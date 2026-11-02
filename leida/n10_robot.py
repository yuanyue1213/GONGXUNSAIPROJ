"""Actual robot link: stopped full-circle captures, then stop-and-wait route upload.

No drive commands are sent from animation time; STM32 executes all motion.
"""
from dataclasses import dataclass
import math
import socket
import time
import zlib

from n10_planner import center, lidar_to_pose, plan_mission
from n10_grid import START_CELLS


def initial_pose(zone):
    return (2250., 2250., 180.) if zone == 2 else (2250., 150., 0.)


def apply_step(pose, kind, direction, amount):
    x, y, h = pose
    if kind == 'TURN':
        return x, y, (h + (amount if direction == 'R' else -amount)) % 360
    forward = amount if direction == 'F' else -amount if direction == 'B' else 0
    left = amount if direction == 'L' else -amount if direction == 'R' else 0
    return (*lidar_to_pose(forward, left, pose), h)


@dataclass
class Step:
    leg: int
    kind: str
    direction: str
    amount: int
    pose: tuple
    plan_index: int = 0

    def line(self, run, index):
        return f'NAV_STEP,{run},{index},{self.leg},{self.kind},{self.direction},{self.amount}\n'


def route_steps(plan, scan_pose):
    """Connect actual scan position to grid, then return to the original start area."""
    if plan.start[2] != 3 or scan_pose[2] != 270:
        raise ValueError('Hardware plan must start facing map left')
    pose = tuple(scan_pose)
    result = []

    def add(leg, kind, direction, amount, index=0):
        nonlocal pose
        amount = round(amount)
        if not amount:
            return
        if not 0 < amount <= (90 if kind == 'TURN' else 10000):
            raise ValueError('Navigation amount out of range')
        pose = apply_step(pose, kind, direction, amount)
        result.append(Step(leg, kind, direction, amount, pose, index))

    gx, gy = center(*plan.start[:2])
    # At heading270, body-right is map-up. Lower start315 -> grid center350.
    add(1, 'MOVE', 'F' if gx < pose[0] else 'B', abs(gx-pose[0]))
    add(1, 'MOVE', 'R' if gy > pose[1] else 'L', abs(gy-pose[1]))
    leg = 1
    for index, action in enumerate(plan.actions):
        if action.kind == 'VISIT':
            leg += 1
            continue
        add(leg, 'MOVE' if action.kind in ('F', 'B') else 'TURN', action.kind,
            action.distance if action.kind in ('F', 'B') else 90, index)
    if leg != 9:
        raise ValueError('Expected all eight mission visits')
    target = initial_pose(plan.start_zone)
    # Restore initial heading, then reverse the entrance translation.
    delta = (target[2]-pose[2]) % 360
    while delta:
        direction = 'L' if delta == 270 else 'R'
        add(8, 'TURN', direction, 90, len(plan.actions)-1)
        delta = (target[2]-pose[2]) % 360
    add(8, 'MOVE', 'B', abs(target[1]-pose[1]), len(plan.actions)-1)
    add(8, 'MOVE', 'L' if plan.start_zone == 2 else 'R', target[0]-pose[0], len(plan.actions)-1)
    if math.dist(pose[:2], target[:2]) > .1 or pose[2] != target[2]:
        raise ValueError('Return connector mismatch')
    if len(result) > 128 or {s.leg for s in result} != set(range(1, 9)):
        raise ValueError('Invalid route length/legs')
    return result


def upload_frames(run, zone, steps):
    lines = [s.line(run, i) for i, s in enumerate(steps)]
    body = f'{run},{zone},{len(lines)}\n' + ''.join(lines)
    crc = zlib.crc32(body.encode('ascii')) & 0xffffffff
    return [f'NAV_BEGIN,{run},{zone},{len(lines)},{crc:08X}\n', *lines, f'NAV_END,{run}\n']


class FullScan:
    """Discard the first partial revolution; collect a fresh, ordered full turn."""
    def __init__(self, first_sequence):
        self.floor = first_sequence
        self.last_sequence = self.last_angle = None
        self.started = False
        self.points = []
        self.coverage = set()

    def feed(self, frame, points):
        if ((frame.sequence-self.floor) & 0xffffffff) >= 0x80000000:
            return None
        if self.last_sequence is not None:
            delta = (frame.sequence-self.last_sequence) & 0xffffffff
            if not 0 < delta < 0x80000000:
                return None
        self.last_sequence = frame.sequence
        wrapped = self.last_angle is not None and self.last_angle-frame.start > 180
        self.last_angle = frame.start
        if wrapped:
            if self.started and len(self.coverage) >= 35 and self.points:
                return list(self.points)
            self.started = True
            self.points.clear()
            self.coverage.clear()
        if self.started:
            # Coverage includes zero returns, while obstacle points exclude them.
            for angle, distance, intensity in points:
                self.coverage.add(int(angle//10))
                if distance:
                    self.points.append((angle, distance, intensity))
        return None


class RobotLink:
    def __init__(self, host, zone, recording, player, port=3335):
        self.host, self.port, self.zone = host, port, zone
        self.recording, self.player = recording, player
        self.sock = None
        self.rx = bytearray()
        self.tx = bytearray()
        self.next_connect = self.hello_at = 0.
        self.run = None
        self.active = False
        self.scan_key = None
        self.collector = None
        self.completed = set()
        self.pose = initial_pose(zone)
        self.status = 'Connecting PC control link'
        self.frames = []
        self.steps = []
        self.pending = None
        self.sent_at = 0.
        self.frame_index = 0

    def set_zone(self, zone):
        if zone not in (1, 2):
            raise ValueError('Invalid start zone')
        if self.active:
            raise ValueError('Robot running: start selection is locked')
        self.zone = zone
        self.pose = initial_pose(zone)
        self.hello_at = 0.
        self.status = 'Waiting for start selection confirmation'
        self.player.clear()

    def send(self, line):
        self.tx.extend(line.encode('ascii'))

    def disconnect(self):
        if self.sock:
            self.sock.close()
        self.sock = None
        self.rx.clear()
        self.tx.clear()

    def close(self):
        self.disconnect()

    def _send_upload(self, now):
        if self.frame_index >= len(self.frames):
            return
        self.pending = self.frames[self.frame_index]
        self.send(self.pending)
        self.sent_at = now

    def line(self, line, now):
        fields = line.split(',')
        if fields[0] == 'PC_READY' and len(fields) == 2:
            if not self.active and int(fields[1]) == self.zone:
                self.status = f'Start {self.zone} confirmed; press App START'
        elif fields[0] == 'SCAN_READY' and len(fields) == 7:
            run, scan = map(int, fields[1:3])
            if scan not in (1, 2) or run <= 0:
                raise ValueError('Invalid scan request')
            key = (run, scan)
            if key in self.completed:
                self.send(f'SCAN_OK,{run},{scan}\n')
                return
            first_sequence = int(fields[6])
            if key == self.scan_key and self.collector and first_sequence == self.collector.floor:
                return
            if run != self.run:
                if scan != 1:
                    raise ValueError('First scan missing; restart robot at start area')
                self.run = run
                self.completed.clear()
                self.recording.reset()
                self.player.clear()
                self.frames.clear()
                self.pending = None
            self.active = True
            pose = tuple(map(float, fields[3:6]))
            expected = (2125., 2085. if self.zone == 2 else 315.,
                        (180. if self.zone == 2 else 0.) if scan == 1 else 270.)
            if not all(math.isfinite(a) and abs(a-b) <= .1 for a, b in zip(pose, expected)):
                raise ValueError('Scan pose/start configuration mismatch')
            self.scan_key, self.pose = key, pose
            self.collector = FullScan(first_sequence)
            self.status = f'Scan {scan}/2: waiting for complete 360 deg'
        elif fields[0] == 'NAV_ACK' and len(fields) == 3 and int(fields[1]) == self.run and self.pending:
            expected = 'BEGIN' if self.frame_index == 0 else str(self.frame_index-1)
            if fields[2] == expected:
                self.frame_index += 1
                self._send_upload(now)
        elif fields[0] == 'NAV_READY' and len(fields) == 3 and int(fields[1]) == self.run:
            if self.pending and self.pending.startswith('NAV_END,'):
                self.pending = None
                self.status = 'Path ready; STM32 executing'
        elif fields[0] == 'NAV_STATE' and len(fields) == 5 and int(fields[1]) == self.run:
            index = int(fields[3])
            if not 0 <= index < len(self.steps):
                raise ValueError('Unknown robot step')
            self.status = f'Leg {fields[2]}/8 step {index+1}/{len(self.steps)} {fields[4]} (command pose)'
            self.player.index = self.steps[index].plan_index
            if fields[4] == 'DONE':
                self.pose = self.steps[index].pose
        elif fields[0] == 'AUTO' and len(fields) >= 8:
            if fields[2] in ('STARTING', 'RUNNING', 'WORK', 'SCANNING', 'TURNING'):
                self.active = True
            if fields[2] in ('DONE', 'STOPPED', 'ERROR'):
                self.active = False
                self.pending = None
                self.collector = None
                self.status = ' / '.join((fields[2], fields[-1]))
        elif fields[0] in ('NAV_ERROR', 'BRIDGE_ERROR'):
            self.pending = None
            self.status = line

    def feed_scan(self, frame, points, now):
        if self.collector is None:
            return
        complete = self.collector.feed(frame, points)
        if complete is None:
            return
        run, number = self.scan_key
        mapped = []
        for angle, distance, intensity in complete:
            rad = math.radians(angle)
            x, y = lidar_to_pose(distance*math.cos(rad), distance*math.sin(rad), self.pose)
            mapped.append(dict(x_mm=x, y_mm=y, angle_deg=angle, distance_mm=distance, intensity=intensity))
        self.recording.capture(mapped, dict(zone=self.zone, run=run, scan=number,
            car_radar_pose_mm_deg=list(self.pose), first_sequence=self.collector.floor))
        self.completed.add(self.scan_key)
        self.collector = None
        self.send(f'SCAN_OK,{run},{number}\n')
        if number == 1:
            self.status = 'Scan1 saved; waiting for turn and Scan2'
            return
        plan = plan_mission(self.recording.blocked, self.zone, start_heading=3)
        self.steps = route_steps(plan, self.pose)
        self.frames = upload_frames(run, self.zone, self.steps)
        self.player.load(plan, now)
        self.player.paused = True
        self.frame_index = 0
        self._send_upload(now)
        self.status = f'Uploading {len(self.steps)} navigation steps'

    def poll(self, now):
        if self.sock is None:
            if now < self.next_connect:
                return
            self.next_connect = now+2
            try:
                self.sock = socket.create_connection((self.host, self.port), timeout=.2)
                self.sock.setblocking(False)
                self.hello_at = 0
                self.status = 'PC connected; waiting for start selection confirmation'
                if self.pending:
                    self.send(self.pending)
            except OSError:
                self.disconnect()
                self.status = 'PC control offline; reconnecting'
                return
        try:
            if len(self.tx) > 8192:
                raise ConnectionError('Control send queue full')
            if now-self.hello_at >= 1:
                self.send(f'PC_HELLO,{self.zone}\n')
                self.hello_at = now
            if self.pending and now-self.sent_at >= 1:
                self.send(self.pending)
                self.sent_at = now
            if self.tx:
                try:
                    n = self.sock.send(self.tx)
                    if n <= 0:
                        raise ConnectionError('Closed socket')
                    del self.tx[:n]
                except BlockingIOError:
                    pass
            for _ in range(16):
                try:
                    data = self.sock.recv(2048)
                except BlockingIOError:
                    break
                if not data:
                    raise ConnectionError('Closed socket')
                self.rx.extend(data)
                if len(self.rx) > 16384:
                    raise ValueError('Oversized control response')
                while b'\n' in self.rx:
                    line, _, rest = self.rx.partition(b'\n')
                    self.rx[:] = rest
                    try:
                        self.line(line.decode('ascii').rstrip('\r'), now)
                    except (ValueError, UnicodeError) as exc:
                        self.status = str(exc)
        except (OSError, ValueError):
            self.disconnect()
            self.status = 'PC disconnected; STM32 retains its current stage'
