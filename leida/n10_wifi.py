"""N10 wireless point-cloud viewer. Run --help for options; no serial dependency."""
import argparse
from dataclasses import dataclass
import math
import socket
import struct
import time
from pathlib import Path
from n10_grid import TwoSnapshots, FIXED_BLOCKED, X_EDGES, Y_FROM_TOP, cell_rect
from n10_planner import RoutePlayer, handle_control, vehicle_pose, lidar_to_pose
from n10_plan_ui import control_at, draw_route, draw_toolbar

SUBSCRIBE = b"N10_SUB"


@dataclass
class Frame:
    sequence: int
    period_us: int
    start: float
    end: float
    samples: list


def decode_packet(packet):
    if len(packet) != 66 or packet[:4] != b"N10U":
        raise ValueError("bad UDP header/length")
    raw = packet[8:]
    if raw[:3] != b"\xa5\x5a\x3a" or (sum(raw[:57]) & 255) != raw[57]:
        raise ValueError("bad N10 header/checksum")
    period, start = struct.unpack_from(">HH", raw, 3)
    end = struct.unpack_from(">H", raw, 55)[0]
    if start >= 36000 or end >= 36000:
        raise ValueError("bad angle")
    return Frame(struct.unpack_from(">I", packet, 4)[0], period, start / 100,
                 end / 100, [struct.unpack_from(">HB", raw, 7 + 3*i) for i in range(16)])


def polar_points(frame):
    step = ((frame.end - frame.start) % 360) / 15
    for i, (distance, intensity) in enumerate(frame.samples):
        angle = (frame.start + i * step) % 360
        yield angle, distance, intensity


class Cloud:
    def __init__(self, max_age=0.35):
        self.max_age = max_age
        self.bins = {}
        self.last_seq = None
        self.last_seen = None
        self.frames = self.lost = self.reordered = 0

    def accept(self, frame, now):
        # After a timeout accept a reset sequence (e.g. ESP reboot).
        if self.last_seq is not None and now - self.last_seen < 2:
            delta = (frame.sequence - self.last_seq) & 0xffffffff
            if delta == 0 or delta >= 0x80000000:
                self.reordered += 1
                return False
            self.lost += delta - 1
        self.last_seq, self.last_seen = frame.sequence, now
        self.frames += 1
        # Remove the old sector, including returns now invalid/zero, to avoid ghost points.
        span = (frame.end - frame.start) % 360
        for key in list(self.bins):
            if (key / 10 - frame.start) % 360 <= span + 0.1:
                del self.bins[key]
        for angle, distance, intensity in polar_points(frame):
            if distance:
                self.bins[int(angle * 10) % 3600] = (angle, distance, intensity, now)
        return True

    def points(self, now):
        self.bins = {k: v for k, v in self.bins.items() if now - v[3] <= self.max_age}
        return list(self.bins.values())


def map_cloud_points(points, zone, pose=None):
    pose = vehicle_pose(None, zone) if pose is None else pose
    result = []
    for angle, distance, intensity, seen in points:
        rad = math.radians(angle)
        x, y = lidar_to_pose(distance * math.cos(rad), distance * math.sin(rad), pose)
        result.append(dict(x_mm=x, y_mm=y, angle_deg=angle, distance_mm=distance,
                           intensity=intensity, received_monotonic=seen))
    return result


def render(cloud, now, args, view, zone, bad, error="", recording=None, points=None,
           player=None, start_zone=None):
    import cv2
    import numpy as np
    image = np.full((980, 900, 3), (26, 22, 19), dtype=np.uint8)
    start_zone = zone if start_zone is None else start_zone
    pose = vehicle_pose(player, start_zone)
    points = cloud.points(now) if points is None else points
    origin, size = 60, 780
    def map_pixel(x, y):
        return round(origin + x / 2400 * size), round(origin + (2400-y) / 2400 * size)
    if view == "map":
        blocked = recording.blocked if recording else set(FIXED_BLOCKED)
        for row in range(1, 6):
            for col in range(1, 6):
                rect = cell_rect(row, col)
                color = (65, 65, 65) if (row, col) in FIXED_BLOCKED else (
                    (65, 65, 155) if (row, col) in blocked else (35, 50, 35))
                cv2.rectangle(image, map_pixel(rect[0], rect[3]), map_pixel(rect[2], rect[1]), color, -1)
                px, py = map_pixel((rect[0]+rect[2])/2, (rect[1]+rect[3])/2)
                label = f'{row},{col} ' + ('BLOCK' if (row, col) in blocked else 'OPEN')
                cv2.putText(image, label, (px-43, py+5), 0, .43, (220, 220, 220), 1)
        for x in X_EDGES:
            cv2.line(image, map_pixel(x, 150), map_pixel(x, 2320), (170, 170, 170))
        for down in Y_FROM_TOP:
            cv2.line(image, map_pixel(150, 2400-down), map_pixel(2400, 2400-down), (170, 170, 170))
        if recording:
            for snapshot in recording.snapshots:
                for point in snapshot['points']:
                    x, y = point['x_mm'], point['y_mm']
                    if 0 <= x < 2400 and 0 <= y < 2400:
                        cv2.circle(image, map_pixel(x, y), 3, (200, 160, 70), 1)
        for point in map_cloud_points(points, start_zone, pose):
            x, y = point['x_mm'], point['y_mm']
            if 0 <= x < 2400 and 0 <= y < 2400:
                cv2.circle(image, map_pixel(x, y), 2, (60, 210, 255), -1)
        draw_route(image, player, start_zone, map_pixel, pose)
        title = f"Car + radar | X {pose[0]:.0f} Y {pose[1]:.0f} mm | heading {pose[2]:.0f} deg | preview pose"
    else:
        center = (450, 450)
        for radius in range(1, 5):
            pixels = radius * size // 8
            cv2.circle(image, center, pixels, (65, 60, 55), 1)
            cv2.putText(image, f"{args.range_mm * radius / 4:.0f} mm", (455, 450-pixels+16), 0, .45, (140, 140, 140), 1)
        cv2.line(image, (60, 450), (840, 450), (65, 60, 55))
        cv2.line(image, (450, 60), (450, 840), (65, 60, 55))
        for angle, distance, intensity, _ in points:
            if distance > args.range_mm: continue
            rad = math.radians(angle)
            # Same sensor convention as map projection: +Y is the car's left.
            scale = size / 2 / args.range_mm
            pixel = (round(450 - distance * math.sin(rad) * scale),
                     round(450 - distance * math.cos(rad) * scale))
            cv2.circle(image, pixel, 2, (60, 150 + intensity * 105 // 255, 255), -1)
        cv2.circle(image, center, 6, (100, 255, 90), -1)
        title = "360 deg | car + radar at center | 0 deg forward, +90 deg left"
    cv2.putText(image, title, (20, 25), 0, .55, (220, 230, 230), 1)
    stale = cloud.last_seen is None or now - cloud.last_seen > 1
    status = "DEMO (synthetic)" if args.demo else ("WAITING / NO DATA" if stale else "LIVE")
    cv2.putText(image, f"{status} | frames {cloud.frames} | points {len(points)} | lost {cloud.lost} | bad {bad}",
                (20, 50), 0, .52, (90, 160, 255) if stale else (100, 255, 150), 1)
    count = len(recording.snapshots) if recording else 0
    draw_toolbar(image, player, start_zone)
    cv2.putText(image, f"Saved {count}/2 | Click MAP: capture | P: plan | SPACE: pause | R: reset | 1/2: car + radar", (20, 915), 0, .48, (220, 220, 220), 1)
    cv2.putText(image, player.status() if player else 'Virtual car preview | no commands sent to robot',
                (20, 940), 0, .48, (100, 240, 220), 1)
    if error: cv2.putText(image, error[:115], (20, 965), 0, .43, (90, 90, 255), 1)
    return image


def demo_packet(sequence):
    raw = bytearray(b"\xa5\x5a\x3a" + bytes(55))
    start = (sequence % 24) * 15
    struct.pack_into(">HH", raw, 3, 4166, start * 100)
    for i in range(16):
        angle = math.radians(start + i)
        distance = int(1400 + 300 * math.cos(3*angle))
        struct.pack_into(">HB", raw, 7 + i*3, distance, 180)
    struct.pack_into(">H", raw, 55, ((start + 15) % 360) * 100)
    raw[57] = sum(raw[:57]) & 255
    return b"N10U" + struct.pack(">I", sequence & 0xffffffff) + raw


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--esp", default="192.168.4.1")
    parser.add_argument("--port", type=int, default=3334)
    parser.add_argument("--view", choices=["radar", "map"], default="map")
    parser.add_argument("--zone", type=int, choices=[1, 2], default=1)
    parser.add_argument("--range-mm", type=int, default=6000)
    parser.add_argument("--max-age", type=float, default=.35, help="point expiry in seconds")
    parser.add_argument("--demo", action="store_true", help="synthetic data, no ESP needed")
    parser.add_argument("--save-dir", type=Path, default=Path(__file__).parent / 'captures',
                        help="directory for two-click point-cloud/grid JSON records")
    args = parser.parse_args()
    if args.range_mm <= 0 or args.max_age <= 0 or not 1 <= args.port <= 65535:
        parser.error("range, max-age and port must be positive (port <= 65535)")
    import cv2
    cloud = Cloud(args.max_age)
    player = RoutePlayer()
    start_zone = args.zone
    recording = TwoSnapshots(args.save_dir, args.demo)
    view, zone, bad, demo_seq = args.view, args.zone, 0, 0
    last_subscribe, next_demo, error = 0, time.monotonic(), ""
    target = (socket.gethostbyname(args.esp), args.port)
    title = "N10 Wi-Fi Point Cloud"
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("", 0))
        sock.setblocking(False)
        # Avoid Windows UDP 'connection reset' exceptions after transient ICMP errors.
        if hasattr(socket, "SIO_UDP_CONNRESET"):
            sock.ioctl(socket.SIO_UDP_CONNRESET, False)
        cv2.namedWindow(title, cv2.WINDOW_NORMAL)
        displayed = {}
        capture_message = ''
        def control(key):
            nonlocal start_zone, zone, view, capture_message
            try:
                start_zone = handle_control(key, recording, player, start_zone, time.monotonic())
                zone = start_zone
                if key in ('plan', 'top', 'bottom', 'replay'):
                    view = 'map'
                capture_message = 'Virtual preview only; no robot commands'
                if key == 'plan':
                    print(f'Planned {len(player.plan.actions)} actions; motion estimate {player.plan.cost:.1f}s', flush=True)
            except ValueError as exc:
                capture_message = str(exc)
        def mouse(event, x, y, flags, param):
            nonlocal capture_message
            if event != cv2.EVENT_LBUTTONDOWN or not displayed:
                return
            button = control_at(x, y)
            if button:
                control(button)
                return
            if not (0 <= x < 900 and 60 <= y < 840):
                return
            try:
                clicked = time.monotonic()
                if not displayed['points'] or clicked-max(p[3] for p in displayed['points']) > args.max_age:
                    raise ValueError('No fresh point cloud; snapshot not saved')
                path = recording.capture(map_cloud_points(displayed['points'], displayed['zone'], displayed['pose']), {
                    'zone': displayed['zone'], 'view': displayed['view'],
                    'car_radar_pose_mm_deg': list(displayed['pose']),
                    'radar_mount': 'car center; 0 deg forward, +90 deg left',
                    'click_pixel': [x, y], 'last_sequence': displayed['sequence'],
                    'clicked_monotonic': clicked,
                    'frames': displayed['frames'], 'lost': displayed['lost'],
                    'range_mm': displayed['range_mm'], 'max_age_seconds': args.max_age,
                })
                capture_message = f'Saved {len(recording.snapshots)}/2: {path.name}'
                player.clear()
                print(f'{capture_message}\n{path}', flush=True)
            except (ValueError, OSError) as exc:
                capture_message = str(exc)
        cv2.setMouseCallback(title, mouse)
        try:
            while True:
                now = time.monotonic()
                if args.demo:
                    while next_demo <= now:
                        cloud.accept(decode_packet(demo_packet(demo_seq)), now)
                        demo_seq += 1
                        next_demo += 1/240
                else:
                    if now - last_subscribe >= 1:
                        try:
                            sock.sendto(SUBSCRIBE, target)
                            error = ""
                        except OSError as exc:
                            error = str(exc)
                        last_subscribe = now
                    # Bound work to keep the window responsive even under a packet flood.
                    for _ in range(512):
                        try:
                            packet, source = sock.recvfrom(2048)
                        except BlockingIOError:
                            break
                        except OSError as exc:
                            error = str(exc)
                            break
                        if source != target: continue
                        try:
                            cloud.accept(decode_packet(packet), now)
                        except ValueError:
                            bad += 1
                shown_points = cloud.points(now)
                player.update(time.monotonic())
                displayed = dict(points=list(shown_points), zone=zone, view=view,
                                 pose=vehicle_pose(player, start_zone),
                                 sequence=cloud.last_seq, frames=cloud.frames, lost=cloud.lost,
                                 range_mm=args.range_mm)
                cv2.imshow(title, render(cloud, now, args, view, zone, bad,
                                        error or capture_message, recording, shown_points, player, start_zone))
                key = cv2.waitKey(16) & 255
                if key in (27, ord('q')) or cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                    break
                if key == ord('m'): view = "map" if view == "radar" else "radar"
                if key == ord('p'): control('plan')
                if key == ord(' '): control('pause')
                if key == ord('r'):
                    recording.reset()
                    player.clear()
                    capture_message = 'New pair started; previous JSON kept'
                if key in (ord('1'), ord('2')):
                    control('bottom' if key == ord('1') else 'top')
                if key in (ord('+'), ord('=')): args.range_mm = min(50000, args.range_mm + 500)
                if key == ord('-'): args.range_mm = max(500, args.range_mm - 500)
        except KeyboardInterrupt:
            pass
        finally:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
