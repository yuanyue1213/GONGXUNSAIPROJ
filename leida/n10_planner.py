"""Heading-aware whole-mission A* and a clock-driven virtual car (no hardware IO)."""
from dataclasses import dataclass, field
import heapq
import math

from n10_grid import FIXED_BLOCKED, START_CELLS, cell_rect

HEADINGS = ((-1, 0), (0, 1), (1, 0), (0, -1))  # up, right, down, left


def center(row, col):
    x0, y0, x1, y1 = cell_rect(row, col)
    return (x0+x1)/2, (y0+y1)/2


def vehicle_pose(player, start_zone):
    """One shared car/radar pose; headings are clockwise degrees from map up."""
    if player is not None and player.plan is not None:
        return player.pose()
    return (*center(*START_CELLS[start_zone]), 0. if start_zone == 1 else 180.)


def lidar_to_pose(forward, left, pose):
    """Radar at car center, 0 deg forward / +90 deg left (original N10 TF)."""
    x, y, heading = pose
    angle = math.radians(heading)
    return (x + math.sin(angle)*forward - math.cos(angle)*left,
            y + math.cos(angle)*forward + math.sin(angle)*left)


@dataclass
class Action:
    kind: str
    start: tuple
    end: tuple
    duration: float
    distance: float = 0
    task: str = ''
    cells: list = field(default_factory=list)


@dataclass
class Plan:
    start_zone: int
    start: tuple
    actions: list
    cost: float
    blocked: frozenset


def mission(start_zone):
    # Car right = east/north/south/west at the respective boundary station.
    return [('QR', (3, 5), 0), ('RAW 1', (1, 3), 3),
            ('COARSE 1: place / take', (5, 3), 1), ('TEMP 1: place', (3, 1), 2),
            ('RAW 2', (1, 3), 3), ('COARSE 2: place / take', (5, 3), 1),
            ('TEMP 2: stack', (3, 1), 2), ('RETURN', START_CELLS[start_zone], None)]


def plan_mission(blocked, start_zone, forward_speed=400, backward_speed=400, turn_seconds=.8):
    """Use mm/s and seconds as preview costs; assume every open cell is passable."""
    if start_zone not in START_CELLS:
        raise ValueError('Unknown start position')
    if any(not math.isfinite(v) or v <= 0 for v in (forward_speed, backward_speed, turn_seconds)):
        raise ValueError('Motion costs must be finite and positive')
    blocked = frozenset(blocked) | FIXED_BLOCKED
    start_cell = START_CELLS[start_zone]
    if start_cell in blocked:
        raise ValueError('Start cell is blocked; capture at this start first')
    targets = mission(start_zone)
    for name, cell, _ in targets:
        if cell in blocked:
            raise ValueError(f'Unreachable {name}: cell {cell} is blocked')
    # Preflight reachability gives the failed task, without relaxing any obstacles.
    reachable, pending = {start_cell}, [start_cell]
    while pending:
        r, c = pending.pop()
        for dr, dc in HEADINGS:
            nxt = (r+dr, c+dc)
            if 1 <= nxt[0] <= 5 and 1 <= nxt[1] <= 5 and nxt not in blocked and nxt not in reachable:
                reachable.add(nxt)
                pending.append(nxt)
    for name, cell, _ in targets:
        if cell not in reachable:
            raise ValueError(f'Unreachable {name}: no open route to {cell}')

    # State = mission stage, row, column, heading. Search all arrival headings
    # jointly, so a locally short leg cannot force a worse next leg.
    start = (0, *start_cell, 0 if start_zone == 1 else 2)
    speed = max(forward_speed, backward_speed)
    def heuristic(state):
        stage, row, col, _ = state
        here = center(row, col)
        distance = 0
        for _, cell, _ in targets[stage:]:
            there = center(*cell)
            distance += abs(here[0]-there[0]) + abs(here[1]-there[1])
            here = there
        return distance/speed

    best, parent = {start: (0., 0)}, {}
    queue = [(heuristic(start), 0., 0, start)]
    end = None
    while queue:
        _, cost, turns, state = heapq.heappop(queue)
        if best.get(state) != (cost, turns):
            continue
        stage, row, col, heading = state
        if stage == len(targets):
            end = state
            break
        pose = (row, col, heading)
        name, goal, required_heading = targets[stage]
        if (row, col) == goal and (required_heading is None or heading == required_heading):
            neighbors = [((stage+1, row, col, heading),
                          Action('VISIT', pose, pose, 1., task=name), 0.)]
        else:
            neighbors = []
            for kind, dh in [('L', -1), ('R', 1)]:
                dest = (row, col, (heading+dh) % 4)
                neighbors.append(((stage, *dest), Action(kind, pose, dest, turn_seconds), turn_seconds))
            dr, dc = HEADINGS[heading]
            for kind, sign, velocity in [('F', 1, forward_speed), ('B', -1, backward_speed)]:
                nr, nc = row+sign*dr, col+sign*dc
                if not (1 <= nr <= 5 and 1 <= nc <= 5) or (nr, nc) in blocked:
                    continue
                p, q = center(row, col), center(nr, nc)
                distance = abs(p[0]-q[0])+abs(p[1]-q[1])
                dest = (nr, nc, heading)
                neighbors.append(((stage, *dest), Action(kind, pose, dest, distance/velocity,
                                                        distance, cells=[(row, col), (nr, nc)]), distance/velocity))
        for dest, action, increment in neighbors:
            candidate = (cost+increment, turns+(action.kind in ('L', 'R')))
            if candidate < best.get(dest, (float('inf'), float('inf'))):
                best[dest], parent[dest] = candidate, (state, action)
                heapq.heappush(queue, (candidate[0]+heuristic(dest), *candidate, dest))
    if end is None:
        raise ValueError('No complete mission route')
    actions = []
    cursor = end
    while cursor != start:
        cursor, action = parent[cursor]
        actions.append(action)
    actions.reverse()
    merged = []
    for action in actions:
        if (merged and action.kind in ('F', 'B') and merged[-1].kind == action.kind
                and merged[-1].end == action.start):
            previous = merged[-1]
            previous.end = action.end
            previous.duration += action.duration
            previous.distance += action.distance
            previous.cells.extend(action.cells[1:])
        else:
            merged.append(action)
    return Plan(start_zone, start[1:], merged, best[end][0], blocked)


class RoutePlayer:
    def __init__(self, speed=2.):
        self.speed = speed
        self.clear()

    def clear(self):
        self.plan = None
        self.index = 0
        self.elapsed = 0.
        self.last_time = None
        self.paused = False

    def load(self, plan, now):
        self.clear()
        self.plan, self.last_time = plan, now

    @property
    def finished(self):
        return self.plan is not None and self.index >= len(self.plan.actions)

    def update(self, now):
        if self.last_time is None:
            self.last_time = now
        dt = max(0., now-self.last_time)
        self.last_time = now
        if not self.plan or self.paused or self.finished:
            return
        self.elapsed += dt*self.speed
        while not self.finished and self.elapsed >= self.plan.actions[self.index].duration:
            self.elapsed -= self.plan.actions[self.index].duration
            self.index += 1
        if self.finished:
            self.elapsed = 0.

    def toggle_pause(self, now):
        self.update(now)
        if self.plan and not self.finished:
            self.paused = not self.paused

    def pose(self):
        if not self.plan:
            return None
        if self.finished:
            row, col, heading = self.plan.actions[-1].end
            return (*center(row, col), heading*90.)
        action = self.plan.actions[self.index]
        fraction = min(1., self.elapsed/action.duration)
        x0, y0 = center(*action.start[:2])
        x1, y1 = center(*action.end[:2])
        angle = action.start[2]*90.
        if action.kind in ('L', 'R'):
            angle += (-90 if action.kind == 'L' else 90)*fraction
        return (x0+(x1-x0)*fraction, y0+(y1-y0)*fraction, angle % 360)

    def status(self):
        if not self.plan:
            return 'Complete two captures, choose a start, then click PLAN'
        if self.finished:
            return 'DONE: returned to the original start'
        action = self.plan.actions[self.index]
        label = action.task if action.kind == 'VISIT' else {
            'F': 'FORWARD', 'B': 'BACKWARD', 'L': 'LEFT 90', 'R': 'RIGHT 90'}[action.kind]
        return f'{"PAUSED | " if self.paused else ""}{self.index+1}/{len(self.plan.actions)} | {label} | {action.distance:.0f} mm | preview {self.speed:g}x'


def handle_control(control, recording, player, start_zone, now):
    """Shared by mouse and keyboard; a toolbar action never captures lidar data."""
    if control == 'plan':
        player.clear()
        if len(recording.snapshots) != 2:
            raise ValueError('Save two point-cloud snapshots before planning')
        player.load(plan_mission(recording.blocked, start_zone), now)
    elif control == 'pause':
        player.toggle_pause(now)
    elif control == 'replay' and player.plan:
        player.load(player.plan, now)
    elif control in ('top', 'bottom'):
        start_zone = 2 if control == 'top' else 1
        player.clear()
    return start_zone
