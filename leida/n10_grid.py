"""Confirmed 5x5 track geometry and two-snapshot occupancy recording."""
from bisect import bisect_right
from datetime import datetime, timezone
import json
from pathlib import Path
from uuid import uuid4

X_EDGES = (0, 400, 850, 1250, 1700, 2250)
Y_FROM_TOP = (0, 470, 920, 1320, 1770, 2170)
WIDTHS = (400, 450, 400, 450, 550)
HEIGHTS = (470, 450, 400, 450, 400)
# 红线区域左下角为(0,0)，X向右、Y向上；STM32与PC共用此坐标系。
GRID_LEFT_MM, GRID_RIGHT_MM = X_EDGES[0], X_EDGES[-1]
GRID_BOTTOM_MM = 0
GRID_TOP_MM = Y_FROM_TOP[-1]
GRID_WIDTH_MM = GRID_RIGHT_MM-GRID_LEFT_MM
GRID_HEIGHT_MM = GRID_TOP_MM-GRID_BOTTOM_MM
FIXED_BLOCKED = frozenset({(2, 2), (2, 4), (4, 2), (4, 4)})
WORK_CELLS = frozenset({(1, 3), (5, 3), (3, 1)})  # 原料、粗加工、暂存：点云不计障碍。
START_CELLS = {1: (5, 5), 2: (1, 5)}  # 通信zone：1右下、2右上；键盘：1右上、2右下。


def cell_at(x, y):
    """Map mm (bottom-left origin) to 1-based (row from top, column from left).

    Internal boundaries belong to the right/lower cell; outside the grid is ignored.
    """
    down = GRID_TOP_MM - y
    if not (GRID_LEFT_MM <= x < GRID_RIGHT_MM and 0 <= down < GRID_HEIGHT_MM):
        return None
    return bisect_right(Y_FROM_TOP, down), bisect_right(X_EDGES, x)


def cell_rect(row, col):
    return (X_EDGES[col-1], GRID_TOP_MM-Y_FROM_TOP[row],
            X_EDGES[col], GRID_TOP_MM-Y_FROM_TOP[row-1])


class TwoSnapshots:
    def __init__(self, directory, demo=False):
        self.directory = Path(directory)
        self.demo = demo
        self.reset()

    def reset(self):
        self.snapshots = []
        self.path = None
        self.detected = set()
        self.start_cells = set()

    @property
    def blocked(self):
        return set(FIXED_BLOCKED) | self.detected

    def capture(self, points, metadata):
        if len(self.snapshots) >= 2:
            raise ValueError("Two snapshots complete; press R to start a new pair")
        if not points:
            raise ValueError("No fresh point cloud; snapshot not saved")
        # Copy JSON values so later cloud updates cannot change an accepted snapshot.
        points = json.loads(json.dumps(points, allow_nan=False))
        start_cell = START_CELLS.get(metadata.get('zone'))
        start_cells = self.start_cells | ({start_cell} if start_cell else set())
        ignored = start_cells | WORK_CELLS
        hits = sorted({cell for p in points if (cell := cell_at(p['x_mm'], p['y_mm']))}
                      - ignored)
        # Both samples share the same start exclusions, even if the second start
        # was seen as an obstacle from the first viewpoint before being selected.
        detected = (self.detected | set(hits)) - ignored
        blocked = set(FIXED_BLOCKED) | detected
        snapshot = {
            'number': len(self.snapshots) + 1,
            'clicked_at_utc': datetime.now(timezone.utc).isoformat(),
            'metadata': dict(metadata), 'points': points,
            'hit_cells': [list(c) for c in hits],
            'start_cell': list(start_cell) if start_cell else None,
        }
        snapshots = self.snapshots + [snapshot]
        document = {
            'schema': 'n10-two-snapshots-v2', 'demo': self.demo,
            'complete': len(snapshots) == 2,
            'grid': {'units': 'mm', 'origin': 'red-grid-bottom-left',
                     'rows': 'top-to-bottom, 1-based', 'columns': 'left-to-right, 1-based',
                     'widths': WIDTHS, 'heights': HEIGHTS,
                     'x_edges': X_EDGES, 'y_edges_from_top': Y_FROM_TOP,
                     'boundary_rule': 'internal edge -> right/lower cell; outer max excluded'},
            'fixed_blocked': [list(c) for c in sorted(FIXED_BLOCKED)],
            'ignored_start_cells': [list(c) for c in sorted(start_cells)],
            'detected_blocked': [list(c) for c in sorted(detected)],
            'blocked': [list(c) for c in sorted(blocked)],
            'passable': [[(r, c) not in blocked for c in range(1, 6)] for r in range(1, 6)],
            'snapshots': snapshots,
        }
        self.directory.mkdir(parents=True, exist_ok=True)
        path = self.path or self.directory / (
            datetime.now().strftime('%Y%m%d-%H%M%S-') + uuid4().hex[:8] + '.json')
        temp = path.with_suffix('.tmp')
        temp.write_text(json.dumps(document, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
        temp.replace(path)
        # Commit only after the file has been written successfully.
        self.snapshots, self.detected, self.path = snapshots, detected, path
        self.start_cells = start_cells
        return path
