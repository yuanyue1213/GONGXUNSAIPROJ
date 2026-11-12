import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from n10_grid import (TwoSnapshots, cell_at, cell_rect, FIXED_BLOCKED, WIDTHS, HEIGHTS,
                      GRID_LEFT_MM, GRID_RIGHT_MM, GRID_BOTTOM_MM, GRID_TOP_MM,
                      GRID_WIDTH_MM, GRID_HEIGHT_MM)


class GridTests(unittest.TestCase):
    def test_confirmed_geometry_and_all_centers(self):
        self.assertEqual((GRID_WIDTH_MM, GRID_HEIGHT_MM), (2250, 2170))
        self.assertEqual((GRID_LEFT_MM, GRID_RIGHT_MM, GRID_BOTTOM_MM, GRID_TOP_MM),
                         (0, 2250, 0, 2170))
        self.assertEqual(sum(WIDTHS), 2250)
        self.assertEqual(sum(HEIGHTS), 2170)
        for r in range(1, 6):
            for c in range(1, 6):
                x0, y0, x1, y1 = cell_rect(r, c)
                self.assertEqual((x1-x0, y1-y0), (WIDTHS[c-1], HEIGHTS[r-1]))
                self.assertEqual(cell_at((x0+x1)/2, (y0+y1)/2), (r, c))
                self.assertEqual((r, c) in FIXED_BLOCKED, x1-x0 == y1-y0 == 450)

    def test_edges_and_outside(self):
        self.assertEqual(cell_at(0, 2170), (1, 1))
        self.assertEqual(cell_at(400, 1700), (2, 2))
        self.assertEqual(cell_at(2249.9, 0.1), (5, 5))
        for point in [(-1, 1850), (2250, 1850), (350, 2171), (350, 0), (0, -1)]:
            self.assertIsNone(cell_at(*point))

    def test_two_snapshots_union_and_immutable_saved_state(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = TwoSnapshots(directory)
            self.assertEqual(recorder.blocked, set(FIXED_BLOCKED))
            points = [{'x_mm': 150, 'y_mm': 1950}]
            path = recorder.capture(points, {'zone': 2})
            self.assertFalse(json.loads(path.read_text())['complete'])
            points[0]['x_mm'] = 9999
            self.assertEqual(recorder.snapshots[0]['points'][0]['x_mm'], 150)
            self.assertEqual(path, recorder.capture([{'x_mm': 1550, 'y_mm': 150}], {'zone': 1}))
            self.assertEqual(recorder.blocked, set(FIXED_BLOCKED) | {(1, 1), (5, 4)})
            saved = json.loads(path.read_text())
            self.assertTrue(saved['complete'])
            self.assertEqual(saved['schema'], 'n10-two-snapshots-v2')
            self.assertEqual(saved['grid']['origin'], 'red-grid-bottom-left')
            self.assertEqual(len(saved['snapshots']), 2)
            self.assertFalse(saved['passable'][0][0])
            self.assertFalse(saved['passable'][1][1])
            self.assertTrue(saved['passable'][2][2])
            with self.assertRaises(ValueError): recorder.capture(points, {})
            recorder.reset()
            self.assertTrue(path.exists())
            self.assertEqual(recorder.blocked, set(FIXED_BLOCKED))
            new_path = recorder.capture([{'x_mm': 300, 'y_mm': 2100}], {})
            self.assertNotEqual(path, new_path)

    def test_no_data_and_failed_save_do_not_consume_click(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = TwoSnapshots(directory)
            with self.assertRaises(ValueError): recorder.capture([], {})
            with patch.object(Path, 'replace', side_effect=OSError('disk error')):
                with self.assertRaises(OSError):
                    recorder.capture([{'x_mm': 300, 'y_mm': 2100}], {})
            self.assertEqual(recorder.snapshots, [])
            self.assertEqual(recorder.blocked, set(FIXED_BLOCKED))

    def test_start_points_saved_but_never_block_used_starts(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = TwoSnapshots(directory)
            bottom = {'x_mm': 2050, 'y_mm': 150}
            top = {'x_mm': 2050, 'y_mm': 1950}
            neighbor = {'x_mm': 1550, 'y_mm': 150}
            path = recorder.capture([bottom, top, neighbor], {'zone': 1})
            self.assertNotIn((5, 5), recorder.blocked)
            self.assertIn((1, 5), recorder.blocked)
            recorder.capture([top, bottom], {'zone': 2})
            self.assertNotIn((1, 5), recorder.blocked)
            self.assertNotIn((5, 5), recorder.blocked)
            self.assertIn((5, 4), recorder.blocked)
            self.assertTrue(set(FIXED_BLOCKED) <= recorder.blocked)
            saved = json.loads(path.read_text())
            self.assertEqual(len(saved['snapshots'][0]['points']), 3)
            self.assertEqual(saved['ignored_start_cells'], [[1, 5], [5, 5]])
            self.assertTrue(saved['passable'][0][4])
            self.assertTrue(saved['passable'][4][4])
            recorder.reset()
            self.assertEqual(recorder.start_cells, set())

    def test_occupied_map_render(self):
        import cv2
        from types import SimpleNamespace
        from n10_wifi import Cloud, render
        with tempfile.TemporaryDirectory() as directory:
            recorder = TwoSnapshots(directory, demo=True)
            recorder.capture([{'x_mm': 300, 'y_mm': 2100}], {'zone': 2})
            recorder.capture([{'x_mm': 1550, 'y_mm': 150}], {'zone': 1})
            image = render(Cloud(), 1, SimpleNamespace(demo=True, range_mm=6000),
                           'map', 1, 0, recording=recorder)
            out = Path(__file__).parent / 'preview'
            out.mkdir(exist_ok=True)
            self.assertTrue(cv2.imwrite(str(out / 'two-snapshots.png'), image))

    def test_work_area_cloud_saved_without_blocking_mission(self):
        from n10_planner import center, plan_mission
        work_cells = {(1, 3), (5, 3), (3, 1)}
        cells = work_cells | set(FIXED_BLOCKED) | {(3, 3)}
        points = [dict(zip(('x_mm', 'y_mm'), center(*cell))) for cell in sorted(cells)]
        with tempfile.TemporaryDirectory() as directory:
            recorder = TwoSnapshots(directory)
            for zone in (1, 2):
                path = recorder.capture(points, {'zone': zone})
                self.assertEqual(recorder.blocked, set(FIXED_BLOCKED) | {(3, 3)})
            saved = json.loads(path.read_text())
            for snapshot in saved['snapshots']:
                self.assertEqual(snapshot['points'], points)
                self.assertTrue(work_cells.isdisjoint(map(tuple, snapshot['hit_cells'])))
            for row, col in work_cells:
                self.assertTrue(saved['passable'][row-1][col-1])
            self.assertFalse(saved['passable'][2][2])
            for zone in (1, 2):
                self.assertIsNotNone(plan_mission(recorder.blocked, zone))


if __name__ == '__main__':
    unittest.main()
