import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from n10_grid import TwoSnapshots, cell_at, cell_rect, FIXED_BLOCKED, WIDTHS, HEIGHTS


class GridTests(unittest.TestCase):
    def test_confirmed_geometry_and_all_centers(self):
        self.assertEqual(sum(WIDTHS), 2250)
        self.assertEqual(sum(HEIGHTS), 2170)
        for r in range(1, 6):
            for c in range(1, 6):
                x0, y0, x1, y1 = cell_rect(r, c)
                self.assertEqual((x1-x0, y1-y0), (WIDTHS[c-1], HEIGHTS[r-1]))
                self.assertEqual(cell_at((x0+x1)/2, (y0+y1)/2), (r, c))
                self.assertEqual((r, c) in FIXED_BLOCKED, x1-x0 == y1-y0 == 450)

    def test_edges_and_outside(self):
        self.assertEqual(cell_at(150, 2320), (1, 1))
        self.assertEqual(cell_at(550, 1850), (2, 2))
        self.assertEqual(cell_at(2399.9, 150.1), (5, 5))
        for point in [(149, 2000), (2400, 2000), (500, 2321), (500, 150), (0, 0)]:
            self.assertIsNone(cell_at(*point))

    def test_two_snapshots_union_and_immutable_saved_state(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = TwoSnapshots(directory)
            self.assertEqual(recorder.blocked, set(FIXED_BLOCKED))
            points = [{'x_mm': 300, 'y_mm': 2100}]
            path = recorder.capture(points, {'zone': 2})
            self.assertFalse(json.loads(path.read_text())['complete'])
            points[0]['x_mm'] = 9999
            self.assertEqual(recorder.snapshots[0]['points'][0]['x_mm'], 300)
            self.assertEqual(path, recorder.capture([{'x_mm': 1700, 'y_mm': 300}], {'zone': 1}))
            self.assertEqual(recorder.blocked, set(FIXED_BLOCKED) | {(1, 1), (5, 4)})
            saved = json.loads(path.read_text())
            self.assertTrue(saved['complete'])
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
            bottom = {'x_mm': 2200, 'y_mm': 300}
            top = {'x_mm': 2200, 'y_mm': 2100}
            neighbor = {'x_mm': 1700, 'y_mm': 300}
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
            recorder.capture([{'x_mm': 1700, 'y_mm': 300}], {'zone': 1})
            image = render(Cloud(), 1, SimpleNamespace(demo=True, range_mm=6000),
                           'map', 1, 0, recording=recorder)
            out = Path(__file__).parent / 'preview'
            out.mkdir(exist_ok=True)
            self.assertTrue(cv2.imwrite(str(out / 'two-snapshots.png'), image))


if __name__ == '__main__':
    unittest.main()
