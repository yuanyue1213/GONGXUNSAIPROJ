"""Synthetic image and UART dispatch tests; no MaixCAM hardware required."""
import importlib.util
import math
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('merged_vision', ROOT / 'camera/MaixVision/vision.py')
v = importlib.util.module_from_spec(spec)
spec.loader.exec_module(v)
BGR = {'red': (30, 30, 220), 'yellow': (30, 220, 220), 'blue': (220, 30, 30),
       'black': (15, 15, 15), 'light_blue': (220, 220, 30), 'green': (30, 220, 30)}

def calibration(path, colors=('red', 'yellow', 'blue')):
    centers = {name: np.array(lab, np.float32) for name, lab in v.DEFAULT_LAB_CENTER.items()}
    tolerances = {name: np.array((8, 8, 8), np.float32) for name in centers}
    for name in colors:
        centers[name] = cv2.cvtColor(np.array([[BGR[name]]], np.float32) / 255, cv2.COLOR_BGR2LAB)[0, 0]
    assert v.save_lab_thresholds(centers, tolerances, set(colors), str(path))

def frame(colors=('red', 'yellow', 'blue'), angle=0, missing=None, origin=(256, 160), invalid=False):
    img = np.full((320, 512, 3), 110, np.uint8)
    for i, color in enumerate(colors):
        if color == missing:
            continue
        t = angle + i * 2 * math.pi / 3
        p = (round(origin[0] + 70 * math.cos(t)), round(origin[1] + 70 * math.sin(t)))
        if invalid:
            p = (150 + i * 70, 160)
        cv2.circle(img, p, 13, BGR[color], -1)
    return img

def elliptical_frame(colors=('red', 'yellow', 'blue'), angle=0., axes=(100., 60.), tilt=0.,
                     origin=(256., 160.), extras=(), missing=()):
    img = np.full((320, 512, 3), 110, np.uint8)
    rotation = np.array(((math.cos(tilt), -math.sin(tilt)), (math.sin(tilt), math.cos(tilt))))
    points = {}
    for i, color in enumerate(colors):
        t = angle + i*2*math.pi/3
        point = np.array(origin) + rotation.dot((axes[0]*math.cos(t), axes[1]*math.sin(t)))
        points[color] = point
        if color not in missing:
            cv2.circle(img, tuple(round(float(x)) for x in point), 13, BGR[color], -1)
    for color, point, radius in extras:
        cv2.circle(img, point, radius, BGR[color], -1)
    return img, points


class TurntableTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.path = Path(self.temp.name) / 'lab.json'
        calibration(self.path)
        self.table = v.TurntableTaskDetector(str(self.path))

    def tearDown(self):
        self.temp.cleanup()

    def test_idle_preview_explains_calibration_and_detects_default_colors(self):
        preview = v.IdleTablePreview(self.table)
        self.assertIsNone(preview.draw(frame(('red', 'black', 'light_blue'))))
        self.assertIn('NEED LAB', preview.state)
        self.assertIn('black', preview.state)
        calibration(self.path, ('red', 'black', 'light_blue'))
        table = v.TurntableTaskDetector(str(self.path))
        preview = v.IdleTablePreview(table)
        for i in range(4):
            self.assertIsNone(preview.draw(frame(('red', 'black', 'light_blue'), angle=i*.1)))
        center = preview.draw(frame(('red', 'black', 'light_blue'), angle=.4))
        self.assertIsNotNone(center)
        self.assertLessEqual(np.linalg.norm(np.array(center) - (256, 160)), 1)
        self.assertEqual(len(preview.detector.last_points), 3)
        self.assertIsNone(table.detector)  # Preview never primes the task detector.
        self.assertIsNone(preview.draw(frame(('red', 'black', 'light_blue'), missing='black')))
        self.assertIn('MISSING: black', preview.state)
        for _ in range(4):
            self.assertIsNone(preview.draw(frame(('red', 'black', 'light_blue'), angle=.4)))
        self.assertIsNotNone(preview.draw(frame(('red', 'black', 'light_blue'), angle=.4)))

    def test_idle_preview_does_not_emit_uart_or_prime_task_samples(self):
        calibration(self.path, ('red', 'black', 'light_blue'))
        count, sent, preview_results = [0], [], []
        serial = types.SimpleNamespace(
            read=lambda: b'CAM,77,TABLE,156,256.0,160.0\n' if count[0] == 5 else None,
            write=lambda data: sent.append(data.decode()))
        def read_frame():
            count[0] += 1
            return frame(('red', 'black', 'light_blue'), angle=count[0]*.1)
        maix = types.ModuleType('maix')
        maix.camera = types.SimpleNamespace(Camera=lambda *a: types.SimpleNamespace(read=read_frame))
        maix.display = types.SimpleNamespace(Display=lambda: types.SimpleNamespace(show=lambda img: None))
        maix.app = types.SimpleNamespace(need_exit=lambda: count[0] >= 10)
        maix.image = types.SimpleNamespace(image2cv=lambda img, **k: img.copy(), cv2image=lambda img, **k: img)
        maix.uart = types.SimpleNamespace(UART=lambda *a: serial)
        factory, draw = v.TurntableTaskDetector, v.IdleTablePreview.draw
        def record_preview(preview, img):
            result = draw(preview, img)
            preview_results.append(result)
            return result
        with patch.dict(sys.modules, {'maix': maix}), \
                patch.object(v, 'TurntableTaskDetector', lambda: factory(str(self.path))), \
                patch.object(v.IdleTablePreview, 'draw', record_preview):
            v.main()
        self.assertEqual(len(preview_results), 5)
        self.assertIsNotNone(preview_results[-1])
        self.assertEqual(sent[:4], ['VISION,77,NONE\n'] * 4)
        self.assertEqual(len(sent), 5)
        self.assertTrue(sent[-1].startswith('VISION,77,TABLE,156,'))

    def test_mild_ellipse_near_point_no_longer_waits_forever(self):
        for _ in range(5):
            img, points = elliptical_frame(axes=(100., 80.))
            center = self.table.detect(img, 123)
        self.assertIsNotNone(center)
        near = v.NearPickup()
        self.assertTrue(near.bind(self.table.detector.last_points, 123,
                                  geometry=self.table.detector.last_geometry))
        self.assertLess(np.linalg.norm(near.point - (256, 240)), 1.)
        img, points = elliptical_frame(axes=(100., 80.), angle=math.pi/2)
        for _ in range(4):
            self.assertIsNone(near.update(points, 1))
        self.assertEqual(near.update(points, 1), '2')

    def test_rotated_slanted_ellipse_geometry_survives_rotation(self):
        tilt = .4
        rotation = np.array(((math.cos(tilt), -math.sin(tilt)), (math.sin(tilt), math.cos(tilt))))
        matrix = rotation.dot(np.diag((100.**2, 55.**2))).dot(rotation.T)
        expected = np.array((256, 160)) + (matrix[0, 1]/math.sqrt(matrix[1, 1]), math.sqrt(matrix[1, 1]))
        for i in range(5):
            img, _ = elliptical_frame(axes=(100., 55.), tilt=tilt, angle=i*.15)
            center = self.table.detect(img, 123)
        self.assertIsNotNone(center)
        self.assertLess(np.linalg.norm(self.table.detector.last_geometry['bottom'] - expected), 1.5)
        self.assertGreater(abs(expected[0] - 256), 10.)
        # Keep tracking the same ellipse while color positions continue rotating.
        for i in range(5, 15):
            img, _ = elliptical_frame(axes=(100., 55.), tilt=tilt, angle=i*.15)
            center = self.table.detect(img, 123)
        self.assertIsNotNone(center)

    def test_large_black_background_does_not_replace_material(self):
        calibration(self.path, ('red', 'black', 'light_blue'))
        table = v.TurntableTaskDetector(str(self.path))
        for _ in range(5):
            img, points = elliptical_frame(('red', 'black', 'light_blue'), axes=(100., 80.),
                                           extras=(('black', (430, 250), 28),))
            center = table.detect(img, 156)
        self.assertIsNotNone(center)
        self.assertLess(np.linalg.norm(np.array(table.detector.last_points['black']) - points['black']), 1.)
        self.assertLess(np.linalg.norm(np.array(center) - (256, 160)), 1.)

    def test_ambiguous_same_color_groups_return_no_coordinate(self):
        for _ in range(6):
            img, _ = elliptical_frame(axes=(70., 70.), extras=(('yellow', (291, 221), 13),))
            self.assertIsNone(self.table.detect(img, 123))
        self.assertEqual(self.table.state, 'ambiguous_colors')

    def test_near_roi_rejects_background_and_ambiguous_track_blobs(self):
        for _ in range(5):
            img, points = elliptical_frame(axes=(70., 70.))
            self.table.detect(img, 123)
        near = v.NearPickup()
        self.assertTrue(near.bind(self.table.detector.last_points, 123,
                                  geometry=self.table.detector.last_geometry,
                                  areas={c: obj['area'] for c, obj in self.table.detector.selected_candidates.items()}))
        for i in range(5):
            img, _ = elliptical_frame(axes=(70., 70.), angle=math.pi/2, missing=('yellow', 'blue'),
                                      extras=(('red', (430, 250), 28),))
            result = near.detect(img, 1, self.table)
        self.assertEqual(result, '2')
        img, _ = elliptical_frame(axes=(70., 70.), angle=math.pi/2, missing=('yellow', 'blue'),
                                  extras=(('red', (326, 160), 13),))
        self.assertIsNone(near.detect(img, 1, self.table))
        self.assertEqual(len(near.history), 0)

    def test_bad_geometry_rejected_and_idle_camera_can_reacquire(self):
        self.assertIsNone(v.turntable_geometry(((100, 100), (150, 100), (200, 100))))
        self.assertIsNone(v.turntable_geometry(((100, 100), (150, float('nan')), (200, 100))))
        self.assertIsNone(v.turntable_geometry(((100, 100), (100, 100), (200, 200))))
        for _ in range(5):
            self.table.detect(frame(), 123)
        for _ in range(14):
            self.assertIsNone(self.table.detect(frame(origin=(300, 160)), 123))
        for _ in range(6):
            center = self.table.detect(frame(origin=(300, 160)), 123)
        self.assertIsNotNone(center)
        self.assertLess(np.linalg.norm(np.array(center) - (300, 160)), 1.)

    def test_protocol_and_task_colors(self):
        self.assertEqual(v.parse_request('CAM,12,TABLE,156,256.0,160.0'), (12, 'TABLE', 156, 256., 160.))
        self.assertEqual(v.table_colors(156), ('red', 'black', 'light_blue'))
        for target in ('111', '1266', '120', '127', '15'):
            self.assertIsNone(v.parse_request('CAM,12,TABLE,' + target))
        self.assertIsNone(v.parse_request('CAM,0,TABLE,123'))
        self.assertIsNone(v.parse_request('CAM,12,TABLE,123,512.0,160.0'))
        self.assertEqual(v.parse_request('CAM,13,MAT,5'), (13, 'MAT', 5))
        self.assertEqual(v.parse_request('CAM,14,RING,2'), (14, 'RING', 2))
        self.assertEqual(v.parse_request('CAM,15,OCCUPIED,156,256.0,160.0'), (15, 'OCCUPIED', 156, 256., 160.))
        self.assertIsNone(v.parse_request('CAM,15,OCCUPIED,111'))

    def test_fixed_positions_survive_removal_and_color_permutation(self):
        refs = v.PhysicalPositions()
        self.assertEqual(v.parse_request('CAM,16,PICK,6'), (16, 'PICK', 6))
        self.assertIsNone(v.parse_request('CAM,16,PICK,7'))
        self.assertIsNone(v.parse_request('CAM,16,PICK,1,256.0,160.0'))
        self.assertTrue(refs.bind({'red': (350., 160.), 'black': (110., 160.),
                                  'light_blue': (230., 160.)}, 156))
        objects = [{'color': 'red', 'center': (350., 160.)},
                   {'color': 'black', 'center': (110., 160.)},
                   {'color': 'light_blue', 'center': (230., 160.)}]
        with patch.object(v, 'detect_materials', lambda *a, **k: objects):
            self.assertEqual(refs.detect(None, 1, self.table), '3')
            self.assertEqual(refs.detect(None, 5, self.table), '1')
            objects.pop(1)  # Removing left does not renumber middle/right.
            self.assertEqual(refs.detect(None, 6, self.table), '2')
            self.assertIsNone(refs.detect(None, 5, self.table))
            objects.pop(1)
            self.assertEqual(refs.detect(None, 1, self.table), '3')
            objects.append(dict(objects[0]))
            self.assertIsNone(refs.detect(None, 1, self.table))
        self.assertIsNone(refs.position((170., 160.)))
        self.assertIsNone(refs.position((float('nan'), 160.)))
        self.assertFalse(refs.bind({'red': (100, 160)}, 156))
        self.assertIsNone(refs.position((350, 160)))

    def test_occupied_middle_is_spatial_not_pickup_color(self):
        objects = [{'color': 'red', 'center': (350., 160.)},
                   {'color': 'black', 'center': (110., 160.)},
                   {'color': 'light_blue', 'center': (230.4, 160.2)}]
        self.assertEqual(v.occupied_middle_center(objects, 156), (230.4, 160.2))
        self.assertEqual(v.occupied_middle_center(objects[::-1], 516), (230.4, 160.2))
        self.assertIsNone(v.occupied_middle_center(objects[:2], 156))
        self.assertIsNone(v.occupied_middle_center(objects + [objects[0]], 156))
        bad = [dict(obj) for obj in objects]
        bad[2]['center'] = (230, 250)
        self.assertIsNone(v.occupied_middle_center(bad, 156))
        bad[2]['center'] = (float('nan'), 160)
        self.assertIsNone(v.occupied_middle_center(bad, 156))

    def test_main_uart_occupied_center_after_three_frames(self):
        count, sent = [0], []
        objects = [{'color': 'red', 'center': (350., 160.)},
                   {'color': 'black', 'center': (110., 160.)},
                   {'color': 'light_blue', 'center': (230., 160.)}]
        serial = types.SimpleNamespace(
            read=lambda: b'CAM,80,OCCUPIED,156,256.0,160.0\n' if count[0] == 0 else None,
            write=lambda data: sent.append(data.decode()))
        def read_frame():
            count[0] += 1
            return np.full((320, 512, 3), 110, np.uint8)
        maix = types.ModuleType('maix')
        maix.camera = types.SimpleNamespace(Camera=lambda *args: types.SimpleNamespace(read=read_frame))
        maix.display = types.SimpleNamespace(Display=lambda: types.SimpleNamespace(show=lambda img: None))
        maix.app = types.SimpleNamespace(need_exit=lambda: count[0] >= 3)
        maix.image = types.SimpleNamespace(image2cv=lambda img, **kwargs: img.copy(), cv2image=lambda img, **kwargs: img)
        maix.uart = types.SimpleNamespace(UART=lambda *args: serial)
        factory = v.TurntableTaskDetector
        with patch.dict(sys.modules, {'maix': maix}), patch.object(v, 'detect_materials', lambda *args, **kwargs: objects), \
                patch.object(v, 'TurntableTaskDetector', lambda: factory(str(self.path))):
            v.main()
        self.assertEqual(sent, ['VISION,80,NONE\n', 'VISION,80,NONE\n', 'VISION,80,OCCUPIED,156,230.0,160.0\n'])

    def test_near_waits_for_requested_color_at_fixed_bottom(self):
        near = v.NearPickup()
        points = {'red': (256., 230.), 'yellow': (195.4, 125.), 'blue': (316.6, 125.)}
        self.assertTrue(near.bind(points, 123))
        self.assertLess(np.linalg.norm(near.point - (256, 230)), .1)
        self.assertEqual(v.parse_request('CAM,83,NEAR,1'), (83, 'NEAR', 1))
        self.assertIsNone(v.parse_request('CAM,83,NEAR,7'))
        self.assertIsNone(v.parse_request('CAM,83,NEAR,1,256.0,230.0'))
        for _ in range(8):
            self.assertIsNone(near.update({'red': (256, 90), 'yellow': (256, 230)}, 1))
        # A moving target passes through the ROI; this is not a stable arrival.
        for dx in (-8, -4, 0, 4, 8):
            self.assertIsNone(near.update({'red': (256 + dx, 230)}, 1))
        near.reset()
        for _ in range(4):
            self.assertIsNone(near.update({'red': (256, 230)}, 1))
        self.assertEqual(near.update({'red': (256, 230)}, 1), '2')
        original = near.point.copy()
        # Previous target's ready history cannot qualify the next requested color.
        for _ in range(4):
            self.assertIsNone(near.update({'blue': (256, 230)}, 3))
        self.assertEqual(near.update({'blue': (256, 230)}, 3), '2')
        self.assertTrue(np.array_equal(near.point, original))
        self.assertIsNone(near.update({}, 3))
        self.assertIsNone(near.update({'blue': (256, 230)}, 3))
        self.assertIsNone(near.update({'red': (float('nan'), 230)}, 1))
        self.assertIsNone(near.update({'black': (256, 230)}, 5))

    def test_uart_near_after_preparation_with_only_one_color_remaining(self):
        count, sent = [0], []
        requests = {0: b'CAM,80,TABLE,123,256.0,160.0\n',
                    5: b'CAM,81,IDLE\n', 6: b'CAM,82,NEAR,1\n'}
        serial = types.SimpleNamespace(read=lambda: requests.get(count[0]),
                                       write=lambda data: sent.append(data.decode()))
        def read_frame():
            img = frame(angle=math.pi / 2)
            if count[0] >= 6:
                img[:200] = 110  # Only red remains, at the fixed bottom position.
            count[0] += 1
            return img
        maix = types.ModuleType('maix')
        maix.camera = types.SimpleNamespace(Camera=lambda *a: types.SimpleNamespace(read=read_frame))
        maix.display = types.SimpleNamespace(Display=lambda: types.SimpleNamespace(show=lambda img: None))
        maix.app = types.SimpleNamespace(need_exit=lambda: count[0] >= 11)
        maix.image = types.SimpleNamespace(image2cv=lambda img, **k: img.copy(), cv2image=lambda img, **k: img)
        maix.uart = types.SimpleNamespace(UART=lambda *a: serial)
        factory = v.TurntableTaskDetector
        with patch.dict(sys.modules, {'maix': maix}), \
                patch.object(v, 'TurntableTaskDetector', lambda: factory(str(self.path))):
            v.main()
        self.assertEqual(sent[-5:], ['VISION,82,NONE\n'] * 4 + ['VISION,82,NEAR,1,2\n'])

    def test_raw_pick_uses_calibrated_color_after_table_request_reset(self):
        for _ in range(5):
            center = self.table.detect(frame(angle=.2), 123)
        self.assertIsNotNone(center)
        refs = v.PhysicalPositions()
        self.assertTrue(refs.bind(self.table.detector.last_points, 123, raw=True))
        self.table.reset()  # The UART changes request from TABLE/IDLE to PICK.
        img = frame(angle=.2, missing='yellow')
        # Remove blue too: target color can still select its original right pose.
        img[:, :260] = 110
        self.assertEqual(refs.detect(img, 1, self.table), '3')
        self.assertIsNone(refs.detect(img, 2, self.table))
        self.assertIsNone(refs.detect(img, 6, self.table))

    def test_uart_pick_keeps_fixed_references_through_idle_and_removal(self):
        count, sent = [0], []
        objects = [{'color': 'red', 'center': (350., 160.)},
                   {'color': 'black', 'center': (110., 160.)},
                   {'color': 'light_blue', 'center': (230., 160.)}]
        requests = {0: b'CAM,80,OCCUPIED,156,256.0,160.0\n',
                    3: b'CAM,81,IDLE\n', 4: b'CAM,82,PICK,1\n'}
        serial = types.SimpleNamespace(read=lambda: requests.get(count[0]),
                                       write=lambda data: sent.append(data.decode()))
        def read_frame():
            count[0] += 1
            return np.full((320, 512, 3), 110, np.uint8)
        maix = types.ModuleType('maix')
        maix.camera = types.SimpleNamespace(Camera=lambda *a: types.SimpleNamespace(read=read_frame))
        maix.display = types.SimpleNamespace(Display=lambda: types.SimpleNamespace(show=lambda img: None))
        maix.app = types.SimpleNamespace(need_exit=lambda: count[0] >= 7)
        maix.image = types.SimpleNamespace(image2cv=lambda img, **k: img.copy(), cv2image=lambda img, **k: img)
        maix.uart = types.SimpleNamespace(UART=lambda *a: serial)
        factory = v.TurntableTaskDetector
        with patch.dict(sys.modules, {'maix': maix}), \
                patch.object(v, 'detect_materials', lambda *a, **k: objects if count[0] <= 3 else objects[:1]), \
                patch.object(v, 'TurntableTaskDetector', lambda: factory(str(self.path))):
            v.main()
        self.assertEqual(sent[-3:], ['VISION,82,NONE\n', 'VISION,82,NONE\n', 'VISION,82,PICK,1,3\n'])

    def test_rotating_blocks_stable_table_center(self):
        for i in range(4):
            self.assertIsNone(self.table.detect(frame(angle=i * .2), 123))
        result = self.table.detect(frame(angle=.8), 123)
        self.assertLessEqual(np.linalg.norm(np.array(result) - [256, 160]), 1)
        points = self.table.detector.last_points
        self.assertEqual(set(points), {'red', 'yellow', 'blue'})
        self.assertGreater(np.linalg.norm(np.array(points['red']) - [256, 160]), 60)

    def test_missing_color_and_invalid_triangle_reset_history(self):
        for _ in range(4):
            self.assertIsNone(self.table.detect(frame(), 123))
        self.assertIsNone(self.table.detect(frame(missing='blue'), 123))
        for _ in range(4):
            self.assertIsNone(self.table.detect(frame(), 123))
        self.assertIsNotNone(self.table.detect(frame(), 123))
        self.assertIsNone(self.table.detect(frame(invalid=True), 123))
        self.assertEqual(self.table.state, 'invalid_triangle')

    def test_new_request_discards_prior_frames_and_unstable_origin(self):
        for _ in range(4):
            self.table.detect(frame(), 123)
        self.table.reset()
        self.assertIsNone(self.table.detect(frame(), 123))
        self.table.reset()
        for i in range(8):
            self.assertIsNone(self.table.detect(frame(origin=(256 + i * 4, 160)), 123))

    def test_calibration_required_and_other_task_colors_supported(self):
        empty = v.TurntableTaskDetector(str(Path(self.temp.name) / 'absent.json'))
        self.assertIsNone(empty.detect(frame(), 123))
        self.assertIn('NEED LAB', empty.state)
        calibration(self.path, ('red', 'black', 'light_blue'))
        table = v.TurntableTaskDetector(str(self.path))
        for i in range(4):
            self.assertIsNone(table.detect(frame(('red', 'black', 'light_blue'), angle=i*.1), 156))
        self.assertIsNotNone(table.detect(frame(('red', 'black', 'light_blue'), angle=.4), 156))
        self.assertIsNone(table.detect(frame(('red', 'black', 'light_blue')), 516))

    def test_main_uart_emits_table_after_five_frames_only(self):
        count, sent = [0], []
        class Serial:
            def read(self):
                return b'CAM,77,TABLE,123,256.0,160.0\n' if count[0] == 0 else None
            def write(self, data):
                sent.append(data.decode())
        serial = Serial()
        def read_frame():
            count[0] += 1
            return frame(angle=count[0]*.1)
        maix = types.ModuleType('maix')
        maix.camera = types.SimpleNamespace(Camera=lambda *args: types.SimpleNamespace(read=read_frame))
        maix.display = types.SimpleNamespace(Display=lambda: types.SimpleNamespace(show=lambda image: None))
        maix.app = types.SimpleNamespace(need_exit=lambda: count[0] >= 5)
        maix.image = types.SimpleNamespace(image2cv=lambda image, **kwargs: image.copy(), cv2image=lambda image, **kwargs: image)
        maix.uart = types.SimpleNamespace(UART=lambda *args: serial)
        factory = v.TurntableTaskDetector
        with patch.dict(sys.modules, {'maix': maix}), patch.object(v, 'TurntableTaskDetector', lambda: factory(str(self.path))):
            v.main()
        self.assertEqual(sent[:4], ['VISION,77,NONE\n'] * 4)
        self.assertTrue(sent[4].startswith('VISION,77,TABLE,123,'))
        fields = sent[4].strip().split(',')
        self.assertLessEqual(abs(float(fields[4]) - 256), 1)
        self.assertLessEqual(abs(float(fields[5]) - 160), 1)

if __name__ == '__main__':
    unittest.main()
