"""Exercise the installed single-file camera entry without Maix hardware."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
import itertools
import numpy as np

path = Path(__file__).resolve().parents[2]/'camera/MaixVision/main.py'
spec = importlib.util.spec_from_file_location('camera_main', path)
vision = importlib.util.module_from_spec(spec)
spec.loader.exec_module(vision)


class Detector:
    def rings(self, _):
        return np.array([(100., 150.), (200., 151.5), (300., 153.)])

    def materials(self, _, mode, colors):
        assert mode == 'OCCUPIED'
        return [dict(color=c, center=(100.+i*100, 150.+i*1.5), area=100.)
                for i, c in enumerate(colors)]


class ParallelVisionTests(unittest.TestCase):
    def test_request_validation(self):
        for target in (2, 156):
            self.assertEqual(vision.parse_request(f'CAM,1,PARALLEL,{target}'), (1, 'PARALLEL', target))
        for target in (0, 1, 3, 111, 157):
            self.assertIsNone(vision.parse_request(f'CAM,1,PARALLEL,{target}'))

    def test_rings_and_materials_return_all_three_and_stop_on_ack(self):
        for target in (2, 156):
            engine = vision.Engine({})
            engine.detector = Detector()
            engine.table = SimpleNamespace(calibrated=set(vision.COLORS.values()))
            engine.select((42, 'PARALLEL', target))
            self.assertEqual(engine.process(None),
                f'VISION,42,PARALLEL,{target},100.0,150.0,200.0,151.5,300.0,153.0\n')
            engine.select((42, 'ACK', 0))
            self.assertIsNone(engine.process(None))
            engine.select((43, 'RING', 2))
            for _ in range(3):
                result = engine.process(None)
            self.assertEqual(result, 'VISION,43,RING,2,200.0,151.5\n')

    def test_saved_offset_is_ignored(self):
        cfg = vision.defaults()
        cfg['material_offset'] = [30., -20.]
        self.assertNotIn('material_offset', vision.validate_config(cfg))
        # 实际检测函数：旧配置即使带偏移，返回坐标也不能移动。
        table = SimpleNamespace(calibrated=set(vision.COLORS.values()),
                                centers=vision.DEFAULT_HSV_CENTER, tolerances=vision.DEFAULT_HSV_TOL)
        detector = vision.Detector(cfg, table)
        self.assertEqual(detector.cfg['material_offset'], [30., -20.])
        image = np.full((320, 512, 3), 255, np.uint8)
        # 用同一红色物料图像比较实际检测输出。
        vision.cv2.circle(image, (150, 120), 30, (0, 0, 255), -1)
        raw = detector.materials(image, 'OCCUPIED', (1,))
        cfg['material_offset'] = [0., 0.]
        plain = detector.materials(image, 'OCCUPIED', (1,))
        self.assertTrue(raw)
        self.assertEqual(raw, plain)


class OccupiedInferenceTests(unittest.TestCase):
    @staticmethod
    def objects(colors, points, missing=()):
        return [dict(color=c, center=p, area=100.)
                for i, (c, p) in enumerate(zip(colors, points)) if i not in missing]

    @staticmethod
    def engine(target, objects, mode='OCCUPIED', calibrated=None):
        engine = vision.Engine(vision.defaults())
        class Objects:
            def materials(self, _, mode, colors):
                return [o for o in objects if o['color'] in colors]
        engine.detector = Objects()
        engine.table = SimpleNamespace(calibrated=set(vision.COLORS.values()) if calibrated is None else calibrated)
        engine.select((42, mode, target))
        return engine

    def test_all_color_orders_and_each_missing_slot(self):
        points = np.array(((100., 145.), (250., 160.), (400., 175.)))
        for colors in itertools.permutations((1, 5, 6)):
            target = int(''.join(map(str, colors)))
            for missing in range(3):
                objects = self.objects(colors, points, (missing,))
                predicted, index = vision.occupied_points(objects, colors)
                np.testing.assert_allclose(predicted, points)
                self.assertEqual(index, missing)
                engine = self.engine(target, objects)
                for _ in range(3):
                    result = engine.process(None)
                self.assertEqual(result, f'VISION,42,OCCUPIED,{target},250.0,160.0\n')
                self.assertEqual(engine.inferred_slot, missing)
                self.assertIn('2 COLORS + EST', engine.state)
                self.assertEqual(len(engine.objects), 2)
                engine.select((42, 'ACK', 0))
                self.assertIsNone(engine.process(None))
                engine.select((43, 'PARALLEL', target))
                self.assertEqual(engine.process(None),
                    f'VISION,43,PARALLEL,{target},100.0,145.0,250.0,160.0,400.0,175.0\n')

    def test_three_measured_colors_keep_uneven_geometry(self):
        points = np.array(((90., 170.), (210., 140.), (430., 180.)))
        objects = self.objects((6, 5, 1), points)
        predicted, inferred = vision.occupied_points(objects, (1, 5, 6))
        np.testing.assert_allclose(predicted, points)
        self.assertIsNone(inferred)  # 三色齐全仍按实测位置，不恢复一排或等间距校验。

    def test_offscreen_missing_endpoint_is_preserved_for_parallel(self):
        points = ((-100., 130.), (100., 150.), (300., 170.))
        objects = self.objects((6, 5, 1), points, (0,))
        engine = self.engine(651, objects, 'PARALLEL')
        self.assertEqual(engine.process(None),
                         'VISION,42,PARALLEL,651,-100.0,130.0,100.0,150.0,300.0,170.0\n')
        engine.select((43, 'OCCUPIED', 651))
        for _ in range(3):
            result = engine.process(None)
        self.assertEqual(result, 'VISION,43,OCCUPIED,651,100.0,150.0\n')

    def test_two_calibrated_colors_are_enough(self):
        objects = self.objects((6, 5, 1), ((100., 150.), (250., 160.), (400., 170.)), (1,))
        engine = self.engine(651, objects, calibrated={'red', 'light_blue'})
        for _ in range(3):
            result = engine.process(None)
        self.assertEqual(result, 'VISION,42,OCCUPIED,651,250.0,160.0\n')

    def test_one_color_or_ambiguous_candidates_are_not_inferred(self):
        points = ((100., 150.), (250., 160.), (400., 170.))
        cases = [self.objects((6, 5, 1), points, (0, 1)),
                 self.objects((6, 5, 1), points, (0,))]
        cases[1].append(dict(cases[1][0]))
        for objects in cases:
            self.assertEqual(vision.occupied_points(objects, (6, 5, 1)), (None, None))
            engine = self.engine(651, objects)
            for _ in range(4):
                self.assertEqual(engine.process(None), 'VISION,42,NONE\n')

    def test_reversed_two_color_order_or_overlapping_centers_are_rejected(self):
        for points in (((300., 150.), (100., 160.), (400., 170.)),
                       ((100., 150.), (105., 150.), (400., 170.))):
            objects = self.objects((6, 5, 1), points, (2,))
            self.assertEqual(vision.occupied_points(objects, (6, 5, 1)), (None, None))

    def test_inferred_point_is_marked_on_display(self):
        objects = self.objects((6, 5, 1), ((100., 145.), (250., 160.), (400., 175.)), (1,))
        engine = self.engine(651, objects)
        for _ in range(3):
            engine.process(None)
        frame = np.zeros((420, 512, 3), np.uint8)
        vision.draw_status(frame, engine, 0, 'TEST')
        self.assertTrue(np.any(frame[154:167, 244:257] != 0))
        engine.select((43, 'IDLE', 0))
        self.assertIsNone(engine.ring_points)
        self.assertIsNone(engine.inferred_slot)


if __name__ == '__main__':
    unittest.main()
