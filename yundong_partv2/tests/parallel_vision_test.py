"""Exercise the installed single-file camera entry without Maix hardware."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
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


if __name__ == '__main__':
    unittest.main()
