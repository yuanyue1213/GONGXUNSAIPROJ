"""Ring detection regressions using synthetic camera frames, without Maix hardware."""
import importlib.util
from pathlib import Path
import unittest
import cv2
import numpy as np

path = Path(__file__).resolve().parents[2] / 'camera/MaixVision/main.py'
spec = importlib.util.spec_from_file_location('ring_vision', path)
vision = importlib.util.module_from_spec(spec)
spec.loader.exec_module(vision)


def rings(centers=((120, 180), (256, 180), (392, 180)), ratio=.8, gradient=False):
    mask = np.zeros((320, 512), np.uint8)
    for index, center in enumerate(centers):
        for radius in (24, 27, 30, 34, 39, 44):
            cv2.ellipse(mask, center, (radius, round(radius*ratio)), 0, 0, 360, 255, 1)
        cv2.putText(mask, str(index+1), (center[0]-4, center[1]+4),
                    cv2.FONT_HERSHEY_SIMPLEX, .4, 255, 1)
    gray = np.tile(np.linspace(85, 245, 512, dtype=np.float32), (320, 1)) if gradient else np.full((320, 512), 235., np.float32)
    gray[mask > 0] -= 55 if gradient else 225
    return cv2.cvtColor(gray.astype(np.uint8), cv2.COLOR_GRAY2BGR)


class RingVisionTests(unittest.TestCase):
    def setUp(self):
        self.detector = vision.Detector(vision.defaults())

    def check_centers(self, frame, centers):
        points = self.detector.rings(frame)
        self.assertIsNotNone(points)
        np.testing.assert_allclose(points, centers, atol=2.)

    def test_uneven_illumination(self):
        self.check_centers(rings(gradient=True), ((120, 180), (256, 180), (392, 180)))

    def test_oblique_rings(self):
        self.check_centers(rings(ratio=.5), ((120, 180), (256, 180), (392, 180)))

    def test_spacing_is_not_a_detection_gate(self):
        centers = ((75, 150), (175, 180), (425, 150))
        self.check_centers(rings(centers), centers)

    def test_existing_circle_v2_camera_sample(self):
        sample = path.parent / 'circle_v2_subpixel_preview.jpg'
        frame = cv2.imread(str(sample))
        self.assertIsNotNone(frame)
        frame = cv2.resize(frame, (512, 320))
        before = frame.copy()
        self.check_centers(frame, ((117, 163), (256, 170), (398, 178)))
        np.testing.assert_array_equal(frame, before)

    def test_partial_targets_are_visible_but_not_reported_as_three(self):
        frame = rings(((120, 180), (256, 180)))
        engine = vision.Engine(vision.defaults())
        engine.select((42, 'RING', 2))
        self.assertEqual(engine.process(frame), 'VISION,42,NONE\n')
        self.assertIn('2/3', engine.state)
        np.testing.assert_allclose(engine.ring_points, ((120, 180), (256, 180)), atol=2.)
        before = frame.copy()
        vision.draw_status(frame, engine, 0, 'TEST')
        self.assertFalse(np.array_equal(before[170:190, 110:130], frame[170:190, 110:130]))

    def test_incomplete_ambiguous_and_solid_targets_are_rejected(self):
        for centers in ((), ((120, 180), (256, 180)),
                        ((70, 180), (190, 180), (310, 180), (440, 180))):
            self.assertIsNone(self.detector.rings(rings(centers)))
        frame = np.full((320, 512, 3), 235, np.uint8)
        for x in (120, 256, 392):
            cv2.circle(frame, (x, 180), 30, (10, 10, 10), -1)
        self.assertIsNone(self.detector.rings(frame))
        frame[:] = 235
        for x in (120, 256, 392):
            cv2.circle(frame, (x, 180), 30, (10, 10, 10), 1)
        self.assertIsNone(self.detector.rings(frame))


if __name__ == '__main__':
    unittest.main()
