"""Synthetic images + existing CAM/VISION lifecycle, no Maix hardware needed."""
import copy
import importlib.util
import math
import tempfile
import sys
import types
from pathlib import Path
import unittest
from unittest.mock import patch

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("reference_vision", ROOT / "camera/MaixVision/vision_reference.py")
v = importlib.util.module_from_spec(spec)
spec.loader.exec_module(v)
BGR = {1: (25, 25, 220), 2: (25, 220, 220), 3: (145, 30, 30),
       4: (25, 200, 25), 5: (20, 20, 20), 6: (230, 195, 50)}


def canvas():
    return np.full((320, 512, 3), 235, np.uint8)


def blocks(items):
    img = canvas()
    for color, center in items:
        cv2.ellipse(img, center, (20, 16), 0, 0, 360, BGR[color], -1)
    return img


def table(angle=0):
    items = [(c, (round(256+95*math.cos(angle+i*math.tau/3)),
                   round(160+70*math.sin(angle+i*math.tau/3)))) for i, c in enumerate((1, 5, 6))]
    return blocks(items)


def stable(engine, img, codes=(), count=3):
    result = None
    for _ in range(count):
        result = engine.process(img, codes)
    return result


class ReferenceTests(unittest.TestCase):
    def setUp(self):
        self.cfg = v.defaults()
        self.engine = v.Engine(self.cfg)

    def test_default_validation_and_independent_config(self):
        self.assertEqual(v.validate_config(self.cfg), self.cfg)
        with tempfile.TemporaryDirectory() as folder:
            path = str(Path(folder) / "new.json")
            self.assertIn("DEFAULT", v.load_config(path)[1])
            v.save_config(self.cfg, path)
            self.assertEqual(v.load_config(path)[0], self.cfg)
            Path(path).write_text("{}")
            with self.assertRaises(ValueError):
                v.load_config(path)
        bad = copy.deepcopy(self.cfg)
        bad["near_point"] = [10, 10]
        with self.assertRaises(ValueError):
            v.validate_config(bad)

    def test_calibration_wrap_red_and_black(self):
        img = blocks([(1, (200, 150)), (5, (300, 150))])
        v.sample_hsv(self.cfg, 1, img, (200, 150))
        self.assertEqual(len(self.cfg["hsv"]["1"]), 2)
        v.sample_hsv(self.cfg, 5, img, (300, 150))
        self.assertLess(self.cfg["hsv"]["5"][0][1][2], 100)
        self.assertEqual(self.cfg["calibrated"], [1, 5])
        with self.assertRaises(ValueError):
            v.sample_hsv(self.cfg, 1, canvas(), (200, 150))

    def test_protocol_validation_and_fragmented_overlong_lines(self):
        self.assertTrue(v.valid_code("156+123+516+231"))
        for bad in ("123+123+156+231", "116+123+611+231", "123+321", "156+111+516+231"):
            self.assertFalse(v.valid_code(bad))
        for mode, target in (("TABLE", 156), ("OCCUPIED", 516), ("RING", 2), ("MAT", 5), ("PICK", 6), ("NEAR", 1)):
            self.assertEqual(v.parse_request("CAM,17,%s,%d" % (mode, target)), (17, mode, target))
        for bad in ("CAM,0,QR", "CAM,4294967296,QR", "CAM,1,NEAR,7", "CAM,1,RING,4",
                    "CAM,1,TABLE,116", "CAM,1,MAT,1,nan,160.0", "CAM,1,MAT,1,512.0,160.0",
                    "CAM,1,NEAR,1,256.0,160.0", "CAM,1,TABLE,156,256.00,160.0"):
            self.assertIsNone(v.parse_request(bad), bad)
        receiver = v.LineReceiver()
        self.assertEqual(receiver.feed(b"CAM,7,TA"), [])
        self.assertEqual(receiver.feed(b"BLE,156,256.0,160.0\r\n"), [(7, "TABLE", 156, 256., 160.)])
        self.assertEqual(receiver.feed(b"x"*80+b"\nCAM,8,IDLE\n"), [(8, "IDLE", 0)])

    def test_six_color_material_centers(self):
        for color in BGR:
            img = blocks([(color, (256, 160))])
            self.engine.select((color, "MAT", color))
            result = stable(self.engine, img)
            self.assertIn(",MAT,%d," % color, result)
            xy = np.array(list(map(float, result.strip().split(",")[-2:])))
            self.assertLess(np.linalg.norm(xy-(256, 160)), 2.)

    def test_cylinder_lower_base_center(self):
        img = canvas()
        # Top cap at (230,130), larger base at (256,160): a visible stepped body.
        cv2.fillConvexPoly(img, np.array([[211, 130], [249, 130], [279, 160], [233, 160]]), BGR[1])
        cv2.ellipse(img, (230, 130), (19, 13), 0, 0, 360, BGR[1], -1)
        cv2.ellipse(img, (256, 160), (23, 17), 0, 0, 360, BGR[1], -1)
        found = v.Detector(self.cfg).materials(img, "MAT", (1,))
        self.assertEqual(len(found), 1)
        self.assertLess(np.linalg.norm(np.array(found[0]["center"])-(256, 160)), 5.)

    def test_table_rotation_uses_center_stability_not_vertices(self):
        self.engine.select((10, "TABLE", 156, 256., 160.))
        for i in range(4):
            self.assertEqual(self.engine.process(table(i*.13)), "VISION,10,NONE\n")
        result = self.engine.process(table(.52))
        self.assertIn("VISION,10,TABLE,156,", result)
        xy = np.array(list(map(float, result.strip().split(",")[-2:])))
        self.assertLess(np.linalg.norm(xy-(256, 160)), 2.)

    def test_missing_or_duplicate_color_does_not_output_center(self):
        self.engine.select((10, "TABLE", 156))
        for img in (blocks([(1, (160, 180)), (5, (300, 180))]),
                    blocks([(1, (160, 180)), (1, (220, 90)), (5, (300, 180)), (6, (250, 240))])):
            self.assertEqual(stable(self.engine, img, count=6), "VISION,10,NONE\n")

    def test_near_independent_of_three_blocks_and_no_stale_result(self):
        self.engine.select((11, "NEAR", 1))
        img = blocks([(1, (256, 260))])
        self.assertEqual(stable(self.engine, img), "VISION,11,NEAR,1,2\n")
        self.assertEqual(self.engine.process(canvas()), "VISION,11,NONE\n")
        for _ in range(2):
            self.assertEqual(self.engine.process(img), "VISION,11,NONE\n")
        self.assertEqual(self.engine.process(img), "VISION,11,NEAR,1,2\n")
        self.engine.select((12, "NEAR", 5))
        self.assertEqual(stable(self.engine, img), "VISION,12,NONE\n")
        self.engine.select((13, "NEAR", 1))
        far = blocks([(1, (256, 205))])
        self.assertEqual(stable(self.engine, far), "VISION,13,NONE\n")

    def test_near_duplicate_and_moving_target_rejected(self):
        self.engine.select((11, "NEAR", 1))
        img = blocks([(1, (256, 260)), (1, (330, 230))])
        self.assertEqual(stable(self.engine, img), "VISION,11,NONE\n")
        for x in (245, 250, 255, 260, 265):
            self.assertEqual(self.engine.process(blocks([(1, (x, 260))])), "VISION,11,NONE\n")

    def test_occupied_middle_and_persistent_left_middle_right(self):
        self.engine.select((20, "OCCUPIED", 156))
        row = blocks([(1, (140, 190)), (5, (256, 190)), (6, (372, 190))])
        self.assertIn("VISION,20,OCCUPIED,156,", stable(self.engine, row))
        self.engine.select((21, "IDLE", 0))
        self.assertIsNone(self.engine.process(row))
        # Leftmost removed: rightmost must remain position3, not become position2.
        self.engine.select((22, "PICK", 6))
        remaining = blocks([(5, (256, 190)), (6, (372, 190))])
        self.assertEqual(stable(self.engine, remaining), "VISION,22,PICK,6,3\n")
        self.engine.select((23, "TABLE", 156))
        self.engine.select((24, "PICK", 6))
        self.assertEqual(stable(self.engine, remaining), "VISION,24,NONE\n")

    def test_occupied_rejects_two_or_non_row(self):
        self.engine.select((20, "OCCUPIED", 156))
        self.assertEqual(stable(self.engine, table(), count=5), "VISION,20,NONE\n")
        self.assertEqual(stable(self.engine, blocks([(1, (140, 190)), (5, (256, 190))])), "VISION,20,NONE\n")

    def test_black_concentric_rings_and_left_right(self):
        img = canvas()
        for x in (135, 256, 377):
            for radius in (17, 28, 40):
                cv2.ellipse(img, (x, 180), (radius, round(radius*.8)), 0, 0, 360, (10, 10, 10), 3)
        self.engine.select((30, "RING", 2))
        result = stable(self.engine, img)
        self.assertIn("VISION,30,RING,2,", result)
        xy = np.array(list(map(float, result.strip().split(",")[-2:])))
        self.assertLess(np.linalg.norm(xy-(256, 180)), 2.)
        # Solid black disks are not concentric ring targets.
        img = blocks([(5, (135, 180)), (5, (256, 180)), (5, (377, 180))])
        self.assertEqual(stable(self.engine, img), "VISION,30,NONE\n")

    def test_qr_stability_new_request_and_idle_silence(self):
        self.engine.select((40, "QR", 0))
        code = "156+123+516+231"
        self.assertEqual(stable(self.engine, canvas(), [code]), "VISION,40,QR,"+code+"\n")
        self.assertEqual(self.engine.process(canvas(), [code, "156+231+516+123"]), "VISION,40,NONE\n")
        self.engine.select((41, "QR", 0))
        self.assertEqual(self.engine.process(canvas(), [code]), "VISION,41,NONE\n")
        self.engine.select((42, "IDLE", 0))
        self.assertIsNone(self.engine.process(canvas()))

    def test_duplicate_camera_retry_does_not_reset_stability(self):
        request = (50, "MAT", 1)
        img = blocks([(1, (256, 160))])
        for _ in range(2):
            self.engine.select(request)
            self.assertEqual(self.engine.process(img), "VISION,50,NONE\n")
        self.engine.select(request)
        self.assertIn("VISION,50,MAT,1,", self.engine.process(img))

    def test_maix_main_uart_lifecycle_and_idle_silence(self):
        count, current, sent, closed = [0], ["QR"], [], []
        schedule = {0: "CAM,1,QR\n", 3: "CAM,2,TABLE,156,256.0,160.0\n",
                    8: "CAM,3,IDLE\n", 9: "CAM,4,NEAR,1\n",
                    12: "CAM,5,OCCUPIED,156,256.0,160.0\n", 15: "CAM,6,IDLE\n",
                    16: "CAM,7,PICK,6\n", 19: "CAM,8,IDLE\n"}
        def read_serial():
            command = schedule.get(count[0], "")
            if command:
                current[0] = command.strip().split(",")[2]
            return command.encode()
        def read_camera():
            mode = current[0]
            img = (table() if mode == "TABLE" else blocks([(1, (256, 260))]) if mode == "NEAR" else
                   blocks([(1, (140, 190)), (5, (256, 190)), (6, (372, 190))]) if mode == "OCCUPIED" else
                   blocks([(6, (372, 190))]) if mode == "PICK" else canvas())
            count[0] += 1
            return types.SimpleNamespace(img=img, find_qrcodes=lambda: [types.SimpleNamespace(payload=lambda: "156+123+516+231")])
        clock = [0.]
        def monotonic():
            clock[0] += .2
            return clock[0]
        serial = types.SimpleNamespace(read=read_serial, write=lambda b: sent.append(b.decode()), close=lambda: closed.append("uart"))
        camera = types.SimpleNamespace(read=read_camera, close=lambda: closed.append("camera"))
        display = types.SimpleNamespace(show=lambda img: None)
        maix = types.ModuleType("maix")
        maix.camera = types.SimpleNamespace(Camera=lambda w, h: camera)
        maix.display = types.SimpleNamespace(Display=lambda: display)
        maix.uart = types.SimpleNamespace(UART=lambda dev, baud: serial)
        maix.app = types.SimpleNamespace(need_exit=lambda: count[0] >= 21)
        maix.image = types.SimpleNamespace(image2cv=lambda f, **kw: f.img.copy(), cv2image=lambda img, **kw: img)
        with patch.dict(sys.modules, {"maix": maix}), patch.object(v, "load_config", return_value=(self.cfg, "TEST")), patch.object(v.time, "monotonic", monotonic):
            v.main()
        for prefix in ("VISION,1,QR,", "VISION,2,TABLE,156,", "VISION,4,NEAR,1,2", "VISION,5,OCCUPIED,156,", "VISION,7,PICK,6,3"):
            self.assertTrue(any(line.startswith(prefix) for line in sent), (prefix, sent))
        self.assertFalse(any(line.startswith(("VISION,3,", "VISION,6,", "VISION,8,")) for line in sent))
        self.assertEqual(closed, ["uart", "camera"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
