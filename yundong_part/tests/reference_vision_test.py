"""Synthetic images + existing CAM/VISION lifecycle, no Maix hardware needed."""
import copy
import ast
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
spec = importlib.util.spec_from_file_location("reference_vision", ROOT / "camera/MaixVision/main.py")
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
        centers = {name: np.array(value, np.float32) for name, value in v.DEFAULT_LAB_CENTER.items()}
        for color, bgr in BGR.items():
            centers[v.COLORS[color]] = cv2.cvtColor(np.array([[bgr]], np.float32)/255., cv2.COLOR_BGR2LAB)[0, 0]
        self.lab_data = (centers, {name: np.array([8,8,8], np.float32) for name in centers}, set(centers))
        self.lab_patch = patch.object(v, "load_lab_thresholds", return_value=self.lab_data)
        self.lab_patch.start()
        self.addCleanup(self.lab_patch.stop)
        self.engine = v.Engine(self.cfg)

    def prepare_near(self):
        self.engine.select((100, "TABLE", 156))
        self.assertIn(",TABLE,156,", stable(self.engine, table(), count=5))

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
        bad["material_offset"] = [100, 0]
        with self.assertRaises(ValueError):
            v.validate_config(bad)

    def test_original_algorithms_and_calibration_are_preserved(self):
        original = ast.parse((ROOT / "camera/MaixVision/vision_original_20261025.py").read_text(encoding="utf-8"))
        restored_text = (ROOT / "camera/MaixVision/main.py").read_text(encoding="utf-8")
        restored = ast.parse(restored_text.replace("ORIGINAL_COLORS.items()", "COLORS.items()").replace("class OriginalStableTarget:", "class StableTarget:"))
        def functions(tree):
            return {n.name: ast.dump(n, include_attributes=False) for n in tree.body if isinstance(n, (ast.FunctionDef, ast.ClassDef))}
        expected, actual = functions(original), functions(restored)
        for name in ("save_lab_thresholds", "turntable_geometry", "on_turntable_track",
                     "TurntableCenterDetector", "TurntableTaskDetector", "table_colors", "NearPickup", "StableTarget"):
            self.assertEqual(actual[name], expected[name], name)
        centers, tolerances, calibrated = self.lab_data
        with tempfile.TemporaryDirectory() as folder:
            path = str(Path(folder)/"lab_thresholds.json")
            self.assertTrue(v.save_lab_thresholds(centers, tolerances, calibrated, path))
            # Bypass setUp's in-memory loader to test the original persistence code.
            self.lab_patch.stop()
            loaded = v.load_lab_thresholds(path)
            self.assertEqual(loaded[2], calibrated)
            np.testing.assert_allclose(loaded[0]["red"], centers["red"])
            self.lab_patch.start()

    def test_malformed_lab_types_fall_back_without_startup_failure(self):
        self.lab_patch.stop()
        try:
            with tempfile.TemporaryDirectory() as folder:
                path = Path(folder)/"lab.json"
                for payload in ("[]", "null", "123", "{}",
                                '{"version":1,"lab_center":[],"lab_tol":{}}',
                                '{"version":1,"lab_center":{},"lab_tol":null}',
                                '{"version":1,"lab_center":{},"lab_tol":{},"calibrated_colors":[{}]}'):
                    path.write_text(payload)
                    centers, tolerances, calibrated = v.load_lab_thresholds(str(path))
                    self.assertEqual(calibrated, set())
                    for name in v.DEFAULT_LAB_CENTER:
                        np.testing.assert_allclose(centers[name], v.DEFAULT_LAB_CENTER[name])
                        np.testing.assert_allclose(tolerances[name], v.DEFAULT_LAB_TOL[name])
        finally:
            self.lab_patch.start()

    def test_command_footer_does_not_cover_camera_pixels(self):
        img = blocks([(2, (256, 260))])
        display = cv2.copyMakeBorder(img, 0, v.FOOTER_HEIGHT, 0, 0, cv2.BORDER_CONSTANT)
        receiver = v.LineReceiver()
        receiver.feed(b"CAM,1,TABLE,123\n")
        v.draw_command_status(display, receiver)
        np.testing.assert_array_equal(display[:v.HEIGHT], img)
        self.assertTrue(np.any(display[v.HEIGHT:]))

    def test_original_idle_preview_does_not_prime_task(self):
        preview = v.IdleTablePreview(self.engine.table, target=156)
        for _ in range(5):
            center = preview.draw(table())
        self.assertIsNotNone(center)
        self.assertIsNone(self.engine.table.detector)
        self.assertIsNone(self.engine.near.point)
        self.engine.select((80, "TABLE", 156))
        for _ in range(4):
            self.assertEqual(self.engine.process(table()), "VISION,80,NONE\n")
        self.assertIn("VISION,80,TABLE,156,", self.engine.process(table()))

    def test_default_preview_and_table_include_yellow(self):
        img = blocks([(1, (160, 130)), (2, (350, 130)), (3, (256, 270))])
        preview = v.IdleTablePreview(self.engine.table)
        self.assertEqual(preview.target, 123)
        for _ in range(5):
            center = preview.draw(img.copy())
        self.assertIsNotNone(center)
        self.assertEqual(set(preview.detector.last_points), {"red", "yellow", "blue"})
        self.engine.select((90, "TABLE", 123))
        self.assertIn("VISION,90,TABLE,123,", stable(self.engine, img, count=5))
        self.assertEqual({obj["color"] for obj in self.engine.objects}, {1, 2, 3})
        # 屏幕绘制路径必须包含yellow标签，而不只是内部检测到了黄色。
        with patch.object(v.cv2, "putText", wraps=cv2.putText) as text:
            v.draw_status(img.copy(), self.engine, 0., "TEST")
        self.assertIn("yellow", [call.args[1] for call in text.call_args_list])

    def test_protocol_validation_and_fragmented_overlong_lines(self):
        self.assertTrue(v.valid_code("156+123+516+231"))
        for bad in ("123+123+156+231", "116+123+611+231", "123+321", "156+111+516+231"):
            self.assertFalse(v.valid_code(bad))
        for mode, target in (("TABLE", 156), ("OCCUPIED", 516), ("RING", 2), ("PICK", 6), ("NEAR", 1)):
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
            objects = v.Detector(self.cfg).materials(img, "PICK", (color,))
            self.assertEqual(len(objects), 1)
            xy = np.array(objects[0]["center"])
            self.assertLess(np.linalg.norm(xy-(256, 160)), 2.)

    def test_uart_partial_noise_and_repeated_qr_request_are_visible(self):
        receiver = v.LineReceiver()
        self.assertEqual(receiver.feed(b"\x00"), [])
        self.assertEqual(receiver.last_data, b"\x00")
        self.assertEqual((receiver.accepted, receiver.rejected), (0, 0))
        # 启动噪声破坏首帧，下一次同编号补发即可恢复。
        self.assertEqual(receiver.feed(b"CAM,1,QR\n"), [])
        self.assertEqual((receiver.accepted, receiver.rejected), (0, 1))
        self.assertEqual(receiver.feed(b"CAM,1,Q"), [])
        request = receiver.feed(b"R\n")
        self.assertEqual(request, [(1, "QR", 0)])
        self.engine.select(request[0])
        img = canvas()
        self.assertEqual(self.engine.process(img, ["156+123+516+231"]), "VISION,1,NONE\n")
        for _ in range(2):
            repeat = receiver.feed(b"CAM,1,QR\n")
            self.engine.select(repeat[0])
            result = self.engine.process(img, ["156+123+516+231"])
        self.assertEqual(result, "VISION,1,QR,156+123+516+231\n")
        self.assertEqual((receiver.accepted, receiver.rejected), (3, 1))

    def test_cylinder_lower_base_center(self):
        img = canvas()
        # Top cap at (230,130), larger base at (256,160): a visible stepped body.
        cv2.fillConvexPoly(img, np.array([[211, 130], [249, 130], [279, 160], [233, 160]]), BGR[1])
        cv2.ellipse(img, (230, 130), (19, 13), 0, 0, 360, BGR[1], -1)
        cv2.ellipse(img, (256, 160), (23, 17), 0, 0, 360, BGR[1], -1)
        found = v.Detector(self.cfg).materials(img, "PICK", (1,))
        self.assertEqual(len(found), 1)
        self.assertLess(np.linalg.norm(np.array(found[0]["center"])-(256, 160)), 5.)

    def test_competition_tapered_material_base_six_colors(self):
        # 规则图5：顶部直径30、底部50、高60；用斜视轮廓验证底部中心。
        for color, bgr in BGR.items():
            img = canvas()
            cv2.fillConvexPoly(img, np.array([[241, 120], [271, 120], [281, 180], [231, 180]]), bgr)
            cv2.ellipse(img, (256, 120), (15, 8), 0, 0, 360, bgr, -1)
            cv2.ellipse(img, (256, 180), (25, 12), 0, 0, 360, bgr, -1)
            found = v.Detector(self.cfg).materials(img, "PICK", (color,))
            self.assertEqual(len(found), 1)
            self.assertLess(np.linalg.norm(np.array(found[0]["center"])-(256, 180)), 6.)

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

    def test_near_requires_table_then_survives_removed_blocks(self):
        self.engine.select((99, "NEAR", 1))
        self.assertEqual(stable(self.engine, blocks([(1, (256, 230))]), count=5), "VISION,99,NONE\n")
        self.prepare_near()
        self.engine.select((11, "NEAR", 1))
        img = blocks([(1, (256, 230))])
        self.assertEqual(stable(self.engine, img, count=5), "VISION,11,NEAR,1,2\n")
        self.assertEqual(self.engine.process(canvas()), "VISION,11,NONE\n")
        for _ in range(4):
            self.assertEqual(self.engine.process(img), "VISION,11,NONE\n")
        self.assertEqual(self.engine.process(img), "VISION,11,NEAR,1,2\n")
        self.engine.select((12, "NEAR", 5))
        self.assertEqual(stable(self.engine, img), "VISION,12,NONE\n")
        self.engine.select((13, "NEAR", 1))
        far = blocks([(1, (256, 205))])
        self.assertEqual(stable(self.engine, far), "VISION,13,NONE\n")

    def test_near_duplicate_and_moving_target_rejected(self):
        self.prepare_near()
        self.engine.select((11, "NEAR", 1))
        img = blocks([(1, (256, 230)), (1, (310, 210))])
        self.assertEqual(stable(self.engine, img), "VISION,11,NONE\n")
        for x in (245, 250, 255, 260, 265):
            self.assertEqual(self.engine.process(blocks([(1, (x, 230))])), "VISION,11,NONE\n")

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

    def test_final_pose_refresh_before_pick_and_near(self):
        # OCCUPIED 粗/精调的坐标不能直接用于车体移动后的 PICK。
        self.engine.select((1, "OCCUPIED", 156))
        self.assertIn(",OCCUPIED,", stable(self.engine, blocks([(1, (140, 190)), (5, (256, 190)), (6, (372, 190))])))
        moved = blocks([(1, (140, 150)), (5, (256, 150)), (6, (372, 150))])
        self.engine.select((2, "IDLE", 0))
        self.engine.select((3, "OCCUPIED", 156))  # 最终位置只刷新，不再移动。
        self.assertIn(",OCCUPIED,", stable(self.engine, moved))
        self.engine.select((4, "IDLE", 0))
        self.engine.select((5, "PICK", 6))
        remaining = blocks([(6, (372, 150))])
        self.assertEqual(stable(self.engine, remaining), "VISION,5,PICK,6,3\n")
        # TABLE 的最终刷新同样要更新 NEAR 抓取点，且允许其余两件已被取走。
        self.prepare_near()
        shifted_table = cv2.warpAffine(table(), np.float32([[1, 0, 0], [0, 1, 20]]),
                                      (512, 320), borderValue=(235, 235, 235))
        self.engine.select((6, "IDLE", 0))
        self.engine.select((7, "TABLE", 156))
        self.assertIn(",TABLE,", stable(self.engine, shifted_table, count=5))
        self.engine.select((8, "IDLE", 0))
        self.engine.select((9, "NEAR", 1))
        self.assertEqual(stable(self.engine, blocks([(1, (256, 250))]), count=5), "VISION,9,NEAR,1,2\n")

    def test_unused_modes_and_legacy_config_removed(self):
        self.assertIsNone(v.parse_request("CAM,1,MAT,1"))
        legacy = copy.deepcopy(self.cfg)
        legacy.update(near_point=[10, 10], near_window=[0, 0], calibrated=[99], hsv={"unused": "old thresholds"})
        legacy["rois"]["NEAR"] = [0, 0, 0, 0]
        self.assertEqual(v.validate_config(legacy), self.cfg)

    def test_calibration_polls_task_before_reading_frame(self):
        maix = types.ModuleType("maix")
        maix.app = types.SimpleNamespace(need_exit=lambda: False)
        maix.image = types.SimpleNamespace()
        maix.time = types.SimpleNamespace()
        def task_received():
            raise InterruptedError("new CAM task")
        with patch.dict(sys.modules, {"maix": maix}):
            with self.assertRaises(InterruptedError):
                # cam=None 确保先轮询任务，再进行任何阻塞的图像/触屏操作。
                v.calibrate_table_colors(None, None, None, self.engine.table, ("red",), task_received)

    def test_shared_lab_updates_material_detection_without_restart(self):
        self.assertIs(self.engine.detector.lab_source, self.engine.table)
        red = blocks([(1, (256, 190))])
        green = blocks([(4, (256, 190))])
        detector = self.engine.detector
        self.assertEqual(len(detector.materials(red, "PICK", (1,))), 1)
        # 触屏采样修改颜色中心，所有物料模式立即使用新阈值。
        self.engine.table.centers["red"] = self.engine.table.centers["green"].copy()
        self.assertEqual(detector.materials(red, "PICK", (1,)), [])
        self.assertEqual(len(detector.materials(green, "PICK", (1,))), 1)
        self.assertEqual(len(detector.materials(green, "OCCUPIED", (1,))), 1)
        # 中断标定后整体重新加载字典，也必须跟随新的阈值。
        self.engine.table.centers = dict(self.engine.table.centers)
        self.engine.table.centers["red"] = cv2.cvtColor(
            np.array([[BGR[1]]], np.float32)/255., cv2.COLOR_BGR2LAB)[0, 0]
        self.assertEqual(len(detector.materials(red, "PICK", (1,))), 1)
        self.assertEqual(detector.materials(green, "PICK", (1,)), [])

    def test_material_modes_require_same_lab_calibration_as_table(self):
        self.engine.table.calibrated = set(self.engine.table.calibrated)-{"red"}
        row = blocks([(1, (140, 190)), (5, (256, 190)), (6, (372, 190))])
        for req, mode, target in ((80, "TABLE", 156), (81, "OCCUPIED", 156), (82, "PICK", 1)):
            self.engine.select((req, mode, target))
            self.assertEqual(stable(self.engine, row, count=6), "VISION,%d,NONE\n" % req)
            self.assertIn("NEED LAB: red", self.engine.state)
        self.assertEqual(self.engine.detector.materials(row, "PICK", (1,)), [])
        self.engine.table.calibrated.add("red")
        self.engine.select((83, "OCCUPIED", 156))
        self.assertIn(",OCCUPIED,156,", stable(self.engine, row))

    def test_overlapping_blue_lab_masks_are_not_double_labelled(self):
        self.engine.table.centers["light_blue"] = self.engine.table.centers["blue"].copy()
        img = blocks([(3, (256, 190))])
        self.assertEqual(self.engine.detector.materials(img, "PICK", (3, 6)), [])

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

    def test_black_material_on_ring_background(self):
        for thickness in (1, 2):
            background = canvas()
            for index, x in enumerate((140, 256, 372)):
                for radius in (24, 28, 32, 36, 40, 44):
                    cv2.ellipse(background, (x, 190), (radius, round(radius*.8)),
                                0, 0, 360, (10, 10, 10), thickness)
                cv2.putText(background, str(3-index), (x-4, 194),
                            cv2.FONT_HERSHEY_SIMPLEX, .4, (10, 10, 10), 1)
            self.assertEqual(v.Detector(self.cfg).materials(background, "PICK", (5,)), [])
            for black_position in range(3):
                colors = [1, 6]
                colors.insert(black_position, 5)
                img = background.copy()
                for color, x in zip(colors, (140, 256, 372)):
                    cv2.ellipse(img, (x, 190), (20, 16), 0, 0, 360, BGR[color], -1)
                self.engine.select((100+black_position, "OCCUPIED", 156))
                self.assertIn(",OCCUPIED,156,", stable(self.engine, img))
                self.engine.select((200+black_position, "IDLE", 0))
                self.engine.select((300+black_position, "PICK", 5))
                self.assertEqual(stable(self.engine, img).strip(),
                                 "VISION,%d,PICK,5,%d" % (300+black_position, black_position+1))
                # 取走其他物料后保留圆环与数字，工位编号仍不能变化。
                remaining = background.copy()
                cv2.ellipse(remaining, ((140, 256, 372)[black_position], 190),
                            (20, 16), 0, 0, 360, BGR[5], -1)
                self.assertEqual(stable(self.engine, remaining).strip(),
                                 "VISION,%d,PICK,5,%d" % (300+black_position, black_position+1))

    def test_tapered_black_material_on_competition_rings(self):
        for thickness in (1, 2):
            img = canvas()
            for x in (119, 256, 393):
                for radius in (24, 27, 30, 34, 39, 44):
                    cv2.ellipse(img, (x, 190), (radius, round(radius*.8)),
                                0, 0, 360, (10, 10, 10), thickness)
            for color, x in ((1, 119), (5, 256), (6, 393)):
                bgr = BGR[color]
                cv2.fillConvexPoly(img, np.array([[x-15, 130], [x+15, 130],
                                                 [x+25, 190], [x-25, 190]]), bgr)
                cv2.ellipse(img, (x, 130), (15, 8), 0, 0, 360, bgr, -1)
                cv2.ellipse(img, (x, 190), (25, 12), 0, 0, 360, bgr, -1)
            self.engine.select((10, "OCCUPIED", 156))
            self.assertIn(",OCCUPIED,156,", stable(self.engine, img))
            self.engine.select((11, "IDLE", 0))
            self.engine.select((12, "PICK", 5))
            self.assertEqual(stable(self.engine, img), "VISION,12,PICK,5,2\n")

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
        request = (50, "QR", 0)
        img = blocks([(1, (256, 160))])
        for _ in range(2):
            self.engine.select(request)
            self.assertEqual(self.engine.process(img, ["156+123+516+231"]), "VISION,50,NONE\n")
        self.engine.select(request)
        self.assertIn("VISION,50,QR,", self.engine.process(img, ["156+123+516+231"]))

    def test_ack_stops_result_until_new_request(self):
        receiver = v.LineReceiver()
        self.assertEqual(receiver.feed(b"ACK,50\n"), [(50, "ACK", 0)])
        for bad in ("ACK,0", "ACK,4294967296", "ACK,-1", "ACK,1,QR"):
            self.assertIsNone(v.parse_request(bad))
        request = (50, "QR", 0)
        self.engine.select(request)
        self.engine.select((50, "ACK", 0))  # 没有回复前不接受确认。
        self.assertFalse(self.engine.stopped)
        code = "156+123+516+231"
        self.assertIn(",QR,", stable(self.engine, canvas(), [code]))
        self.engine.select((49, "ACK", 0))
        self.assertFalse(self.engine.stopped)
        self.engine.select((50, "ACK", 0))
        self.assertTrue(self.engine.stopped)
        self.engine.select(request)  # 已确认请求的补发不能重新启动。
        with patch.object(self.engine, "_process", side_effect=AssertionError("must stop")):
            self.assertIsNone(self.engine.process(canvas(), [code]))
        self.engine.select((51, "QR", 0))
        self.engine.select((50, "ACK", 0))  # 迟到确认不能停止下一任务。
        self.assertFalse(self.engine.stopped)
        self.assertEqual(self.engine.process(canvas(), [code]), "VISION,51,NONE\n")

    def test_ack_none_and_station_cache_preserved(self):
        self.engine.select((20, "OCCUPIED", 156))
        img = blocks([(1, (140, 190)), (5, (256, 190)), (6, (372, 190))])
        self.assertIn(",OCCUPIED,", stable(self.engine, img))
        anchors = self.engine.anchors.copy()
        self.engine.select((20, "ACK", 0))
        np.testing.assert_array_equal(self.engine.anchors, anchors)
        self.engine.select((21, "PICK", 6))
        self.assertEqual(stable(self.engine, blocks([(6, (372, 190))])), "VISION,21,PICK,6,3\n")
        self.engine.select((22, "RING", 2))
        self.assertEqual(self.engine.process(canvas()), "VISION,22,NONE\n")
        self.engine.select((22, "ACK", 0))
        self.assertIsNone(self.engine.process(canvas()))

    def test_maix_main_uart_lifecycle_and_idle_silence(self):
        count, current, sent, closed = [0], ["QR"], [], []
        schedule = {0: "CAM,1,QR\n", 3: "ACK,1\n", 5: "CAM,2,TABLE,156,256.0,160.0\n",
                    10: "CAM,3,IDLE\n", 11: "CAM,4,NEAR,1\n",
                    16: "CAM,5,OCCUPIED,156,256.0,160.0\n", 19: "CAM,6,IDLE\n",
                    20: "CAM,7,PICK,6\n", 23: "CAM,8,IDLE\n"}
        def read_serial():
            command = schedule.get(count[0], "")
            if command:
                current[0] = "IDLE" if command.startswith("ACK,") else command.strip().split(",")[2]
            return command.encode()
        def read_camera():
            self.assertNotEqual(current[0], "QR", "QR must not read the material camera")
            mode = current[0]
            img = (table() if mode == "TABLE" else blocks([(1, (256, 230))]) if mode == "NEAR" else
                   blocks([(1, (140, 190)), (5, (256, 190)), (6, (372, 190))]) if mode == "OCCUPIED" else
                   blocks([(6, (372, 190))]) if mode == "PICK" else canvas())
            count[0] += 1
            return types.SimpleNamespace(img=img, find_qrcodes=lambda: [types.SimpleNamespace(payload=lambda: "156+123+516+231")])
        clock = [0.]
        def monotonic():
            clock[0] += .2
            return clock[0]
        def write_serial(data):
            self.assertNotIn(count[0], (4, 5), "ACK后、下个请求前不能再发结果")
            sent.append(data.decode())
        serial = types.SimpleNamespace(read=read_serial, write=write_serial, close=lambda: closed.append("uart"))
        def read_usb():
            self.assertEqual(current[0], "QR")
            count[0] += 1
            return True, np.zeros((480, 640, 3), np.uint8)
        usb_settings = []
        usb = types.SimpleNamespace(set=lambda key, value: usb_settings.append((key, value)),
                                    isOpened=lambda: True, read=read_usb, release=lambda: closed.append("usb"))
        camera = types.SimpleNamespace(read=read_camera, close=lambda: closed.append("camera"))
        displayed = []
        display = types.SimpleNamespace(show=lambda img: displayed.append(img))
        original_preview_draw = v.IdleTablePreview.draw
        def draw_preview(preview, img):
            result = original_preview_draw(preview, img)
            img[200, 20] = (12, 34, 56)  # 预览层的绘制必须到达最终显示帧。
            return result
        maix = types.ModuleType("maix")
        maix.camera = types.SimpleNamespace(Camera=lambda w, h: camera)
        maix.display = types.SimpleNamespace(Display=lambda: display)
        maix.uart = types.SimpleNamespace(UART=lambda dev, baud: serial)
        maix.sys = types.SimpleNamespace(device_id=lambda: "maixcam2")
        mapped_pins = []
        maix.pinmap = types.SimpleNamespace(set_pin_function=lambda pin, func: mapped_pins.append((pin, func)) or 0)
        maix.err = types.SimpleNamespace(check_raise=lambda code, message: None if code == 0 else self.fail(message))
        light_values = []
        maix.gpio = types.SimpleNamespace(Mode=types.SimpleNamespace(OUT=1),
            GPIO=lambda pin, mode: types.SimpleNamespace(value=lambda level: light_values.append(level)))
        maix.app = types.SimpleNamespace(need_exit=lambda: count[0] >= 25)
        def cv2image(img, **kw):
            if img.shape[:2] == (480, 640):
                return types.SimpleNamespace(find_qrcodes=lambda: [types.SimpleNamespace(
                    payload=lambda: "156+123+516+231",
                    corners=lambda: [(100, 100), (300, 100), (300, 300), (100, 300)])])
            return img
        maix.image = types.SimpleNamespace(image2cv=lambda f, **kw: f.img.copy(), cv2image=cv2image)
        with patch.dict(sys.modules, {"maix": maix}), patch.object(v, "load_config", return_value=(self.cfg, "TEST")), patch.object(v.time, "monotonic", monotonic), patch.object(v.IdleTablePreview, "draw", draw_preview), patch.object(v.cv2, "VideoCapture", return_value=usb) as capture:
            v.main()
        capture.assert_called_once_with(0)
        self.assertEqual(usb_settings, [(cv2.CAP_PROP_FPS, 30), (cv2.CAP_PROP_FRAME_WIDTH, 640), (cv2.CAP_PROP_FRAME_HEIGHT, 480)])
        self.assertEqual(mapped_pins, [("B0", "UART2_TX"), ("B1", "UART2_RX"), ("B3", "GPIOB3")])
        self.assertEqual(light_values, [0, 1, 0, 1, 0, 1, 0, 1, 0, 0])
        for prefix in ("VISION,1,QR,", "VISION,2,TABLE,156,", "VISION,4,NEAR,1,2", "VISION,5,OCCUPIED,156,", "VISION,7,PICK,6,3"):
            self.assertTrue(any(line.startswith(prefix) for line in sent), (prefix, sent))
        self.assertFalse(any(line.startswith(("VISION,3,", "VISION,6,", "VISION,8,")) for line in sent))
        self.assertEqual(closed, ["usb", "uart", "camera"])
        self.assertGreaterEqual(len(displayed), 23)
        self.assertTrue(any(np.array_equal(img[200, 20], (12, 34, 56)) for img in displayed))
        self.assertTrue(any(img.shape[0] == v.HEIGHT + v.FOOTER_HEIGHT for img in displayed))
        # USB 全分辨率二维码框缩放到屏幕，证明显示的是该摄像头的内容。
        self.assertTrue(any(np.array_equal(img[100, 80], (0, 0, 255)) for img in displayed))

    def test_usb_qr_failure_retry_and_release(self):
        cap = types.SimpleNamespace(set=lambda *args: None, isOpened=lambda: False, release=lambda: None)
        qr = v.UsbQrCamera()
        with patch.object(v.cv2, "VideoCapture", return_value=cap) as capture, patch.object(cap, "release") as release:
            qr.select(None)
            capture.assert_not_called()
            qr.select((1, "QR", 0))
            self.assertIsNone(qr.read())
            self.assertIn("OPEN FAILED", qr.state)
            qr.select((1, "QR", 0))  # same request retry does not repeatedly reopen
            capture.assert_called_once_with(0)
            release.assert_called_once()
            qr.select((2, "IDLE", 0))
            cap.isOpened = lambda: True
            cap.read = lambda: (False, None)
            qr.select((3, "QR", 0))
            self.assertIsNone(qr.read())
            self.assertIn("READ FAILED", qr.state)
            cap.read = lambda: (True, np.zeros((480, 640, 3), np.uint8))
            with patch.object(v.time, "monotonic", return_value=qr.retry_at+1):
                qr.select((3, "QR", 0))
            self.assertEqual(qr.read().shape, (480, 640, 3))
            qr.select((4, "TABLE", 156))
            self.assertIsNone(qr.cap)
            self.assertEqual(release.call_count, 3)

    def test_uart_pin_mapping_matches_selected_port_without_switching(self):
        calls = []
        maix = types.ModuleType("maix")
        maix.sys = types.SimpleNamespace(device_id=lambda: "maixcam2")
        maix.pinmap = types.SimpleNamespace(set_pin_function=lambda pin, func: calls.append((pin, func)) or 0)
        def check(code, message):
            if code:
                raise RuntimeError(message)
        maix.err = types.SimpleNamespace(check_raise=check)
        maix.uart = types.SimpleNamespace(UART=lambda device, baud: calls.append((device, baud)))
        with patch.dict(sys.modules, {"maix": maix}):
            for device, expected in (("/dev/ttyS2", [("B0", "UART2_TX"), ("B1", "UART2_RX")]),
                                     ("/dev/ttyS4", [("A21", "UART4_TX"), ("A22", "UART4_RX")])):
                calls.clear()
                with patch.object(v, "CAMERA_UART_DEVICE", device):
                    v.open_camera_uart()
                self.assertEqual(calls, expected+[(device, 115200)])
            calls.clear()
            maix.pinmap.set_pin_function = lambda pin, func: 1
            with self.assertRaises(RuntimeError):
                v.open_camera_uart()
            self.assertEqual(calls, [])  # no UART opened after a failed pin configuration

    def test_task_light_modes_and_exception_cleanup(self):
        values, mapped = [], []
        maix = types.ModuleType("maix")
        maix.pinmap = types.SimpleNamespace(set_pin_function=lambda pin, func: mapped.append((pin, func)) or 0)
        maix.err = types.SimpleNamespace(check_raise=lambda code, message: None)
        maix.gpio = types.SimpleNamespace(Mode=types.SimpleNamespace(OUT=1),
            GPIO=lambda pin, mode: types.SimpleNamespace(value=lambda level: values.append(level)))
        with patch.dict(sys.modules, {"maix": maix}):
            lamp = v.TaskLight()
            self.assertEqual(mapped, [("B3", "GPIOB3")])
            lamp.update(None)
            for i, mode in enumerate(("QR", "TABLE", "NEAR", "RING", "OCCUPIED", "PICK")):
                lamp.update((i+1, mode, 1))
            self.assertEqual(values, [0, 1])
            lamp.update((9, "IDLE", 0))
            self.assertEqual(values[-1], 0)
            try:
                lamp.set_active(True)  # original calibration runs with the same illumination
                raise RuntimeError("simulated processing failure")
            except RuntimeError:
                pass
            finally:
                lamp.close()
            self.assertEqual(values[-2:], [1, 0])


if __name__ == "__main__":
    unittest.main(verbosity=2)
