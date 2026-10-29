"""Synthetic images + existing CAM/VISION lifecycle, no Maix hardware needed."""
import copy
import ast
import importlib.util
import math
import json
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
        centers = {name: np.array(value, np.float32) for name, value in v.DEFAULT_HSV_CENTER.items()}
        for color, bgr in BGR.items():
            centers[v.COLORS[color]] = cv2.cvtColor(np.array([[bgr]], np.uint8), cv2.COLOR_BGR2HSV)[0, 0]
        self.hsv_data = (centers, {name: np.array([8,8,8], np.float32) for name in centers}, set(centers))
        self.hsv_patch = patch.object(v, "load_hsv_thresholds", return_value=self.hsv_data)
        self.hsv_patch.start()
        self.addCleanup(self.hsv_patch.stop)
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
        bad["material_offset"] = [100, 0]
        with self.assertRaises(ValueError):
            v.validate_config(bad)

    def test_original_qr_stability_and_hsv_persistence(self):
        original = ast.parse((ROOT / "camera/MaixVision/vision_original_20261025.py").read_text(encoding="utf-8"))
        restored_text = (ROOT / "camera/MaixVision/main.py").read_text(encoding="utf-8")
        restored = ast.parse(restored_text.replace("ORIGINAL_COLORS.items()", "COLORS.items()").replace("class OriginalStableTarget:", "class StableTarget:"))
        def functions(tree):
            return {n.name: ast.dump(n, include_attributes=False) for n in tree.body if isinstance(n, (ast.FunctionDef, ast.ClassDef))}
        expected, actual = functions(original), functions(restored)
        for name in ("StableTarget",):
            self.assertEqual(actual[name], expected[name], name)
        centers, tolerances, calibrated = self.hsv_data
        with tempfile.TemporaryDirectory() as folder:
            path = str(Path(folder)/"hsv_thresholds.json")
            self.assertTrue(v.save_hsv_thresholds(centers, tolerances, calibrated, path))
            # Bypass setUp's in-memory loader to test the original persistence code.
            self.hsv_patch.stop()
            loaded = v.load_hsv_thresholds(path)
            self.assertEqual(loaded[2], calibrated)
            np.testing.assert_allclose(loaded[0]["red"], centers["red"])
            self.hsv_patch.start()

    def test_malformed_hsv_types_fall_back_without_startup_failure(self):
        self.hsv_patch.stop()
        try:
            with tempfile.TemporaryDirectory() as folder:
                path = Path(folder)/"hsv.json"
                for payload in ("[]", "null", "123", "{}",
                                '{"version":2,"color_space":"HSV","hsv_center":[],"hsv_tol":{}}',
                                '{"version":2,"color_space":"HSV","hsv_center":{},"hsv_tol":null}',
                                '{"version":2,"color_space":"HSV","hsv_center":{},"hsv_tol":{},"calibrated_colors":[{}]}'):
                    path.write_text(payload)
                    centers, tolerances, calibrated = v.load_hsv_thresholds(str(path))
                    self.assertEqual(calibrated, set())
                    for name in v.DEFAULT_HSV_CENTER:
                        np.testing.assert_allclose(centers[name], v.DEFAULT_HSV_CENTER[name])
                        np.testing.assert_allclose(tolerances[name], v.DEFAULT_HSV_TOL[name])
        finally:
            self.hsv_patch.start()

    def test_hsv_hue_wrap_sampling_and_mask(self):
        hsv = np.array([[[179, 220, 220], [1, 220, 220], [90, 220, 220],
                         [0, 0, 220], [0, 220, 30]]], np.uint8)
        center = v.sample_hsv(cv2.cvtColor(hsv[:, :2], cv2.COLOR_HSV2BGR))
        self.assertLess(min(center[0], 180-center[0]), 2)
        mask = v.hsv_mask(hsv, center, [5, 20, 20])
        self.assertEqual(mask.tolist(), [[255, 255, 0, 0, 0]])
        # 黑色H可以任意，亮度门限仍要排除白底。
        gray = np.array([[[0, 0, 20], [120, 10, 30], [0, 0, 235]]], np.uint8)
        self.assertEqual(v.hsv_mask(gray, v.DEFAULT_HSV_CENTER['black'], v.DEFAULT_HSV_TOL['black']).tolist(), [[255, 255, 0]])

    def test_old_lab_file_is_not_loaded_as_hsv(self):
        self.hsv_patch.stop()
        try:
            with tempfile.TemporaryDirectory() as folder:
                path = Path(folder)/"old.json"
                path.write_text(json.dumps({"version":1,"lab_center":{"red":[47,65,35]},"lab_tol":{"red":[15,12,12]},"calibrated_colors":["red"]}))
                self.assertEqual(v.load_hsv_thresholds(str(path))[2], set())
                centers, tolerances, calibrated = self.hsv_data
                v.save_hsv_thresholds(centers, tolerances, calibrated, str(path))
                data = json.loads(path.read_text())
                self.assertEqual(data['color_space'], 'HSV')
                data['hsv_center']['red'][0] = 180
                path.write_text(json.dumps(data))
                self.assertNotIn('red', v.load_hsv_thresholds(str(path))[2])
        finally:
            self.hsv_patch.start()

    def test_hsv_touch_calibration_samples_and_adjusts_all_channels(self):
        touches = iter([(10,10,True),(0,0,False),(10,10,True),(0,0,False),
                        (10,10,True),(0,0,False),(50,70,True),(0,0,False),
                        (50,130,True),(0,0,False),(50,190,True),(0,0,False),
                        (200,270,True),(0,0,False)])
        maix = types.ModuleType('maix')
        maix.app = types.SimpleNamespace(need_exit=lambda:False)
        ticks = iter([1000,1200,1400])
        maix.time = types.SimpleNamespace(ticks_ms=lambda:next(ticks))
        maix.image = types.SimpleNamespace(image2cv=lambda frame, **kw:frame.copy(), cv2image=lambda img, **kw:img,
            resize_map_pos_reverse=lambda a,b,c,d,fit,x,y:(x,y), Fit=types.SimpleNamespace(FIT_CONTAIN=1))
        cam = types.SimpleNamespace(read=lambda:np.full((320,512,3), BGR[1], np.uint8))
        disp = types.SimpleNamespace(show=lambda img:None,width=lambda:512,height=lambda:320)
        touch = types.SimpleNamespace(read=lambda:next(touches))
        with tempfile.TemporaryDirectory() as folder, patch.dict(sys.modules, {'maix':maix}):
            self.engine.table.path = str(Path(folder)/'hsv.json')
            old_tol = self.engine.table.tolerances['red'].copy()
            v.calibrate_table_colors(cam, disp, touch, self.engine.table, ('red',))
            np.testing.assert_allclose(self.engine.table.centers['red'], v.sample_hsv(cam.read()))
            np.testing.assert_allclose(self.engine.table.tolerances['red'], old_tol+1)
            data = json.loads(Path(self.engine.table.path).read_text())
            self.assertEqual(data['color_space'], 'HSV')

    def test_command_footer_does_not_cover_camera_pixels(self):
        img = blocks([(2, (256, 260))])
        display = cv2.copyMakeBorder(img, 0, v.FOOTER_HEIGHT, 0, 0, cv2.BORDER_CONSTANT)
        receiver = v.LineReceiver()
        receiver.feed(b"CAM,1,TABLE,123\n")
        v.draw_command_status(display, receiver)
        np.testing.assert_array_equal(display[:v.HEIGHT], img)
        self.assertTrue(np.any(display[v.HEIGHT:]))

    def test_crash_log_keeps_stage_request_and_traceback(self):
        with tempfile.TemporaryDirectory() as folder:
            path = str(Path(folder)/"vision_error.log")
            try:
                raise ValueError("test NEAR exception")
            except ValueError:
                v.save_error_log("Process NEAR", (12, "NEAR", 1), path)
            log = Path(path).read_text(encoding="utf-8")
            self.assertIn("Process NEAR", log)
            self.assertIn("(12, 'NEAR', 1)", log)
            self.assertIn("Traceback", log)
            self.assertIn("ValueError: test NEAR exception", log)
            # 日志文件无法写入也不能掩盖原始异常。
            with patch("builtins.open", side_effect=PermissionError("read only")), patch("builtins.print") as output:
                v.save_error_log("Display", None, path)
            self.assertIn("VISION ERROR", output.call_args_list[0].args[0])
            self.assertTrue(output.call_args_list[0].kwargs["flush"])

    def test_initialization_failure_prints_stage_and_traceback(self):
        maix = types.ModuleType("maix")
        for name in ("camera", "display", "app", "image"):
            setattr(maix, name, types.SimpleNamespace())
        with patch.dict(sys.modules, {"maix": maix}), patch.object(v, "load_config", side_effect=ValueError("bad config")), patch("builtins.open", side_effect=PermissionError("read only")), patch("builtins.print") as output:
            with self.assertRaisesRegex(ValueError, "bad config"):
                v.main()
        error = next(call for call in output.call_args_list if call.args and "VISION ERROR" in str(call.args[0]))
        self.assertTrue(error.kwargs["flush"])
        self.assertIn("Load configuration / HSV", error.args[0])
        self.assertIn("ValueError: bad config", error.args[0])

    def test_idle_single_material_preview_does_not_prime_task(self):
        preview = v.IdleMaterialPreview(self.engine.detector)
        img = blocks([(2, (256, 160))])
        for _ in range(5):
            preview.draw(img.copy())
        self.assertEqual(len(preview.objects), 1)
        self.assertEqual(preview.objects[0]["color"], 2)
        self.assertIsNone(self.engine.request)
        self.engine.select((80, "TABLE", 156))
        for _ in range(2):
            self.assertEqual(self.engine.process(img), "VISION,80,NONE\n")
        self.assertIn("VISION,80,TABLE,156,", self.engine.process(img))


    def test_preview_and_table_display_only_selected_yellow(self):
        img = blocks([(1, (140, 130)), (2, (256, 160)), (3, (372, 270))])
        preview = v.IdleMaterialPreview(self.engine.detector)
        preview.draw(img.copy())
        self.assertEqual([obj["color"] for obj in preview.objects], [2])
        self.engine.select((90, "TABLE", 156))  # Yellow need not be in the batch.
        self.assertIn("VISION,90,TABLE,156,", stable(self.engine, img))
        self.assertEqual([obj["color"] for obj in self.engine.objects], [2])
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

    def test_table_accepts_any_single_calibrated_color_without_other_colors(self):
        calibrated = set(self.engine.table.calibrated)
        for color in BGR:
            self.engine.table.calibrated = {v.COLORS[color]}
            self.engine.select((100+color, "TABLE", 156))
            reply = stable(self.engine, blocks([(color, (280, 180))]))
            self.assertTrue(reply.startswith("VISION,%d,TABLE,156," % (100+color)), reply)
            xy = np.array(list(map(float, reply.strip().split(",")[-2:])))
            self.assertLess(np.linalg.norm(xy-(280, 180)), 2.)
            self.assertEqual([obj["color"] for obj in self.engine.objects], [color])
        self.engine.table.calibrated = calibrated


    def test_table_selects_nearest_one_and_does_not_apply_station_offset(self):
        self.cfg["material_offset"] = [60, 40]
        self.engine.select((10, "TABLE", 156))
        img = blocks([(1, (180, 160)), (1, (286, 160)), (4, (420, 250))])
        reply = stable(self.engine, img)
        self.assertIn("VISION,10,TABLE,156,", reply)
        xy = np.array(list(map(float, reply.strip().split(",")[-2:])))
        self.assertLess(np.linalg.norm(xy-(286, 160)), 2.)
        self.assertEqual(len(self.engine.objects), 1)
        self.assertEqual(stable(self.engine, canvas()), "VISION,10,NONE\n")

    def test_centered_table_reply_uses_whole_pixels_for_fixed_coarse_move(self):
        self.engine.select((10, "TABLE", 156, 256., 160.))
        img = blocks([(2, (256, 160))])
        # 缩放拟合产生的亚像素误差不能触发STM32固定1000/5mm粗调。
        self.assertEqual(stable(self.engine, img), "VISION,10,TABLE,156,256.0,160.0\n")


    def test_near_fixed_window_accepts_each_color_without_table(self):
        for color in BGR:
            self.engine.select((100+color, "NEAR", color))
            img = blocks([(color, (256, 160))])
            self.assertEqual(self.engine.process(img), "VISION,%d,NONE\n" % (100+color))
            self.assertEqual(self.engine.process(img), "VISION,%d,NONE\n" % (100+color))
            self.assertEqual(self.engine.process(img), "VISION,%d,NEAR,%d,2\n" % (100+color, color))


    def test_near_ignores_outside_same_color_and_rejects_wrong_inside_color(self):
        self.engine.select((11, "NEAR", 1))
        img = blocks([(1, (350, 230)), (4, (256, 160))])
        self.assertEqual(stable(self.engine, img, count=5), "VISION,11,NONE\n")
        self.assertEqual(stable(self.engine, blocks([(1, (350, 230))]), count=5), "VISION,11,NONE\n")
        self.assertEqual(stable(self.engine, blocks([(1, (256, 160)), (1, (350, 230))])), "VISION,11,NEAR,1,2\n")


    def test_table_stability_resets_on_position_color_and_missing_frames(self):
        self.engine.select((10, "TABLE", 156))
        red = blocks([(1, (230, 160))])
        green = blocks([(4, (280, 160))])
        for _ in range(2):
            self.assertEqual(self.engine.process(red), "VISION,10,NONE\n")
        for _ in range(2):
            self.assertEqual(self.engine.process(green), "VISION,10,NONE\n")
        self.assertIn(",TABLE,156,", self.engine.process(green))
        # At the same pixel a new color must collect three new frames too.
        yellow = blocks([(2, (280, 160))])
        for _ in range(2):
            self.assertEqual(self.engine.process(yellow), "VISION,10,NONE\n")
        self.assertIn(",TABLE,156,", self.engine.process(yellow))
        moved = blocks([(2, (320, 160))])
        self.assertEqual(self.engine.process(moved), "VISION,10,NONE\n")
        self.assertEqual(self.engine.process(canvas()), "VISION,10,NONE\n")
        for _ in range(2):
            self.assertEqual(self.engine.process(moved), "VISION,10,NONE\n")
        self.assertIn(",TABLE,156,", self.engine.process(moved))


    def test_near_stability_restarts_after_interruption_or_color_change(self):
        self.engine.select((11, "NEAR", 1))
        img = blocks([(1, (256, 160)), (5, (180, 200)), (5, (332, 200))])
        for _ in range(2):
            self.assertEqual(self.engine.process(img), "VISION,11,NONE\n")
        self.assertEqual(self.engine.process(canvas()), "VISION,11,NONE\n")
        for _ in range(2):
            self.assertEqual(self.engine.process(img), "VISION,11,NONE\n")
        self.assertEqual(self.engine.process(img), "VISION,11,NEAR,1,2\n")
        self.engine.select((12, "NEAR", 4))
        green = blocks([(4, (256, 160))])
        for _ in range(2):
            self.assertEqual(self.engine.process(green), "VISION,12,NONE\n")
        self.assertEqual(self.engine.process(green), "VISION,12,NEAR,4,2\n")

    def test_near_requires_majority_color_coverage_in_fixed_window(self):
        self.engine.select((11, "NEAR", 1))
        small = canvas()
        cv2.rectangle(small, (244, 148), (253, 172), BGR[1], -1)
        self.assertEqual(stable(self.engine, small, count=5), "VISION,11,NONE\n")
        mostly = canvas()
        cv2.rectangle(mostly, (244, 148), (260, 172), BGR[1], -1)
        self.assertEqual(stable(self.engine, mostly), "VISION,11,NEAR,1,2\n")

    def test_table_custom_target_and_ack_preserve_fixed_pickup_point(self):
        request = (20, "TABLE", 156, 330., 210.)
        self.engine.select(request)
        img = blocks([(1, (330, 210)), (2, (256, 160))])
        self.assertIn(",TABLE,156,", stable(self.engine, img))
        self.assertEqual([obj["color"] for obj in self.engine.objects], [1])
        self.engine.select((20, "ACK", 0))
        self.assertTrue(self.engine.stopped)
        np.testing.assert_array_equal(self.engine.near.point, [330,210])
        self.engine.select((21, "IDLE", 0))
        self.engine.select((22, "NEAR", 1))
        self.assertEqual(stable(self.engine, blocks([(1, (256, 160))])), "VISION,22,NONE\n")
        self.assertEqual(stable(self.engine, blocks([(1, (330, 210))])), "VISION,22,NEAR,1,2\n")


    def test_material_frame_validation(self):
        maix = types.ModuleType("maix")
        convert = types.SimpleNamespace(image2cv=lambda frame, **kw: frame)
        maix.image = convert
        with patch.dict(sys.modules, {"maix": maix}):
            for bad in (None, np.zeros((320, 512), np.uint8), np.zeros((10, 10, 3), np.uint8), np.zeros((320, 512, 3), np.float32)):
                with self.assertRaises(ValueError):
                    v.read_material_frame(types.SimpleNamespace(read=lambda: bad))
            good = canvas()
            self.assertIs(v.read_material_frame(types.SimpleNamespace(read=lambda: good)), good)
            with patch.object(convert, "image2cv", side_effect=RuntimeError("conversion failed")):
                with self.assertRaisesRegex(RuntimeError, "conversion failed"):
                    v.read_material_frame(types.SimpleNamespace(read=lambda: good))

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

    def test_final_pose_refresh_before_pick(self):
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

    def test_shared_hsv_updates_material_detection_without_restart(self):
        self.assertIs(self.engine.detector.hsv_source, self.engine.table)
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
            np.array([[BGR[1]]], np.uint8), cv2.COLOR_BGR2HSV)[0, 0]
        self.assertEqual(len(detector.materials(red, "PICK", (1,))), 1)
        self.assertEqual(detector.materials(green, "PICK", (1,)), [])

    def test_station_and_fixed_pickup_require_requested_hsv_calibration(self):
        self.engine.table.calibrated = set(self.engine.table.calibrated)-{"red"}
        row = blocks([(1, (140, 190)), (5, (256, 190)), (6, (372, 190))])
        for req, mode, target in ((81, "OCCUPIED", 156), (82, "PICK", 1), (84, "NEAR", 1)):
            self.engine.select((req, mode, target))
            self.assertEqual(stable(self.engine, row, count=6), "VISION,%d,NONE\n" % req)
            self.assertIn("NEED HSV: red", self.engine.state)
        self.assertEqual(self.engine.detector.materials(row, "PICK", (1,)), [])
        self.engine.table.calibrated.add("red")
        self.engine.select((83, "OCCUPIED", 156))
        self.assertIn(",OCCUPIED,156,", stable(self.engine, row))

    def test_overlapping_blue_hsv_masks_are_not_double_labelled(self):
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
                    19: "CAM,5,OCCUPIED,156,256.0,160.0\n", 22: "CAM,6,IDLE\n",
                    23: "CAM,7,PICK,6\n", 26: "CAM,8,IDLE\n"}
        def read_serial():
            command = schedule.get(count[0], "")
            if command:
                current[0] = "IDLE" if command.startswith("ACK,") else command.strip().split(",")[2]
            return command.encode()
        def read_camera():
            self.assertNotEqual(current[0], "QR", "QR must not read the material camera")
            mode = current[0]
            if count[0] in (11, 12, 13):
                failed = count[0]
                count[0] += 1
                if failed == 11:
                    raise RuntimeError("temporary material capture failure")
                if failed == 12:
                    return None
                return types.SimpleNamespace(img=np.zeros((320, 512), np.uint8))
            img = (table() if mode == "TABLE" else blocks([(1, (256, 160))]) if mode == "NEAR" else
                   blocks([(1, (140, 190)), (5, (256, 190)), (6, (372, 190))]) if mode == "OCCUPIED" else
                   blocks([(6, (372, 190))]) if mode == "PICK" else canvas())
            count[0] += 1
            return types.SimpleNamespace(img=img, find_qrcodes=lambda: [types.SimpleNamespace(payload=lambda: "156+123+516+231")])
        clock = [0.]
        def monotonic():
            clock[0] += .2
            return clock[0]
        def write_serial(data):
            self.assertNotIn(count[0], (4, 5, 12, 13, 14), "ACK后或取帧失败时不能再发结果，包括NONE")
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
        original_preview_draw = v.IdleMaterialPreview.draw
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
        maix.app = types.SimpleNamespace(need_exit=lambda: count[0] >= 28)
        def cv2image(img, **kw):
            if img.shape[:2] == (480, 640):
                return types.SimpleNamespace(find_qrcodes=lambda: [types.SimpleNamespace(
                    payload=lambda: "156+123+516+231",
                    corners=lambda: [(100, 100), (300, 100), (300, 300), (100, 300)])])
            return img
        maix.image = types.SimpleNamespace(image2cv=lambda f, **kw: f.img.copy(), cv2image=cv2image)
        with patch.dict(sys.modules, {"maix": maix}), patch.object(v, "load_config", return_value=(self.cfg, "TEST")), patch.object(v.time, "monotonic", monotonic), patch.object(v.IdleMaterialPreview, "draw", draw_preview), patch.object(v.cv2, "VideoCapture", return_value=usb) as capture:
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
