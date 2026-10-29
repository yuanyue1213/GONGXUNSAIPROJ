# -*- coding: utf-8 -*-
# MaixCAM / MaixPy v4 application entry: main.py
# Standalone full-flow vision. Open this file in MaixVision, run, then Install App.
# Auto-start: device Settings -> Auto Start -> select the installed application.
# Official: https://wiki.sipeed.com/maixpy/doc/zh/basic/maixvision.html
# Official: https://wiki.sipeed.com/maixpy/doc/zh/basic/auto_start.html
# Original vision.py and vision_original_20261025.py are preserved.

"""MaixCAM standalone vision, fixed-ROI reference algorithm edition.

Upload this file alone. Original vision.py is neither imported nor modified.
512x320 BGR camera; /dev/ttyS2 115200; existing CAM / VISION protocol.
Color IDs: 1 red, 2 yellow, 3 blue, 4 green, 5 black, 6 light_blue.
Only the requested task runs. No motors are controlled by this program.
QR, turntable and touch LAB calibration are restored from the original.
All material tasks share the touch-calibrated LAB thresholds.
"""
import json
import math
import os
import re
import time
from collections import deque
from itertools import product

import cv2
import numpy as np

WIDTH, HEIGHT = 512, 320
WORK_WIDTH = 384                 # 56% of full-resolution pixels
PROCESS_INTERVAL = 0.10          # target ceiling 10 Hz, NOT a measured speed
VISION_BUILD = "20261029-preview-fix"
FOOTER_HEIGHT = 100              # 状态区放在图像外，不遮挡物料和识别标记。
CAMERA_UART_DEVICE = "/dev/ttyS2"  # must match the installed TX/RX wiring
CAMERA_UART_PINS = None           # optional explicit {"pin": "UARTn_TX/RX"}
TASK_LIGHT_PIN = "B3"             # MaixCAM2 external PW5/PWM5 pin, GPIO high/low
TASK_LIGHT_GPIO = "GPIOB3"
QR_CAMERA_DEVICE = 0             # original qrcodes.py: cv2.VideoCapture(0)
IDLE_PREVIEW_TARGET = 123         # 空闲预览红、黄、蓝；正式任务颜色由STM32指令决定。
CONFIG_PATH = os.path.expanduser("~/vision_reference_config.json")
COLORS = {1: "red", 2: "yellow", 3: "blue", 4: "green", 5: "black", 6: "light_blue"}
# Rectangles are [left, top, right, bottom] in ORIGINAL 512x320 pixels.
# TABLE/NEAR use the measured turntable ellipse, not a configured fixed window.
DEFAULT_ROIS = {"OCCUPIED": [12, 12, 500, 308], "PICK": [12, 12, 500, 308],
                "RING": [12, 12, 500, 308]}
KERNEL = np.ones((3, 3), np.uint8)
BLACK_KERNEL = np.ones((5, 5), np.uint8)


def defaults():
    return {"version": 1,
            "rois": {k: list(v) for k, v in DEFAULT_ROIS.items()},
            "material_offset": [0.0, 0.0]}


def validate_config(cfg):
    if cfg.get("version") != 1:
        raise ValueError("config version")
    for mode in DEFAULT_ROIS:
        x0, y0, x1, y1 = cfg["rois"][mode]
        if (any(type(v) is not int for v in (x0, y0, x1, y1)) or
                not 0 <= x0 < x1 <= WIDTH or not 0 <= y0 < y1 <= HEIGHT):
            raise ValueError("ROI " + mode)
    for key in ("material_offset",):
        if len(cfg[key]) != 2 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in cfg[key]):
            raise ValueError(key)
    if any(abs(v) > 80 for v in cfg["material_offset"]):
        raise ValueError("material offset")
    # 旧 HSV、TABLE/NEAR 固定窗口等字段忽略；颜色统一读取 LAB 标定文件。
    return {"version": 1,
            "rois": {mode: cfg["rois"][mode] for mode in DEFAULT_ROIS},
            "material_offset": cfg["material_offset"]}


def load_config(path=CONFIG_PATH):
    if not os.path.exists(path):
        return defaults(), "DEFAULT ROI / SHARED LAB"
    # An invalid saved config stops startup; never silently replace a tuned gate.
    with open(path, "r", encoding="utf-8") as handle:
        return validate_config(json.load(handle)), "SAVED ROI / SHARED LAB"


def save_config(cfg, path=CONFIG_PATH):
    validate_config(cfg)
    with open(path + ".tmp", "w", encoding="utf-8") as handle:
        json.dump(cfg, handle)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(path + ".tmp", path)


def valid_code(text):
    groups = text.split("+")
    return (len(groups) == 4 and all(len(g) == 3 for g in groups)
            and all(c in "123456" for c in groups[0]+groups[2])
            and len(set(groups[0])) == 3 and set(groups[0]) == set(groups[2])
            and all(sorted(groups[i]) == list("123") for i in (1, 3)))


def triple(target):
    text = str(target)
    return tuple(int(c) for c in text) if len(text) == 3 and len(set(text)) == 3 and all(c in "123456" for c in text) else None


def parse_request(line):
    f = line.split(",")
    if len(f) == 2 and f[0] == "ACK" and re.fullmatch(r"[0-9]{1,10}", f[1]):
        req = int(f[1])
        return (req, "ACK", 0) if 1 <= req <= 0xFFFFFFFF else None
    if len(f) not in (3, 4, 6) or f[0] != "CAM" or not re.match(r"^[0-9]{1,10}$", f[1]):
        return None
    req = int(f[1])
    if not 1 <= req <= 0xFFFFFFFF:
        return None
    mode = f[2]
    if len(f) == 3:
        return (req, mode, 0) if mode in ("QR", "IDLE") else None
    if not re.match(r"^[0-9]{1,3}$", f[3]):
        return None
    target = int(f[3])
    if mode in ("TABLE", "OCCUPIED"):
        if triple(target) is None:
            return None
    elif mode not in ("PICK", "NEAR", "RING") or not 1 <= target <= (3 if mode == "RING" else 6):
        return None
    if len(f) == 4:
        return req, mode, target
    if mode not in ("TABLE", "OCCUPIED"):
        return None
    if any(not re.match(r"^[0-9]{1,3}(\.[0-9])?$", value) for value in f[4:]):
        return None
    x, y = map(float, f[4:])
    return (req, mode, target, x, y) if 0 <= x < WIDTH and 0 <= y < HEIGHT else None


class LineReceiver:
    def __init__(self):
        self.pending = bytearray()
        self.dropping = False
        self.accepted = 0
        self.rejected = 0
        self.last_data = b""
        self.last_command = ""

    def feed(self, data):
        requests = []
        if data:
            self.last_data = bytes(data[-16:])
        for byte in data or b"":
            if byte == 10:
                request = None
                if not self.dropping:
                    try:
                        request = parse_request(self.pending.decode("ascii").rstrip("\r"))
                    except UnicodeError:
                        request = None
                    if request is not None:
                        self.last_command = self.pending.decode("ascii").rstrip("\r")
                        self.accepted += 1
                        requests.append(request)
                if request is None and (self.pending or self.dropping):
                    self.rejected += 1
                    print("CAM RX rejected:", bytes(self.pending), "overflow:", self.dropping)
                self.pending.clear()
                self.dropping = False
            elif not self.dropping:
                if len(self.pending) >= 63:
                    self.pending.clear()
                    self.dropping = True
                else:
                    self.pending.append(byte)
        return requests


class Stable:
    def __init__(self, frames=3, tolerance=2.0):
        self.frames, self.tolerance = frames, tolerance
        self.reset()

    def reset(self):
        self.samples = []

    def update(self, value):
        if value is None:
            self.reset()
            return None
        if self.samples:
            old = self.samples[-1]
            different = (old != value) if isinstance(value, str) else np.max(np.linalg.norm(
                np.asarray(value).reshape(-1, 2)-np.mean(self.samples, axis=0).reshape(-1, 2), axis=1)) > self.tolerance
            if different:
                self.reset()
        self.samples.append(value)
        self.samples = self.samples[-self.frames:]
        if len(self.samples) < self.frames:
            return None
        return value if isinstance(value, str) else np.mean(self.samples, axis=0)


def ellipse_fit(contour):
    if len(contour) < 5:
        return None
    points = np.ascontiguousarray(contour[::max(1, len(contour)//80)], dtype=np.float32)
    if len(points) < 5:
        return None
    try:
        ellipse = cv2.fitEllipse(points)
    except cv2.error:
        return None
    (x, y), (a, b), angle = ellipse
    if not all(math.isfinite(v) for v in (x, y, a, b, angle)) or min(a, b) < 4:
        return None
    delta = points.reshape(-1, 2)-np.array([x, y])
    theta = math.radians(angle)
    u = (delta[:, 0]*math.cos(theta)+delta[:, 1]*math.sin(theta))/(a/2)
    v = (-delta[:, 0]*math.sin(theta)+delta[:, 1]*math.cos(theta))/(b/2)
    error = float(np.mean(np.abs(np.hypot(u, v)-1)))
    return ellipse, error


def base_center(contour):
    """Bounded fitting: complete contour + two lower arcs; no exhaustive arc search.

    Prefer the lower base ellipse for a stepped cylinder. If no credible arc
    exists, return None instead of substituting an unrelated blob centroid.
    """
    x, y, w, h = cv2.boundingRect(contour)
    fits = []
    for index, part in enumerate((contour, contour[contour[:, 0, 1] >= y+h*.50], contour[contour[:, 0, 1] >= y+h*.65])):
        result = ellipse_fit(part)
        if result is None:
            continue
        ((cx, cy), (a, b), angle), error = result
        minor, major = min(a, b), max(a, b)
        fill = abs(cv2.contourArea(contour))/(math.pi*a*b/4)
        if (error > .11 or minor/major < .40 or not .65*w <= major <= 1.35*w
                or not x <= cx <= x+w or not y <= cy <= y+h or b > h*2
                or not .55 <= fill <= 3.5):
            continue
        # The ellipse's image-bottom must coincide with the object's bottom.
        theta = math.radians(angle)
        extent = math.hypot(a*.5*math.sin(theta), b*.5*math.cos(theta))
        bottom_error = abs(cy+extent-(y+h-1))/max(w, 1)
        if bottom_error > .18:
            continue
        if index == 0 and error < .04 and bottom_error < .08:
            return cx, cy
        fits.append((error+bottom_error*.5, (cx, cy)))
    return min(fits, key=lambda item: item[0])[1] if fits else None


class Detector:
    def __init__(self, cfg, lab_source=None):
        self.cfg = cfg
        # 与 TABLE/NEAR 共用同一对象，触屏修改和中断后的重新加载立即生效。
        self.lab_source = lab_source if lab_source is not None else TurntableTaskDetector()

    def crop(self, bgr, mode):
        x0, y0, x1, y1 = self.cfg["rois"][mode]
        scale = min(1., WORK_WIDTH/(x1-x0))
        crop = bgr[y0:y1, x0:x1]
        width, height = max(1, round((x1-x0)*scale)), max(1, round((y1-y0)*scale))
        return cv2.resize(crop, (width, height), interpolation=cv2.INTER_AREA), (x0, y0), ((x1-x0)/width, (y1-y0)/height)

    def materials(self, bgr, mode, colors):
        crop, origin, scale = self.crop(bgr, mode)
        source = self.lab_source
        if not all(COLORS[color] in source.calibrated for color in colors):
            return []
        blur = cv2.GaussianBlur(crop, (5, 5), 0)
        lab = cv2.cvtColor(blur.astype(np.float32)/255., cv2.COLOR_BGR2LAB)
        def color_mask(color):
            name = COLORS[color]
            center, tol = source.centers[name], source.tolerances[name]
            lower = np.maximum(center-tol, [0, -128, -128]).astype(np.float32)
            upper = np.minimum(center+tol, [100, 127, 127]).astype(np.float32)
            return cv2.inRange(lab, lower, upper)
        objects = []
        for color in colors:
            mask = color_mask(color)
            if color in (3, 6) and COLORS[9-color] in source.calibrated:
                # Blue/sky-blue can overlap after manual sampling. Ambiguous
                # pixels belong to neither; never label the same block twice.
                mask &= cv2.bitwise_not(color_mask(9-color))
            # 黑色物料有实心区域，圆环只有细线：先去细线，避免外环包住物料轮廓。
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, BLACK_KERNEL if color == 5 else KERNEL)
            if color != 5:
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL)
            solid = cv2.distanceTransform(mask, cv2.DIST_L2, 3) if color == 5 else None
            # 黑色保留轮廓层级，才能找到圆环内部的实心物料；闭运算会连上细环。
            contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE if color == 5 else cv2.RETR_EXTERNAL,
                                                  cv2.CHAIN_APPROX_NONE)[-2:]
            # Area/border filter BEFORE the candidate cap: large dark backgrounds
            # must not displace a real small black cylinder.
            eligible = []
            for index, contour in enumerate(contours):
                area = cv2.contourArea(contour)*scale[0]*scale[1]
                x, y, w, h = cv2.boundingRect(contour)
                if (not 70 <= area <= 14000 or min(w, h) < 6 or max(w, h)/min(w, h) > 3
                        or x <= 0 or y <= 0 or x+w >= mask.shape[1] or y+h >= mask.shape[0]):
                    continue
                if color == 5:
                    # 圆环断片也可能近似椭圆，但没有物料那样宽的实心内核。
                    if solid[y:y+h, x:x+w].max() < max(5., max(w, h)*.20):
                        continue
                    parent, depth = hierarchy[0][index][3], 0
                    while parent >= 0:
                        depth += 1
                        parent = hierarchy[0][parent][3]
                    if depth % 2:  # 白色孔洞不是黑色物料。
                        continue
                    child, holes = hierarchy[0][index][2], 0.
                    while child >= 0:
                        holes += cv2.contourArea(contours[child])*scale[0]*scale[1]
                        child = hierarchy[0][child][0]
                    if holes > area*.20:  # 排除空心环，允许实物上的少量高光。
                        continue
                eligible.append((area, contour))
            for area, contour in sorted(eligible, key=lambda c: c[0], reverse=True)[:6]:
                center = base_center(contour)
                if center is None:
                    continue
                offset = self.cfg["material_offset"]
                center = (origin[0]+center[0]*scale[0]+offset[0], origin[1]+center[1]*scale[1]+offset[1])
                if not 0 <= center[0] < WIDTH or not 0 <= center[1] < HEIGHT:
                    continue
                objects.append({"color": color, "center": center, "area": area})
        return objects

    def rings(self, bgr):
        crop, origin, scale = self.crop(bgr, "RING")
        gray = cv2.GaussianBlur(cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY), (3, 3), 0)
        mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)[1]
        contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)[-2]
        eligible = [c for c in contours if 30 <= abs(cv2.contourArea(c))*scale[0]*scale[1] <= 20000]
        candidates = []
        for contour in sorted(eligible, key=lambda c: abs(cv2.contourArea(c)), reverse=True)[:64]:
            fit = ellipse_fit(contour)
            if fit is None:
                continue
            ((x, y), (a, b), angle), error = fit
            area = abs(cv2.contourArea(contour))/(math.pi*a*b/4)
            if error > .08 or min(a, b)/max(a, b) < .45 or not .78 <= area <= 1.18:
                continue
            center = (origin[0]+x*scale[0], origin[1]+y*scale[1])
            radius = (a*scale[0]+b*scale[1])/4
            if 5 <= radius <= 78:
                candidates.append((center, radius))
        groups = []
        for center, radius in candidates:
            group = next((g for g in groups if np.linalg.norm(np.array(center)-g[0][0]) <= 3), None)
            if group is None:
                groups.append([(center, radius)])
            else:
                group.append((center, radius))
        targets = []
        for group in groups:
            levels = []
            for center, radius in sorted(group, key=lambda c: c[1]):
                if not levels or radius-levels[-1][1] >= max(2.5, radius*.09):
                    levels.append((center, radius))
            if len(levels) >= 3 and levels[-1][1]/levels[0][1] >= 1.3:
                targets.append(np.mean([p for p, r in levels], axis=0))
        # More than three credible targets is ambiguous; do not pick a random row.
        if len(targets) != 3:
            return None
        points = np.array(sorted(targets, key=lambda p: p[0]))
        return points if row_valid(points) else None


def row_valid(points):
    left, middle, right = np.asarray(points)
    gaps = np.diff(np.asarray(points)[:, 0])
    if min(gaps) < 12 or max(gaps) > min(gaps)*1.8:
        return False
    predicted = left[1]+(middle[0]-left[0])/(right[0]-left[0])*(right[1]-left[1])
    return abs(middle[1]-predicted) <= max(6., (right[0]-left[0])*.1)


def unique_triple(objects, colors):
    selected = []
    for color in colors:
        matches = [o for o in objects if o["color"] == color]
        if len(matches) != 1:
            return None
        selected.append(matches[0])
    points = np.array([o["center"] for o in selected])
    if any(np.linalg.norm(points[i]-points[j]) < 18 for i in range(3) for j in range(i)):
        return None
    if max(o["area"] for o in selected) > min(o["area"] for o in selected)*3.5:
        return None
    return selected



# ===== Original QR, LAB calibration and turntable implementation =====
ORIGINAL_COLORS = {name: value for value, name in COLORS.items()}
DEFAULT_CONFIG_PATH = os.path.join(os.path.expanduser("~"), "lab_thresholds.json")

# 与用户原始代码完全一致的浮点 LAB 预设值；不是已经标定成功的值。
DEFAULT_LAB_CENTER = {
    "light_blue": (77, -14, -29),
    "blue": (42, 21, -60),
    "yellow": (81, 6, 75),
    "green": (80, -78, 59),
    "black": (21, 2, 0),
    "red": (47, 65, 35),
}
DEFAULT_LAB_TOL = {
    "light_blue": (8, 8, 8),
    "blue": (8, 8, 8),
    "yellow": (15, 12, 12),
    "green": (10, 10, 10),
    "black": (12, 10, 10),
    "red": (15, 12, 12),
}

DRAW_COLORS_BGR = {
    "light_blue": (255, 255, 0),
    "blue": (255, 0, 0),
    "yellow": (0, 255, 255),
    "green": (0, 255, 0),
    "black": (180, 180, 180),
    "red": (0, 0, 255),
}


def _valid_triplet(obj, positive=False):
    try:
        a = np.asarray(obj, dtype=np.float32)
        if a.shape != (3,) or not bool(np.all(np.isfinite(a))):
            return None
        if positive and not bool(np.all(a >= 1.0)):
            return None
        return a
    except (TypeError, ValueError):
        return None


def load_lab_thresholds(path=DEFAULT_CONFIG_PATH):
    """返回 LAB_CENTER, LAB_TOL, calibrated_colors。

    没有文件时返回原始预设值，但 calibrated_colors 为空：不会误判为已标定。
    """
    centers = {k: np.array(v, dtype=np.float32)
               for k, v in DEFAULT_LAB_CENTER.items()}
    tolerances = {k: np.array(v, dtype=np.float32)
                  for k, v in DEFAULT_LAB_TOL.items()}
    calibrated = set()

    if not os.path.isfile(path):
        print("未找到 LAB 标定文件：", path)
        return centers, tolerances, calibrated

    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        if not isinstance(data, dict) or data.get("version") != 1:
            raise ValueError("不支持的标定文件版本")
        saved_centers = data["lab_center"]
        saved_tols = data["lab_tol"]
        if not isinstance(saved_centers, dict) or not isinstance(saved_tols, dict):
            raise ValueError("LAB 参数必须是颜色字典")
        saved_calibrated = data.get("calibrated_colors", [])
        if not isinstance(saved_calibrated, list):
            raise ValueError("calibrated_colors 格式错误")

        valid = set()
        for name in centers:
            c = _valid_triplet(saved_centers.get(name))
            t = _valid_triplet(saved_tols.get(name), positive=True)
            if c is not None and t is not None:
                # 防止损坏的标定文件生成非法 LAB 范围。
                if (0 <= c[0] <= 100 and -128 <= c[1] <= 127
                        and -128 <= c[2] <= 127):
                    centers[name] = c
                    tolerances[name] = t
                    valid.add(name)

        calibrated = set(saved_calibrated) & valid
        print("已读取 LAB 标定文件：", path)
        print("已完成标定的颜色：", sorted(calibrated))
    except (OSError, ValueError, TypeError, KeyError) as exc:
        print("读取 LAB 标定文件失败，使用原始预设值：", exc)
        centers = {k: np.array(v, dtype=np.float32)
                   for k, v in DEFAULT_LAB_CENTER.items()}
        tolerances = {k: np.array(v, dtype=np.float32)
                      for k, v in DEFAULT_LAB_TOL.items()}
        calibrated.clear()
    return centers, tolerances, calibrated


def save_lab_thresholds(centers, tolerances, calibrated, path=DEFAULT_CONFIG_PATH):
    """原子写入 JSON，避免程序中止导致旧标定文件被写坏。"""
    payload = {
        "version": 1,
        "lab_center": {k: [float(x) for x in v] for k, v in centers.items()},
        "lab_tol": {k: [float(x) for x in v] for k, v in tolerances.items()},
        "calibrated_colors": sorted(calibrated),
    }
    temp_path = path + ".tmp"
    try:
        with open(temp_path, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2, ensure_ascii=False)
            f.flush()
            os.fsync(f.fileno())
        os.replace(temp_path, path)
        print("标定结果已保存：", path)
        return True
    except OSError as exc:
        print("保存 LAB 标定结果失败：", exc)
        try:
            if os.path.exists(temp_path):
                os.remove(temp_path)
        except OSError:
            pass
        return False


def turntable_geometry(vertices, min_edge=15., min_axis_ratio=.4):
    """Affine ellipse of three equal-spaced positions (120 degrees apart).

    For p=c+A*cos(t)+B*sin(t), three equally spaced samples give
    M=A*A.T+B*B.T=2*mean((p-c)*(p-c).T). No equal IMAGE edges are required.
    This is an affine projection model, not a full perspective calibration.
    """
    vertices = np.asarray(vertices, dtype=float)
    if vertices.shape != (3, 2) or not np.all(np.isfinite(vertices)):
        return None
    if any(np.linalg.norm(vertices[i] - vertices[j]) < min_edge
           for i, j in ((0, 1), (1, 2), (2, 0))):
        return None
    center = np.mean(vertices, axis=0)
    centered = vertices - center
    matrix = 2. * centered.T.dot(centered) / 3.
    values = np.linalg.eigvalsh(matrix)
    if values[0] <= 1.e-6 or values[1] <= 1.e-6:
        return None
    minor, major = np.sqrt(values)
    if minor / major < min_axis_ratio or major > 320.:
        return None
    # Lowest image point of an arbitrarily rotated ellipse. It need not be
    # vertically below its center when the ellipse's axes are tilted.
    vertical_radius = math.sqrt(float(matrix[1, 1]))
    bottom = center + (matrix[0, 1] / vertical_radius, vertical_radius)
    return dict(center=center, matrix=matrix, inverse=np.linalg.inv(matrix),
                minor=float(minor), major=float(major), bottom=bottom)


def on_turntable_track(point, geometry, tolerance=.3):
    delta = np.asarray(point, dtype=float) - geometry['center']
    if delta.shape != (2,) or not np.all(np.isfinite(delta)):
        return False
    radius2 = float(delta.dot(geometry['inverse']).dot(delta))
    return (1. - tolerance)**2 <= radius2 <= (1. + tolerance)**2


class TurntableCenterDetector:
    """使用三个彩色圆柱求旋转圆台中心，仅对最终中心做稳定判断。"""

    def __init__(self, lab_center, lab_tol, colors=("red", "yellow", "blue"),
                 min_area=80, min_circularity=0.45,
                 triangle_tol=0.25, min_edge=15.0,
                 stable_count=5, stable_err=1.0, min_axis_ratio=.4, max_area_ratio=3.):
        if len(colors) != 3 or len(set(colors)) != 3:
            raise ValueError("圆台必须配置三种不同颜色")
        if not all(c in lab_center and c in lab_tol for c in colors):
            raise ValueError("圆台颜色必须全部定义在 LAB 阈值字典中")
        self.colors = tuple(colors)
        # 引用外部字典；主程序标定后更新字典，识别模块会自动使用新阈值。
        self.lab_center = lab_center
        self.lab_tol = lab_tol
        self.min_area = float(min_area)
        self.min_circularity = float(min_circularity)
        # triangle_tol retained for source compatibility; image-edge equality is no longer used.
        self.min_axis_ratio = float(min_axis_ratio)
        self.max_area_ratio = float(max_area_ratio)
        self.min_edge = float(min_edge)
        self.stable_count = int(stable_count)
        self.stable_err = float(stable_err)
        if self.stable_count < 1 or self.stable_err < 0:
            raise ValueError("无效的稳定判断参数")
        self.history = deque(maxlen=self.stable_count)
        self.geometry_history = deque(maxlen=self.stable_count)
        self.locked_geometry = self.last_geometry = None
        self.lost_frames = 0
        self.selected_candidates = {}
        self.kernel = np.ones((3, 3), np.uint8)
        self.last_points = {}
        self.last_center = None
        self.last_state = "init"

    def reset(self):
        self.history.clear()
        self.geometry_history.clear()
        self.locked_geometry = self.last_geometry = None
        self.lost_frames = 0
        self.selected_candidates = {}
        self.last_center = None
        self.last_points = {}
        self.last_state = "reset"

    def _candidates_for_color(self, lab_image, color):
        center, tol = self.lab_center[color], self.lab_tol[color]
        lower = np.maximum(center - tol, [0, -128, -128]).astype(np.float32)
        upper = np.minimum(center + tol, [100, 127, 127]).astype(np.float32)
        mask = cv2.inRange(lab_image, lower, upper)
        mask = cv2.erode(mask, self.kernel, iterations=2)
        mask = cv2.dilate(mask, self.kernel, iterations=2)
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        candidates = []
        h, w = mask.shape
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_area or area > h*w*.15:
                continue
            perimeter = cv2.arcLength(contour, True)
            if perimeter <= 0:
                continue
            circularity = 4.*math.pi*area/(perimeter*perimeter)
            if circularity < self.min_circularity:
                continue
            x, y, bw, bh = cv2.boundingRect(contour)
            if x <= 0 or y <= 0 or x+bw >= w or y+bh >= h:
                continue  # A clipped blob's centroid is not a complete material location.
            m = cv2.moments(contour)
            if m['m00'] > 0:
                candidates.append(dict(center=(float(m['m10']/m['m00']), float(m['m01']/m['m00'])),
                                       area=float(area), circularity=float(circularity)))
        return candidates

    def _center_for_color(self, lab_image, color):
        # Legacy single-color callers must not silently choose a larger distractor.
        candidates = self._candidates_for_color(lab_image, color)
        return candidates[0]['center'] if len(candidates) == 1 else None

    def _reject(self, state):
        self.history.clear()
        self.geometry_history.clear()
        self.last_state = state
        self.lost_frames += 1
        if self.lost_frames >= self.stable_count*3:
            # A manually moved camera must be able to reacquire in idle preview.
            # Fresh samples must still pass the full stability window.
            self.locked_geometry = None
        return None

    def _select_group(self, candidates):
        groups = []
        for group in product(*(candidates[c] for c in self.colors)):
            areas = np.array([obj['area'] for obj in group])
            if np.max(areas)/np.min(areas) > self.max_area_ratio:
                continue
            geometry = turntable_geometry([obj['center'] for obj in group], self.min_edge, self.min_axis_ratio)
            if geometry is None:
                continue
            if self.locked_geometry is not None:
                old = self.locked_geometry
                if np.linalg.norm(geometry['center'] - old['center']) > max(6., old['minor']*.12):
                    continue
                if any(not on_turntable_track(obj['center'], old) for obj in group):
                    continue
            score = float(np.std(np.log(areas)))
            score += .2*(1.-min(obj['circularity'] for obj in group))
            score += .1*(1.-geometry['minor']/geometry['major'])
            groups.append((score, group, geometry))
        groups.sort(key=lambda item: item[0])
        if not groups:
            return None, 'invalid_triangle'
        if len(groups)>1 and groups[1][0]-groups[0][0] < .12:
            return None, 'ambiguous_colors'
        return groups[0], None

    def detect(self, bgr_image, draw=False):
        """输入每帧 OpenCV BGR 图像，5 帧中心稳定时返回 (x, y)，否则 None。"""
        if bgr_image is None:
            self.reset()
            return None

        # 注意必须 float32/255 再转 LAB，兼容原来标定的 0~100、-128~127 值。
        blur = cv2.GaussianBlur(bgr_image, (5, 5), 0)
        lab = cv2.cvtColor(blur.astype(np.float32) / 255.0,
                           cv2.COLOR_BGR2LAB)

        candidates = {name: self._candidates_for_color(lab, name) for name in self.colors}
        self.last_points = {c: objects[0]['center'] for c, objects in candidates.items() if len(objects)==1}
        self.last_center = self.last_geometry = None
        self.selected_candidates = {}
        if any(not objects for objects in candidates.values()):
            return self._reject('colors_missing')
        if any(len(objects)>6 for objects in candidates.values()):
            return self._reject('ambiguous_colors')
        selected, error = self._select_group(candidates)
        if selected is None:
            return self._reject(error)
        _, group, geometry = selected
        self.lost_frames = 0
        self.selected_candidates = dict(zip(self.colors, group))
        self.last_points = {c: obj['center'] for c, obj in self.selected_candidates.items()}
        origin = geometry['center']
        self.last_center = tuple(float(v) for v in origin)
        self.last_geometry = geometry
        if draw:
            for name, point in self.last_points.items():
                p = tuple(int(round(v)) for v in point)
                color = DRAW_COLORS_BGR.get(name, (0, 255, 255))
                cv2.circle(bgr_image, p, 4, color, -1)
                cv2.putText(bgr_image, name, (p[0]+6, p[1]-6), cv2.FONT_HERSHEY_SIMPLEX, .45, color, 1)
        if draw:
            o = tuple(int(round(float(v))) for v in origin)
            cv2.drawMarker(bgr_image, o, (0, 255, 255),
                           cv2.MARKER_CROSS, 14, 1)

        self.history.append(self.last_center)
        self.geometry_history.append(geometry)
        self.last_state = "collecting"
        if len(self.history) < self.stable_count:
            return None

        samples = np.array(self.history, dtype=np.float32)
        reference = np.median(samples, axis=0)
        distances = np.linalg.norm(samples - reference, axis=1)
        if not np.all(distances <= self.stable_err):
            # 保留最近五帧滑动窗口，下一帧继续检测。
            self.last_state = "unstable"
            return None

        bottoms = np.asarray([g['bottom'] for g in self.geometry_history])
        if np.max(np.linalg.norm(bottoms - np.median(bottoms, axis=0), axis=1)) > max(2., self.stable_err*2.):
            self.last_state = 'unstable_geometry'
            return None
        # Lock a five-frame ellipse, not a single noisy set of rotating points.
        mean_matrix = np.mean([g['matrix'] for g in self.geometry_history], axis=0)
        mean_center = np.mean(samples, axis=0)
        values = np.linalg.eigvalsh(mean_matrix)
        vy = math.sqrt(float(mean_matrix[1, 1]))
        self.locked_geometry = dict(center=mean_center, matrix=mean_matrix, inverse=np.linalg.inv(mean_matrix),
                                    minor=float(math.sqrt(values[0])), major=float(math.sqrt(values[1])),
                                    bottom=mean_center + (mean_matrix[0, 1]/vy, vy))
        self.last_geometry = self.locked_geometry
        # 五帧全在中位中心 1 像素半径内，才输出一组整数坐标。
        output = np.mean(samples, axis=0)
        stable_point = (int(round(float(output[0]))),
                        int(round(float(output[1]))))
        self.history.clear()
        self.geometry_history.clear()
        self.last_state = "stable"
        if draw:
            cv2.drawMarker(bgr_image, stable_point, (0, 0, 255),
                           cv2.MARKER_CROSS, 22, 2)
        return stable_point


def table_colors(target):
    digits = str(target)
    if len(digits) != 3 or len(set(digits)) != 3 or any(d not in "123456" for d in digits):
        return None
    names = {value: name for name, value in ORIGINAL_COLORS.items()}
    return tuple(names[int(d)] for d in digits)


class TurntableTaskDetector:
    """A batch-specific five-frame center; never fall back to a single block."""
    def __init__(self, path=DEFAULT_CONFIG_PATH):
        self.path = path
        self.centers, self.tolerances, self.calibrated = load_lab_thresholds(path)
        self.reset()

    def reset(self):
        self.target = None
        self.detector = None
        self.state = "waiting"

    def detect(self, frame, target, draw=False):
        colors = table_colors(target)
        if colors is None:
            self.reset()
            self.state = "invalid_colors"
            return None
        if self.target != target:
            self.reset()
            self.target = target
        if not all(color in self.calibrated for color in colors):
            self.detector = None
            self.state = "NEED LAB: " + "/".join(c for c in colors if c not in self.calibrated)
            return None
        if self.detector is None:
            self.detector = TurntableCenterDetector(self.centers, self.tolerances, colors=colors)
        result = self.detector.detect(frame, draw=draw)
        self.state = self.detector.last_state
        if self.state == "colors_missing":
            self.state = "MISSING: " + "/".join(c for c in colors if c not in self.detector.last_points)
        return result




class NearPickup:
    """One fixed pickup point at the image-bottom side of the turntable.

    Retain its geometry after removal. A new color requires fresh, stationary
    observations at that point, rather than choosing one of three block poses.
    """
    def __init__(self):
        self.colors = None
        self.center = self.point = None
        self.radius = None
        self.geometry = None
        self.areas = {}
        self.reset()

    def reset(self):
        self.history = deque(maxlen=5)
        self.target = None

    def bind(self, points, target, geometry=None, areas=None):
        self.geometry = None
        self.areas = {}
        self.colors = table_colors(target)
        self.center = self.point = None
        self.reset()
        if self.colors is None or any(c not in points for c in self.colors):
            return False
        vertices = np.asarray([points[c] for c in self.colors], dtype=float)
        if vertices.shape != (3, 2) or not np.all(np.isfinite(vertices)):
            return False
        geometry = geometry if geometry is not None else turntable_geometry(vertices)
        if geometry is None:
            return False
        point = geometry['bottom']
        if geometry['minor'] < 10 or not (0 <= point[0] < 512 and 0 <= point[1] < 320):
            return False
        self.geometry = geometry
        self.center, self.point, self.radius = geometry['center'].copy(), point.copy(), geometry['minor']
        self.areas = dict(areas or {})
        return True

    def update(self, points, target):
        if target != self.target:
            self.reset()
            self.target = target
        names = {value: name for name, value in ORIGINAL_COLORS.items()}
        name = names.get(target)
        point = points.get(name)
        valid = self.point is not None and name in (self.colors or ()) and point is not None
        if valid:
            point = np.asarray(point, dtype=float)
            valid = point.shape == (2,) and np.all(np.isfinite(point))
        if valid:
            # The fixed bottom ROI remains fixed as other blocks disappear.
            window = min(8., max(3., self.radius * .12))
            valid = np.linalg.norm(point - self.point) <= window
            valid = valid and all(float(p[1]) <= float(point[1]) + 2.
                                  for c, p in points.items() if c != name)
        if not valid:
            self.history.clear()
            return None
        self.history.append(point.copy())
        if len(self.history) < 5:
            return None
        samples = np.asarray(self.history)
        if np.max(np.linalg.norm(samples - np.median(samples, axis=0), axis=1)) > 2.:
            return None
        return "2"  # Dedicated near pose, stored in the R middle profile.

    def detect(self, img, target, table):
        if self.point is None or not all(c in table.calibrated for c in self.colors):
            self.reset()
            return None
        blur = cv2.GaussianBlur(img, (5, 5), 0)
        lab = cv2.cvtColor(blur.astype(np.float32) / 255., cv2.COLOR_BGR2LAB)
        detector = TurntableCenterDetector(table.centers, table.tolerances, colors=self.colors)
        points = {}
        for color in self.colors:
            candidates = [obj for obj in detector._candidates_for_color(lab, color)
                          if on_turntable_track(obj['center'], self.geometry)]
            if color in self.areas:
                candidates = [obj for obj in candidates if 1./3. <= obj['area']/self.areas[color] <= 3.]
            if len(candidates) > 1:
                self.history.clear()
                return None  # Never guess which same-color blob is the real target.
            if candidates:
                points[color] = candidates[0]['center']
        return self.update(points, target)


class OriginalStableTarget:
    def __init__(self):
        self.samples = []
        self.text = None

    def update(self, result):
        if result is None:
            self.samples = []
            self.text = None
            return None
        if isinstance(result, str):
            if result != self.text:
                self.samples = []
            self.text = result
        elif self.samples and np.linalg.norm(np.array(result) - np.mean(self.samples, axis=0)) > 1.5:
            self.samples = []
        self.samples.append(result)
        self.samples = self.samples[-3:]
        if len(self.samples) < 3:
            return None
        return result if isinstance(result, str) else tuple(np.mean(self.samples, axis=0))


def calibrate_table_colors(cam, dis, ts, table, colors, poll=None):
    """Original touch LAB workflow, available only while no task is active."""
    from maix import app, image, time
    LAB_CENTER, LAB_TOL, CALIBRATED_COLORS = table.centers, table.tolerances, table.calibrated
    detector = table
    DOUBLE_CLICK_MS, TOL_STEP, SAMPLE_HALF, SAMPLE_BOX_HALF = 500, 1, 6, 10
    SKIP_RECT = (390, 5, 510, 55)
    L_PLUS_RECT, L_MINUS_RECT = (10, 50, 246, 105), (266, 50, 502, 105)
    A_PLUS_RECT, A_MINUS_RECT = (10, 115, 246, 170), (266, 115, 502, 170)
    B_PLUS_RECT, B_MINUS_RECT = (10, 180, 246, 235), (266, 180, 502, 235)
    NEXT_RECT = (150, 250, 362, 315)

    def read_touch():
        x, y, pressed = ts.read()
        if not pressed:
            return 0, 0, False
        x, y = image.resize_map_pos_reverse(
            WIDTH, HEIGHT, dis.width(), dis.height(),
            image.Fit.FIT_CONTAIN, x, y
        )
        return int(x), int(y), True

    def point_in_rect(x, y, rect):
        x1, y1, x2, y2 = rect
        return x1 <= x <= x2 and y1 <= y <= y2

    def wait_release():
        while not app.need_exit():
            if poll:
                poll()
            _, _, pressed = read_touch()
            if not pressed:
                return

    def draw_button(img, rect, text, color=(0, 255, 0)):
        x1, y1, x2, y2 = rect
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        text_size = cv2.getTextSize(
            text, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2
        )[0]
        tx = x1 + (x2 - x1 - text_size[0]) // 2
        ty = y1 + (y2 - y1 + text_size[1]) // 2
        cv2.putText(img, text, (tx, ty),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)

    def show(cv_img):
        dis.show(image.cv2image(cv_img, bgr=True, copy=True))

    def get_lab_range(color):
        center = LAB_CENTER[color]
        tol = LAB_TOL[color]
        lower = center - tol
        upper = center + tol
        lower[0] = max(0, lower[0])
        upper[0] = min(100, upper[0])
        lower[1] = max(-128, lower[1])
        upper[1] = min(127, upper[1])
        lower[2] = max(-128, lower[2])
        upper[2] = min(127, upper[2])
        return lower.astype(np.float32), upper.astype(np.float32)

    def make_mask(bgr_image, color):
        # 与原代码一致：高斯 -> BGR float LAB -> 阈值 -> 腐蚀/膨胀。
        gs_img = cv2.GaussianBlur(bgr_image, (5, 5), 0)
        img_float = gs_img.astype(np.float32) / 255.0
        lab_img = cv2.cvtColor(img_float, cv2.COLOR_BGR2LAB)
        lower, upper = get_lab_range(color)
        mask = cv2.inRange(lab_img, lower, upper)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=2)
        return mask

    def sample_lab_center(color):
        """双击进入确认模式，在画面中心对颜色取样；SKIP 不覆盖旧值。"""
        last_click_time = 0
        click_count = 0
        pressed_last = False
        sample_ready = False

        while not app.need_exit():
            if poll:
                poll()
            img = cam.read()
            raw_cv_img = image.image2cv(img, ensure_bgr=True, copy=True)
            cv_img = raw_cv_img.copy()
            h, w = cv_img.shape[:2]
            cx, cy = w // 2, h // 2

            cv2.rectangle(cv_img, (0, 0), (300, 40), (0, 0, 0), -1)
            cv2.putText(cv_img, "COLOR: " + color, (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
            draw_button(cv_img, SKIP_RECT, "SKIP", (0, 0, 255))
            cv2.drawMarker(cv_img, (cx, cy), (0, 0, 255),
                           cv2.MARKER_CROSS, 14, 2)
            cv2.circle(cv_img, (cx, cy), 2, (0, 0, 255), -1)

            if not sample_ready:
                cv2.putText(cv_img, "DOUBLE CLICK", (170, 300),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
            else:
                cv2.rectangle(cv_img,
                              (cx - SAMPLE_BOX_HALF, cy - SAMPLE_BOX_HALF),
                              (cx + SAMPLE_BOX_HALF, cy + SAMPLE_BOX_HALF),
                              (0, 255, 0), 2)
                cv2.putText(cv_img, "PRESS TO CONFIRM", (145, 300),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            show(cv_img)
            x, y, pressed = read_touch()
            if pressed and not pressed_last:
                if point_in_rect(x, y, SKIP_RECT):
                    print("跳过颜色：", color)
                    wait_release()
                    return False
                now = time.ticks_ms()
                if not sample_ready:
                    if now - last_click_time <= DOUBLE_CLICK_MS:
                        click_count += 1
                    else:
                        click_count = 1
                    last_click_time = now
                    if click_count >= 2:
                        sample_ready = True
                        click_count = 0
                else:
                    roi = raw_cv_img[
                        cy - SAMPLE_HALF: cy + SAMPLE_HALF,
                        cx - SAMPLE_HALF: cx + SAMPLE_HALF
                    ]
                    roi_float = roi.astype(np.float32) / 255.0
                    roi_lab = cv2.cvtColor(roi_float, cv2.COLOR_BGR2LAB)
                    LAB_CENTER[color] = np.mean(roi_lab, axis=(0, 1)).astype(np.float32)
                    print(color, "LAB_CENTER =", LAB_CENTER[color])
                    wait_release()
                    return True
            pressed_last = pressed
        return False

    def draw_adjust_ui(show_img, color):
        draw_button(show_img, L_PLUS_RECT, "L+")
        draw_button(show_img, L_MINUS_RECT, "L-")
        draw_button(show_img, A_PLUS_RECT, "A+")
        draw_button(show_img, A_MINUS_RECT, "A-")
        draw_button(show_img, B_PLUS_RECT, "B+")
        draw_button(show_img, B_MINUS_RECT, "B-")
        draw_button(show_img, NEXT_RECT, "NEXT", (0, 255, 255))
        text = "L:{} A:{} B:{}".format(*[int(x) for x in LAB_TOL[color]])
        cv2.putText(show_img, text, (20, 45),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    def adjust_lab_tol(color):
        """显示二值图，用原 L±、A±、B± 微调阈值；NEXT 时提交。"""
        pressed_last = False
        while not app.need_exit():
            if poll:
                poll()
            img = cam.read()
            cv_img = image.image2cv(img, ensure_bgr=True, copy=True)
            mask = make_mask(cv_img, color)
            show_img = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            draw_adjust_ui(show_img, color)
            show(show_img)

            x, y, pressed = read_touch()
            if pressed and not pressed_last:
                if point_in_rect(x, y, NEXT_RECT):
                    print(color, "最终 LAB_CENTER =", LAB_CENTER[color])
                    print(color, "最终 LAB_TOL =", LAB_TOL[color])
                    wait_release()
                    return True
                elif point_in_rect(x, y, L_PLUS_RECT):
                    LAB_TOL[color][0] += TOL_STEP
                elif point_in_rect(x, y, L_MINUS_RECT):
                    LAB_TOL[color][0] -= TOL_STEP
                elif point_in_rect(x, y, A_PLUS_RECT):
                    LAB_TOL[color][1] += TOL_STEP
                elif point_in_rect(x, y, A_MINUS_RECT):
                    LAB_TOL[color][1] -= TOL_STEP
                elif point_in_rect(x, y, B_PLUS_RECT):
                    LAB_TOL[color][2] += TOL_STEP
                elif point_in_rect(x, y, B_MINUS_RECT):
                    LAB_TOL[color][2] -= TOL_STEP

                LAB_TOL[color][:] = np.maximum(LAB_TOL[color], 1)
                print(color, "LAB_TOL =", LAB_TOL[color])
            pressed_last = pressed
        return False

    def calibrate_colors(colors):
        """一次标定流程，NEXT 时把结果保存到磁盘；未完成的颜色不标记。"""
        detector.reset()
        print("进入 LAB 颜色标定：", list(colors))
        for color in colors:
            if app.need_exit():
                break
            print("------ 当前颜色：", color, "------")
            # 备份这一个颜色，避免在标定中断时把内存值留成半成品。
            old_center = LAB_CENTER[color].copy()
            old_tol = LAB_TOL[color].copy()
            old_was_calibrated = color in CALIBRATED_COLORS
            if not sample_lab_center(color):
                continue
            if not adjust_lab_tol(color):
                LAB_CENTER[color] = old_center
                LAB_TOL[color] = old_tol
                break

            # 对每个颜色分别保存：意外掉电也不必重新标定已完成的颜色。
            CALIBRATED_COLORS.add(color)
            if not save_lab_thresholds(
                    LAB_CENTER, LAB_TOL, CALIBRATED_COLORS, table.path):
                # 保存失败就恢复此颜色之前的状态，避免误认为下次能复用。
                LAB_CENTER[color] = old_center
                LAB_TOL[color] = old_tol
                if not old_was_calibrated:
                    CALIBRATED_COLORS.discard(color)
                print("该颜色标定未保存，请检查设备磁盘权限：", color)
        detector.reset()
        print("标定过程结束，已完成：", sorted(CALIBRATED_COLORS))

    calibrate_colors(colors)


class IdleTablePreview:
    """Local-only preview. Its samples/geometry never authorize a task response."""
    def __init__(self, table, target=IDLE_PREVIEW_TARGET):
        self.table = table
        self.target = target
        self.reset()

    def reset(self):
        self.detector = None
        self.center = None
        self.state = "waiting"

    def draw(self, img):
        colors = table_colors(self.target)
        self.center = None
        missing_lab = [c for c in colors if c not in self.table.calibrated]
        if missing_lab:
            self.detector = None
            self.state = "NEED LAB: " + "/".join(missing_lab)
        else:
            if self.detector is None:
                self.detector = TurntableCenterDetector(self.table.centers, self.table.tolerances, colors=colors)
            self.center = self.detector.detect(img, draw=True)
            self.state = self.detector.last_state
            if self.state == "colors_missing":
                self.state = "MISSING: " + "/".join(c for c in colors if c not in self.detector.last_points)
        cv2.putText(img, "PREVIEW TABLE %d / WAIT CAM" % self.target, (5, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 255, 0), 1)
        cv2.putText(img, self.state, (5, 40), cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 255, 255), 1)
        count = len(self.detector.last_points) if self.detector is not None else 0
        label = "COLORS %d/3 / HOLD 1.5s: LAB" % count
        if self.center is not None:
            label = "CENTER %d,%d / PREVIEW ONLY" % self.center
            near = NearPickup()
            if near.bind(self.detector.last_points, self.target, geometry=self.detector.last_geometry):
                cv2.circle(img, tuple(int(round(v)) for v in near.point), 8, (0, 255, 255), 1)
        cv2.putText(img, label, (5, 60), cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 255, 255), 1)
        return self.center


class Engine:
    """One task per frame, request-scoped stability, station-scoped pose anchors."""
    def __init__(self, cfg):
        self.cfg = cfg
        self.table = TurntableTaskDetector()
        self.detector = Detector(cfg, self.table)
        self.request = None
        self.stopped = False
        self.responded = False
        self.stable = Stable()
        self.qr_stable = OriginalStableTarget()
        self.near = NearPickup()
        self.anchors = None
        self.station_colors = ()
        self.state = "IDLE / HOLD 1.5s LAB"
        self.objects = []

    def select(self, request):
        if request[1] == "ACK":
            if self.request is not None and request[0] == self.request[0] and self.responded:
                self.stopped = True
            return
        if request == self.request:
            return
        self.request = request
        self.stopped = False
        self.responded = False
        self.stable = Stable()
        self.qr_stable = OriginalStableTarget()
        self.table.reset()
        if request[1] in ("TABLE", "OCCUPIED", "RING", "QR"):
            self.near = NearPickup()
        if request[1] == "NEAR":
            self.near.reset()
        self.objects = []
        if request[1] in ("QR", "TABLE", "RING", "OCCUPIED"):
            self.anchors = None
            self.station_colors = ()

    def process(self, bgr, qr_codes=()):
        if self.stopped:
            self.state = "ACK RECEIVED / WAIT NEXT COMMAND"
            return None
        response = self._process(bgr, qr_codes)
        if response is not None:
            self.responded = True
        return response

    def _process(self, bgr, qr_codes=()):
        self.objects = []
        if self.request is None or self.request[1] == "IDLE":
            self.state = "IDLE / HOLD 1.5s LAB"
            return None
        req, mode, target = self.request[:3]
        value = None
        self.state = mode + " / NOT FOUND OR AMBIGUOUS"
        if mode == "QR":
            codes = {code.strip() for code in qr_codes if valid_code(code.strip())}
            result = self.qr_stable.update(codes.pop() if len(codes) == 1 else None)
            self.state = "QR / STABLE" if result else "QR / WAITING"
            return "VISION,%d,QR,%s\n" % (req, result) if result else "VISION,%d,NONE\n" % req
        elif mode == "TABLE":
            result = self.table.detect(bgr, target, draw=False)
            self.state = "TABLE / " + self.table.state
            if self.table.detector is not None:
                self.objects = [{"color": ORIGINAL_COLORS[c], "center": p}
                                for c, p in self.table.detector.last_points.items()]
            if result is not None:
                detector = self.table.detector
                if not self.near.bind(detector.last_points, target, geometry=detector.last_geometry,
                                      areas={c: obj["area"] for c, obj in detector.selected_candidates.items()}):
                    result = None
            return "VISION,%d,TABLE,%d,%.1f,%.1f\n" % (req, target, *result) if result is not None else "VISION,%d,NONE\n" % req
        elif mode == "NEAR":
            result = self.near.detect(bgr, target, self.table)
            self.state = "NEAR / STABLE" if result else "NEAR / WAIT TARGET AT ORIGINAL ELLIPSE BOTTOM"
            if self.near.point is None:
                self.state = "NEAR / NEED TABLE FIRST"
            return "VISION,%d,NEAR,%d,%s\n" % (req, target, result) if result else "VISION,%d,NONE\n" % req
        elif mode == "RING":
            points = self.detector.rings(bgr)
            value = self.stable.update(points)
            if value is not None:
                self.state = "RING / STABLE"
                return "VISION,%d,RING,%d,%.1f,%.1f\n" % (req, target, *value[target-1])
            return "VISION,%d,NONE\n" % req
        else:
            colors = triple(target) if mode == "OCCUPIED" else (target,)
            missing = [COLORS[c] for c in colors if COLORS[c] not in self.table.calibrated]
            if missing:
                self.stable.reset()
                self.state = mode + " / NEED LAB: " + "/".join(missing)
                return "VISION,%d,NONE\n" % req
            self.objects = self.detector.materials(bgr, mode, colors)
            if mode == "OCCUPIED":
                group = unique_triple(self.objects, colors)
                if group:
                    points = np.array(sorted((o["center"] for o in group), key=lambda p: p[0]))
                    if row_valid(points):
                        value = points
            elif mode == "PICK":
                if self.anchors is None or target not in self.station_colors:
                    self.state = "PICK / NEED OCCUPIED FIRST"
                elif len(self.objects) == 1:
                    distances = np.linalg.norm(self.anchors-np.array(self.objects[0]["center"]), axis=1)
                    index = int(np.argmin(distances))
                    spacing = min(np.linalg.norm(self.anchors[index]-p) for i, p in enumerate(self.anchors) if i != index)
                    if distances[index] <= spacing*.28:
                        value = str(index+1)
        result = self.stable.update(value)
        if result is None:
            if value is not None:
                self.state = mode + " / STABILIZING"
            return "VISION,%d,NONE\n" % req
        self.state = mode + " / STABLE"
        if mode == "OCCUPIED":
            self.anchors = np.array(result, copy=True)
            self.station_colors = colors
            result = result[1]
        if mode == "PICK":
            return "VISION,%d,PICK,%d,%s\n" % (req, target, result)
        return "VISION,%d,%s,%d,%.1f,%.1f\n" % (req, mode, target, *result)


def draw_command_status(img, receiver):
    """只有通过parse_request校验的完整帧才显示接收正确。"""
    height = img.shape[0]
    cv2.rectangle(img, (0, height-82), (WIDTH-1, height-43), (0, 0, 0), -1)
    if receiver.last_command:
        cv2.putText(img, "COMMAND RECEIVED OK", (5, height-66),
                    cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 0), 1)
        cv2.putText(img, receiver.last_command, (5, height-48),
                    cv2.FONT_HERSHEY_SIMPLEX, .4, (0, 255, 0), 1)
    else:
        cv2.putText(img, "WAITING FOR VALID COMMAND", (5, height-58),
                    cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 255, 255), 1)


def draw_status(img, engine, elapsed_ms, config_state):
    cv2.putText(img, "%s / %.0f ms" % (VISION_BUILD, elapsed_ms), (5, 16), cv2.FONT_HERSHEY_SIMPLEX, .4, (0, 255, 0), 1)
    cv2.putText(img, engine.state, (5, 33), cv2.FONT_HERSHEY_SIMPLEX, .4, (0, 255, 255), 1)
    cv2.putText(img, config_state, (5, 50), cv2.FONT_HERSHEY_SIMPLEX, .35, (0, 255, 255), 1)
    if engine.request and engine.request[1] in ("RING", "OCCUPIED", "PICK"):
        x0, y0, x1, y1 = engine.cfg["rois"][engine.request[1]]
        cv2.rectangle(img, (x0, y0), (x1, y1), (0, 180, 0), 1)
    if engine.request and engine.request[1] == "NEAR" and engine.near.point is not None:
        point = tuple(int(round(x)) for x in engine.near.point)
        cv2.circle(img, point, 8, (0, 255, 255), 1)
    for obj in engine.objects:
        x, y = (int(round(v)) for v in obj["center"])
        cv2.drawMarker(img, (x, y), (0, 255, 0), cv2.MARKER_CROSS, 10, 1)
        cv2.putText(img, COLORS[obj["color"]], (x+5, y), cv2.FONT_HERSHEY_SIMPLEX, .35, (0, 255, 0), 1)


def open_camera_uart():
    """Set BOTH UART pin functions before opening the selected port (official API).

    No port scanning or automatic switch to ttyS4: pins must match actual wiring.
    """
    from maix import uart, pinmap, sys, err
    board = sys.device_id()
    mapping = ({"/dev/ttyS2": {"B0": "UART2_TX", "B1": "UART2_RX"},
                "/dev/ttyS4": {"A21": "UART4_TX", "A22": "UART4_RX"}}
               if board == "maixcam2" else
               {"/dev/ttyS0": {"A16": "UART0_TX", "A17": "UART0_RX"},
                "/dev/ttyS1": {"A19": "UART1_TX", "A18": "UART1_RX"}})
    pins = CAMERA_UART_PINS if CAMERA_UART_PINS is not None else mapping.get(CAMERA_UART_DEVICE)
    if pins is None:
        raise RuntimeError("No verified UART pin mapping for %s %s; set CAMERA_UART_DEVICE/PINS to actual wiring" % (board, CAMERA_UART_DEVICE))
    number = CAMERA_UART_DEVICE.rsplit("ttyS", 1)[-1]
    if len(pins) != 2 or set(pins.values()) != {"UART%s_TX" % number, "UART%s_RX" % number}:
        raise ValueError("CAMERA_UART_PINS must contain matching TX and RX functions")
    for pin, function in pins.items():
        err.check_raise(pinmap.set_pin_function(pin, function), "Failed UART pin %s -> %s" % (pin, function))
    print("CAM UART:", board, CAMERA_UART_DEVICE, pins, "115200 8N1")
    return uart.UART(CAMERA_UART_DEVICE, 115200)


class TaskLight:
    """External PW5 (B3) high during tasks/calibration; IDLE/exit drives it low."""
    def __init__(self):
        from maix import gpio, pinmap, err
        err.check_raise(pinmap.set_pin_function(TASK_LIGHT_PIN, TASK_LIGHT_GPIO), "Failed task light pin")
        self.gpio = gpio.GPIO(TASK_LIGHT_GPIO, gpio.Mode.OUT)
        self.active = False
        self.gpio.value(0)

    def set_active(self, active):
        active = bool(active)
        if active != self.active:
            self.gpio.value(1 if active else 0)
            self.active = active

    def update(self, request):
        self.set_active(request is not None and request[1] != "IDLE")

    def close(self):
        self.gpio.value(0)
        self.active = False


class UsbQrCamera:
    """Original QR-only camera settings; never substitute the material camera."""
    def __init__(self):
        self.cap = None
        self.request = None
        self.state = "USB QR IDLE"
        self.retry_at = 0.

    def close(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def select(self, request):
        if request == self.request:
            if request is None or request[1] != "QR" or self.cap is not None or time.monotonic() < self.retry_at:
                return
        self.close()
        self.request = request
        self.state = "USB QR IDLE"
        if request is None or request[1] != "QR":
            return
        self.retry_at = time.monotonic() + 1.
        try:
            self.cap = cv2.VideoCapture(QR_CAMERA_DEVICE)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            if not self.cap.isOpened():
                self.close()
                self.state = "USB QR CAMERA OPEN FAILED"
            else:
                self.state = "USB QR READY"
        except cv2.error:
            self.close()
            self.state = "USB QR CAMERA OPEN FAILED"

    def read(self):
        if self.cap is None:
            return None
        try:
            ok, frame = self.cap.read()
        except cv2.error:
            ok, frame = False, None
        if not ok or frame is None or frame.ndim != 3 or frame.shape[2] != 3:
            self.close()
            self.retry_at = time.monotonic() + 1.
            self.state = "USB QR FRAME READ FAILED"
            return None
        self.state = "USB QR READY"
        return frame


def main():
    from maix import camera, display, app, image
    cfg, config_state = load_config()
    engine = Engine(cfg)
    receiver = LineReceiver()
    print("VISION BUILD:", VISION_BUILD, "ENTRY:", os.path.abspath(__file__))
    serial = open_camera_uart()
    print("CAM UART ready:", CAMERA_UART_DEVICE, "115200; waiting for CAM request")
    cam, disp = camera.Camera(WIDTH, HEIGHT), display.Display()
    preview = IdleTablePreview(engine.table)
    rx_bytes = 0
    last_request = "NONE"
    qr_camera = UsbQrCamera()
    light = TaskLight()
    try:
        from maix import touchscreen
        touch = touchscreen.TouchScreen()
    except Exception as exc:
        print("LAB touch calibration unavailable:", exc)
        touch = None
    press_start = None
    last_process = 0.
    elapsed_ms = 0.
    started = time.monotonic()
    def poll_calibration():
        # 比赛任务优先：标定期间也接收串口，收到任务立即退出标定。
        nonlocal rx_bytes, last_request
        incoming = serial.read()
        rx_bytes += len(incoming or b"")
        requests = receiver.feed(incoming)
        for request in requests:
            engine.select(request)
            if request[1] != "ACK":
                last_request = "%s#%d" % (request[1], request[0])
        if engine.request is not None and engine.request[1] != "IDLE" and not engine.stopped:
            raise InterruptedError("CAM task received")
    try:
        while not app.need_exit():
            incoming = serial.read()
            rx_bytes += len(incoming or b"")
            command_changed = False
            for request in receiver.feed(incoming):
                if request[1] == "ACK":
                    engine.select(request)
                    continue
                last_request = "%s#%d" % (request[1], request[0])
                if request != engine.request:
                    command_changed = True
                    print("CAM RX:", request)
                    last_process = 0.
                    preview.reset()
                engine.select(request)
                light.update(request)
                if request[1] != "IDLE":
                    press_start = None
            if command_changed:
                # 先确认收包，再打开/读取USB；即使USB阻塞也能看到指令已收到。
                received_img = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
                draw_command_status(received_img, receiver)
                disp.show(image.cv2image(received_img, bgr=True, copy=True))
            # 保留原请求以忽略同编号补发；确认后关闭任务设备，等待新请求。
            active_request = None if engine.stopped else engine.request
            light.update(active_request)
            qr_camera.select(active_request)
            qr_mode = active_request is not None and active_request[1] == "QR"
            qr_bgr = qr_camera.read() if qr_mode else None
            if qr_mode:
                # Decode the original 640x480 USB image; resize for display only.
                img = cv2.resize(qr_bgr, (WIDTH, HEIGHT)) if qr_bgr is not None else np.zeros((HEIGHT, WIDTH, 3), np.uint8)
            else:
                frame = cam.read()
                img = image.image2cv(frame, ensure_bgr=True, copy=True)
            idle = active_request is None or active_request[1] == "IDLE"
            now = time.monotonic()
            if not idle and now-last_process >= PROCESS_INTERVAL:
                start = time.monotonic()
                codes = []
                if qr_mode and qr_bgr is not None:
                    # 和原版一样在 USB 原图解码；框和内容映射到屏幕预览。
                    for qr in image.cv2image(qr_bgr, bgr=True, copy=False).find_qrcodes():
                        payload = qr.payload().strip()
                        codes.append(payload)
                        corners = np.asarray(qr.corners(), dtype=np.float32)
                        corners *= (WIDTH/qr_bgr.shape[1], HEIGHT/qr_bgr.shape[0])
                        corners = np.rint(corners).astype(np.int32)
                        cv2.polylines(img, [corners], True, (0, 0, 255), 2)
                        x, y = corners.min(axis=0)
                        cv2.putText(img, payload, (max(0, int(x)), max(65, int(y)-8)),
                                    cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 0, 255), 1)
                response = engine.process(img, codes)
                if qr_mode and qr_bgr is None:
                    engine.state = qr_camera.state
                if response:
                    serial.write(response.encode("ascii"))
                elapsed_ms = (time.monotonic()-start)*1000
                last_process = time.monotonic()
            elif idle:
                if not engine.stopped and now-last_process >= PROCESS_INTERVAL:
                    start = time.monotonic()
                    # Original local-only preview: samples never prime a formal task.
                    preview.draw(img)
                    elapsed_ms = (time.monotonic()-start)*1000
                    last_process = time.monotonic()
                engine.state = "IDLE PREVIEW %d/3 / %s" % (
                    len(preview.detector.last_points) if preview.detector is not None else 0, preview.state)
                if engine.stopped:
                    engine.state = "ACK RECEIVED / WAIT NEXT COMMAND"
                    engine.objects = []
                engine.objects = ([{"color": ORIGINAL_COLORS[c], "center": point}
                                   for c, point in preview.detector.last_points.items()]
                                  if not engine.stopped and preview.detector is not None else [])
            if touch is not None and idle:
                _, _, pressed = touch.read()
                if not pressed:
                    press_start = None
                elif press_start is None:
                    press_start = now
                elif now-press_start >= 1.5:
                    light.set_active(True)
                    try:
                        calibrate_table_colors(cam, disp, touch, engine.table,
                            ("light_blue", "blue", "yellow", "green", "black", "red"), poll_calibration)
                    except InterruptedError:
                        # 丢弃当前颜色未提交的采样；之前 NEXT 保存的颜色仍保留。
                        engine.table.centers, engine.table.tolerances, engine.table.calibrated = load_lab_thresholds(engine.table.path)
                    finally:
                        light.update(None if engine.stopped else engine.request)
                    engine.table.reset()
                    preview.reset()
                    last_process = 0.
                    engine.near = NearPickup()
                    engine.anchors = None
                    engine.station_colors = ()
                    engine.stable.reset()
                    engine.qr_stable = OriginalStableTarget()
                    press_start = None
                    config_state = "SHARED LAB CALIBRATION"
            else:
                press_start = None
            # 识别始终使用原始512x320坐标；只在显示时追加独立状态区。
            img = cv2.copyMakeBorder(img, 0, FOOTER_HEIGHT, 0, 0,
                                     cv2.BORDER_CONSTANT, value=(0, 0, 0))
            display_height = img.shape[0]
            draw_status(img, engine, elapsed_ms, config_state)
            draw_command_status(img, receiver)
            cv2.putText(img, "UP:%ds" % (time.monotonic()-started), (WIDTH-78, 16),
                        cv2.FONT_HERSHEY_SIMPLEX, .35, (0, 255, 0), 1)
            if qr_mode:
                cv2.putText(img, "USB QR / device %s" % QR_CAMERA_DEVICE, (5, display_height-24),
                            cv2.FONT_HERSHEY_SIMPLEX, .4, (0, 255, 255), 1)
            if last_request == "NONE":
                cv2.putText(img, "RX RAW: " + repr(receiver.last_data), (5, display_height-25),
                            cv2.FONT_HERSHEY_SIMPLEX, .35, (0, 255, 255), 1)
            cv2.putText(img, "U%s RX:%d OK:%d BAD:%d LAST:%s" % (
                CAMERA_UART_DEVICE.rsplit("ttyS", 1)[-1], rx_bytes,
                receiver.accepted, receiver.rejected, last_request), (5, display_height-7),
                        cv2.FONT_HERSHEY_SIMPLEX, .35, (0, 255, 255), 1)
            disp.show(image.cv2image(img, bgr=True, copy=True))
    except Exception as exc:
        # 避免异常退出后屏幕保留最后一帧 IDLE，让人误以为程序仍在收串口。
        print("VISION FATAL:", repr(exc))
        error_img = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
        cv2.putText(error_img, "VISION ERROR / " + VISION_BUILD, (5, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 0, 255), 1)
        cv2.putText(error_img, str(exc)[:65], (5, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, .35, (0, 255, 255), 1)
        disp.show(image.cv2image(error_img, bgr=True, copy=True))
        raise
    finally:
        light.close()
        qr_camera.close()
        # Maix APIs differ in whether close is exposed; only close if provided.
        for device in (serial, cam):
            close = getattr(device, "close", None)
            if close:
                close()


if __name__ == "__main__":
    main()
