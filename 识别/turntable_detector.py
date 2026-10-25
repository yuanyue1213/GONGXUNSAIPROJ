# -*- coding: utf-8 -*-
"""MaixCAM2 圆台识别模块（本文件不读取相机，也不控制串口/触摸屏）。

用法：
    centers, tolerances, calibrated = load_lab_thresholds()
    detector = TurntableCenterDetector(centers, tolerances,
                                       colors=("red", "yellow", "blue"))
    stable_center = detector.detect(cv_bgr_image, draw=True)

三色阈值共用统一的 JSON 文件；主程序只需标定并保存一次。
"""

import json
import math
import os
from collections import deque

import cv2
import numpy as np

# 将配置文件放在设备用户主目录，切换脚本路径/导入不同模块时仍共用一份。
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
        if data.get("version") != 1:
            raise ValueError("不支持的标定文件版本")
        saved_centers = data["lab_center"]
        saved_tols = data["lab_tol"]
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


class TurntableCenterDetector:
    """使用三个彩色圆柱求旋转圆台中心，仅对最终中心做稳定判断。"""

    def __init__(self, lab_center, lab_tol, colors=("red", "yellow", "blue"),
                 min_area=80, min_circularity=0.45,
                 triangle_tol=0.25, min_edge=15.0,
                 stable_count=5, stable_err=1.0):
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
        self.triangle_tol = float(triangle_tol)
        self.min_edge = float(min_edge)
        self.stable_count = int(stable_count)
        self.stable_err = float(stable_err)
        if self.stable_count < 1 or self.stable_err < 0:
            raise ValueError("无效的稳定判断参数")
        self.history = deque(maxlen=self.stable_count)
        self.kernel = np.ones((3, 3), np.uint8)
        self.last_points = {}
        self.last_center = None
        self.last_state = "init"

    def reset(self):
        self.history.clear()
        self.last_center = None
        self.last_points = {}
        self.last_state = "reset"

    def _center_for_color(self, lab_image, color):
        center = self.lab_center[color]
        tol = self.lab_tol[color]
        lower = np.maximum(center - tol, [0, -128, -128]).astype(np.float32)
        upper = np.minimum(center + tol, [100, 127, 127]).astype(np.float32)
        mask = cv2.inRange(lab_image, lower, upper)
        # 与原代码一致的先腐蚀两次、再膨胀两次的开运算。
        mask = cv2.erode(mask, self.kernel, iterations=2)
        mask = cv2.dilate(mask, self.kernel, iterations=2)

        contours = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )[-2]
        best_area = 0.0
        best_point = None
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_area:
                continue
            perimeter = cv2.arcLength(contour, True)
            if perimeter <= 0:
                continue
            circularity = 4.0 * math.pi * area / (perimeter * perimeter)
            if circularity < self.min_circularity:
                continue
            m = cv2.moments(contour)
            if m["m00"] <= 0:
                continue
            if area > best_area:
                best_area = area
                best_point = (float(m["m10"] / m["m00"]),
                              float(m["m01"] / m["m00"]))
        return best_point

    def detect(self, bgr_image, draw=False):
        """输入每帧 OpenCV BGR 图像，5 帧中心稳定时返回 (x, y)，否则 None。"""
        if bgr_image is None:
            self.reset()
            return None

        # 注意必须 float32/255 再转 LAB，兼容原来标定的 0~100、-128~127 值。
        blur = cv2.GaussianBlur(bgr_image, (5, 5), 0)
        lab = cv2.cvtColor(blur.astype(np.float32) / 255.0,
                           cv2.COLOR_BGR2LAB)

        points = {}
        for name in self.colors:
            p = self._center_for_color(lab, name)
            if p is not None:
                points[name] = p
                if draw:
                    px, py = int(round(p[0])), int(round(p[1]))
                    color_bgr = DRAW_COLORS_BGR.get(name, (0, 255, 255))
                    cv2.circle(bgr_image, (px, py), 4, color_bgr, -1)
                    cv2.putText(bgr_image, name, (px + 6, py - 6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, color_bgr, 1)
        self.last_points = points
        self.last_center = None

        if len(points) != 3:
            self.history.clear()
            self.last_state = "colors_missing"
            return None

        vertices = np.array([points[name] for name in self.colors],
                            dtype=np.float32)
        lengths = np.array([
            np.linalg.norm(vertices[0] - vertices[1]),
            np.linalg.norm(vertices[1] - vertices[2]),
            np.linalg.norm(vertices[2] - vertices[0]),
        ], dtype=np.float32)
        mean_len = float(np.mean(lengths))
        if (mean_len < self.min_edge or
                float(np.max(lengths) - np.min(lengths)) >
                self.triangle_tol * mean_len):
            self.history.clear()
            self.last_state = "invalid_triangle"
            return None

        # 每帧只对三个圆柱的中心取平均，不对旋转中的单个圆柱做稳定过滤。
        origin = np.mean(vertices, axis=0)
        self.last_center = (float(origin[0]), float(origin[1]))
        if draw:
            o = tuple(int(round(float(v))) for v in origin)
            cv2.drawMarker(bgr_image, o, (0, 255, 255),
                           cv2.MARKER_CROSS, 14, 1)

        self.history.append(self.last_center)
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

        # 五帧全在中位中心 1 像素半径内，才输出一组整数坐标。
        output = np.mean(samples, axis=0)
        stable_point = (int(round(float(output[0]))),
                        int(round(float(output[1]))))
        self.history.clear()
        self.last_state = "stable"
        if draw:
            cv2.drawMarker(bgr_image, stable_point, (0, 0, 255),
                           cv2.MARKER_CROSS, 22, 2)
        return stable_point
