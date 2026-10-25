"""MaixCAM standalone vision, fixed-ROI reference algorithm edition.

Upload this file alone. Original vision.py is neither imported nor modified.
512x320 BGR camera; /dev/ttyS2 115200; existing CAM / VISION protocol.
Color IDs: 1 red, 2 yellow, 3 blue, 4 green, 5 black, 6 light_blue.
Only the requested task runs. No motors are controlled by this program.
HSV settings are independent of the original LAB settings.
"""
import json
import math
import os
import re
import time

import cv2
import numpy as np

WIDTH, HEIGHT = 512, 320
WORK_WIDTH = 384                 # 56% of full-resolution pixels
PROCESS_INTERVAL = 0.10          # target ceiling 10 Hz, NOT a measured speed
CONFIG_PATH = os.path.expanduser("~/vision_reference_config.json")
COLORS = {1: "red", 2: "yellow", 3: "blue", 4: "green", 5: "black", 6: "light_blue"}
DEFAULT_HSV = {
    1: [[[0, 65, 45], [12, 255, 255]], [[168, 65, 45], [179, 255, 255]]],
    2: [[[16, 65, 70], [37, 255, 255]]],
    3: [[[108, 65, 30], [135, 255, 255]]],
    4: [[[38, 60, 40], [88, 255, 255]]],
    5: [[[0, 0, 0], [179, 255, 85]]],
    6: [[[80, 28, 80], [107, 255, 255]]],
}
# Rectangles are [left, top, right, bottom] in ORIGINAL 512x320 pixels.
# Tune to the installed camera; the near point must match the existing raw pose2.
DEFAULT_ROIS = {"TABLE": [12, 12, 500, 308], "MAT": [12, 12, 500, 308],
                "OCCUPIED": [12, 12, 500, 308], "PICK": [12, 12, 500, 308],
                "RING": [12, 12, 500, 308], "NEAR": [128, 176, 384, 308]}
KERNEL = np.ones((3, 3), np.uint8)


def defaults():
    return {"version": 1, "hsv": {str(k): v for k, v in DEFAULT_HSV.items()},
            "rois": {k: list(v) for k, v in DEFAULT_ROIS.items()},
            "near_point": [256, 260], "near_window": [18, 12],
            "material_offset": [0.0, 0.0], "calibrated": []}


def validate_config(cfg):
    if cfg.get("version") != 1:
        raise ValueError("config version")
    for color in COLORS:
        ranges = cfg["hsv"][str(color)]
        if not 1 <= len(ranges) <= 2:
            raise ValueError("HSV ranges")
        for low, high in ranges:
            if len(low) != 3 or len(high) != 3:
                raise ValueError("HSV channels")
            if any(type(v) is not int for v in low + high):
                raise ValueError("HSV integer")
            if any(not 0 <= low[i] <= high[i] <= (179 if i == 0 else 255) for i in range(3)):
                raise ValueError("HSV limits")
    for mode in DEFAULT_ROIS:
        x0, y0, x1, y1 = cfg["rois"][mode]
        if (any(type(v) is not int for v in (x0, y0, x1, y1)) or
                not 0 <= x0 < x1 <= WIDTH or not 0 <= y0 < y1 <= HEIGHT):
            raise ValueError("ROI " + mode)
    for key in ("near_point", "near_window", "material_offset"):
        if len(cfg[key]) != 2 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in cfg[key]):
            raise ValueError(key)
    nx, ny = cfg["near_point"]
    wx, wy = cfg["near_window"]
    x0, y0, x1, y1 = cfg["rois"]["NEAR"]
    if not (1 <= wx <= 50 and 1 <= wy <= 50 and x0 <= nx-wx < nx+wx < x1 and y0 <= ny-wy < ny+wy < y1):
        raise ValueError("near window outside ROI")
    if any(abs(v) > 80 for v in cfg["material_offset"]):
        raise ValueError("material offset")
    if any(type(v) is not int or v not in COLORS for v in cfg["calibrated"]):
        raise ValueError("calibrated colors")
    return cfg


def load_config(path=CONFIG_PATH):
    if not os.path.exists(path):
        return defaults(), "DEFAULT HSV / CALIBRATE"
    # An invalid saved config stops startup; never silently replace a tuned gate.
    with open(path, "r", encoding="utf-8") as handle:
        return validate_config(json.load(handle)), "SAVED HSV"


def save_config(cfg, path=CONFIG_PATH):
    validate_config(cfg)
    with open(path + ".tmp", "w", encoding="utf-8") as handle:
        json.dump(cfg, handle)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(path + ".tmp", path)


def sample_hsv(cfg, color, bgr, point):
    """Circular hue statistics from a small patch; black uses a value ceiling."""
    x, y = (int(v) for v in point)
    if not 5 <= x < WIDTH-5 or not 5 <= y < HEIGHT-5:
        raise ValueError("tap inside object, away from image edge")
    patch = cv2.cvtColor(bgr[y-4:y+5, x-4:x+5], cv2.COLOR_BGR2HSV).reshape(-1, 3)
    s, v = np.median(patch[:, 1:], axis=0)
    if color == 5:
        if v > 145:
            raise ValueError("black sample is too bright")
        ranges = [[[0, 0, 0], [179, 255, int(min(145, v+30))]]]
    else:
        if s < 25 or v < 30:
            raise ValueError("sample has insufficient color")
        angle = patch[:, 0].astype(float) * math.pi / 90
        h = int(round((math.atan2(np.mean(np.sin(angle)), np.mean(np.cos(angle))) * 90 / math.pi) % 180)) % 180
        low_s, low_v = max(25, int(s)-65), max(30, int(v)-65)
        high_v = 255
        lo, hi = h-12, h+12
        if lo < 0:
            ranges = [[[0, low_s, low_v], [hi, 255, high_v]], [[180+lo, low_s, low_v], [179, 255, high_v]]]
        elif hi > 179:
            ranges = [[[lo, low_s, low_v], [179, 255, high_v]], [[0, low_s, low_v], [hi-180, 255, high_v]]]
        else:
            ranges = [[[lo, low_s, low_v], [hi, 255, high_v]]]
    cfg["hsv"][str(color)] = ranges
    cfg["calibrated"] = sorted(set(cfg["calibrated"]) | {color})


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
    elif mode not in ("MAT", "PICK", "NEAR", "RING") or not 1 <= target <= (3 if mode == "RING" else 6):
        return None
    if len(f) == 4:
        return req, mode, target
    if mode not in ("TABLE", "OCCUPIED", "MAT"):
        return None
    if any(not re.match(r"^[0-9]{1,3}(\.[0-9])?$", value) for value in f[4:]):
        return None
    x, y = map(float, f[4:])
    return (req, mode, target, x, y) if 0 <= x < WIDTH and 0 <= y < HEIGHT else None


class LineReceiver:
    def __init__(self):
        self.pending = bytearray()
        self.dropping = False

    def feed(self, data):
        requests = []
        for byte in data or b"":
            if byte == 10:
                if not self.dropping:
                    try:
                        request = parse_request(self.pending.decode("ascii").rstrip("\r"))
                    except UnicodeError:
                        request = None
                    if request is not None:
                        requests.append(request)
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
    def __init__(self, cfg):
        self.cfg = cfg

    def crop(self, bgr, mode):
        x0, y0, x1, y1 = self.cfg["rois"][mode]
        scale = min(1., WORK_WIDTH/(x1-x0))
        crop = bgr[y0:y1, x0:x1]
        width, height = max(1, round((x1-x0)*scale)), max(1, round((y1-y0)*scale))
        return cv2.resize(crop, (width, height), interpolation=cv2.INTER_AREA), (x0, y0), ((x1-x0)/width, (y1-y0)/height)

    def materials(self, bgr, mode, colors):
        crop, origin, scale = self.crop(bgr, mode)
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        objects = []
        for color in colors:
            mask = np.zeros(hsv.shape[:2], np.uint8)
            for low, high in self.cfg["hsv"][str(color)]:
                mask |= cv2.inRange(hsv, np.array(low, np.uint8), np.array(high, np.uint8))
            if color in (3, 6):
                # Blue/sky-blue can overlap after manual sampling. Ambiguous
                # pixels belong to neither; never label the same block twice.
                other = np.zeros(hsv.shape[:2], np.uint8)
                for low, high in self.cfg["hsv"][str(9-color)]:
                    other |= cv2.inRange(hsv, np.array(low, np.uint8), np.array(high, np.uint8))
                mask &= cv2.bitwise_not(other)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL)
            contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            # Area/border filter BEFORE the candidate cap: large dark backgrounds
            # must not displace a real small black cylinder.
            eligible = []
            for contour in contours:
                area = cv2.contourArea(contour)*scale[0]*scale[1]
                x, y, w, h = cv2.boundingRect(contour)
                if (not 70 <= area <= 14000 or min(w, h) < 6 or max(w, h)/min(w, h) > 3
                        or x <= 0 or y <= 0 or x+w >= mask.shape[1] or y+h >= mask.shape[0]):
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


class Engine:
    """One task per frame, request-scoped stability, station-scoped pose anchors."""
    def __init__(self, cfg):
        self.cfg, self.detector = cfg, Detector(cfg)
        self.request = None
        self.stable = Stable()
        self.anchors = None
        self.station_colors = ()
        self.state = "IDLE / HOLD 1.5s HSV"
        self.objects = []

    def select(self, request):
        if request == self.request:
            return
        self.request = request
        self.stable = Stable(frames=5 if request[1] == "TABLE" else 3)
        self.objects = []
        if request[1] in ("QR", "TABLE", "RING", "OCCUPIED"):
            self.anchors = None
            self.station_colors = ()

    def process(self, bgr, qr_codes=()):
        self.objects = []
        if self.request is None or self.request[1] == "IDLE":
            self.state = "IDLE / HOLD 1.5s HSV"
            return None
        req, mode, target = self.request[:3]
        value = None
        self.state = mode + " / NOT FOUND OR AMBIGUOUS"
        if mode == "QR":
            codes = {code.strip() for code in qr_codes if valid_code(code.strip())}
            if len(codes) == 1:
                value = codes.pop()
        elif mode == "RING":
            points = self.detector.rings(bgr)
            value = self.stable.update(points)
            if value is not None:
                self.state = "RING / STABLE"
                return "VISION,%d,RING,%d,%.1f,%.1f\n" % (req, target, *value[target-1])
            return "VISION,%d,NONE\n" % req
        else:
            colors = triple(target) if mode in ("TABLE", "OCCUPIED") else (target,)
            self.objects = self.detector.materials(bgr, mode, colors)
            if mode in ("TABLE", "OCCUPIED"):
                group = unique_triple(self.objects, colors)
                if group:
                    points = np.array([o["center"] for o in group])
                    if mode == "TABLE":
                        # Three equally spaced material positions: affine projection
                        # preserves their centroid. Do NOT infer a moving near gate.
                        u, w = points[1]-points[0], points[2]-points[0]
                        area2 = abs(float(u[0]*w[1]-u[1]*w[0]))
                        longest2 = max(float(np.sum((points[i]-points[j])**2)) for i in range(3) for j in range(i))
                        if area2 >= 500 and area2/max(longest2, 1.) >= .12:
                            value = points
                    else:
                        points = np.array(sorted(points, key=lambda p: p[0]))
                        if row_valid(points):
                            value = points
                if value is not None and mode == "TABLE":
                    # Rotating individual blocks need not stay still: only the center.
                    value = np.mean(value, axis=0)
            elif mode == "PICK":
                if self.anchors is None or target not in self.station_colors:
                    self.state = "PICK / NEED OCCUPIED FIRST"
                elif len(self.objects) == 1:
                    distances = np.linalg.norm(self.anchors-np.array(self.objects[0]["center"]), axis=1)
                    index = int(np.argmin(distances))
                    spacing = min(np.linalg.norm(self.anchors[index]-p) for i, p in enumerate(self.anchors) if i != index)
                    if distances[index] <= spacing*.28:
                        value = str(index+1)
            elif mode == "NEAR":
                # Independent fixed pickup window, never selects the next-nearest
                # object or follows it with a moving arm. No three-color dependency.
                if len(self.objects) == 1:
                    p = np.array(self.objects[0]["center"])
                    if np.all(np.abs(p-self.cfg["near_point"]) <= self.cfg["near_window"]):
                        value = p
            elif mode == "MAT" and len(self.objects) == 1:
                value = self.objects[0]["center"]
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
        if mode in ("PICK", "NEAR"):
            return "VISION,%d,%s,%d,%s\n" % (req, mode, target, "2" if mode == "NEAR" else result)
        if mode == "QR":
            return "VISION,%d,QR,%s\n" % (req, result)
        return "VISION,%d,%s,%d,%.1f,%.1f\n" % (req, mode, target, *result)


def draw_status(img, engine, elapsed_ms, config_state):
    cv2.putText(img, "REFERENCE ROI / %.0f ms" % elapsed_ms, (5, 16), cv2.FONT_HERSHEY_SIMPLEX, .4, (0, 255, 0), 1)
    cv2.putText(img, engine.state, (5, 33), cv2.FONT_HERSHEY_SIMPLEX, .4, (0, 255, 255), 1)
    cv2.putText(img, config_state, (5, 50), cv2.FONT_HERSHEY_SIMPLEX, .35, (0, 255, 255), 1)
    if engine.request and engine.request[1] in DEFAULT_ROIS:
        mode = engine.request[1]
        x0, y0, x1, y1 = engine.cfg["rois"][mode]
        cv2.rectangle(img, (x0, y0), (x1, y1), (0, 180, 0), 1)
        if mode == "NEAR":
            x, y = engine.cfg["near_point"]
            wx, wy = engine.cfg["near_window"]
            cv2.rectangle(img, (int(x-wx), int(y-wy)), (int(x+wx), int(y+wy)), (0, 255, 255), 1)
    for obj in engine.objects:
        x, y = (int(round(v)) for v in obj["center"])
        cv2.drawMarker(img, (x, y), (0, 255, 0), cv2.MARKER_CROSS, 10, 1)
        cv2.putText(img, COLORS[obj["color"]], (x+5, y), cv2.FONT_HERSHEY_SIMPLEX, .35, (0, 255, 0), 1)


def main():
    from maix import camera, display, app, image, uart
    cfg, config_state = load_config()
    engine = Engine(cfg)
    receiver = LineReceiver()
    cam, disp = camera.Camera(WIDTH, HEIGHT), display.Display()
    serial = uart.UART("/dev/ttyS2", 115200)
    try:
        from maix import touchscreen
        touch = touchscreen.TouchScreen()
    except Exception as exc:
        print("Touch unavailable; edit vision_reference_config.json:", exc)
        touch = None
    calibration, chosen, previous_press = False, 1, False
    press_start = None
    dirty = False
    last_process = 0.
    elapsed_ms = 0.
    try:
        while not app.need_exit():
            for request in receiver.feed(serial.read()):
                if request != engine.request:
                    last_process = 0.
                engine.select(request)
                if request[1] != "IDLE":
                    # A motion request interrupts local editing; never delay the
                    # protocol with a blocking calibration dialog.
                    calibration = False
                    press_start = None
            frame = cam.read()
            img = image.image2cv(frame, ensure_bgr=True, copy=True)
            idle = engine.request is None or engine.request[1] == "IDLE"
            now = time.monotonic()
            if not calibration and not idle and now-last_process >= PROCESS_INTERVAL:
                start = time.monotonic()
                codes = [q.payload() for q in frame.find_qrcodes()] if engine.request[1] == "QR" else ()
                response = engine.process(img, codes)
                if response:
                    serial.write(response.encode("ascii"))
                elapsed_ms = (time.monotonic()-start)*1000
                last_process = time.monotonic()
            elif idle:
                engine.state = "IDLE / HOLD 1.5s HSV"
                engine.objects = []
            # Physical touch points must be mapped from display to camera image.
            if touch is not None:
                tx, ty, pressed = touch.read()
                mapped = image.resize_map_pos_reverse(WIDTH, HEIGHT, disp.width(), disp.height(), image.Fit.FIT_CONTAIN, tx, ty)
                tx, ty = int(mapped[0]), int(mapped[1])
                if calibration and pressed and not previous_press:
                    if 282 <= ty < HEIGHT:
                        if tx < 128:
                            chosen = 6 if chosen == 1 else chosen-1
                        elif tx < 256:
                            chosen = chosen % 6+1
                        elif tx < 384:
                            try:
                                save_config(cfg)
                                config_state = "SAVED HSV"
                                dirty = False
                            except (OSError, ValueError) as exc:
                                config_state = "SAVE FAILED: " + str(exc)[:35]
                        else:
                            calibration = False
                            config_state = "UNSAVED HSV" if dirty else config_state
                    elif 0 <= tx < WIDTH and 0 <= ty < 282:
                        try:
                            sample_hsv(cfg, chosen, img, (tx, ty))
                            dirty = True
                            config_state = "SAMPLED " + COLORS[chosen] + " / PRESS SAVE"
                        except ValueError as exc:
                            config_state = str(exc)
                elif idle and not calibration:
                    if not pressed:
                        press_start = None
                    elif press_start is None:
                        press_start = now
                    elif now-press_start >= 1.5:
                        calibration = True
                        press_start = None
                        engine.stable.reset()
                previous_press = pressed
            draw_status(img, engine, elapsed_ms, config_state)
            if calibration:
                cv2.putText(img, "HSV: " + COLORS[chosen] + " / TAP OBJECT", (5, 70), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 255), 1)
                cv2.rectangle(img, (0, 282), (511, 319), (45, 45, 45), -1)
                for i, text in enumerate(("PREV", "NEXT", "SAVE", "EXIT")):
                    cv2.putText(img, text, (i*128+14, 308), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
            disp.show(image.cv2image(img, bgr=True, copy=True))
    finally:
        # Maix APIs differ in whether close is exposed; only close if provided.
        for device in (serial, cam):
            close = getattr(device, "close", None)
            if close:
                close()


if __name__ == "__main__":
    main()
