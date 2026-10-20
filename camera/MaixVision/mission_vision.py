"""Legacy modular entry point; deploy standalone vision.py for the task.

Existing wiring: ttyS2 <-> STM32 UART4; STM32 relays CAM/VISION to ESP32.
Keep material_circle.py and circle_v2.py alongside this file on MaixCAM.
"""
import numpy as np
import cv2
from material_circle import detect_materials
from circle_v2 import color_circle_position

COLORS = {"red": 1, "yellow": 2, "blue": 3, "green": 4, "black": 5, "light_blue": 6}


def valid_code(text):
    groups = text.split("+")
    if len(groups) != 4 or any(len(g) != 3 for g in groups):
        return False
    return (all(c in "123456" for c in groups[0] + groups[2])
            and len(set(groups[0])) == len(set(groups[2])) == 3
            and set(groups[0]) == set(groups[2])
            and all(sorted(groups[i]) == list("123") for i in (1, 3)))


def parse_request(line):
    fields = line.split(",")
    if len(fields) not in (3, 4, 6) or fields[0] != "CAM":
        return None
    if not fields[1].isdigit() or not 1 <= int(fields[1]) <= 0xFFFFFFFF:
        return None
    if fields[2] in ("QR", "IDLE") and len(fields) == 3:
        return int(fields[1]), fields[2], 0
    if len(fields) in (4, 6) and fields[3].isdigit():
        target = int(fields[3])
        if fields[2] in ("MAT", "RING") and 1 <= target <= (6 if fields[2] == "MAT" else 3):
            result = int(fields[1]), fields[2], target
            if len(fields) == 4:
                return result
            if fields[2] != "MAT":
                return None
            for value in fields[4:]:
                parts = value.split(".")
                if (len(parts) > 2 or not parts[0].isdigit()
                        or (len(parts) == 2 and (len(parts[1]) != 1 or not parts[1].isdigit()))):
                    return None
            try:
                anchor = tuple(float(v) for v in fields[4:])
            except ValueError:
                return None
            if (not all(np.isfinite(v) for v in anchor) or not 0 <= anchor[0] < 512
                    or not 0 <= anchor[1] < 320):
                return None
            return result + anchor
    return None


def material_center(objects, target, anchor=None):
    matches = [obj for obj in objects if COLORS[obj["color"]] == target]
    if len(matches) == 1:
        return matches[0]["center"]
    if len(matches) > 1 and anchor is not None:
        anchor = np.array(anchor)
        ranked = sorted((float(np.linalg.norm(np.array(obj["center"]) - anchor)), i)
                        for i, obj in enumerate(matches))
        if ranked[0][0] < 180 and ranked[1][0] - ranked[0][0] > 30:
            return matches[ranked[0][1]]["center"]
    return None


class StableTarget:
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


def main():
    from maix import camera, display, app, image, uart
    cam = camera.Camera(512, 320)
    disp = display.Display()
    serial = uart.UART("/dev/ttyS2", 115200)
    request = None
    stable = StableTarget()
    pending = bytearray()
    dropping = False
    while not app.need_exit():
        incoming = serial.read()
        for byte in incoming or b"":
            if byte == 10:
                if not dropping:
                    try:
                        parsed = parse_request(pending.decode("ascii").rstrip("\r"))
                    except UnicodeError:
                        parsed = None
                    if parsed is not None and parsed != request:
                        request = parsed
                        stable = StableTarget()
                pending.clear()
                dropping = False
            elif not dropping:
                if len(pending) < 63:
                    pending.append(byte)
                else:
                    dropping = True
                    pending.clear()
        frame = cam.read()
        img = image.image2cv(frame, ensure_bgr=True, copy=True)
        found = None
        if request and request[1] != "IDLE":
            req, mode, target = request[:3]
            if mode == "QR":
                codes = {qr.payload().strip() for qr in frame.find_qrcodes()}
                codes = [code for code in codes if valid_code(code)]
                if len(codes) == 1:
                    found = codes[0]
            elif mode == "MAT":
                # All candidates, including the two/one remaining after pickup.
                objects = detect_materials(img, False, select_three=False)
                found = material_center(objects, target, request[3:] if len(request) == 5 else None)
            else:
                centers = color_circle_position(img)
                if centers is not None:
                    found = centers[2 * (target - 1):2 * target]
            result = stable.update(found)
            if result is None:
                serial.write(("VISION,%d,NONE\n" % req).encode())
            elif mode == "QR":
                serial.write(("VISION,%d,QR,%s\n" % (req, result)).encode())
                print(result)
            else:
                serial.write(("VISION,%d,%s,%d,%.1f,%.1f\n" % (req, mode, target, result[0], result[1])).encode())
                x, y = (int(round(v)) for v in result)
                cv2.drawMarker(img, (x, y), (0, 255, 255), cv2.MARKER_CROSS, 12, 1)
            cv2.putText(img, "%s target=%d" % (mode, target), (5, 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
        else:
            cv2.putText(img, "WAIT CAM REQUEST", (5, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
        disp.show(image.cv2image(img, bgr=True, copy=True))


if __name__ == "__main__":
    main()
