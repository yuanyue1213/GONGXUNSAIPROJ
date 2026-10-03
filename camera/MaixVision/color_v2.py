
from maix import camera, display, app, image
import cv2
import numpy as np

LAB_CENTER = {
    "light_blue": np.array([77, -14, -29], dtype=np.float32),
    "blue": np.array([42, 21, -60], dtype=np.float32),
    "yellow": np.array([81, 6, 75], dtype=np.float32),
    "green": np.array([80, -78, 59], dtype=np.float32),
    "black": np.array([21, 2, 0], dtype=np.float32),
    "red": np.array([47, 65, 35], dtype=np.float32)
}

LAB_TOL = {
    "light_blue": np.array([8, 8, 8], dtype=np.float32),
    "blue": np.array([8, 8, 8], dtype=np.float32),
    "yellow": np.array([15, 12, 12], dtype=np.float32),
    "green": np.array([10, 10, 10], dtype=np.float32),
    "black": np.array([12, 10, 10], dtype=np.float32),
    "red": np.array([15, 12, 12], dtype=np.float32)
}

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
    return lower, upper

def color_blocks_position_WL(img, color, size_code):
    if img is None:
        print("无画面")
        return None
    if color not in LAB_CENTER:
        print("未知颜色:", color)
        return None

    gs_img = cv2.GaussianBlur(img, (5, 5), 0)
    img_float = gs_img.astype(np.float32) / 255.0
    lab_img = cv2.cvtColor(img_float, cv2.COLOR_BGR2LAB)
    lower, upper = get_lab_range(color)
    mask = cv2.inRange(lab_img, lower, upper)
    

    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=2)
    mask = cv2.dilate(mask, kernel, iterations=2)

    dis.show(image.cv2image(mask, copy=True))

    cnts = cv2.findContours(
        mask.copy(),
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )[-2]

    if len(cnts) == 0:
        return None

    c = max(cnts, key=cv2.contourArea)
    area = cv2.contourArea(c)

    if area <= size_code:
        return None

    rect = cv2.minAreaRect(c)
    box = cv2.boxPoints(rect).astype(np.int32)
    cv2.drawContours(img, [box], -1, (0, 255, 255), 2)

    x, y, w, h = cv2.boundingRect(c)
    padding = 10

    x1 = max(0, x - padding)
    y1 = max(0, y - padding)
    x2 = min(mask.shape[1], x + w + padding)
    y2 = min(mask.shape[0], y + h + padding)

    roi = mask[y1:y2, x1:x2]

    if roi.size == 0:
        return None

    roi_blur = cv2.GaussianBlur(roi, (5, 5), 1)

    min_side = min(w, h)
    max_side = max(w, h)

    min_radius = max(3, int(min_side * 0.25))
    max_radius = max(min_radius + 2, int(max_side * 0.7))

    circles = cv2.HoughCircles(
        roi_blur,
        cv2.HOUGH_GRADIENT,
        dp=1.5,
        minDist=max(10, min_side // 2),
        param1=90,
        param2=20,
        minRadius=min_radius,
        maxRadius=max_radius
    )

    if circles is not None:
        circles = np.round(circles[0]).astype(np.int32)
        best_circle = max(circles, key=lambda circle: circle[2])

        roi_center_x = int(best_circle[0])
        roi_center_y = int(best_circle[1])
        radius = int(best_circle[2])

        center_x = roi_center_x + x1
        center_y = roi_center_y + y1

        cv2.circle(
            img,
            (center_x, center_y),
            radius,
            (0, 255, 0),
            2
        )

        cv2.circle(
            img,
            (center_x, center_y),
            4,
            (0, 0, 255),
            -1
        )

        print(
            "霍夫圆:",
            "center =",
            (center_x, center_y),
            "radius =",
            radius
        )

        return center_x, center_y

    center_x, center_y = rect[0]

    cv2.circle(
        img,
        (int(center_x), int(center_y)),
        4,
        (255, 0, 255),
        -1
    )

    print("霍夫圆未检测到，使用矩形中心")

    return int(center_x), int(center_y)

cam = camera.Camera(512, 320)
dis = display.Display()

while not app.need_exit():
    img = cam.read()

    cv_img = image.image2cv(
        img,
        ensure_bgr=True,
        copy=True
    )

    center = color_blocks_position_WL(
        cv_img,
        "green",
        300
    )

    if center:
        print("目标中心:", center)

    #dis.show(image.cv2image(cv_img,bgr=True,copy=True))

