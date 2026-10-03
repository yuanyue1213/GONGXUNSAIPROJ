
from maix import camera, display, app, image
import cv2
import numpy as np

A4_SHORT_MM = 210.0
A4_LONG_MM = 297.0

def order_points(pts):
    """将四个角点排序为：左上、右上、右下、左下"""
    pts = np.array(pts, dtype=np.float32)
    s = pts.sum(axis=1)
    diff = np.diff(pts, axis=1).reshape(-1)
    top_left = pts[np.argmin(s)]
    bottom_right = pts[np.argmax(s)]
    top_right = pts[np.argmin(diff)]
    bottom_left = pts[np.argmax(diff)]
    return np.array([top_left, top_right, bottom_right, bottom_left], dtype=np.float32)

def detect_a4_and_scale(cv_img):
    """识别A4纸，并计算长短边像素长度以及实际尺寸比例"""

    gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # A4为白色，背景较暗，因此白纸保留为白色
    _, binary = cv2.threshold(blur, 185, 255, cv2.THRESH_BINARY)

    # 显示二值化结果
    img_show = image.cv2image(binary, copy=True)
    disp.show(img_show)

    contours = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    best_quad = None
    best_area = 0

    for cnt in contours:
        area = cv2.contourArea(cnt)

        # A4通常占画面比较大，因此过滤较小轮廓
        if area < 5000:
            continue

        perimeter = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.02 * perimeter, True)

        if len(approx) != 4:
            continue

        if not cv2.isContourConvex(approx):
            continue

        if area > best_area:
            best_area = area
            best_quad = approx

    if best_quad is None:
        return None

    pts = best_quad.reshape(4, 2)
    ordered = order_points(pts)

    tl = ordered[0]
    tr = ordered[1]
    br = ordered[2]
    bl = ordered[3]

    top = np.linalg.norm(tr - tl)
    right = np.linalg.norm(br - tr)
    bottom = np.linalg.norm(br - bl)
    left = np.linalg.norm(bl - tl)

    # 水平方向两条边平均
    horizontal = (top + bottom) / 2.0

    # 垂直方向两条边平均
    vertical = (left + right) / 2.0

    # 自动判断哪一组是长边、哪一组是短边
    if horizontal > vertical:
        long_px = horizontal
        short_px = vertical
    else:
        long_px = vertical
        short_px = horizontal

    # 计算两个方向的比例
    scale_short = A4_SHORT_MM / short_px
    scale_long = A4_LONG_MM / long_px

    # 两个比例再取平均，作为最终比例
    scale_average = (scale_short + scale_long) / 2.0

    center = np.mean(ordered, axis=0)
    cx = int(center[0])
    cy = int(center[1])

    ordered_int = ordered.astype(np.int32)

    cv2.polylines(cv_img, [ordered_int], True, (0, 255, 0), 2)

    for point in ordered_int:
        cv2.circle(cv_img, tuple(point), 4, (0, 0, 255), -1)

    cv2.circle(cv_img, (cx, cy), 5, (255, 0, 0), -1)

    cv2.putText(cv_img, f"Top: {top:.1f}px", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 2)
    cv2.putText(cv_img, f"Right: {right:.1f}px", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 2)
    cv2.putText(cv_img, f"Bottom: {bottom:.1f}px", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 2)
    cv2.putText(cv_img, f"Left: {left:.1f}px", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 2)

    cv2.putText(cv_img, f"Short: {short_px:.2f}px", (10, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
    cv2.putText(cv_img, f"Long: {long_px:.2f}px", (10, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    cv2.putText(cv_img, f"S scale: {scale_short:.4f}", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 2)
    cv2.putText(cv_img, f"L scale: {scale_long:.4f}", (10, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 2)
    cv2.putText(cv_img, f"Avg: {scale_average:.4f}mm/px", (10, 195), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 2)

    return {
        "top": top,
        "right": right,
        "bottom": bottom,
        "left": left,
        "short_px": short_px,
        "long_px": long_px,
        "scale_short": scale_short,
        "scale_long": scale_long,
        "scale_average": scale_average,
        "center": (cx, cy)
    }

cam = camera.Camera(512, 320)
disp = display.Display()

while not app.need_exit():
    img = cam.read()
    cv_img = image.image2cv(img, ensure_bgr=True, copy=True)

    result = detect_a4_and_scale(cv_img)

    if result is not None:
        print("==============================")
        print(f"上边长度       : {result['top']:.2f} px")
        print(f"右边长度       : {result['right']:.2f} px")
        print(f"下边长度       : {result['bottom']:.2f} px")
        print(f"左边长度       : {result['left']:.2f} px")
        print(f"短边平均像素   : {result['short_px']:.2f} px")
        print(f"长边平均像素   : {result['long_px']:.2f} px")
        print(f"短边比例       : {result['scale_short']:.6f} mm/px")
        print(f"长边比例       : {result['scale_long']:.6f} mm/px")
        print(f"最终平均比例   : {result['scale_average']:.6f} mm/px")
        print(f"A4中心         : {result['center']}")
