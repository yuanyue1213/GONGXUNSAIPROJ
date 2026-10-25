# -*- coding: utf-8 -*-
"""MaixCAM2 主程序：保留原 LAB 触屏标定 + 独立圆台识别模块。

两份文件 main.py、turntable_detector.py 放在同一目录运行。
首次启动：没有三色标定记录时自动进入三色标定；成功后保存至 ~/lab_thresholds.json。
再次启动：自动读取文件，不再标定；运行期间长按屏幕 1.5 秒可手动重新标定六色。
"""

from maix import camera, display, app, image, touchscreen, time, uart, gpio, pinmap, err
import cv2
import numpy as np

from turntable_detector import (
    DEFAULT_CONFIG_PATH,
    TurntableCenterDetector,
    load_lab_thresholds,
    save_lab_thresholds,
)

# -------------------------- 可修改参数 --------------------------
WIDTH, HEIGHT = 512, 320
TARGET_COLORS = ("red", "yellow", "blue")  # 改成圆台上实际的三个颜色
COLOR_LIST = ["light_blue", "blue", "yellow", "green", "black", "red"]

# 只在首次缺失目标颜色标定时，自动标定缺少的目标颜色。
AUTO_CALIBRATE_ON_FIRST_RUN = True
LONG_PRESS_MS = 1500
DOUBLE_CLICK_MS = 500
TOL_STEP = 1
SAMPLE_HALF = 6
SAMPLE_BOX_HALF = 10

# 识别参数
STABLE_COUNT = 5
STABLE_ERR = 1.0         # 相对于最近五帧中位中心的欧氏距离（像素）
MIN_AREA = 80
TRIANGLE_TOL = 0.25
DEBUG_DRAW = True

# 与原代码相同的串口和补光灯配置
ENABLE_UART = True
UART_DEVICE = "/dev/ttyS2"   # 原始代码使用此设备，实际用 UART3 时请确认端口
UART_BAUD = 115200
LIGHT_ON = False              # 原代码 0 = 关灯, 1 = 开灯

# 与原代码相同的触摸按钮布局
SKIP_RECT = (390, 5, 510, 55)
L_PLUS_RECT = (10, 50, 246, 105)
L_MINUS_RECT = (266, 50, 502, 105)
A_PLUS_RECT = (10, 115, 246, 170)
A_MINUS_RECT = (266, 115, 502, 170)
B_PLUS_RECT = (10, 180, 246, 235)
B_MINUS_RECT = (266, 180, 502, 235)
NEXT_RECT = (150, 250, 362, 315)

# -------------------------- 加载一次已标定的阈值 --------------------------
# 这两份字典在主程序和识别器之间共享（识别器保存它们的引用）。
LAB_CENTER, LAB_TOL, CALIBRATED_COLORS = load_lab_thresholds(DEFAULT_CONFIG_PATH)

cam = camera.Camera(WIDTH, HEIGHT)
dis = display.Display()
ts = touchscreen.TouchScreen()

serial_dev = None
if ENABLE_UART:
    try:
        serial_dev = uart.UART(UART_DEVICE, UART_BAUD)
        print("串口已初始化：", UART_DEVICE)
    except Exception as exc:
        print("警告：串口打开失败，仅在屏幕/终端输出：", exc)

detector = TurntableCenterDetector(
    LAB_CENTER, LAB_TOL, colors=TARGET_COLORS,
    min_area=MIN_AREA, triangle_tol=TRIANGLE_TOL,
    stable_count=STABLE_COUNT, stable_err=STABLE_ERR,
)


# -------------------------- 通用触摸/显示功能（保留原逻辑） --------------------------
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


# -------------------------- 原 LAB 分割方式 --------------------------
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


# -------------------------- 兼容原有单色色块识别函数 --------------------------
def color_blocks_position_WL(bgr_image, color, size_code=300):
    """保留原程序的单色霍夫圆定位；圆台检测不调用此函数。"""
    if bgr_image is None or color not in LAB_CENTER:
        return None
    mask = make_mask(bgr_image, color)
    contours = cv2.findContours(
        mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )[-2]
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(contour) <= size_code:
        return None

    rect = cv2.minAreaRect(contour)
    box = cv2.boxPoints(rect).astype(np.int32)
    cv2.drawContours(bgr_image, [box], 0, (0, 255, 255), 1)
    x, y, w, h = cv2.boundingRect(contour)
    roi = mask[y:y + h, x:x + w]
    roi_blur = cv2.GaussianBlur(roi, (5, 5), 1)

    min_side, max_side = min(w, h), max(w, h)
    min_radius = max(3, int(min_side * 0.25))
    max_radius = max(min_radius + 2, int(max_side * 0.7))
    circles = cv2.HoughCircles(
        roi_blur, cv2.HOUGH_GRADIENT,
        dp=1.5, minDist=max(10, min_side // 2),
        param1=90, param2=20,
        minRadius=min_radius, maxRadius=max_radius
    )
    if circles is not None:
        c = max(np.round(circles[0]).astype(np.int32), key=lambda v: v[2])
        cx, cy, radius = int(c[0] + x), int(c[1] + y), int(c[2])
        cv2.circle(bgr_image, (cx, cy), radius, (0, 255, 0), 2)
    else:
        cx, cy = (int(round(v)) for v in rect[0])
    cv2.circle(bgr_image, (cx, cy), 5, (0, 0, 255), -1)
    return cx, cy


# -------------------------- 原触屏 LAB 标定流程 --------------------------
def sample_lab_center(color):
    """双击进入确认模式，在画面中心对颜色取样；SKIP 不覆盖旧值。"""
    last_click_time = 0
    click_count = 0
    pressed_last = False
    sample_ready = False

    while not app.need_exit():
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
                LAB_CENTER, LAB_TOL, CALIBRATED_COLORS, DEFAULT_CONFIG_PATH):
            # 保存失败就恢复此颜色之前的状态，避免误认为下次能复用。
            LAB_CENTER[color] = old_center
            LAB_TOL[color] = old_tol
            if not old_was_calibrated:
                CALIBRATED_COLORS.discard(color)
            print("该颜色标定未保存，请检查设备磁盘权限：", color)
    detector.reset()
    print("标定过程结束，已完成：", sorted(CALIBRATED_COLORS))


def calibrate_all_colors():
    """保留原有的六色标定入口；对不需要的颜色仍可以点击 SKIP。"""
    calibrate_colors(COLOR_LIST)


def target_colors_ready():
    return all(color in CALIBRATED_COLORS for color in TARGET_COLORS)


# -------------------------- 主循环 --------------------------
def main():
    press_start = 0
    long_press_triggered = False

    # 沿用原来的 B25 补光灯引脚。
    try:
        err.check_raise(pinmap.set_pin_function("B25", "GPIOB25"),
                        "set light pin failed")
        light = gpio.GPIO("GPIOB25", gpio.Mode.OUT)
        light.value(1 if LIGHT_ON else 0)
    except Exception as exc:
        print("警告：补光灯 GPIO 初始化失败：", exc)

    # 第一次只要求圆台三色完成标定；其他颜色仍可之后长按标定。
    if AUTO_CALIBRATE_ON_FIRST_RUN and not target_colors_ready():
        missing = [c for c in TARGET_COLORS if c not in CALIBRATED_COLORS]
        print("首次运行，缺少实际标定的颜色：", missing)
        calibrate_colors(missing)

    while not app.need_exit():
        img = cam.read()
        cv_img = image.image2cv(img, ensure_bgr=True, copy=True)

        if target_colors_ready():
            # 每帧识别三个移动色块，只稳定判断最终圆台中心。
            stable_center = detector.detect(cv_img, draw=DEBUG_DRAW)
            if stable_center is not None:
                center_x, center_y = stable_center
                msg = f"x={center_x:03d},y={center_y:03d}\n"
                print("最终输出圆台中心：", msg.strip())
                if serial_dev is not None:
                    serial_dev.write_str(msg)
                cv2.putText(cv_img, f"Origin: ({center_x},{center_y})",
                            (10, 24), cv2.FONT_HERSHEY_SIMPLEX,
                            0.65, (0, 0, 255), 2)
            elif DEBUG_DRAW:
                state = "{} {}/{}".format(
                    detector.last_state, len(detector.history), STABLE_COUNT
                )
                cv2.putText(cv_img, state, (8, 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                            (0, 255, 255), 1)
        else:
            detector.reset()
            cv2.putText(cv_img, "NEED LAB CALIBRATION", (10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)

        cv2.putText(cv_img, "LONG PRESS: CALIBRATE", (110, 300),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        show(cv_img)

        _, _, pressed = read_touch()
        if pressed:
            if press_start == 0:
                press_start = time.ticks_ms()
            else:
                now = time.ticks_ms()
                if now - press_start >= LONG_PRESS_MS and not long_press_triggered:
                    long_press_triggered = True
                    print("长按屏幕，进入六色标定")
                    wait_release()
                    calibrate_all_colors()
                    press_start = 0
                    long_press_triggered = False
        else:
            press_start = 0
            long_press_triggered = False


if __name__ == "__main__":
    main()
