from maix import camera, display, app, image, touchscreen, time
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
COLOR_LIST = ["light_blue", "blue", "yellow", "green", "black", "red"]

LONG_PRESS_MS = 1500
DOUBLE_CLICK_MS = 500
TOL_STEP = 1
SAMPLE_HALF = 6
SAMPLE_BOX_HALF = 10
SKIP_RECT = (390, 5, 510, 55)
L_PLUS_RECT = (10, 50, 246, 105)
L_MINUS_RECT = (266, 50, 502, 105)
A_PLUS_RECT = (10, 115, 246, 170)
A_MINUS_RECT = (266, 115, 502, 170)
B_PLUS_RECT = (10, 180, 246, 235)
B_MINUS_RECT = (266, 180, 502, 235)
NEXT_RECT = (150, 250, 362, 315)

cam = camera.Camera(512, 320)
dis = display.Display()
ts = touchscreen.TouchScreen()

def read_touch():
    x, y, pressed = ts.read()
    if not pressed:
        return 0, 0, False
    x, y = image.resize_map_pos_reverse(512, 320, dis.width(), dis.height(), image.Fit.FIT_CONTAIN, x, y)
    return int(x), int(y), True

def point_in_rect(x, y, rect):
    x1, y1, x2, y2 = rect
    return x1 <= x <= x2 and y1 <= y <= y2

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

def make_mask(img, color):
    gs_img = cv2.GaussianBlur(img, (5, 5), 0)
    img_float = gs_img.astype(np.float32) / 255.0
    lab_img = cv2.cvtColor(img_float, cv2.COLOR_BGR2LAB)
    lower, upper = get_lab_range(color)
    mask = cv2.inRange(lab_img, lower, upper)
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=2)
    mask = cv2.dilate(mask, kernel, iterations=2)
    return mask

def color_blocks_position_WL(img, color, size_code):
    if img is None:
        print("无画面")
        return None
    if color not in LAB_CENTER:
        print("未知颜色:", color)
        return None
    mask = make_mask(img, color)
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
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
    circles = cv2.HoughCircles(roi_blur, cv2.HOUGH_GRADIENT, dp=1.5, minDist=max(10, min_side // 2), param1=90, param2=20, minRadius=min_radius, maxRadius=max_radius)
    if circles is not None:
        circles = np.round(circles[0]).astype(np.int32)
        best_circle = max(circles, key=lambda circle: circle[2])
        roi_center_x = int(best_circle[0])
        roi_center_y = int(best_circle[1])
        radius = int(best_circle[2])
        center_x = roi_center_x + x1
        center_y = roi_center_y + y1
        cv2.circle(img, (center_x, center_y), radius, (0, 255, 0), 2)
        cv2.circle(img, (center_x, center_y), 5, (0, 0, 255), -1)
        print("霍夫圆: center =", (center_x, center_y), "radius =", radius)
        return center_x, center_y
    center_x, center_y = rect[0]
    cv2.circle(img, (int(center_x), int(center_y)), 5, (255, 0, 255), -1)
    print("霍夫圆未检测到，使用矩形中心")
    return int(center_x), int(center_y)

def wait_release():
    while not app.need_exit():
        _, _, pressed = read_touch()
        if not pressed:
            return

def draw_button(img, rect, text, color=(0, 255, 0)):
    x1, y1, x2, y2 = rect
    cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
    text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2)[0]
    tx = x1 + (x2 - x1 - text_size[0]) // 2
    ty = y1 + (y2 - y1 + text_size[1]) // 2
    cv2.putText(img, text, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)

def sample_lab_center(color):
    last_click_time = 0
    click_count = 0
    pressed_last = False
    sample_ready = False
    while not app.need_exit():
        img = cam.read()
        raw_cv_img = image.image2cv(img, ensure_bgr=True, copy=True)
        cv_img = raw_cv_img.copy()
        h, w = cv_img.shape[:2]
        cx = w // 2
        cy = h // 2
        cv2.rectangle(cv_img, (0, 0), (300, 40), (0, 0, 0), -1)
        cv2.putText(cv_img, "COLOR: " + color, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
        draw_button(cv_img, SKIP_RECT, "SKIP", (0, 0, 255))
        cv2.drawMarker(cv_img, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 14, 2)
        cv2.circle(cv_img, (cx, cy), 2, (0, 0, 255), -1)
        if not sample_ready:
            cv2.putText(cv_img, "DOUBLE CLICK", (170, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
        else:
            cv2.rectangle(cv_img, (cx - SAMPLE_BOX_HALF, cy - SAMPLE_BOX_HALF), (cx + SAMPLE_BOX_HALF, cy + SAMPLE_BOX_HALF), (0, 255, 0), 2)
            cv2.putText(cv_img, "PRESS TO CONFIRM", (145, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        dis.show(image.cv2image(cv_img, bgr=True, copy=True))
        x, y, pressed = read_touch()
        if pressed and not pressed_last:
            print("touch:", x, y)
            if point_in_rect(x, y, SKIP_RECT):
                print("跳过颜色:", color)
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
                    print(color, "中心采样区域已确定")
            else:
                roi = raw_cv_img[cy - SAMPLE_HALF: cy + SAMPLE_HALF, cx - SAMPLE_HALF: cx + SAMPLE_HALF]
                roi_float = roi.astype(np.float32) / 255.0
                roi_lab = cv2.cvtColor(roi_float, cv2.COLOR_BGR2LAB)
                LAB_CENTER[color] = np.mean(roi_lab, axis=(0, 1)).astype(np.float32)
                print(color, "LAB_CENTER =", LAB_CENTER[color])
                wait_release()
                return True
        pressed_last = pressed

def draw_adjust_ui(show_img, color):
    h, w = show_img.shape[:2]
    cv2.rectangle(show_img, (0, 0), (w - 1, h - 1), (0, 255, 0), 2)
    cv2.rectangle(show_img, (0, 0), (w - 1, 40), (0, 0, 0), -1)
    cv2.putText(show_img, "COLOR: " + color, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
    draw_button(show_img, L_PLUS_RECT, "L+ %d" % int(LAB_TOL[color][0]))
    draw_button(show_img, L_MINUS_RECT, "L-")
    draw_button(show_img, A_PLUS_RECT, "A+ %d" % int(LAB_TOL[color][1]))
    draw_button(show_img, A_MINUS_RECT, "A-")
    draw_button(show_img, B_PLUS_RECT, "B+ %d" % int(LAB_TOL[color][2]))
    draw_button(show_img, B_MINUS_RECT, "B-")
    draw_button(show_img, NEXT_RECT, "NEXT", (0, 255, 255))

def adjust_lab_tol(color):
    pressed_last = False
    while not app.need_exit():
        img = cam.read()
        cv_img = image.image2cv(img, ensure_bgr=True, copy=True)
        mask = make_mask(cv_img, color)
        show_img = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        draw_adjust_ui(show_img, color)
        dis.show(image.cv2image(show_img, bgr=True, copy=True))
        x, y, pressed = read_touch()
        if pressed and not pressed_last:
            print("touch:", x, y)
            if point_in_rect(x, y, NEXT_RECT):
                print(color, "最终 LAB_CENTER =", LAB_CENTER[color])
                print(color, "最终 LAB_TOL =", LAB_TOL[color])
                wait_release()
                return
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
            LAB_TOL[color][0] = max(1, LAB_TOL[color][0])
            LAB_TOL[color][1] = max(1, LAB_TOL[color][1])
            LAB_TOL[color][2] = max(1, LAB_TOL[color][2])
            print(color, "LAB_TOL =", LAB_TOL[color])
        pressed_last = pressed

def calibrate_all_colors():
    print("开始六种颜色标定")
    for color in COLOR_LIST:
        print("====================")
        print("当前颜色:", color)
        print("====================")
        result = sample_lab_center(color)
        if result is False:
            continue
        adjust_lab_tol(color)
    print("六种颜色全部标定完成")
    print("LAB_CENTER:")
    for color in COLOR_LIST:
        print(color, LAB_CENTER[color])
    print("LAB_TOL:")
    for color in COLOR_LIST:
        print(color, LAB_TOL[color])


#----------------------------------------------------------------主函数
press_start = 0
long_press_triggered = False

while not app.need_exit():
    img = cam.read()
    cv_img = image.image2cv(img, ensure_bgr=True, copy=True)
    center = color_blocks_position_WL(cv_img, "green", 300)
    if center:
        center_x, center_y = center
        print("返回目标圆心:", center_x, center_y)
    cv2.putText(cv_img, "LONG PRESS: CALIBRATE", (110, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
    dis.show(image.cv2image(cv_img, bgr=True, copy=True))
    _, _, pressed = read_touch()
    if pressed:
        if press_start == 0:
            press_start = time.ticks_ms()
        else:
            now = time.ticks_ms()
            if now - press_start >= LONG_PRESS_MS and not long_press_triggered:
                long_press_triggered = True
                print("检测到长按，进入六色标定")
                wait_release()
                calibrate_all_colors()
                press_start = 0
                long_press_triggered = False
    else:
        press_start = 0
        long_press_triggered = False
