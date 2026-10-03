from maix import camera, display, app, image
import cv2
import numpy as np

# LAB中心值
LAB_CENTER = {
    "light_blue": np.array([68, -9, -24], dtype=np.float32),
    "blue": np.array([34, 10, -37], dtype=np.float32),
    "yellow": np.array([81, 6, 75], dtype=np.float32),
    "green": np.array([74, -74, 67], dtype=np.float32),
    "black": np.array([21, 2, 0], dtype=np.float32),
    "red": np.array([47, 65, 35], dtype=np.float32)
}
# LAB余量
LAB_TOL = {
    "light_blue": np.array([15, 12, 12], dtype=np.float32),
    "blue": np.array([15, 12, 12], dtype=np.float32),
    "yellow": np.array([15, 12, 12], dtype=np.float32),
    "green": np.array([15, 12, 12], dtype=np.float32),
    "black": np.array([12, 10, 10], dtype=np.float32),
    "red": np.array([15, 12, 12], dtype=np.float32)
}

def get_lab_range(color):
    center = LAB_CENTER[color]
    tol = LAB_TOL[color]
    lower = center - tol
    upper = center + tol
    # 限制标准LAB范围
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
    # 1. 高斯滤波
    gs_img = cv2.GaussianBlur(img, (5, 5), 0)
    # 2. 转float并归一化
    img_float = gs_img.astype(np.float32) / 255.0
    # 3. BGR -> LAB
    lab_img = cv2.cvtColor(img_float, cv2.COLOR_BGR2LAB)
    # 4. 得到LAB上下限
    lower, upper = get_lab_range(color)
    # 5. LAB阈值二值化
    mask = cv2.inRange(lab_img, lower, upper)
    # 6. 腐蚀去噪
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=2)
    # 7. 找轮廓
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if len(cnts) == 0:
        return None
    # 8. 找最大轮廓
    c = max(cnts, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area <= size_code:
        return None
    # 9. 最小外接旋转矩形
    rect = cv2.minAreaRect(c)
    box = cv2.boxPoints(rect).astype(np.int32)
    # 10. 画矩形
    cv2.drawContours(img, [box], -1, (0, 255, 255), 2)
    # 11. 获取中心
    center_x, center_y = rect[0]
    # 12. 画中心点
    cv2.circle(img, (int(center_x), int(center_y)), 4, (0, 0, 255), -1)
    return int(center_x), int(center_y)


cam = camera.Camera(512, 320)
dis = display.Display()


while not app.need_exit():
    img = cam.read()
    # Maix图像 -> OpenCV BGR
    cv_img = image.image2cv(img, ensure_bgr=True, copy=True)
    # 改这里选择颜色
    center = color_blocks_position_WL(cv_img, "red", 300)
    if center:
        print("目标中心:", center)
    dis.show(image.cv2image(cv_img, bgr=True, copy=True))

