from maix import camera, display, app, image, uart
import cv2
import numpy as np


def color_circle_position(img):
    """
    色环识别辅助实现定位
    :param img: 输入的图像（OpenCV BGR格式）
    :return: 三个色环的中心坐标 x1,y1,x2,y2,x3,y3；失败返回 None
    """
    erode_hsv= cv2.erode(img, None, iterations=2)  # 腐蚀 粗的变细
    kernel = np.ones((7, 7), np.uint8)
    diRange_hsv = cv2.dilate(erode_hsv, kernel, 1)  # 膨胀 填补空洞
    gray_img = cv2.cvtColor(diRange_hsv, cv2.COLOR_BGR2GRAY)  # 转化为单通道灰度图

    # 限制对比度自适应直方图均衡，增强对光线鲁棒性
    clahe = cv2.createCLAHE(clipLimit=4.0, tileGridSize=(8, 8))
    clahed = clahe.apply(gray_img)  # 对灰度图做 CLAHE 均衡

    # 计算形态学梯度（增强物体边缘）
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    gradient = cv2.morphologyEx(clahed, cv2.MORPH_GRADIENT, kernel)  # 注意：这里用 clahed 而不是 gray_img


    result = cv2.GaussianBlur(gradient, (3, 3),1)  # 高斯模糊 平滑边缘

    eqal_img = cv2.convertScaleAbs(result, alpha=2, beta=0)  # 再整体增强对比度

    eqal_img = cv2.GaussianBlur(eqal_img, (7, 7), 3, 3)  # 再一次高斯模糊 平滑边缘

    retval, threshold_img = cv2.threshold(eqal_img, 170, 255, cv2.THRESH_BINARY)  # 二值化



    threshold_img = cv2.GaussianBlur(threshold_img, (9, 9), 3, 3)  # 最后再来一次高斯模糊 平滑边缘

    # 霍夫圆检测
    circles = cv2.HoughCircles(threshold_img, cv2.HOUGH_GRADIENT_ALT, 1.5, 50,
                               param1=100, param2=0.85, minRadius=15,
                               maxRadius=30)

    try:
        if len(circles[0]) == 3:
            circles = np.uint16(np.around(circles))
            # 遍历
            for circle in circles[0, :]:
                cv2.circle(img, (circle[0], circle[1]), circle[2], (0, 0, 255), 2)
                cv2.circle(img, (circle[0], circle[1]), 2, (255, 0, 0), 2)
            circle_all = [circles[0][0], circles[0][1], circles[0][2]]
            circle_list = sorted(circle_all, key=lambda x: x[0])
            return (circle_list[0][0], circle_list[0][1],
                    circle_list[1][0], circle_list[1][1],
                    circle_list[2][0], circle_list[2][1])
    except:
        pass
    return None


# ==================== 主程序 ====================
cam = camera.Camera(512, 320)
disp = display.Display()
# Same UART as color_v4.py: connect TX to STM32 UART4 RX (PA1), common ground.
serial_dev = uart.UART("/dev/ttyS2", 115200)

while not app.need_exit():
    # 1. 从 MaixPy 摄像头读取图像
    img = cam.read()

    # 2. 将 MaixPy 图像转为 OpenCV BGR 格式
    cv_img = image.image2cv(img, ensure_bgr=True, copy=True)

    # 3. 用 OpenCV 处理
    result = color_circle_position(cv_img)

    if result:
        x1, y1, x2, y2, x3, y3 = result
        print(f"色环中心: ({x1},{y1}) ({x2},{y2}) ({x3},{y3})")
        # Send only complete three-ring detections; indices are spatial, not colors.
        serial_dev.write(f"RINGS,{int(x1)},{int(y1)},{int(x2)},{int(y2)},{int(x3)},{int(y3)}\n".encode())

    # 4. 将 OpenCV 处理后的图像转回 MaixPy 格式并显示
    img_show = image.cv2image(cv_img, bgr=True, copy=True)
    disp.show(img_show)
