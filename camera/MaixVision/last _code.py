from maix import camera, display, app, image
import cv2,sys 
import numpy as np

def color_blocks_position_WL(img, color, size_code):
    color_dist = {
        "red": {"Lower1": np.array([156, 60, 60]),"Upper1": np.array([179, 255, 255]),"Lower2": np.array([0, 60, 60]),"Upper2": np.array([6, 255, 255])},
        "yellow": {"Lower": np.array([20, 80, 80]),"Upper": np.array([35, 255, 255])},
        "blue": {"Lower": np.array([100, 120, 50]),"Upper": np.array([124, 255, 200])},
        "green": {"Lower": np.array([38, 80, 45]),"Upper": np.array([90, 255, 255])},
        "black": {"Lower": np.array([0, 0, 0]),"Upper": np.array([179, 255, 50])},
        "light_blue": {"Lower": np.array([90, 40, 160]),"Upper": np.array([120, 200, 255])}
    }
    if img is None: print("无画面");return None
    if color not in color_dist: print("未知颜色:", color);return None
    gs_img = cv2.GaussianBlur(img, (5, 5), 0)
    hsv_img = cv2.cvtColor(gs_img, cv2.COLOR_BGR2HSV)
    if color == "red":
        mask1 = cv2.inRange(hsv_img, color_dist["red"]["Lower1"], color_dist["red"]["Upper1"])
        mask2 = cv2.inRange(hsv_img, color_dist["red"]["Lower2"], color_dist["red"]["Upper2"])
        mask = cv2.bitwise_or(mask1, mask2)
    else:
        mask = cv2.inRange(hsv_img, color_dist[color]["Lower"], color_dist[color]["Upper"])
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=2)
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if len(cnts) == 0: return None
    c = max(cnts, key=cv2.contourArea)
    if int(cv2.contourArea(c)) <= size_code: return None
    rect = cv2.minAreaRect(c)
    box = cv2.boxPoints(rect).astype(np.int32)
    cv2.drawContours(img, [box], -1, (0, 255, 255), 2)
    center_x, center_y = rect[0]
    cv2.circle(img, (int(center_x), int(center_y)), 4, (0, 0, 255), -1)
    return int(center_x), int(center_y)



cam, dis = camera.Camera(512, 320), display.Display()
while not app.need_exit():
    img = cam.read()
    cv_img = image.image2cv(img, ensure_bgr=True, copy=True)
    center = color_blocks_position_WL(cv_img, "red", 300)
    if center: print("目标中心:", center)
    dis.show(image.cv2image(cv_img, bgr=True, copy=True))

#----------------------------------------------------------------------向下是色环识别

def color_circle_position (img):     
    """
    色环识别辅助实现定位
    :param img: 输入的图像
    :return: 三个色环的中心坐标 x1,y1,x2,y2,x3,y3
    """
    
    erode_hsv = cv2.erode(img, None, iterations=2)  # 腐蚀 粗的变细
    kernel = np.ones((7, 7), np.uint8)#5,5
    diRange_hsv = cv2.dilate(erode_hsv, kernel, 1) # 膨胀 填补空洞
    gray_img = cv2.cvtColor(diRange_hsv, cv2.COLOR_BGR2GRAY) # 转化为单通道灰度图
    
    # 限制对比度自适应直方图均衡，非常好的增强对光线鲁棒性的方法，但是阈值过大容易出现噪点
    clahe = cv2.createCLAHE(clipLimit=5.0, tileGridSize=(8, 8))#5.0 
    clahed = clahe.apply(gray_img) # 对灰度图做 CLAHE 均衡
    
    # 计算形态学梯度（增强物体边缘）
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    gradient = cv2.morphologyEx(gray_img, cv2.MORPH_GRADIENT, kernel)
    
    result = cv2.GaussianBlur(gradient, (7, 7), 3, 3) # 高斯模糊 平滑边缘
    
    eqal_img = cv2.convertScaleAbs(result, alpha=4, beta=0) # 再整体增强对比度
    cv2.imshow("video2", eqal_img)
    eqal_img = cv2.GaussianBlur(eqal_img, (7, 7), 3, 3) # 再一次高斯模糊 平滑边缘

    retval, threshold_img = cv2.threshold(eqal_img, 70, 255, cv2.THRESH_BINARY) # 二值化

    threshold_img = cv2.GaussianBlur(threshold_img, (9, 9), 3, 3) # 最后再来一次高斯模糊 平滑边缘

    # canny_img = cv2.Canny(gradient,120,200)

    # diRange_img = cv2.dilate(canny_img, kernel, 1)

    # 霍夫圆检测
    circles = cv2.HoughCircles(threshold_img, cv2.HOUGH_GRADIENT_ALT, 1.5, 50, param1=100, param2=0.95, minRadius=15,
                               maxRadius=50)
    cv2.imshow("video", gray_img)

    cv2.imshow("video3", threshold_img)

    try:
        if(len(circles[0])==3):
            circles = np.uint16(np.around(circles))
            # 遍历
            for circle in circles[0, :]:
                cv2.circle(img, (circle[0], circle[1]), circle[2], (0, 0, 255), 2)
                cv2.circle(img, (circle[0], circle[1]), 2, (255, 0, 0), 2)          
            circle_all=[circles[0][0],circles[0][1],circles[0][2]]
            circle_list =  sorted(circle_all, key=lambda x: x[0])
            return (circle_list[0][0],circle_list[0][1],circle_list[1][0],circle_list[1][1],circle_list[2][0],circle_list[2][1]) # 依次返回色环的中心坐标
    except:
        pass

