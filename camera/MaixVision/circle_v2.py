"""黑色同心环定位；串口仍输出 RINGS,x1,y1,x2,y2,x3,y3\n。

坐标是完整画面坐标，三个靶标从左到右排序，不识别中心数字。
倾斜较大时需先做平面透视校正；这里输出的是图像拟合中心。
"""
from itertools import combinations
import cv2
import numpy as np

FRAME_WIDTH, FRAME_HEIGHT = 512, 320
# 按画面比例设置 ROI；默认全图，现场固定安装后可缩小范围。
ROI = (0.0, 0.0, 1.0, 1.0)
MIN_DIAMETER = 20
MAX_DIAMETER = 150
MIN_AXIS_RATIO = 0.60
MAX_FIT_ERROR = 0.09
MIN_RING_LEVELS = 3
MIN_RADIUS_SPAN = 1.20
STABLE_FRAMES = 3
STABLE_ERROR_PX = 4.0


def refine_ring_edge(gray, ellipse, contour):
    """在灰度原图沿椭圆法线插值边缘，再用浮点边缘点拟合。"""
    (cx,cy),(a,b),angle=ellipse
    points=contour.reshape(-1,2).astype(np.float32)
    points=points[::max(1,int(np.ceil(len(points)/96)))]
    co,si=np.cos(np.deg2rad(angle)),np.sin(np.deg2rad(angle))
    delta=points-np.array([cx,cy])
    u=delta[:,0]*co+delta[:,1]*si
    v=-delta[:,0]*si+delta[:,1]*co
    nx,ny=u/(a*a),v/(b*b)
    normals=np.column_stack((nx*co-ny*si,nx*si+ny*co))
    normals/=np.maximum(np.linalg.norm(normals,axis=1)[:,None],1e-6)
    # 环线较密，只搜索邻近边缘，避免跳到相邻环线。
    offsets=np.arange(-2.0,2.01,0.5,dtype=np.float32)
    locations=points[:,None,:]+normals[:,None,:]*offsets[None,:,None]
    valid=(locations[:,:,0].min(axis=1)>=0)&(locations[:,:,1].min(axis=1)>=0)
    valid&=(locations[:,:,0].max(axis=1)<gray.shape[1]-1)&(locations[:,:,1].max(axis=1)<gray.shape[0]-1)
    samples=cv2.remap(gray,locations[:,:,0].astype(np.float32),
                     locations[:,:,1].astype(np.float32),cv2.INTER_LINEAR).astype(np.float32)
    gradients=np.abs(samples[:,2:]-samples[:,:-2])
    peak=np.argmax(gradients,axis=1)
    valid&=(peak>0)&(peak<gradients.shape[1]-1)
    rows=np.arange(len(points)); safe=np.clip(peak,1,gradients.shape[1]-2)
    left=gradients[rows,safe-1]; middle=gradients[rows,safe]; right=gradients[rows,safe+1]
    denominator=left-2*middle+right
    valid&=(middle>=8)&(denominator<-1e-3)
    correction=np.clip(0.5*(left-right)/np.minimum(denominator,-1e-3),-0.5,0.5)
    shift=offsets[safe+1]+0.5*correction
    refined=(points+normals*shift[:,None])[valid].astype(np.float32)
    if len(refined)<12:
        return ellipse
    # 保留原来的完整环线覆盖要求。
    bins=np.floor((np.arctan2(v[valid]/b,u[valid]/a)+np.pi)*16/(2*np.pi)).astype(int)%16
    if len(np.unique(bins))<14:
        return ellipse
    fitted=cv2.fitEllipse(refined.reshape(-1,1,2))
    (fx,fy),(fa,fb),_=fitted
    if np.hypot(fx-cx,fy-cy)>1.5 or not 0.9<fa/a<1.1 or not 0.9<fb/b<1.1:
        return ellipse
    return fitted


def ellipse_candidate(contour):
    """只接受覆盖完整椭圆且拟合误差较小的轮廓，过滤数字/胶带。"""
    if len(contour) < 12:
        return None
    ellipse = cv2.fitEllipse(contour)
    (cx, cy), (a, b), angle = ellipse
    minor, major = min(a, b), max(a, b)
    if not MIN_DIAMETER <= minor <= major <= MAX_DIAMETER:
        return None
    if minor / major < MIN_AXIS_RATIO:
        return None
    area = abs(cv2.contourArea(contour))
    fill = area / (np.pi * a * b / 4.0)
    if not 0.78 <= fill <= 1.18:
        return None
    points = contour.reshape(-1, 2).astype(np.float32)
    delta = points - np.array([cx, cy], dtype=np.float32)
    theta = np.deg2rad(angle)
    u = (delta[:, 0] * np.cos(theta) + delta[:, 1] * np.sin(theta)) / (a / 2)
    v = (-delta[:, 0] * np.sin(theta) + delta[:, 1] * np.cos(theta)) / (b / 2)
    error = float(np.mean(np.abs(np.sqrt(u * u + v * v) - 1)))
    if error > MAX_FIT_ERROR:
        return None
    # 至少覆盖 16 个角度区间中的 14 个，拒绝局部圆弧拟合出的假椭圆。
    bins = np.floor((np.arctan2(v, u) + np.pi) * 16 / (2 * np.pi)).astype(int) % 16
    if len(np.unique(bins)) < 14:
        return None
    return {"center": np.array([cx, cy]), "radius": (a + b) / 4,
            "ellipse": ellipse, "error": error, "ratio": minor / major,
            "contour":contour}


def group_targets(candidates):
    groups = []
    for candidate in sorted(candidates, key=lambda item: item["error"]):
        matched = None
        for group in groups:
            anchor = group[0]
            tolerance = max(3.0, min(anchor["radius"], candidate["radius"]) * 0.12)
            if (np.linalg.norm(candidate["center"] - anchor["center"]) <= tolerance
                    and abs(candidate["ratio"] - anchor["ratio"]) <= 0.15):
                matched = group
                break
        if matched is None:
            groups.append([candidate])
        else:
            matched.append(candidate)
    targets = []
    for group in groups:
        # 内外边界距离很近，不将同一条粗线的两侧算作两层环。
        levels = []
        for candidate in sorted(group, key=lambda item: item["radius"]):
            if not levels or candidate["radius"] - levels[-1]["radius"] >= max(2.0, candidate["radius"] * 0.06):
                levels.append(candidate)
            elif candidate["error"] < levels[-1]["error"]:
                levels[-1] = candidate
        if len(levels) < MIN_RING_LEVELS:
            continue
        if levels[-1]["radius"] / levels[0]["radius"] < MIN_RADIUS_SPAN:
            continue
        weights = np.array([1 / (0.02 + item["error"]) for item in levels])
        center = np.average([item["center"] for item in levels], axis=0, weights=weights)
        targets.append({"center": center, "radius": levels[-1]["radius"],
                        "levels": levels, "score": len(levels)})
    return targets


def select_three(targets):
    best, best_score = None, -float("inf")
    # 限制候选数量，避免复杂背景下组合数过大。
    targets = sorted(targets, key=lambda item: item["score"], reverse=True)[:10]
    for triple in combinations(targets, 3):
        triple = sorted(triple, key=lambda item: item["center"][0])
        centers = np.array([item["center"] for item in triple])
        radii = np.array([item["radius"] for item in triple])
        if radii.max() / radii.min() > 1.5:
            continue
        distances = np.linalg.norm(np.diff(centers, axis=0), axis=1)
        if np.any(distances < (radii[:-1] + radii[1:]) * 0.9):
            continue
        if distances.max() / distances.min() > 1.8:
            continue
        baseline = centers[2] - centers[0]
        middle = centers[1] - centers[0]
        line_error = abs(baseline[0] * middle[1] - baseline[1] * middle[0]) / max(np.linalg.norm(baseline), 1)
        if line_error > radii.mean() * 0.45:
            continue
        score = sum(item["score"] for item in triple) - line_error / radii.mean()
        if score > best_score:
            best, best_score = triple, score
    return best


def color_circle_position(img):
    """检测三个靶标并绘制结果；未找到完整三组时返回 None。"""
    if img is None or img.size == 0:
        return None
    height, width = img.shape[:2]
    x0, y0, x1, y1 = [int(value * size) for value, size in zip(ROI, (width, height, width, height))]
    if not (0 <= x0 < x1 <= width and 0 <= y0 < y1 <= height):
        raise ValueError("ROI must be inside the image")
    edge_gray = cv2.cvtColor(img[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(edge_gray, (3, 3), 0)
    masks = [cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)[1],
             cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                   cv2.THRESH_BINARY_INV, 31, 7)]
    # 两种分割独立评估，避免同一轮廓重复计入同心层数。
    best = None
    for mask in masks:
        contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)[-2]
        candidates = []
        for contour in contours:
            candidate = ellipse_candidate(contour)
            if candidate is not None:
                candidates.append(candidate)
        triple = select_three(group_targets(candidates))
        if triple is not None and (best is None or sum(t["score"] for t in triple) > sum(t["score"] for t in best)):
            best = triple
    if best is None:
        return None
    # 完成三组筛选后才细化，避免所有背景轮廓都做亚像素计算。
    for target in best:
        for level in target["levels"]:
            level["ellipse"]=refine_ring_edge(edge_gray,level["ellipse"],level["contour"])
            level["center"]=np.array(level["ellipse"][0])
        weights=np.array([1/(0.02+level["error"]) for level in target["levels"]])
        target["center"]=np.average([level["center"] for level in target["levels"]],axis=0,weights=weights)
    result = []
    for index, target in enumerate(best, 1):
        center = target["center"] + (x0, y0)
        cx, cy = [int(round(value)) for value in center]
        result.extend((cx, cy))
        for level in target["levels"]:
            (ex, ey), axes, angle = level["ellipse"]
            cv2.ellipse(img, ((ex + x0, ey + y0), axes, angle), (0, 255, 0), 1)
        cv2.drawMarker(img, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 12, 1)
        cv2.putText(img, str(index), (cx + 6, cy - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv2.putText(img, "%.2f,%.2f" % tuple(center), (max(0,cx-30),min(height-4,cy+24)),
                    cv2.FONT_HERSHEY_SIMPLEX,0.35,(0,0,255),1)
        # 结果保留拟合小数，标记位置仍使用整数像素。
        result[-2:] = [float(center[0]),float(center[1])]
    return tuple(result)


class StableCenters:
    """连续稳定后逐帧输出滑动均值；漏检或跳变立即停止输出。"""
    def __init__(self):
        self.samples = []

    def update(self, result):
        if result is None:
            self.samples = []
            return None
        current = np.array(result, dtype=np.float32).reshape(3, 2)
        if self.samples:
            average = np.mean(self.samples, axis=0)
            if np.max(np.linalg.norm(current - average, axis=1)) > STABLE_ERROR_PX:
                self.samples = []
        self.samples.append(current)
        self.samples = self.samples[-STABLE_FRAMES:]
        if len(self.samples) < STABLE_FRAMES:
            return None
        return tuple(float(value) for value in np.mean(self.samples, axis=0).reshape(-1))


def main():
    from maix import camera, display, app, image, uart
    cam = camera.Camera(FRAME_WIDTH, FRAME_HEIGHT)
    disp = display.Display()
    serial_dev = uart.UART("/dev/ttyS2", 115200)
    stable = StableCenters()
    while not app.need_exit():
        frame = cam.read()
        cv_img = image.image2cv(frame, ensure_bgr=True, copy=True)
        result = stable.update(color_circle_position(cv_img))
        if result is not None:
            # 滤波后发送一位小数，STM32保留至最终脉冲换算。
            serial_dev.write(("RINGS," + ",".join("%.1f"%value for value in result) + "\n").encode())
            print("RINGS(float):", ",".join("%.1f"%value for value in result))
        cv2.putText(cv_img, "STABLE" if result is not None else "SEARCHING", (5, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0) if result is not None else (0, 0, 255), 1)
        disp.show(image.cv2image(cv_img, bgr=True, copy=True))


if __name__ == "__main__":
    main()
