"""六色锥形物料底座圆定位（落在纸面上的大圆的图像拟合中心）。

MaixCAM: 512x320, ttyS2/115200, RINGS,x1,y1,x2,y2,x3,y3\n。
仅恰好找到三个可靠底座且连续稳定时发送；六色是可识别种类。
可在电脑上导入 detect_materials() 调试，不依赖 maix。
"""
import cv2
import numpy as np
from itertools import combinations

WIDTH, HEIGHT = 512, 320
ROI = (0.0, 0.0, 1.0, 1.0)  # 全图搜索，距离变化不再截掉上移的目标。
MIN_BODY_AREA = 80
MIN_BASE_DIAMETER = 10
MAX_BASE_DIAMETER = 160
STABLE_FRAMES = 3
STABLE_ERROR = 4
# HSV 范围用于找到物料整体，不直接用整体重心。需按实际灯光调节。
COLOR_RANGES = {
    "red": [((0, 65, 45), (10, 255, 255)), ((170, 65, 45), (179, 255, 255))],
    "yellow": [((15, 65, 70), (37, 255, 255))],
    "green": [((38, 60, 40), (88, 255, 255))],
    "light_blue": [((89, 30, 130), (120, 255, 255))],
    "blue": [((89, 40, 35), (130, 255, 200))],
    "black": [((0, 0, 0), (179, 255, 105))],
}


def ellipse_points(ellipse, count=72, scale=1.0):
    (cx, cy), (a, b), angle = ellipse
    t = np.arange(count) * (2 * np.pi / count)
    angle = np.deg2rad(angle)
    u, v = a * scale / 2 * np.cos(t), b * scale / 2 * np.sin(t)
    return np.column_stack((cx + u * np.cos(angle) - v * np.sin(angle),
                            cy + u * np.sin(angle) + v * np.cos(angle)))


def base_candidates(mask, contour):
    """从物料外轮廓的连续圆弧拟合大底座，拒绝小顶面和细长侧面。"""
    x,y,w,h = cv2.boundingRect(contour)
    body = np.zeros_like(mask)
    cv2.drawContours(body,[contour],-1,255,-1)
    boundary = np.zeros_like(mask)
    cv2.drawContours(boundary,[contour],-1,255,1)
    distance = cv2.distanceTransform(255-boundary,cv2.DIST_L2,3)
    # 循环窗口可跨越 findContours 的起点，避免圆弧被起点切断。
    doubled = np.concatenate((contour,contour))
    candidates=[]
    for fraction in (0.40,0.55,0.70,0.85,1.0):
        length=max(12,int(len(contour)*fraction))
        for start in range(0,len(contour),max(4,len(contour)//16)):
            piece=doubled[start:start+length]
            if len(piece)<12:
                continue
            ellipse=cv2.fitEllipse(piece)
            (cx,cy),(a,b),angle=ellipse
            minor,major=min(a,b),max(a,b)
            if minor<MIN_BASE_DIAMETER or major>MAX_BASE_DIAMETER:
                continue
            if minor/major<0.70 or not 0.85<=major/min(w,h)<=1.20:
                continue
            pts=np.rint(ellipse_points(ellipse)).astype(int)
            inner=np.rint(ellipse_points(ellipse,scale=0.8)).astype(int)
            if (np.any(pts<0) or np.any(pts[:,0]>=mask.shape[1])
                    or np.any(pts[:,1]>=mask.shape[0])):
                continue
            # 底座投影应基本落在物料轮廓里；不能跨入背景。
            if np.mean(body[inner[:,1],inner[:,0]]>0)<0.90:
                continue
            if np.mean(mask[inner[:,1],inner[:,0]]>0)<0.70:
                continue
            distances=distance[pts[:,1],pts[:,0]]
            supported=distances<=max(1.5,minor*0.035)
            support=float(np.mean(supported))
            if support<0.50:
                continue
            # 拟合窗口自身也应贴合椭圆，防止直边影响圆心。
            delta=piece.reshape(-1,2)-np.array([cx,cy])
            theta=np.deg2rad(angle)
            u=(delta[:,0]*np.cos(theta)+delta[:,1]*np.sin(theta))/(a/2)
            v=(-delta[:,0]*np.sin(theta)+delta[:,1]*np.cos(theta))/(b/2)
            residual=float(np.mean(np.abs(np.sqrt(u*u+v*v)-1)))
            if residual>0.07:
                continue
            score=support-residual*2-abs(major/min(w,h)-1.0)*0.20
            candidates.append({"ellipse":ellipse,"score":score})
    return sorted(candidates,key=lambda item:item["score"],reverse=True)


def detect_materials(img, draw=True):
    """返回所有可靠物料底座：color、center、ellipse、score，从左到右排序。"""
    height,width = img.shape[:2]
    x0,y0,x1,y1 = [int(v*s) for v,s in zip(ROI,(width,height,width,height))]
    if not 0 <= x0 < x1 <= width or not 0 <= y0 < y1 <= height:
        raise ValueError("ROI must be inside image")
    crop = img[y0:y1,x0:x1]
    hsv = cv2.cvtColor(crop,cv2.COLOR_BGR2HSV)
    gray = cv2.GaussianBlur(cv2.cvtColor(crop,cv2.COLOR_BGR2GRAY),(3,3),0)
    found = []
    # 蓝与浅蓝范围有重叠：先合并为蓝色族，再按物料内部明暗分类。
    for color in ("red","yellow","green","blue","black"):
        mask = np.zeros(gray.shape,np.uint8)
        ranges = COLOR_RANGES[color] + (COLOR_RANGES["light_blue"] if color=="blue" else [])
        for lower,upper in ranges:
            mask |= cv2.inRange(hsv,np.array(lower,np.uint8),np.array(upper,np.uint8))
        if color == "black":
            # 黑色细环线与物料同色：先去掉窄线，再补物料内部小孔。
            mask = cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
        mask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((5,5),np.uint8))
        contours = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)[-2]
        for contour in contours:
            if cv2.contourArea(contour) < MIN_BODY_AREA:
                continue
            bx,by,bw,bh=cv2.boundingRect(contour)
            if bx<=1 or by<=1 or bx+bw>=gray.shape[1]-1 or by+bh>=gray.shape[0]-1:
                continue
            candidates = base_candidates(mask,contour)
            if not candidates:
                continue
            # 高度明显大于宽度时，底座位于轮廓下半部。
            if bh > bw*1.15:
                candidates = [c for c in candidates if c["ellipse"][0][1] > by + bh*0.45]
            if not candidates:
                continue
            best = candidates[0]
            (cx,cy),axes,angle = best["ellipse"]
            label = color
            if color=="blue":
                sample = np.zeros_like(mask)
                cv2.ellipse(sample,((cx,cy),axes,angle),255,-1)
                values = hsv[:,:,2][(sample>0)&(mask>0)]
                label = "light_blue" if len(values) and np.median(values)>190 else "blue"
            item = {"color":label,"center":(cx+x0,cy+y0),
                    "ellipse":((cx+x0,cy+y0),axes,angle),"score":best["score"]}
            found.append(item)
    # 跨颜色去重，特别是深色物料在 black 掩膜中的重复检测。
    unique=[]
    for item in sorted(found,key=lambda t:t["score"],reverse=True):
        if any(np.linalg.norm(np.array(item["center"])-other["center"]) < min(item["ellipse"][1])*0.4 for other in unique):
            continue
        unique.append(item)
    unique.sort(key=lambda t:t["center"][0])
    if len(unique) >= 3:
        unique = select_three_materials(unique)
    if draw:
        for item in unique:
            center=tuple(int(round(v)) for v in item["center"])
            cv2.ellipse(img,item["ellipse"],(255,0,255),1)
            cv2.drawMarker(img,center,(255,0,255),cv2.MARKER_CROSS,10,1)
            cv2.putText(img,item["color"],(center[0]+5,center[1]-5),cv2.FONT_HERSHEY_SIMPLEX,0.4,(255,0,255),1)
    return unique


def select_three_materials(materials):
    """选取大小相近、沿一条直线排列的三物料，排除画面外散放物料。"""
    best, best_score = [], -float("inf")
    candidates = sorted(materials,key=lambda t:t["score"],reverse=True)[:10]
    for triple in combinations(candidates,3):
        triple=sorted(triple,key=lambda t:t["center"][0])
        centers=np.array([t["center"] for t in triple])
        sizes=np.array([max(t["ellipse"][1]) for t in triple])
        if sizes.max()/sizes.min()>1.65:
            continue
        distances=np.linalg.norm(np.diff(centers,axis=0),axis=1)
        if np.any(distances<(sizes[:-1]+sizes[1:])*0.55):
            continue
        if distances.max()/distances.min()>1.8:
            continue
        baseline=centers[2]-centers[0]
        middle=centers[1]-centers[0]
        deviation=abs(baseline[0]*middle[1]-baseline[1]*middle[0])/max(np.linalg.norm(baseline),1)
        if deviation>sizes.mean()*0.35:
            continue
        score=sum(t["score"] for t in triple)-deviation/sizes.mean()-0.2*(distances.max()/distances.min()-1)
        if score>best_score:
            best,best_score=triple,score
    return best


class StableMaterials:
    def __init__(self):
        self.samples=[]
        self.colors=None

    def update(self,materials):
        if len(materials)!=3:
            self.samples=[]
            self.colors=None
            return None
        colors=tuple(m["color"] for m in materials)
        centers=np.array([m["center"] for m in materials])
        if colors!=self.colors or (self.samples and np.max(np.linalg.norm(centers-np.mean(self.samples,axis=0),axis=1))>STABLE_ERROR):
            self.samples=[]
        self.colors=colors
        self.samples.append(centers)
        self.samples=self.samples[-STABLE_FRAMES:]
        if len(self.samples)<STABLE_FRAMES:
            return None
        return tuple(int(v) for v in np.rint(np.mean(self.samples,axis=0)).reshape(-1))


def encode_rings(result):
    return ("RINGS,"+",".join(str(int(v)) for v in result)+"\n").encode()


def main():
    from maix import camera,display,app,image,uart
    cam=camera.Camera(WIDTH,HEIGHT)
    disp=display.Display()
    serial=uart.UART("/dev/ttyS2",115200)
    stable=StableMaterials()
    while not app.need_exit():
        img=image.image2cv(cam.read(),ensure_bgr=True,copy=True)
        materials=detect_materials(img)
        result=stable.update(materials)
        if result is not None:
            serial.write(encode_rings(result))
            print("RINGS:",result)
        cv2.putText(img,"STABLE" if result else "SEARCHING: %d/3"%len(materials),(5,18),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,255),1)
        disp.show(image.cv2image(img,bgr=True,copy=True))


if __name__=="__main__":
    main()
