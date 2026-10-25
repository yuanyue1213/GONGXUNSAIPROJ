"""一体化视觉：二维码、六色物料底座圆心、黑色同心圆环。

只上传本文件即可；运行时只依赖设备已有的 maix、cv2、numpy。
串口 /dev/ttyS2，115200；等待STM32的CAM请求后执行对应任务。
空闲时本地预览156（红/黑/浅蓝）圆台识别，不通过串口回传；长按1.5秒标定。
CAM,101,QR -> VISION,101,QR,156+123+516+231
CAM,102,MAT,5,256.0,160.0 -> VISION,102,MAT,5,113.2,160.7
CAM,103,RING,2 -> VISION,103,RING,2,256.4,160.3
CAM,104,TABLE,156,256.0,160.0 -> VISION,104,TABLE,156,256.0,160.0
CAM,105,OCCUPIED,156,256.0,160.0 -> VISION,105,OCCUPIED,156,256.0,160.0
OCCUPIED将三件物料按画面X排序，取中间物料底座中心；三帧稳定后输出。
TABLE联合三色候选估计椭圆轨迹，五帧稳定后输出整体中心；NEAR使用椭圆下端等待取料。
LAB标定与识别/main.py共用~/lab_thresholds.json；空闲时长按屏幕1.5秒可标定六色。
漏检/未稳定：VISION,请求编号,NONE；圆环目标1/2/3为画面左/中/右。
同一请求持续识别；换请求重置稳定窗口。保留亚像素拟合及一位小数输出。
"""

# ===== 六色物料检测 =====
import cv2
import numpy as np
from itertools import combinations, product
import time

WIDTH, HEIGHT = 512, 320
ROI = (0.0, 0.0, 1.0, 1.0)  # 全图搜索，距离变化不再截掉上移的目标。
MIN_BODY_AREA = 80
MIN_BASE_DIAMETER = 10
MAX_BASE_DIAMETER = 160
STABLE_FRAMES = 3
STABLE_ERROR = 4
SEARCH_WIDTH = 256  # 只用缩小图找区域；底座拟合始终使用原图。
REDETECT_INTERVAL = 20  # 周期性轻量全图核对，异常才做完整圆弧搜索。
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


def refine_subpixel(img, ellipse, arc):
    """沿底座圆弧法线找原图颜色梯度峰，抛物线插值得到亚像素边缘。"""
    (cx,cy),(a,b),angle=ellipse
    points=arc.reshape(-1,2).astype(np.float32)
    # 等间隔抽样，限制每个物料的开销；只细化已确认的底座圆弧。
    points=points[::max(1,int(np.ceil(len(points)/96)))]
    theta=np.deg2rad(angle)
    co,si=np.cos(theta),np.sin(theta)
    delta=points-np.array([cx,cy])
    u=delta[:,0]*co+delta[:,1]*si
    v=-delta[:,0]*si+delta[:,1]*co
    nx=u/(a*a); ny=v/(b*b)
    normals=np.column_stack((nx*co-ny*si,nx*si+ny*co))
    normals/=np.maximum(np.linalg.norm(normals,axis=1)[:,None],1e-6)
    offsets=np.arange(-3.0,3.01,0.5,dtype=np.float32)
    locations=points[:,None,:]+normals[:,None,:]*offsets[None,:,None]
    valid=(locations[:,:,0].min(axis=1)>=0)&(locations[:,:,1].min(axis=1)>=0)
    valid&=(locations[:,:,0].max(axis=1)<img.shape[1]-1)&(locations[:,:,1].max(axis=1)<img.shape[0]-1)
    samples=cv2.remap(img,locations[:,:,0].astype(np.float32),
                      locations[:,:,1].astype(np.float32),cv2.INTER_LINEAR).astype(np.float32)
    # 使用三通道梯度，红/绿等与背景灰度相近时仍能定位边缘。
    gradients=np.linalg.norm(samples[:,2:]-samples[:,:-2],axis=2)
    peak=np.argmax(gradients,axis=1)
    valid&=(peak>0)&(peak<gradients.shape[1]-1)
    rows=np.arange(len(points)); safe=np.clip(peak,1,gradients.shape[1]-2)
    left=gradients[rows,safe-1]; middle=gradients[rows,safe]; right=gradients[rows,safe+1]
    denominator=left-2*middle+right
    valid&=(middle>=12)&(denominator<-1e-3)
    correction=np.clip(0.5*(left-right)/np.minimum(denominator,-1e-3),-0.5,0.5)
    shift=offsets[safe+1]+0.5*correction
    valid&=np.abs(shift)<=2.0
    refined=(points+normals*shift[:,None])[valid].astype(np.float32)
    if len(refined)<12:
        return ellipse
    fitted=cv2.fitEllipse(refined.reshape(-1,1,2))
    (fx,fy),(fa,fb),_=fitted
    # 局部弱边缘或遮挡不能把底座中心拉走。
    if (np.hypot(fx-cx,fy-cy)>2.0 or not 0.9<fa/a<1.1
            or not 0.9<fb/b<1.1):
        return ellipse
    return fitted


def base_candidates(mask, contour, prior=None):
    """从物料外轮廓的连续圆弧拟合大底座，拒绝小顶面和细长侧面。"""
    x,y,w,h = cv2.boundingRect(contour)
    # 距离变换仅在物料附近的小图上计算，不再每个候选扫描全图。
    pad=max(5,int(max(w,h)*0.15))
    ox,oy=max(0,x-pad),max(0,y-pad)
    mask=mask[oy:min(mask.shape[0],y+h+pad),ox:min(mask.shape[1],x+w+pad)]
    contour=contour-np.array([[[ox,oy]]],dtype=np.int32)
    body = np.zeros_like(mask)
    cv2.drawContours(body,[contour],-1,255,-1)
    boundary = np.zeros_like(mask)
    cv2.drawContours(boundary,[contour],-1,255,1)
    distance = cv2.distanceTransform(255-boundary,cv2.DIST_L2,3)
    pieces=[]
    if prior is not None:
        (px,py),(pa,pb),angle=prior
        theta=np.deg2rad(angle)
        delta=contour.reshape(-1,2)-np.array([px-ox,py-oy])
        u=(delta[:,0]*np.cos(theta)+delta[:,1]*np.sin(theta))/(pa/2)
        v=(-delta[:,0]*np.sin(theta)+delta[:,1]*np.cos(theta))/(pb/2)
        radial=np.sqrt(u*u+v*v)
        bins=(np.floor((np.arctan2(v,u)+np.pi)*16/(2*np.pi)).astype(int)%16)
        # 使用本帧实际轮廓；只保留靠近上帧底座的点，最多拟合三次。
        for band in (0.06,0.12,0.20):
            keep=np.abs(radial-1)<band
            if np.count_nonzero(keep)>=12 and len(np.unique(bins[keep]))>=8:
                pieces.append(contour[keep])
    else:
        doubled=np.concatenate((contour,contour))
        for fraction in (0.40,0.55,0.70,0.85,1.0):
            length=max(12,int(len(contour)*fraction))
            for start in range(0,len(contour),max(4,len(contour)//16)):
                pieces.append(doubled[start:start+length])
    candidates=[]
    for piece in pieces:
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
        candidates.append({"ellipse":((cx+ox,cy+oy),(a,b),angle),"score":score,
                           "arc":piece+np.array([[[ox,oy]]],dtype=np.int32)})
    return sorted(candidates,key=lambda item:item["score"],reverse=True)


def color_mask(hsv, color, coarse=False):
    mask=np.zeros(hsv.shape[:2],np.uint8)
    ranges=COLOR_RANGES[color]+(COLOR_RANGES["light_blue"] if color=="blue" else [])
    for lower,upper in ranges:
        mask |= cv2.inRange(hsv,np.array(lower,np.uint8),np.array(upper,np.uint8))
    if color=="black":
        mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
    kernel=3 if coarse else 5
    return cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((kernel,kernel),np.uint8))


def _detect_region(img, colors=None, prior=None):
    """在原分辨率的局部区域拟合；坐标相对传入图像。"""
    height,width = img.shape[:2]
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    found = []
    # 蓝与浅蓝范围有重叠：先合并为蓝色族，再按物料内部明暗分类。
    for color in (colors or ("red","yellow","green","blue","black")):
        mask = color_mask(hsv,color)
        contours = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)[-2]
        for contour in contours:
            if cv2.contourArea(contour) < MIN_BODY_AREA:
                continue
            bx,by,bw,bh=cv2.boundingRect(contour)
            if bx<=1 or by<=1 or bx+bw>=width-1 or by+bh>=height-1:
                continue
            candidates = base_candidates(mask,contour,prior)
            if not candidates:
                continue
            # 高度明显大于宽度时，底座位于轮廓下半部。
            if bh > bw*1.15:
                candidates = [c for c in candidates if c["ellipse"][0][1] > by + bh*0.45]
            if not candidates:
                continue
            best = candidates[0]
            tracking_ellipse = best["ellipse"]
            best["ellipse"] = refine_subpixel(img,best["ellipse"],best["arc"])
            (cx,cy),axes,angle = best["ellipse"]
            label = color
            if color=="blue":
                sample = np.zeros_like(mask)
                cv2.ellipse(sample,((cx,cy),axes,angle),255,-1)
                values = hsv[:,:,2][(sample>0)&(mask>0)]
                label = "light_blue" if len(values) and np.median(values)>190 else "blue"
            item = {"color":label,"center":(cx,cy),
                    "ellipse":((cx,cy),axes,angle),"score":best["score"],
                    "bbox":(bx,by,bw,bh),"mask_color":color,
                    "tracking_ellipse":tracking_ellipse}
            found.append(item)
    # 跨颜色去重，特别是深色物料在 black 掩膜中的重复检测。
    unique=[]
    for item in sorted(found,key=lambda t:t["score"],reverse=True):
        if any(np.linalg.norm(np.array(item["center"])-other["center"]) < min(item["ellipse"][1])*0.4 for other in unique):
            continue
        unique.append(item)
    unique.sort(key=lambda t:t["center"][0])
    return unique


def draw_materials(img, materials):
    for item in materials:
        center=tuple(int(round(v)) for v in item["center"])
        cv2.ellipse(img,item["ellipse"],(255,0,255),1)
        cv2.drawMarker(img,center,(255,0,255),cv2.MARKER_CROSS,10,1)
        cx,cy=item["center"]
        cv2.putText(img,item["color"],(center[0]+5,center[1]-5),cv2.FONT_HERSHEY_SIMPLEX,0.4,(255,0,255),1)
        cv2.putText(img,"%.2f,%.2f"%(cx,cy),(max(0,center[0]-30),min(img.shape[0]-4,center[1]+24)),
                    cv2.FONT_HERSHEY_SIMPLEX,0.35,(255,0,255),1)


def padded_box(box, shape, fraction=0.50):
    x,y,w,h=box
    pad=max(10,int(max(w,h)*fraction))
    return (max(0,x-pad),max(0,y-pad),min(shape[1],x+w+pad),min(shape[0],y+h+pad))


def shift_material(item, x, y):
    result=dict(item)
    cx,cy=item["center"]
    _,axes,angle=item["ellipse"]
    bx,by,w,h=item["bbox"]
    result.update(center=(cx+x,cy+y),ellipse=((cx+x,cy+y),axes,angle),bbox=(bx+x,by+y,w,h))
    (tx,ty),ta,tangle=item["tracking_ellipse"]
    result["tracking_ellipse"]=((tx+x,ty+y),ta,tangle)
    return result


def coarse_boxes(hsv, sx, sy, color):
    mask=color_mask(hsv,color,coarse=True)
    count,_,stats,_=cv2.connectedComponentsWithStats(mask,8)
    proposals=[]
    for i in range(1,count):
        x,y,w,h,area=stats[i]
        if area<max(10,MIN_BODY_AREA/(sx*sy)*0.5):
            continue
        if min(w*sx,h*sy)<MIN_BASE_DIAMETER*0.65:
            continue
        # 背景大块不必进入昂贵拟合阶段。
        if min(w*sx,h*sy)>MAX_BASE_DIAMETER*1.6:
            continue
        proposals.append((int(x*sx),int(y*sy),int(np.ceil(w*sx)),int(np.ceil(h*sy))))
    return proposals


def detect_materials(img, draw=True, select_three=True):
    """缩小图查找连通区域，再在原图局部拟合底座；返回全图坐标。"""
    height,width=img.shape[:2]
    x0,y0,x1,y1=[int(v*s) for v,s in zip(ROI,(width,height,width,height))]
    if not 0<=x0<x1<=width or not 0<=y0<y1<=height:
        raise ValueError("ROI must be inside image")
    crop=img[y0:y1,x0:x1]
    factor=min(1.0,SEARCH_WIDTH/crop.shape[1])
    small=cv2.resize(crop,(max(1,round(crop.shape[1]*factor)),max(1,round(crop.shape[0]*factor))),interpolation=cv2.INTER_AREA)
    sx,sy=crop.shape[1]/small.shape[1],crop.shape[0]/small.shape[0]
    hsv=cv2.cvtColor(small,cv2.COLOR_BGR2HSV)
    found=[]
    for color in ("red","yellow","green","blue","black"):
        proposals=coarse_boxes(hsv,sx,sy,color)
        for box in proposals:
            left,top,right,bottom=padded_box(box,crop.shape)
            for item in _detect_region(crop[top:bottom,left:right],(color,)):
                found.append(shift_material(item,left+x0,top+y0))
    unique=[]
    for item in sorted(found,key=lambda t:t["score"],reverse=True):
        if any(np.linalg.norm(np.array(item["center"])-other["center"])<min(item["ellipse"][1])*0.4 for other in unique):
            continue
        unique.append(item)
    unique.sort(key=lambda t:t["center"][0])
    if select_three and len(unique)>=3:
        unique=select_three_materials(unique)
    if draw:
        draw_materials(img,unique)
    return unique


def coarse_track_audit(img, previous):
    """缩小全图核对已有物料区域，不执行底座拟合；不能代替本帧边缘验证。"""
    height,width=img.shape[:2]
    scale=min(1.0,SEARCH_WIDTH/width)
    small=cv2.resize(img,(max(1,round(width*scale)),max(1,round(height*scale))),interpolation=cv2.INTER_AREA)
    hsv=cv2.cvtColor(small,cv2.COLOR_BGR2HSV)
    sx,sy=width/small.shape[1],height/small.shape[0]
    by_color={}
    for item in previous:
        color=item["mask_color"]
        if color not in by_color:
            by_color[color]=coarse_boxes(hsv,sx,sy,color)
        px,py,pw,ph=item["bbox"]
        matched=False
        for x,y,w,h in by_color[color]:
            intersection=max(0,min(px+pw,x+w)-max(px,x))*max(0,min(py+ph,y+h)-max(py,y))
            union=pw*ph+w*h-intersection
            if union>0 and intersection/union>0.45:
                matched=True
                break
        if not matched:
            return False
    return True


class MaterialTracker:
    """每帧重新拟合局部边缘，不输出预测圆心；失败当帧恢复全图查找。"""
    def __init__(self):
        self.previous=[]
        self.age=0
        self.shape=None
        self.mode="SEARCH"

    def update(self,img,draw=True):
        materials=[]
        audit_due=self.age>=REDETECT_INTERVAL
        valid_previous=len(self.previous)==3 and self.shape==img.shape[:2]
        if valid_previous and audit_due:
            valid_previous=coarse_track_audit(img,self.previous)
            self.age=0
        if valid_previous:
            for previous in self.previous:
                left,top,right,bottom=padded_box(previous["bbox"],img.shape,0.45)
                (cx,cy),axes,angle=previous["tracking_ellipse"]
                prior=((cx-left,cy-top),axes,angle)
                options=[shift_material(item,left,top) for item in
                         _detect_region(img[top:bottom,left:right],(previous["mask_color"],),prior)]
                size=max(previous["ellipse"][1])
                options=[item for item in options if
                         np.linalg.norm(np.array(item["center"])-previous["center"])<size*0.6
                         and 0.65<max(item["ellipse"][1])/size<1.5]
                if not options:
                    break
                materials.append(min(options,key=lambda t:np.linalg.norm(np.array(t["center"])-previous["center"])))
            materials=select_three_materials(materials) if len(materials)==3 else []
            self.age+=1
            self.mode="AUDIT" if audit_due else "TRACK"
        if len(materials)!=3:
            materials=detect_materials(img,False)
            self.age=0
            self.mode="SEARCH"
        self.previous=materials if len(materials)==3 else []
        self.shape=img.shape[:2]
        if draw:
            draw_materials(img,materials)
        return materials


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
        # 保留浮点平均圆心；绘图像素和整数串口编码分别在使用时取整。
        return tuple(float(v) for v in np.mean(self.samples,axis=0).reshape(-1))


def encode_rings(result):
    # 滤波后以一位小数传输，STM32在最终电机脉冲计算时取整。
    return ("RINGS,"+",".join("%.1f"%v for v in result)+"\n").encode()

# ===== 黑色同心圆环检测（独立参数） =====
from itertools import combinations
import cv2
import numpy as np

RING_FRAME_WIDTH, RING_FRAME_HEIGHT = 512, 320
# 按画面比例设置 RING_ROI；默认全图，现场固定安装后可缩小范围。
RING_ROI = (0.0, 0.0, 1.0, 1.0)
RING_MIN_DIAMETER = 20
RING_MAX_DIAMETER = 150
RING_MIN_AXIS_RATIO = 0.60
RING_MAX_FIT_ERROR = 0.09
RING_MIN_RING_LEVELS = 3
RING_MIN_RADIUS_SPAN = 1.20
RING_STABLE_FRAMES = 3
RING_STABLE_ERROR_PX = 4.0


def ring_refine_ring_edge(gray, ellipse, contour):
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


def ring_ellipse_candidate(contour):
    """只接受覆盖完整椭圆且拟合误差较小的轮廓，过滤数字/胶带。"""
    if len(contour) < 12:
        return None
    ellipse = cv2.fitEllipse(contour)
    (cx, cy), (a, b), angle = ellipse
    minor, major = min(a, b), max(a, b)
    if not RING_MIN_DIAMETER <= minor <= major <= RING_MAX_DIAMETER:
        return None
    if minor / major < RING_MIN_AXIS_RATIO:
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
    if error > RING_MAX_FIT_ERROR:
        return None
    # 至少覆盖 16 个角度区间中的 14 个，拒绝局部圆弧拟合出的假椭圆。
    bins = np.floor((np.arctan2(v, u) + np.pi) * 16 / (2 * np.pi)).astype(int) % 16
    if len(np.unique(bins)) < 14:
        return None
    return {"center": np.array([cx, cy]), "radius": (a + b) / 4,
            "ellipse": ellipse, "error": error, "ratio": minor / major,
            "contour":contour}


def ring_group_targets(candidates):
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
        if len(levels) < RING_MIN_RING_LEVELS:
            continue
        if levels[-1]["radius"] / levels[0]["radius"] < RING_MIN_RADIUS_SPAN:
            continue
        weights = np.array([1 / (0.02 + item["error"]) for item in levels])
        center = np.average([item["center"] for item in levels], axis=0, weights=weights)
        targets.append({"center": center, "radius": levels[-1]["radius"],
                        "levels": levels, "score": len(levels)})
    return targets


def ring_select_three(targets):
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


def ring_color_circle_position(img):
    """检测三个靶标并绘制结果；未找到完整三组时返回 None。"""
    if img is None or img.size == 0:
        return None
    height, width = img.shape[:2]
    x0, y0, x1, y1 = [int(value * size) for value, size in zip(RING_ROI, (width, height, width, height))]
    if not (0 <= x0 < x1 <= width and 0 <= y0 < y1 <= height):
        raise ValueError("RING_ROI must be inside the image")
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
            candidate = ring_ellipse_candidate(contour)
            if candidate is not None:
                candidates.append(candidate)
        triple = ring_select_three(ring_group_targets(candidates))
        if triple is not None and (best is None or sum(t["score"] for t in triple) > sum(t["score"] for t in best)):
            best = triple
    if best is None:
        return None
    # 完成三组筛选后才细化，避免所有背景轮廓都做亚像素计算。
    for target in best:
        for level in target["levels"]:
            level["ellipse"]=ring_refine_ring_edge(edge_gray,level["ellipse"],level["contour"])
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


class ring_StableCenters:
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
            if np.max(np.linalg.norm(current - average, axis=1)) > RING_STABLE_ERROR_PX:
                self.samples = []
        self.samples.append(current)
        self.samples = self.samples[-RING_STABLE_FRAMES:]
        if len(self.samples) < RING_STABLE_FRAMES:
            return None
        return tuple(float(value) for value in np.mean(self.samples, axis=0).reshape(-1))

# ===== 三物块圆台中心与持久化 LAB 标定（来自识别目录） =====
# -*- coding: utf-8 -*-
"""MaixCAM2 圆台识别模块（本文件不读取相机，也不控制串口/触摸屏）。

用法：
    centers, tolerances, calibrated = load_lab_thresholds()
    detector = TurntableCenterDetector(centers, tolerances,
                                       colors=("red", "yellow", "blue"))
    stable_center = detector.detect(cv_bgr_image, draw=True)

三色阈值共用统一的 JSON 文件；主程序只需标定并保存一次。
"""

import json
import math
import os
from collections import deque

import cv2
import numpy as np

# 将配置文件放在设备用户主目录，切换脚本路径/导入不同模块时仍共用一份。
DEFAULT_CONFIG_PATH = os.path.join(os.path.expanduser("~"), "lab_thresholds.json")

# 与用户原始代码完全一致的浮点 LAB 预设值；不是已经标定成功的值。
DEFAULT_LAB_CENTER = {
    "light_blue": (77, -14, -29),
    "blue": (42, 21, -60),
    "yellow": (81, 6, 75),
    "green": (80, -78, 59),
    "black": (21, 2, 0),
    "red": (47, 65, 35),
}
DEFAULT_LAB_TOL = {
    "light_blue": (8, 8, 8),
    "blue": (8, 8, 8),
    "yellow": (15, 12, 12),
    "green": (10, 10, 10),
    "black": (12, 10, 10),
    "red": (15, 12, 12),
}

DRAW_COLORS_BGR = {
    "light_blue": (255, 255, 0),
    "blue": (255, 0, 0),
    "yellow": (0, 255, 255),
    "green": (0, 255, 0),
    "black": (180, 180, 180),
    "red": (0, 0, 255),
}


def _valid_triplet(obj, positive=False):
    try:
        a = np.asarray(obj, dtype=np.float32)
        if a.shape != (3,) or not bool(np.all(np.isfinite(a))):
            return None
        if positive and not bool(np.all(a >= 1.0)):
            return None
        return a
    except (TypeError, ValueError):
        return None


def load_lab_thresholds(path=DEFAULT_CONFIG_PATH):
    """返回 LAB_CENTER, LAB_TOL, calibrated_colors。

    没有文件时返回原始预设值，但 calibrated_colors 为空：不会误判为已标定。
    """
    centers = {k: np.array(v, dtype=np.float32)
               for k, v in DEFAULT_LAB_CENTER.items()}
    tolerances = {k: np.array(v, dtype=np.float32)
                  for k, v in DEFAULT_LAB_TOL.items()}
    calibrated = set()

    if not os.path.isfile(path):
        print("未找到 LAB 标定文件：", path)
        return centers, tolerances, calibrated

    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        if data.get("version") != 1:
            raise ValueError("不支持的标定文件版本")
        saved_centers = data["lab_center"]
        saved_tols = data["lab_tol"]
        saved_calibrated = data.get("calibrated_colors", [])
        if not isinstance(saved_calibrated, list):
            raise ValueError("calibrated_colors 格式错误")

        valid = set()
        for name in centers:
            c = _valid_triplet(saved_centers.get(name))
            t = _valid_triplet(saved_tols.get(name), positive=True)
            if c is not None and t is not None:
                # 防止损坏的标定文件生成非法 LAB 范围。
                if (0 <= c[0] <= 100 and -128 <= c[1] <= 127
                        and -128 <= c[2] <= 127):
                    centers[name] = c
                    tolerances[name] = t
                    valid.add(name)

        calibrated = set(saved_calibrated) & valid
        print("已读取 LAB 标定文件：", path)
        print("已完成标定的颜色：", sorted(calibrated))
    except (OSError, ValueError, TypeError, KeyError) as exc:
        print("读取 LAB 标定文件失败，使用原始预设值：", exc)
        centers = {k: np.array(v, dtype=np.float32)
                   for k, v in DEFAULT_LAB_CENTER.items()}
        tolerances = {k: np.array(v, dtype=np.float32)
                      for k, v in DEFAULT_LAB_TOL.items()}
        calibrated.clear()
    return centers, tolerances, calibrated


def save_lab_thresholds(centers, tolerances, calibrated, path=DEFAULT_CONFIG_PATH):
    """原子写入 JSON，避免程序中止导致旧标定文件被写坏。"""
    payload = {
        "version": 1,
        "lab_center": {k: [float(x) for x in v] for k, v in centers.items()},
        "lab_tol": {k: [float(x) for x in v] for k, v in tolerances.items()},
        "calibrated_colors": sorted(calibrated),
    }
    temp_path = path + ".tmp"
    try:
        with open(temp_path, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2, ensure_ascii=False)
            f.flush()
            os.fsync(f.fileno())
        os.replace(temp_path, path)
        print("标定结果已保存：", path)
        return True
    except OSError as exc:
        print("保存 LAB 标定结果失败：", exc)
        try:
            if os.path.exists(temp_path):
                os.remove(temp_path)
        except OSError:
            pass
        return False


def turntable_geometry(vertices, min_edge=15., min_axis_ratio=.4):
    """Affine ellipse of three equal-spaced positions (120 degrees apart).

    For p=c+A*cos(t)+B*sin(t), three equally spaced samples give
    M=A*A.T+B*B.T=2*mean((p-c)*(p-c).T). No equal IMAGE edges are required.
    This is an affine projection model, not a full perspective calibration.
    """
    vertices = np.asarray(vertices, dtype=float)
    if vertices.shape != (3, 2) or not np.all(np.isfinite(vertices)):
        return None
    if any(np.linalg.norm(vertices[i] - vertices[j]) < min_edge
           for i, j in ((0, 1), (1, 2), (2, 0))):
        return None
    center = np.mean(vertices, axis=0)
    centered = vertices - center
    matrix = 2. * centered.T.dot(centered) / 3.
    values = np.linalg.eigvalsh(matrix)
    if values[0] <= 1.e-6 or values[1] <= 1.e-6:
        return None
    minor, major = np.sqrt(values)
    if minor / major < min_axis_ratio or major > 320.:
        return None
    # Lowest image point of an arbitrarily rotated ellipse. It need not be
    # vertically below its center when the ellipse's axes are tilted.
    vertical_radius = math.sqrt(float(matrix[1, 1]))
    bottom = center + (matrix[0, 1] / vertical_radius, vertical_radius)
    return dict(center=center, matrix=matrix, inverse=np.linalg.inv(matrix),
                minor=float(minor), major=float(major), bottom=bottom)


def on_turntable_track(point, geometry, tolerance=.3):
    delta = np.asarray(point, dtype=float) - geometry['center']
    if delta.shape != (2,) or not np.all(np.isfinite(delta)):
        return False
    radius2 = float(delta.dot(geometry['inverse']).dot(delta))
    return (1. - tolerance)**2 <= radius2 <= (1. + tolerance)**2


class TurntableCenterDetector:
    """使用三个彩色圆柱求旋转圆台中心，仅对最终中心做稳定判断。"""

    def __init__(self, lab_center, lab_tol, colors=("red", "yellow", "blue"),
                 min_area=80, min_circularity=0.45,
                 triangle_tol=0.25, min_edge=15.0,
                 stable_count=5, stable_err=1.0, min_axis_ratio=.4, max_area_ratio=3.):
        if len(colors) != 3 or len(set(colors)) != 3:
            raise ValueError("圆台必须配置三种不同颜色")
        if not all(c in lab_center and c in lab_tol for c in colors):
            raise ValueError("圆台颜色必须全部定义在 LAB 阈值字典中")
        self.colors = tuple(colors)
        # 引用外部字典；主程序标定后更新字典，识别模块会自动使用新阈值。
        self.lab_center = lab_center
        self.lab_tol = lab_tol
        self.min_area = float(min_area)
        self.min_circularity = float(min_circularity)
        # triangle_tol retained for source compatibility; image-edge equality is no longer used.
        self.min_axis_ratio = float(min_axis_ratio)
        self.max_area_ratio = float(max_area_ratio)
        self.min_edge = float(min_edge)
        self.stable_count = int(stable_count)
        self.stable_err = float(stable_err)
        if self.stable_count < 1 or self.stable_err < 0:
            raise ValueError("无效的稳定判断参数")
        self.history = deque(maxlen=self.stable_count)
        self.geometry_history = deque(maxlen=self.stable_count)
        self.locked_geometry = self.last_geometry = None
        self.lost_frames = 0
        self.selected_candidates = {}
        self.kernel = np.ones((3, 3), np.uint8)
        self.last_points = {}
        self.last_center = None
        self.last_state = "init"

    def reset(self):
        self.history.clear()
        self.geometry_history.clear()
        self.locked_geometry = self.last_geometry = None
        self.lost_frames = 0
        self.selected_candidates = {}
        self.last_center = None
        self.last_points = {}
        self.last_state = "reset"

    def _candidates_for_color(self, lab_image, color):
        center, tol = self.lab_center[color], self.lab_tol[color]
        lower = np.maximum(center - tol, [0, -128, -128]).astype(np.float32)
        upper = np.minimum(center + tol, [100, 127, 127]).astype(np.float32)
        mask = cv2.inRange(lab_image, lower, upper)
        mask = cv2.erode(mask, self.kernel, iterations=2)
        mask = cv2.dilate(mask, self.kernel, iterations=2)
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        candidates = []
        h, w = mask.shape
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_area or area > h*w*.15:
                continue
            perimeter = cv2.arcLength(contour, True)
            if perimeter <= 0:
                continue
            circularity = 4.*math.pi*area/(perimeter*perimeter)
            if circularity < self.min_circularity:
                continue
            x, y, bw, bh = cv2.boundingRect(contour)
            if x <= 0 or y <= 0 or x+bw >= w or y+bh >= h:
                continue  # A clipped blob's centroid is not a complete material location.
            m = cv2.moments(contour)
            if m['m00'] > 0:
                candidates.append(dict(center=(float(m['m10']/m['m00']), float(m['m01']/m['m00'])),
                                       area=float(area), circularity=float(circularity)))
        return candidates

    def _center_for_color(self, lab_image, color):
        # Legacy single-color callers must not silently choose a larger distractor.
        candidates = self._candidates_for_color(lab_image, color)
        return candidates[0]['center'] if len(candidates) == 1 else None

    def _reject(self, state):
        self.history.clear()
        self.geometry_history.clear()
        self.last_state = state
        self.lost_frames += 1
        if self.lost_frames >= self.stable_count*3:
            # A manually moved camera must be able to reacquire in idle preview.
            # Fresh samples must still pass the full stability window.
            self.locked_geometry = None
        return None

    def _select_group(self, candidates):
        groups = []
        for group in product(*(candidates[c] for c in self.colors)):
            areas = np.array([obj['area'] for obj in group])
            if np.max(areas)/np.min(areas) > self.max_area_ratio:
                continue
            geometry = turntable_geometry([obj['center'] for obj in group], self.min_edge, self.min_axis_ratio)
            if geometry is None:
                continue
            if self.locked_geometry is not None:
                old = self.locked_geometry
                if np.linalg.norm(geometry['center'] - old['center']) > max(6., old['minor']*.12):
                    continue
                if any(not on_turntable_track(obj['center'], old) for obj in group):
                    continue
            score = float(np.std(np.log(areas)))
            score += .2*(1.-min(obj['circularity'] for obj in group))
            score += .1*(1.-geometry['minor']/geometry['major'])
            groups.append((score, group, geometry))
        groups.sort(key=lambda item: item[0])
        if not groups:
            return None, 'invalid_triangle'
        if len(groups)>1 and groups[1][0]-groups[0][0] < .12:
            return None, 'ambiguous_colors'
        return groups[0], None

    def detect(self, bgr_image, draw=False):
        """输入每帧 OpenCV BGR 图像，5 帧中心稳定时返回 (x, y)，否则 None。"""
        if bgr_image is None:
            self.reset()
            return None

        # 注意必须 float32/255 再转 LAB，兼容原来标定的 0~100、-128~127 值。
        blur = cv2.GaussianBlur(bgr_image, (5, 5), 0)
        lab = cv2.cvtColor(blur.astype(np.float32) / 255.0,
                           cv2.COLOR_BGR2LAB)

        candidates = {name: self._candidates_for_color(lab, name) for name in self.colors}
        self.last_points = {c: objects[0]['center'] for c, objects in candidates.items() if len(objects)==1}
        self.last_center = self.last_geometry = None
        self.selected_candidates = {}
        if any(not objects for objects in candidates.values()):
            return self._reject('colors_missing')
        if any(len(objects)>6 for objects in candidates.values()):
            return self._reject('ambiguous_colors')
        selected, error = self._select_group(candidates)
        if selected is None:
            return self._reject(error)
        _, group, geometry = selected
        self.lost_frames = 0
        self.selected_candidates = dict(zip(self.colors, group))
        self.last_points = {c: obj['center'] for c, obj in self.selected_candidates.items()}
        origin = geometry['center']
        self.last_center = tuple(float(v) for v in origin)
        self.last_geometry = geometry
        if draw:
            for name, point in self.last_points.items():
                p = tuple(int(round(v)) for v in point)
                color = DRAW_COLORS_BGR.get(name, (0, 255, 255))
                cv2.circle(bgr_image, p, 4, color, -1)
                cv2.putText(bgr_image, name, (p[0]+6, p[1]-6), cv2.FONT_HERSHEY_SIMPLEX, .45, color, 1)
        if draw:
            o = tuple(int(round(float(v))) for v in origin)
            cv2.drawMarker(bgr_image, o, (0, 255, 255),
                           cv2.MARKER_CROSS, 14, 1)

        self.history.append(self.last_center)
        self.geometry_history.append(geometry)
        self.last_state = "collecting"
        if len(self.history) < self.stable_count:
            return None

        samples = np.array(self.history, dtype=np.float32)
        reference = np.median(samples, axis=0)
        distances = np.linalg.norm(samples - reference, axis=1)
        if not np.all(distances <= self.stable_err):
            # 保留最近五帧滑动窗口，下一帧继续检测。
            self.last_state = "unstable"
            return None

        bottoms = np.asarray([g['bottom'] for g in self.geometry_history])
        if np.max(np.linalg.norm(bottoms - np.median(bottoms, axis=0), axis=1)) > max(2., self.stable_err*2.):
            self.last_state = 'unstable_geometry'
            return None
        # Lock a five-frame ellipse, not a single noisy set of rotating points.
        mean_matrix = np.mean([g['matrix'] for g in self.geometry_history], axis=0)
        mean_center = np.mean(samples, axis=0)
        values = np.linalg.eigvalsh(mean_matrix)
        vy = math.sqrt(float(mean_matrix[1, 1]))
        self.locked_geometry = dict(center=mean_center, matrix=mean_matrix, inverse=np.linalg.inv(mean_matrix),
                                    minor=float(math.sqrt(values[0])), major=float(math.sqrt(values[1])),
                                    bottom=mean_center + (mean_matrix[0, 1]/vy, vy))
        self.last_geometry = self.locked_geometry
        # 五帧全在中位中心 1 像素半径内，才输出一组整数坐标。
        output = np.mean(samples, axis=0)
        stable_point = (int(round(float(output[0]))),
                        int(round(float(output[1]))))
        self.history.clear()
        self.geometry_history.clear()
        self.last_state = "stable"
        if draw:
            cv2.drawMarker(bgr_image, stable_point, (0, 0, 255),
                           cv2.MARKER_CROSS, 22, 2)
        return stable_point


def table_colors(target):
    digits = str(target)
    if len(digits) != 3 or len(set(digits)) != 3 or any(d not in "123456" for d in digits):
        return None
    names = {value: name for name, value in COLORS.items()}
    return tuple(names[int(d)] for d in digits)


class TurntableTaskDetector:
    """A batch-specific five-frame center; never fall back to a single block."""
    def __init__(self, path=DEFAULT_CONFIG_PATH):
        self.path = path
        self.centers, self.tolerances, self.calibrated = load_lab_thresholds(path)
        self.reset()

    def reset(self):
        self.target = None
        self.detector = None
        self.state = "waiting"

    def detect(self, frame, target, draw=False):
        colors = table_colors(target)
        if colors is None:
            self.reset()
            self.state = "invalid_colors"
            return None
        if self.target != target:
            self.reset()
            self.target = target
        if not all(color in self.calibrated for color in colors):
            self.detector = None
            self.state = "NEED LAB: " + "/".join(c for c in colors if c not in self.calibrated)
            return None
        if self.detector is None:
            self.detector = TurntableCenterDetector(self.centers, self.tolerances, colors=colors)
        result = self.detector.detect(frame, draw=draw)
        self.state = self.detector.last_state
        if self.state == "colors_missing":
            self.state = "MISSING: " + "/".join(c for c in colors if c not in self.detector.last_points)
        return result


class IdleTablePreview:
    """Local-only preview. Its samples/geometry never authorize a task response."""
    def __init__(self, table, target=156):
        self.table = table
        self.target = target
        self.reset()

    def reset(self):
        self.detector = None
        self.center = None
        self.state = "waiting"

    def draw(self, img):
        colors = table_colors(self.target)
        self.center = None
        missing_lab = [c for c in colors if c not in self.table.calibrated]
        if missing_lab:
            self.detector = None
            self.state = "NEED LAB: " + "/".join(missing_lab)
        else:
            if self.detector is None:
                self.detector = TurntableCenterDetector(self.table.centers, self.table.tolerances, colors=colors)
            self.center = self.detector.detect(img, draw=True)
            self.state = self.detector.last_state
            if self.state == "colors_missing":
                self.state = "MISSING: " + "/".join(c for c in colors if c not in self.detector.last_points)
        cv2.putText(img, "PREVIEW TABLE %d / WAIT CAM" % self.target, (5, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 255, 0), 1)
        cv2.putText(img, self.state, (5, 40), cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 255, 255), 1)
        count = len(self.detector.last_points) if self.detector is not None else 0
        label = "COLORS %d/3 / HOLD 1.5s: LAB" % count
        if self.center is not None:
            label = "CENTER %d,%d / PREVIEW ONLY" % self.center
            near = NearPickup()
            if near.bind(self.detector.last_points, self.target, geometry=self.detector.last_geometry):
                cv2.circle(img, tuple(int(round(v)) for v in near.point), 8, (0, 255, 255), 1)
        cv2.putText(img, label, (5, 60), cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 255, 255), 1)
        return self.center


def calibrate_table_colors(cam, dis, ts, table, colors):
    """Original touch LAB workflow, available only while no task is active."""
    from maix import app, image, time
    LAB_CENTER, LAB_TOL, CALIBRATED_COLORS = table.centers, table.tolerances, table.calibrated
    detector = table
    DOUBLE_CLICK_MS, TOL_STEP, SAMPLE_HALF, SAMPLE_BOX_HALF = 500, 1, 6, 10
    SKIP_RECT = (390, 5, 510, 55)
    L_PLUS_RECT, L_MINUS_RECT = (10, 50, 246, 105), (266, 50, 502, 105)
    A_PLUS_RECT, A_MINUS_RECT = (10, 115, 246, 170), (266, 115, 502, 170)
    B_PLUS_RECT, B_MINUS_RECT = (10, 180, 246, 235), (266, 180, 502, 235)
    NEXT_RECT = (150, 250, 362, 315)

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
                    LAB_CENTER, LAB_TOL, CALIBRATED_COLORS, table.path):
                # 保存失败就恢复此颜色之前的状态，避免误认为下次能复用。
                LAB_CENTER[color] = old_center
                LAB_TOL[color] = old_tol
                if not old_was_calibrated:
                    CALIBRATED_COLORS.discard(color)
                print("该颜色标定未保存，请检查设备磁盘权限：", color)
        detector.reset()
        print("标定过程结束，已完成：", sorted(CALIBRATED_COLORS))

    calibrate_colors(colors)


# ===== STM32协议分发、二维码及主循环 =====
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
        if ((fields[2] in ("MAT", "RING", "PICK", "NEAR") and 1 <= target <= (6 if fields[2] in ("MAT", "PICK", "NEAR") else 3))
                or (fields[2] in ("TABLE", "OCCUPIED") and table_colors(target) is not None)):
            result = int(fields[1]), fields[2], target
            if len(fields) == 4:
                return result
            if fields[2] not in ("MAT", "TABLE", "OCCUPIED"):
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


def occupied_middle_center(objects, target):
    """Middle of three occupied ring positions, independent of pickup order/color.

    Never re-label two remaining objects as a new middle; calibrate before pickup.
    Ambiguous same-color candidates or an implausible row produce no coordinate.
    """
    colors = table_colors(target)
    if colors is None:
        return None
    points = []
    for color in colors:
        matches = [obj for obj in objects if obj.get("color") == color]
        if len(matches) != 1:
            return None
        p = np.asarray(matches[0]["center"], dtype=float)
        if p.shape != (2,) or not np.all(np.isfinite(p)) or not (0 <= p[0] < 512 and 0 <= p[1] < 320):
            return None
        points.append(p)
    points.sort(key=lambda p: p[0])
    left, middle, right = points
    gaps = (middle[0] - left[0], right[0] - middle[0])
    if min(gaps) < 8 or max(gaps) > min(gaps) * 1.8:
        return None
    # Three ring positions lie along one projected row; permit a tilted row.
    ratio = (middle[0] - left[0]) / (right[0] - left[0])
    row_y = left[1] + ratio * (right[1] - left[1])
    if abs(middle[1] - row_y) > max(8., (right[0] - left[0]) * .12):
        return None
    return tuple(float(value) for value in middle)


class NearPickup:
    """One fixed pickup point at the image-bottom side of the turntable.

    Retain its geometry after removal. A new color requires fresh, stationary
    observations at that point, rather than choosing one of three block poses.
    """
    def __init__(self):
        self.colors = None
        self.center = self.point = None
        self.radius = None
        self.geometry = None
        self.areas = {}
        self.reset()

    def reset(self):
        self.history = deque(maxlen=5)
        self.target = None

    def bind(self, points, target, geometry=None, areas=None):
        self.geometry = None
        self.areas = {}
        self.colors = table_colors(target)
        self.center = self.point = None
        self.reset()
        if self.colors is None or any(c not in points for c in self.colors):
            return False
        vertices = np.asarray([points[c] for c in self.colors], dtype=float)
        if vertices.shape != (3, 2) or not np.all(np.isfinite(vertices)):
            return False
        geometry = geometry if geometry is not None else turntable_geometry(vertices)
        if geometry is None:
            return False
        point = geometry['bottom']
        if geometry['minor'] < 10 or not (0 <= point[0] < 512 and 0 <= point[1] < 320):
            return False
        self.geometry = geometry
        self.center, self.point, self.radius = geometry['center'].copy(), point.copy(), geometry['minor']
        self.areas = dict(areas or {})
        return True

    def update(self, points, target):
        if target != self.target:
            self.reset()
            self.target = target
        names = {value: name for name, value in COLORS.items()}
        name = names.get(target)
        point = points.get(name)
        valid = self.point is not None and name in (self.colors or ()) and point is not None
        if valid:
            point = np.asarray(point, dtype=float)
            valid = point.shape == (2,) and np.all(np.isfinite(point))
        if valid:
            # The fixed bottom ROI remains fixed as other blocks disappear.
            window = min(8., max(3., self.radius * .12))
            valid = np.linalg.norm(point - self.point) <= window
            valid = valid and all(float(p[1]) <= float(point[1]) + 2.
                                  for c, p in points.items() if c != name)
        if not valid:
            self.history.clear()
            return None
        self.history.append(point.copy())
        if len(self.history) < 5:
            return None
        samples = np.asarray(self.history)
        if np.max(np.linalg.norm(samples - np.median(samples, axis=0), axis=1)) > 2.:
            return None
        return "2"  # Dedicated near pose, stored in the R middle profile.

    def detect(self, img, target, table):
        if self.point is None or not all(c in table.calibrated for c in self.colors):
            self.reset()
            return None
        blur = cv2.GaussianBlur(img, (5, 5), 0)
        lab = cv2.cvtColor(blur.astype(np.float32) / 255., cv2.COLOR_BGR2LAB)
        detector = TurntableCenterDetector(table.centers, table.tolerances, colors=self.colors)
        points = {}
        for color in self.colors:
            candidates = [obj for obj in detector._candidates_for_color(lab, color)
                          if on_turntable_track(obj['center'], self.geometry)]
            if color in self.areas:
                candidates = [obj for obj in candidates if 1./3. <= obj['area']/self.areas[color] <= 3.]
            if len(candidates) > 1:
                self.history.clear()
                return None  # Never guess which same-color blob is the real target.
            if candidates:
                points[color] = candidates[0]['center']
        return self.update(points, target)


class PhysicalPositions:
    """Fixed image L/M/R references, captured before any block is removed.

    Never sort the remaining one/two blocks into newly numbered positions.
    A moving target must return close to a reference before selecting its pose.
    """
    def __init__(self):
        self.points = None
        self.colors = None
        self.raw = False

    def bind(self, points, target, raw=False):
        self.points = None
        self.colors = table_colors(target)
        self.raw = raw
        if self.colors is None or any(c not in points for c in self.colors):
            return False
        samples = np.array([points[c] for c in self.colors], dtype=float)
        if (samples.shape != (3, 2) or not np.all(np.isfinite(samples))
                or np.any(samples < 0) or np.any(samples[:, 0] >= 512)
                or np.any(samples[:, 1] >= 320)):
            return False
        samples = samples[np.argsort(samples[:, 0])]
        if np.min(np.diff(samples[:, 0])) < 8:
            return False  # Left/middle/right is ambiguous at this view.
        self.points = samples
        return True

    def position(self, point):
        if self.points is None or point is None:
            return None
        point = np.asarray(point, dtype=float)
        if point.shape != (2,) or not np.all(np.isfinite(point)):
            return None
        distances = np.linalg.norm(self.points - point, axis=1)
        selected = int(np.argmin(distances))
        spacing = min(np.linalg.norm(self.points[selected] - p)
                      for i, p in enumerate(self.points) if i != selected)
        # Leave gaps between poses: never guess a pose between two references.
        if distances[selected] > spacing * .30:
            return None
        return str(selected + 1)

    def detect(self, img, target, table):
        if self.points is None or self.colors is None:
            return None
        names = {value: name for name, value in COLORS.items()}
        color = names.get(target)
        if color not in self.colors:
            return None
        if self.raw:
            if color not in table.calibrated:
                return None
            blur = cv2.GaussianBlur(img, (5, 5), 0)
            lab = cv2.cvtColor(blur.astype(np.float32) / 255., cv2.COLOR_BGR2LAB)
            detector = TurntableCenterDetector(table.centers, table.tolerances, colors=self.colors)
            point = detector._center_for_color(lab, color)
        else:
            point = material_center(detect_materials(img, False, select_three=False), target)
        return self.position(point)


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
    table = TurntableTaskDetector()
    preview = IdleTablePreview(table)
    positions = PhysicalPositions()
    near = NearPickup()
    try:
        from maix import touchscreen
        touch = touchscreen.TouchScreen()
    except Exception as exc:
        touch = None
        print("LAB touch calibration unavailable:", exc)
    press_start = None
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
                        table.reset()
                        preview.reset()
                        if parsed[1] in ("TABLE", "OCCUPIED", "QR"):
                            positions = PhysicalPositions()
                            near = NearPickup()
                        if parsed[1] == "NEAR":
                            near.reset()
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
            elif mode == "NEAR":
                found = near.detect(img, target, table)
                if near.point is not None:
                    cv2.circle(img, tuple(int(round(x)) for x in near.point), 8, (0, 255, 255), 1)
            elif mode == "PICK":
                found = None if positions.raw else positions.detect(img, target, table)
            elif mode == "MAT":
                # All candidates, including the two/one remaining after pickup.
                objects = detect_materials(img, False, select_three=False)
                found = material_center(objects, target, request[3:] if len(request) == 5 else None)
            elif mode == "TABLE":
                found = table.detect(img, target, draw=True)
                cv2.putText(img, table.state, (5, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
            elif mode == "OCCUPIED":
                objects = detect_materials(img, False, select_three=False)
                found = occupied_middle_center(objects, target)
            else:
                centers = ring_color_circle_position(img)
                if centers is not None:
                    found = centers[2 * (target - 1):2 * target]
            # TABLE already requires five stable frame centers, not three extra batches.
            result = found if mode in ("TABLE", "NEAR") else stable.update(found)
            if result is not None and mode in ("TABLE", "OCCUPIED"):
                points = (table.detector.last_points if mode == "TABLE" else
                          {obj["color"]: obj["center"] for obj in objects})
                # Capture only at the aligned station, before the first pickup.
                if not (near.bind(points, target, geometry=table.detector.last_geometry,
                                  areas={c: obj['area'] for c, obj in table.detector.selected_candidates.items()})
                        if mode == "TABLE" else positions.bind(points, target)):
                    result = None
            if result is None:
                serial.write(("VISION,%d,NONE\n" % req).encode())
            elif mode in ("PICK", "NEAR"):
                serial.write(("VISION,%d,%s,%d,%s\n" % (req, mode, target, result)).encode())
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
            preview.draw(img)
        if touch is not None and (request is None or request[1] == "IDLE"):
            _, _, pressed = touch.read()
            if not pressed:
                press_start = None
            elif press_start is None:
                press_start = time.monotonic()
            elif time.monotonic() - press_start >= 1.5:
                calibrate_table_colors(cam, disp, touch, table,
                    ("light_blue", "blue", "yellow", "green", "black", "red"))
                table.reset()
                preview.reset()
                stable = StableTarget()
                press_start = None
        else:
            press_start = None
        disp.show(image.cv2image(img, bgr=True, copy=True))


if __name__ == "__main__":
    main()
