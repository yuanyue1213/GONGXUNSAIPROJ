"""一体化视觉：二维码、六色物料底座圆心、黑色同心圆环。

只上传本文件即可；运行时只依赖设备已有的 maix、cv2、numpy。
串口 /dev/ttyS2，115200；等待STM32的CAM请求后执行对应任务。
CAM,101,QR -> VISION,101,QR,156+123+516+231
CAM,102,MAT,5,256.0,160.0 -> VISION,102,MAT,5,113.2,160.7
CAM,103,RING,2 -> VISION,103,RING,2,256.4,160.3
漏检/未稳定：VISION,请求编号,NONE；圆环目标1/2/3为画面左/中/右。
同一请求持续识别；换请求重置稳定窗口。保留亚像素拟合及一位小数输出。
"""

# ===== 六色物料检测 =====
import cv2
import numpy as np
from itertools import combinations
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
        if fields[2] in ("MAT", "RING") and 1 <= target <= (6 if fields[2] == "MAT" else 3):
            result = int(fields[1]), fields[2], target
            if len(fields) == 4:
                return result
            if fields[2] != "MAT":
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
            elif mode == "MAT":
                # All candidates, including the two/one remaining after pickup.
                objects = detect_materials(img, False, select_three=False)
                found = material_center(objects, target, request[3:] if len(request) == 5 else None)
            else:
                centers = ring_color_circle_position(img)
                if centers is not None:
                    found = centers[2 * (target - 1):2 * target]
            result = stable.update(found)
            if result is None:
                serial.write(("VISION,%d,NONE\n" % req).encode())
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
            cv2.putText(img, "WAIT CAM REQUEST", (5, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
        disp.show(image.cv2image(img, bgr=True, copy=True))


if __name__ == "__main__":
    main()
