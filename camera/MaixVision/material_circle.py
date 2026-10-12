"""六色锥形物料底座圆定位（落在纸面上的大圆的图像拟合中心）。

MaixCAM: 512x320, ttyS2/115200, RINGS,x1,y1,x2,y2,x3,y3\n。
仅恰好找到三个可靠底座且连续稳定时发送；六色是可识别种类。
可在电脑上导入 detect_materials() 调试，不依赖 maix。
"""
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


def detect_materials(img, draw=True):
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
    if len(unique)>=3:
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


def main():
    from maix import camera,display,app,image,uart
    cam=camera.Camera(WIDTH,HEIGHT)
    disp=display.Display()
    serial=uart.UART("/dev/ttyS2",115200)
    stable=StableMaterials()
    tracker=MaterialTracker()
    last_log=0.0
    while not app.need_exit():
        img=image.image2cv(cam.read(),ensure_bgr=True,copy=True)
        start=time.monotonic()
        materials=tracker.update(img)
        detection_ms=(time.monotonic()-start)*1000
        result=stable.update(materials)
        if result is not None:
            serial.write(encode_rings(result))
        now=time.monotonic()
        if now-last_log>=1.0:
            coordinates=None if result is None else ",".join("%.1f"%v for v in result)
            print(tracker.mode,"detect_ms:",round(detection_ms,1),"RINGS(float):",coordinates)
            last_log=now
        cv2.putText(img,"STABLE" if result else "SEARCHING: %d/3"%len(materials),(5,18),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,255),1)
        cv2.putText(img,"%s %.1fms"%(tracker.mode,detection_ms),(5,36),cv2.FONT_HERSHEY_SIMPLEX,0.45,(255,0,255),1)
        disp.show(image.cv2image(img,bgr=True,copy=True))


if __name__=="__main__":
    main()
