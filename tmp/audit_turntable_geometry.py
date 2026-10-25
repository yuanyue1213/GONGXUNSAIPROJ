import importlib.util, math, json
from pathlib import Path
import cv2
import numpy as np
spec=importlib.util.spec_from_file_location('vision_audit',Path('camera/MaixVision/vision.py'))
v=importlib.util.module_from_spec(spec);spec.loader.exec_module(v)
colors=('red','black','light_blue')
bgr={'red':(30,30,220),'black':(15,15,15),'light_blue':(220,220,30)}
centers={c:cv2.cvtColor(np.array([[bgr[c]]],np.float32)/255,cv2.COLOR_BGR2LAB)[0,0] for c in colors}
tols={c:np.array((8,8,8),np.float32) for c in colors}
def ellipse_frame(angle=0, distractor=False):
 img=np.full((320,512,3),110,np.uint8)
 for i,c in enumerate(colors):
  a=angle+i*2*math.pi/3
  xy=(round(256+100*math.cos(a)),round(160+80*math.sin(a)))
  cv2.circle(img,xy,13,bgr[c],-1)
 if distractor:cv2.circle(img,(430,250),28,bgr['black'],-1)
 return img
obj=v.TurntableCenterDetector(centers,tols,colors=colors)
for i in range(5): result=obj.detect(ellipse_frame())
near=v.NearPickup(); bound=near.bind(obj.last_points,156)
real_bottom={'red':(256.,240.),'black':(169.4,120.),'light_blue':(342.6,120.)}
responses=[near.update(real_bottom,1) for _ in range(8)]
print(json.dumps({'case':'mild_ellipse','TABLE_center':result,'state':obj.last_state,
 'NEAR_bind':bound,'predicted_near':near.point.tolist(),'actual_bottom':[256,240],
 'distance_error_px':float(np.linalg.norm(near.point-(256,240))),
 'acceptance_radius_px':min(8.,max(3.,near.radius*.12)),
 'stationary_actual_near_responses':responses},ensure_ascii=False))
obj2=v.TurntableCenterDetector(centers,tols,colors=colors)
obj2.detect(ellipse_frame(distractor=True))
print(json.dumps({'case':'outside_black_distractor','selected_black':obj2.last_points.get('black'),
 'actual_black':[206,229],'state':obj2.last_state},ensure_ascii=False))
print('labels are whole-mask centroids; no fitted block-base circle in _center_for_color')
