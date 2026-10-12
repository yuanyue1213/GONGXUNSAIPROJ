import importlib.util
from pathlib import Path
import cv2
import numpy as np

root=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location("rings",root/"camera/MaixVision/circle_v2.py")
rings=importlib.util.module_from_spec(spec)
spec.loader.exec_module(rings)
coarse=[]; refined=[]
for phase in (0.1,0.3,0.6,0.8):
    scale=16
    yy,xx=np.indices((96*scale,96*scale),dtype=float)
    center=np.array([48+phase,47+phase*0.7])
    radius=np.hypot((xx+0.5)/scale-0.5-center[0],(yy+0.5)/scale-0.5-center[1])
    high=np.uint8(np.where((radius>23)&(radius<26),30,220))
    gray=cv2.resize(high,(96,96),interpolation=cv2.INTER_AREA)
    gray=cv2.GaussianBlur(gray,(3,3),0.65)
    mask=np.uint8(gray<120)*255
    contour=max(cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)[-2],key=cv2.contourArea)
    initial=cv2.fitEllipse(contour)
    fine=rings.refine_ring_edge(gray,initial,contour)
    coarse.append(np.linalg.norm(np.array(initial[0])-center))
    refined.append(np.linalg.norm(np.array(fine[0])-center))
    assert rings.refine_ring_edge(np.full_like(gray,128),initial,contour)==initial
assert np.mean(refined)<np.mean(coarse),(coarse,refined)
assert max(refined)<0.2,refined
img=cv2.imread("C:/Users/ASUS/AppData/Local/Temp/codex-clipboard-9a5cee6e-437e-4f46-b710-f5130cc0fc9f.png")
results=[rings.color_circle_position(img.copy()) for _ in range(4)]
assert all(result is not None for result in results)
assert results[0][0]<results[0][2]<results[0][4]
assert np.allclose(results,results[0])
assert any(abs(v-round(v))>0.01 for v in results[0])
stable=rings.StableCenters()
for result in results:
    filtered=stable.update(result)
assert filtered is not None
assert len(("RINGS,"+",".join("%.1f"%v for v in filtered)+"\n").split(","))==7
assert stable.update(None) is None
assert rings.color_circle_position(np.full_like(img,255)) is None
cv2.imwrite(str(root/"camera/MaixVision/circle_v2_subpixel_preview.jpg"),img)
print("Ring subpixel tests passed; mean synthetic error %.4f -> %.4f px; screenshot %s"%
      (np.mean(coarse),np.mean(refined),tuple(round(v,3) for v in results[0])))
