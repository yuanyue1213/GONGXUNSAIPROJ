"""Verify fractional centers against supersampled synthetic ground truth."""
import importlib.util
from pathlib import Path
import cv2
import numpy as np

path=Path(__file__).resolve().parents[1]/"camera/MaixVision/material_circle.py"
spec=importlib.util.spec_from_file_location("material",path)
material=importlib.util.module_from_spec(spec)
spec.loader.exec_module(material)
coarse_errors=[]
fine_errors=[]
for diameter in (16,30,54):
    for phase in (0.1,0.3,0.6,0.8):
        center=np.array([48+phase,47+phase*0.7])
        # Pixel area integration with a known continuous ellipse boundary.
        scale=16
        yy,xx=np.indices((96*scale,96*scale),dtype=float)
        xx=(xx+0.5)/scale-0.5
        yy=(yy+0.5)/scale-0.5
        inside=((xx-center[0])/(diameter/2))**2+((yy-center[1])/(diameter*0.43))**2<=1
        high=np.full((96*scale,96*scale,3),210,np.uint8)
        high[inside]=(35,45,180)
        img=cv2.resize(high,(96,96),interpolation=cv2.INTER_AREA)
        img=cv2.GaussianBlur(img,(3,3),0.65)
        mask=np.uint8(img[:,:,0]<120)*255
        contour=max(cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)[-2],key=cv2.contourArea)
        initial=cv2.fitEllipse(contour)
        refined=material.refine_subpixel(img,initial,contour)
        coarse_errors.append(np.linalg.norm(np.array(initial[0])-center))
        fine_errors.append(np.linalg.norm(np.array(refined[0])-center))
assert np.mean(fine_errors)<np.mean(coarse_errors)*0.8,(coarse_errors,fine_errors)
assert max(fine_errors)<0.2,fine_errors
# A featureless image must retain the validated coarse ellipse.
assert material.refine_subpixel(np.full_like(img,128),initial,contour)==initial
print("Synthetic mean center error: integer contour %.4f px -> subpixel %.4f px; max %.4f px" %
      (np.mean(coarse_errors),np.mean(fine_errors),max(fine_errors)))
