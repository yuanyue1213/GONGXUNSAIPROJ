"""Run with python tmp/test_material_optimization.py; user screenshots are local fixtures."""
import importlib.util
import statistics
import time
from pathlib import Path
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]


def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    result=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


new=module("optimized",ROOT/"camera/MaixVision/material_circle.py")
old=module("baseline",ROOT/"tmp/material_circle_before_optimization.py")
keys=["304ea3c0-0f64-4a47-a65c-80c354106de3",
      "93b62d6e-69f1-4fe7-9a39-c3ebaf18569a",
      "46d1af66-1dd8-4bb5-9420-bd0c5bbd3f1e"]
images=[cv2.imread("C:/Users/ASUS/AppData/Local/Temp/codex-clipboard-"+key+".png") for key in keys]
assert all(img is not None for img in images)
old_times,search_times,track_times=[],[],[]
for index,img in enumerate(images):
    baseline=old.detect_materials(img,False)
    current=new.detect_materials(img,False)
    assert len(current)==3
    assert [m["color"] for m in current]==[m["color"] for m in baseline]
    assert np.max(np.linalg.norm(np.array([m["center"] for m in current])-np.array([m["center"] for m in baseline]),axis=1))<2.1
    for brightness in (0.85,1.1):
        found=new.detect_materials(np.clip(img.astype(float)*brightness,0,255).astype(np.uint8),False)
        assert len(found)==3,(index,brightness)
    for scale in (0.65,0.85):
        small=cv2.resize(img,None,fx=scale,fy=scale)
        canvas=np.full_like(img,190)
        canvas[10:10+small.shape[0],10:10+small.shape[1]]=small
        assert len(new.detect_materials(canvas,False))==3,(index,scale)
    resized=cv2.resize(img,(512,320))
    assert len(new.detect_materials(resized,False))==3
    tracker=new.MaterialTracker()
    first=tracker.update(img,False)
    assert len(first)==3 and tracker.mode=="SEARCH"
    original_centers=np.array([m["center"] for m in first])
    for _ in range(new.REDETECT_INTERVAL):
        followed=tracker.update(img,False)
        assert len(followed)==3 and tracker.mode=="TRACK"
        assert np.max(np.linalg.norm(np.array([m["center"] for m in followed])-original_centers,axis=1))<2.5
    # Bound expensive ellipse fits in the ordinary three-object tracking path.
    fit=cv2.fitEllipse
    calls=[0]
    def counted_fit(points):
        calls[0]+=1
        return fit(points)
    cv2.fitEllipse=counted_fit
    tracker.age=0
    try:
        assert len(tracker.update(img,False))==3 and tracker.mode=="TRACK"
        # Three extra fits refine the selected arcs using subpixel image edges.
        assert calls[0]<=12,calls[0]
    finally:
        cv2.fitEllipse=fit
    tracker.age=new.REDETECT_INTERVAL
    assert len(tracker.update(img,False))==3 and tracker.mode=="AUDIT"
    # Stable scene must not invoke the expensive search during periodic audits.
    original_search=new.detect_materials
    def unexpected_search(*args,**kwargs):
        raise AssertionError("Full search during stable tracking")
    new.detect_materials=unexpected_search
    try:
        for _ in range(new.REDETECT_INTERVAL*3):
            assert len(tracker.update(img,False))==3
    finally:
        new.detect_materials=original_search
    # Entire row moves beyond the previous windows: reacquire in the same frame.
    offset=75 if index==2 else -65
    moved=cv2.warpAffine(img,np.float32([[1,0,0],[0,1,offset]]),(img.shape[1],img.shape[0]),borderValue=(190,190,190))
    assert len(tracker.update(moved,False))==3 and tracker.mode=="SEARCH"
    assert tracker.update(np.full_like(img,255),False)==[]
    assert tracker.previous==[]
    assert len(tracker.update(img,False))==3 and tracker.mode=="SEARCH"
    stable=new.StableMaterials()
    assert stable.update(first) is None
    assert stable.update(first) is None
    assert stable.update(first) is not None
    assert stable.update([]) is None
    preview=img.copy()
    tracker.update(preview)
    cv2.imwrite(str(ROOT/"camera/MaixVision"/f"material_fast_preview_{index}.jpg"),preview)
    # Benchmark at the actual camera resolution, excluding overlays and I/O.
    for _ in range(5):
        start=time.perf_counter();old.detect_materials(resized,False);old_times.append((time.perf_counter()-start)*1000)
        start=time.perf_counter();new.detect_materials(resized,False);search_times.append((time.perf_counter()-start)*1000)
    tracker=new.MaterialTracker();tracker.update(resized,False)
    for _ in range(5):
        start=time.perf_counter();tracker.update(resized,False);track_times.append((time.perf_counter()-start)*1000)
fractional=(110.2,160.7,256.4,160.3,400.8,160.2)
samples=[{"color":color,"center":fractional[2*i:2*i+2]} for i,color in enumerate(("red","yellow","green"))]
stable=new.StableMaterials()
for _ in range(new.STABLE_FRAMES):
    retained=stable.update(samples)
assert np.allclose(retained,fractional)
assert new.encode_rings(retained)==b"RINGS,110.2,160.7,256.4,160.3,400.8,160.2\n"
assert new.encode_rings((1,2,3,4,5,6))==b"RINGS,1.0,2.0,3.0,4.0,5.0,6.0\n"
print("Regression passed: three fixtures, brightness, scale, movement, periodic search, disappearance, reacquisition, UART.")
print("Desktop median detection ms: baseline=%.1f search=%.1f track=%.1f" % tuple(statistics.median(t) for t in (old_times,search_times,track_times)))
