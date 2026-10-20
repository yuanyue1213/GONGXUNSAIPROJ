import sys
from pathlib import Path
import cv2
import numpy as np

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / "camera/MaixVision"))
from mission_vision import parse_request, valid_code, StableTarget, COLORS, material_center
from material_circle import detect_materials

assert valid_code("156+123+516+231")
assert not valid_code("123+321")
assert not valid_code("156+123+234+231")
assert not valid_code("116+123+611+231")
assert parse_request("CAM,1,QR") == (1, "QR", 0)
assert parse_request("CAM,8,IDLE") == (8, "IDLE", 0)
assert parse_request("CAM,2,MAT,5,256.4,160.3") == (2,"MAT",5,256.4,160.3)
assert parse_request("CAM,2,MAT,5,nan,160.3") is None
assert parse_request("CAM,2,MAT,5,256.45,160.3") is None
objects = [{"color":"blue","center":(250,160)}, {"color":"blue","center":(400,270)}]
assert material_center(objects,3,(256,160)) == (250,160)
assert material_center(objects,3) is None
assert material_center([objects[0],{"color":"blue","center":(265,160)}],3,(256,160)) is None
assert parse_request("CAM,4294967295,MAT,6") == (4294967295, "MAT", 6)
assert all(parse_request(s) is None for s in ("CAM,0,QR", "CAM,1,MAT,7", "CAM,1,RING,4", "CAM,1,QR,0", "CAM,4294967296,QR"))
stable = StableTarget()
assert stable.update((110.2, 160.7)) is None
assert stable.update((110.3, 160.8)) is None
assert np.allclose(stable.update((110.4, 160.9)), (110.3, 160.8))
assert stable.update((120, 160)) is None
assert stable.update(None) is None
stable = StableTarget()
assert stable.update("156+123+516+231") is None
assert stable.update("156+123+516+231") is None
assert stable.update("156+123+516+231") == "156+123+516+231"

palette = [(1,(30,30,190)), (2,(20,210,230)), (3,(180,70,30)),
           (4,(35,190,35)), (5,(45,45,45)), (6,(230,190,130))]
for count in (1,2,3,6):
    img = np.full((320,512,3),220,np.uint8)
    for index,(_,color) in enumerate(palette[:count]):
        cv2.circle(img,(80+176*(index%3),90+140*(index//3)),25,color,-1,cv2.LINE_AA)
    found = detect_materials(img,False,select_three=False)
    assert len(found)==count,(count,found)
    assert sorted(COLORS[obj["color"]] for obj in found)==sorted(code for code,_ in palette[:count])
print("PASS: request framing, six-color numbering, four-group task validation, stability reset, one/two/three/six objects")
