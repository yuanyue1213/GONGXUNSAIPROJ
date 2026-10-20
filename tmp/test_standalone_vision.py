"""Single-file detector parity and protocol-driven main-loop tests."""
from pathlib import Path
import importlib.util
import ast
import sys
import types
import numpy as np
import cv2

root = Path(__file__).resolve().parents[1]
folder = root / "camera/MaixVision"
sys.path.insert(0, str(folder))
import vision
import material_circle
import circle_v2

tree = ast.parse((folder / "vision.py").read_text(encoding="utf-8"))
assert not any(isinstance(n, ast.ImportFrom) and n.module in ("material_circle", "circle_v2", "mission_vision") for n in ast.walk(tree))
# Run the six-color/single-object/protocol suite against the single-file module.
suite = (root / "tmp/test_mission_vision.py").read_text(encoding="utf-8")
exec(compile(suite.replace("from mission_vision import", "from vision import")
             .replace("from material_circle import", "from vision import"), str(root / "tmp/test_mission_vision.py"), "exec"),
     {"__file__": str(root / "tmp/test_mission_vision.py")})

img = np.full((320,512,3),230,np.uint8)
for x in (105,255,405):
    for radius in (15,22,29,36,43):
        cv2.circle(img,(x,160),radius,(25,25,25),2,cv2.LINE_AA)
single = vision.ring_color_circle_position(img.copy())
original = circle_v2.color_circle_position(img.copy())
assert single is not None and np.allclose(single,original)
cv2.imwrite(str(root / "tmp/standalone_ring_preview.png"),img)

# Verify fragmented serial input, duplicate request handling and actual switch
# dispatch without requiring camera hardware or executing more than one mode.
chunks = [b"", b"CA", b"M,10,QR\n", b"", b"",
          b"CAM,20,MAT,5,256.0,160.0\n", b"CAM,20,MAT,5,256.0,160.0\n", b"",
          b"CAM,30,RING,2\n", b"", b"", b""]
chunks += [b"CAM,40,IDLE\n", b"", b""]
outputs=[]; calls=[]; index=[0]
class Frame:
    def find_qrcodes(self):
        calls.append("QR")
        return [types.SimpleNamespace(payload=lambda: "156+123+516+231")]
class Serial:
    def read(self):
        value=chunks[index[0]]; index[0]+=1; return value
    def write(self, data): outputs.append(data.decode())
fake = types.ModuleType("maix")
fake.app=types.SimpleNamespace(need_exit=lambda: index[0]>=len(chunks))
fake.camera=types.SimpleNamespace(Camera=lambda *_: types.SimpleNamespace(read=lambda: Frame()))
fake.display=types.SimpleNamespace(Display=lambda: types.SimpleNamespace(show=lambda _: None))
fake.uart=types.SimpleNamespace(UART=lambda *_: Serial())
fake.image=types.SimpleNamespace(image2cv=lambda *args,**kwargs: np.zeros((320,512,3),np.uint8),
                                cv2image=lambda *args,**kwargs: args[0])
sys.modules["maix"]=fake
detect=vision.detect_materials; ring_detect=vision.ring_color_circle_position
def material(*args,**kwargs):
    calls.append("MAT"); return [{"color":"black","center":(113.2,160.7)}]
def rings(*args,**kwargs):
    calls.append("RING"); return (113.2,160.7,256.4,160.3,399.8,160.2)
vision.detect_materials=material; vision.ring_color_circle_position=rings
try:
    vision.main()
finally:
    vision.detect_materials=detect; vision.ring_color_circle_position=ring_detect
    del sys.modules["maix"]
assert calls==["QR"]*3+["MAT"]*3+["RING"]*4,calls
assert "VISION,10,QR,156+123+516+231\n" in outputs
assert "VISION,20,MAT,5,113.2,160.7\n" in outputs
assert "VISION,30,RING,2,256.4,160.3\n" in outputs
assert not any(line.startswith("VISION,40,") for line in outputs)
assert not any("VISION,20,RING" in line or "VISION,30,MAT" in line for line in outputs)
print("PASS: standalone six-color detection, subpixel ring parity, idle/split-frame/duplicate request/QR-MAT-RING switching")
