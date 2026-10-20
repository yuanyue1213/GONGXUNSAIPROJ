"""Bundle the reviewed detectors without runtime imports of local modules."""
import ast
from pathlib import Path
import re

root = Path(__file__).resolve().parents[1]
folder = root / "camera/MaixVision"


def detector(filename):
    text = (folder / filename).read_text(encoding="utf-8")
    tree = ast.parse(text)
    main = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "main")
    start = next(node.lineno for node in tree.body if isinstance(node, (ast.Import, ast.ImportFrom)))
    return "\n".join(text.splitlines()[start - 1:main.lineno - 1]).rstrip() + "\n", tree


materials, _ = detector("material_circle.py")
rings, tree = detector("circle_v2.py")
# Separate ring parameters/functions from the material detector's globals.
names = set()
for node in tree.body:
    if isinstance(node, (ast.FunctionDef, ast.ClassDef)) and node.name != "main":
        names.add(node.name)
    elif isinstance(node, ast.Assign):
        for target in node.targets:
            names.update(child.id for child in ast.walk(target) if isinstance(child, ast.Name))
renames = {name: ("RING_" + name if name.isupper() else "ring_" + name) for name in names}
rings = re.sub(r"\b(" + "|".join(map(re.escape, sorted(names, key=len, reverse=True))) + r")\b",
               lambda match: renames[match.group()], rings)
mission = (folder / "mission_vision.py").read_text(encoding="utf-8")
mission_tree = ast.parse(mission)
start = next(node.lineno for node in mission_tree.body if isinstance(node, ast.Assign))
mission = "\n".join(mission.splitlines()[start - 1:]) + "\n"
mission = re.sub(r"\bcolor_circle_position\b", "ring_color_circle_position", mission)
header = '''"""一体化视觉：二维码、六色物料底座圆心、黑色同心圆环。

只上传本文件即可；运行时只依赖设备已有的 maix、cv2、numpy。
串口 /dev/ttyS2，115200；等待STM32的CAM请求后执行对应任务。
CAM,101,QR -> VISION,101,QR,156+123+516+231
CAM,102,MAT,5,256.0,160.0 -> VISION,102,MAT,5,113.2,160.7
CAM,103,RING,2 -> VISION,103,RING,2,256.4,160.3
漏检/未稳定：VISION,请求编号,NONE；圆环目标1/2/3为画面左/中/右。
同一请求持续识别；换请求重置稳定窗口。保留亚像素拟合及一位小数输出。
"""

'''
result = header + "# ===== 六色物料检测 =====\n" + materials + \
    "\n# ===== 黑色同心圆环检测（独立参数） =====\n" + rings + \
    "\n# ===== STM32协议分发、二维码及主循环 =====\n" + mission
ast.parse(result)
(folder / "vision.py").write_text(result, encoding="utf-8")
print("Wrote standalone vision.py (%d lines)" % len(result.splitlines()))
