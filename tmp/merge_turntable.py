"""Merge the supplied detector and calibration UI into standalone vision.py."""
from pathlib import Path
import ast
import textwrap

root = Path(__file__).resolve().parents[1]
path = root / 'camera/MaixVision/vision.py'
source = path.read_text(encoding='utf-8-sig')
detector = (root / '识别/turntable_detector.py').read_text(encoding='utf-8-sig')
original_main = (root / '识别/main.py').read_text(encoding='utf-8-sig')
assert 'class TurntableCenterDetector:' not in source
wrapper = '''

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
        return result


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
'''
names = {'read_touch', 'point_in_rect', 'wait_release', 'draw_button', 'show',
         'get_lab_range', 'make_mask', 'sample_lab_center', 'draw_adjust_ui',
         'adjust_lab_tol', 'calibrate_colors'}
tree = ast.parse(original_main)
for node in tree.body:
    if isinstance(node, ast.FunctionDef) and node.name in names:
        function = ast.get_source_segment(original_main, node).replace('DEFAULT_CONFIG_PATH', 'table.path')
        wrapper += '\n' + textwrap.indent(function, '    ') + '\n'
wrapper += '\n    calibrate_colors(colors)\n\n'
marker = '# ===== STM32协议分发、二维码及主循环 ====='
assert source.count(marker) == 1
source = source.replace(marker, '# ===== 三物块圆台中心与持久化 LAB 标定（来自识别目录） =====\n' + detector + wrapper + '\n' + marker)
ast.parse(source)
path.write_text(source, encoding='utf-8', newline='\n')
print('Merged detector, persistence, and original touch calibration into', path)
