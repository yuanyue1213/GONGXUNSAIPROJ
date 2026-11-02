"""OpenCV map overlay and clickable virtual-car controls."""
from functools import lru_cache
import math
from pathlib import Path

from n10_planner import center, vehicle_pose

BUTTONS = (
    ('plan', (20, 850, 185, 890), '路径规划 / PLAN'),
    ('pause', (195, 850, 335, 890), '暂停 / 继续'),
    ('replay', (345, 850, 475, 890), '重新播放'),
    ('top', (485, 850, 670, 890), '起点：右上 ↓'),
    ('bottom', (680, 850, 875, 890), '起点：右下 ↑'),
)


def control_at(x, y):
    for key, (x0, y0, x1, y1), _ in BUTTONS:
        if x0 <= x < x1 and y0 <= y < y1:
            return key
    return None


@lru_cache(maxsize=1)
def ui_font():
    from PIL import ImageFont
    # Windows target; Noto also supports Chinese on Linux.
    for path in ('C:/Windows/Fonts/msyh.ttc',
                 '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'):
        if Path(path).is_file():
            return ImageFont.truetype(path, 17)
    return ImageFont.load_default(size=17)


def draw_toolbar(image, player, start_zone, hardware=False):
    import cv2
    import numpy as np
    from PIL import Image, ImageDraw
    for key, (x0, y0, x1, y1), _ in BUTTONS:
        selected = (key == 'top' and start_zone == 2) or (key == 'bottom' and start_zone == 1)
        cv2.rectangle(image, (x0, y0), (x1-1, y1-1), (80, 95, 45) if selected else (65, 58, 48), -1)
        cv2.rectangle(image, (x0, y0), (x1-1, y1-1), (160, 160, 160), 1)
    rgb = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(rgb)
    for key, (x0, y0, x1, y1), label in BUTTONS:
        if hardware and key in ('plan','pause','replay'):
            label = {'plan':'自动采样规划','pause':'App一键启动','replay':'执行状态回传'}[key]
        if key == 'pause' and not hardware:
            label = '继续 / RESUME' if player and player.paused else '暂停 / PAUSE'
        draw.text(((x0+x1)/2, (y0+y1)/2), label, font=ui_font(), fill=(240, 240, 240), anchor='mm')
    image[:] = cv2.cvtColor(np.asarray(rgb), cv2.COLOR_RGB2BGR)


def draw_route(image, player, start_zone, map_pixel, pose=None):
    import cv2
    import numpy as np
    if player and player.plan:
        for action in player.plan.actions:
            if action.kind in ('F', 'B'):
                cv2.line(image, map_pixel(*center(*action.start[:2])),
                         map_pixel(*center(*action.end[:2])), (150, 120, 65), 2, cv2.LINE_AA)
        if not player.finished:
            action = player.plan.actions[player.index]
            if action.kind in ('F', 'B'):
                cv2.line(image, map_pixel(*center(*action.start[:2])),
                         map_pixel(*center(*action.end[:2])), (40, 210, 255), 3, cv2.LINE_AA)
    pose = vehicle_pose(player, start_zone) if pose is None else pose
    for label, cell in [('QR', (3, 5)), ('RAW', (1, 3)), ('COARSE', (5, 3)), ('TEMP', (3, 1))]:
        px, py = map_pixel(*center(*cell))
        cv2.putText(image, label, (px-25, py-30), 0, .5, (230, 210, 130), 1, cv2.LINE_AA)
    # Position comes from interpolated cell centers; only the heading rotates.
    cx, cy = map_pixel(*pose[:2])
    angle = math.radians(pose[2])
    forward = np.array([math.sin(angle), -math.cos(angle)])
    right = np.array([math.cos(angle), math.sin(angle)])
    middle = np.array([cx, cy])
    corners = np.array([middle+forward*a+right*b for a, b in [(17, 12), (17, -12), (-17, -12), (-17, 12)]], dtype=np.int32)
    cv2.fillConvexPoly(image, corners, (70, 220, 100), cv2.LINE_AA)
    cv2.polylines(image, [corners], True, (230, 255, 230), 1, cv2.LINE_AA)
    # Cyan side is the car's right; concentric center marks its rigidly mounted radar.
    right_a, right_b = middle-forward*12+right*13, middle+forward*12+right*13
    cv2.line(image, tuple(right_a.astype(int)), tuple(right_b.astype(int)), (255, 220, 70), 3, cv2.LINE_AA)
    cv2.circle(image, (cx, cy), 5, (30, 30, 30), -1, cv2.LINE_AA)
    cv2.circle(image, (cx, cy), 5, (255, 230, 80), 1, cv2.LINE_AA)
    # Bright arrow marks the front, including while the vehicle reverses.
    nose = middle+forward*25
    cv2.arrowedLine(image, (cx, cy), tuple(nose.astype(int)), (50, 60, 250), 3, cv2.LINE_AA, tipLength=.4)
