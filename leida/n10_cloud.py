"""N10点云和距离查看器：连接ESP32热点后运行 python n10_cloud.py。"""
import argparse
import math
import socket
import time

import cv2
import numpy as np

# 只复用现有UDP解码和短时点云缓存，不创建导航连接或发送运动指令。
from n10_wifi import Cloud, decode_packet

TITLE = 'N10 Point Cloud and Distance'
CENTER = (450, 470)
RADIUS = 370


def point_pixel(angle, distance, range_mm):
    angle = math.radians(angle)
    scale = RADIUS / range_mm
    # 雷达原始坐标：0度朝上，90度朝左；不使用地图、车位或首扫修正。
    return (round(CENTER[0] - distance * math.sin(angle) * scale),
            round(CENTER[1] - distance * math.cos(angle) * scale))


def render(points, range_mm, cursor=None):
    image = np.full((930, 900, 3), (24, 24, 24), np.uint8)
    font = cv2.FONT_HERSHEY_SIMPLEX
    for i in range(1, 5):
        radius = round(RADIUS * i / 4)
        cv2.circle(image, CENTER, radius, (65, 65, 65), 1)
        cv2.putText(image, f'{range_mm*i/4:g} mm',
                    (CENTER[0]+8, CENTER[1]-radius+18), font, .45, (160, 160, 160), 1)
    cv2.line(image, (80, CENTER[1]), (820, CENTER[1]), (65, 65, 65))
    cv2.line(image, (CENTER[0], 100), (CENTER[0], 840), (65, 65, 65))
    for label, pixel in [('0 deg', (428, 93)), ('90', (45, 475)),
                         ('180', (430, 865)), ('270', (830, 475))]:
        cv2.putText(image, label, pixel, font, .45, (170, 170, 170), 1)
    visible = [(p, point_pixel(p[0], p[1], range_mm)) for p in points if 0 < p[1] <= range_mm]
    for _, pixel in visible:
        cv2.circle(image, pixel, 2, (80, 230, 120), -1)
    cv2.drawMarker(image, CENTER, (255, 255, 255), cv2.MARKER_CROSS, 14, 1)

    selected = None
    if visible:
        selected = min(visible, key=lambda item: item[0][1])
        if cursor is not None:
            hovered = min(visible, key=lambda item: math.dist(item[1], cursor))
            if math.dist(hovered[1], cursor) <= 15:
                selected = hovered
    if selected is not None:
        (angle, distance, _, _), pixel = selected
        cv2.line(image, CENTER, pixel, (100, 150, 200), 1)
        cv2.circle(image, pixel, 6, (0, 200, 255), 2)
        cv2.putText(image, f'{distance:g} mm',
                    (min(pixel[0]+10, 780), max(pixel[1]-10, 105)), font, .5, (0, 220, 255), 1)
        text = f'Distance: {distance:g} mm    Angle: {angle:.1f} deg'
    else:
        text = 'No points in range' if points else 'Waiting for radar data...'
    cv2.putText(image, text, (25, 35), font, .65, (0, 220, 255), 1)
    cv2.putText(image, f'Range: {range_mm:g} mm    Points: {len(points)}',
                (25, 65), font, .5, (190, 190, 190), 1)
    cv2.putText(image, 'Hover a point: distance | Default: nearest visible point',
                (25, 900), font, .5, (190, 190, 190), 1)
    cv2.putText(image, '+/-: zoom    Q / Esc: exit', (25, 923), font, .45, (160, 160, 160), 1)
    return image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--esp', default='192.168.4.1')
    parser.add_argument('--port', type=int, default=3334)
    parser.add_argument('--range-mm', type=int, default=3000)
    parser.add_argument('--max-age', type=float, default=.35)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535 or args.range_mm <= 0 or not math.isfinite(args.max_age) or args.max_age <= 0:
        parser.error('port must be 1..65535; range-mm and max-age must be positive')
    target = (socket.gethostbyname(args.esp), args.port)
    cloud = Cloud(args.max_age)
    cursor = [None]
    last_subscribe = -float('inf')
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(('', 0))
        sock.setblocking(False)
        if hasattr(socket, 'SIO_UDP_CONNRESET'):
            sock.ioctl(socket.SIO_UDP_CONNRESET, False)
        cv2.namedWindow(TITLE, cv2.WINDOW_AUTOSIZE)
        def mouse(event, x, y, flags, param):
            cursor[0] = (x, y)
        cv2.setMouseCallback(TITLE, mouse)
        try:
            while True:
                now = time.monotonic()
                if now-last_subscribe >= 1:
                    sock.sendto(b'N10_SUB', target)
                    last_subscribe = now
                for _ in range(512):
                    try:
                        packet, source = sock.recvfrom(2048)
                    except BlockingIOError:
                        break
                    if source != target:
                        continue
                    try:
                        cloud.accept(decode_packet(packet), now)
                    except ValueError:
                        continue
                cv2.imshow(TITLE, render(cloud.points(now), args.range_mm, cursor[0]))
                key = cv2.waitKey(16) & 255
                if key in (27, ord('q')) or cv2.getWindowProperty(TITLE, cv2.WND_PROP_VISIBLE) < 1:
                    break
                if key in (ord('+'), ord('=')):
                    args.range_mm = max(100, args.range_mm//2)
                elif key == ord('-'):
                    args.range_mm = min(60000, args.range_mm*2)
        finally:
            cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        main()
    except (KeyboardInterrupt, OSError) as exc:
        print(f'Viewer stopped: {exc}')
