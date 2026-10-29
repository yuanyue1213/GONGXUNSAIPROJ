"""MaixCAM2 UART2回环测试。单独在MaixVision打开、运行本文件。

先断开STM32到相机的TX/RX线，再用跳线连接相机B0和B1。
不打开摄像头，不运行比赛；只需本文件，无需导入main.py。
"""
import time
import cv2
import numpy as np
from maix import uart, pinmap, err, display, image, app

err.check_raise(pinmap.set_pin_function("B0", "UART2_TX"), "B0 UART2_TX")
err.check_raise(pinmap.set_pin_function("B1", "UART2_RX"), "B1 UART2_RX")
serial = uart.UART("/dev/ttyS2", 115200)
screen = display.Display()
serial.read()  # 丢弃测试开始前的残留数据。
message = b"CAM,1,QR\n"
pending = bytearray()
sent = good = bad = total = 0
streak = 0
last_raw = b""
next_send = 0.
try:
    while not app.need_exit():
        now = time.monotonic()
        if now >= next_send:
            written = serial.write(message)
            if written != len(message):
                raise RuntimeError("UART TX short write: %s" % written)
            sent += 1
            next_send = now + 1.
        received = serial.read()
        if received:
            total += len(received)
            last_raw = bytes(received[-24:])
            print("UART2 RX:", repr(received))
            for byte in received:
                if byte == 10:
                    if pending == message[:-1]:
                        good += 1
                        streak += 1
                    elif pending:
                        bad += 1
                        streak = 0
                    pending.clear()
                else:
                    pending.append(byte)
                    if len(pending) > 128:
                        bad += 1
                        streak = 0
                        pending.clear()
        frame = np.zeros((320, 512, 3), np.uint8)
        lines = ["MAIXCAM2 UART2 LOOPBACK / 115200 8N1",
                 "Disconnect STM32. Connect B0 TX -> B1 RX.",
                 "TX:%d  RX BYTES:%d  OK:%d  BAD:%d" % (sent, total, good, bad),
                 "LOOPBACK PASS" if streak >= 3 else "WAIT / CHECK RAW",
                 "RAW: " + repr(last_raw)]
        for i, line in enumerate(lines):
            cv2.putText(frame, line, (8, 30 + 48*i), cv2.FONT_HERSHEY_SIMPLEX,
                        .43, (0, 255, 0) if i == 3 and streak >= 3 else (0, 255, 255), 1)
        screen.show(image.cv2image(frame, bgr=True, copy=True))
        time.sleep(.02)
finally:
    serial.close()
