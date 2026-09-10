"""N10 point-cloud monitor for the ESP32-C3 Wi-Fi bridge.

Connect the PC to Wi-Fi ``N10-LiDAR`` (password: ``lidar12345``), then run:
    python lidar_monitor.py

The script uses only the Python standard library.  Press Escape or close the
window to stop it.
"""

from __future__ import annotations

import argparse
import math
import socket
import struct
import time
import tkinter as tk
from dataclasses import dataclass, field


FRAME_SIZE = 58
HEADER_FORMAT = "<4sIHHH"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
DEFAULT_ESP_IP = "192.168.4.1"
UDP_PORT = 3333


def valid_n10_frame(frame: bytes) -> bool:
    return (
        len(frame) == FRAME_SIZE
        and frame[:3] == b"\xA5\x5A\x3A"
        and (sum(frame[:-1]) & 0xFF) == frame[-1]
    )


def decode_points(frame: bytes) -> list[tuple[float, int, int]]:
    """Return (angle_degrees, distance_mm, intensity) for one N10 packet."""
    start = int.from_bytes(frame[5:7], "big")
    stop = int.from_bytes(frame[55:57], "big")
    if stop < start:
        stop += 36000

    points: list[tuple[float, int, int]] = []
    for index in range(16):
        offset = 7 + index * 3
        distance = int.from_bytes(frame[offset : offset + 2], "big")
        intensity = frame[offset + 2]
        angle = ((start + (stop - start) * index / 15.0) % 36000) / 100.0
        if distance:
            points.append((angle, distance, intensity))
    return points


@dataclass
class PendingScan:
    chunk_count: int
    chunks: dict[int, bytes] = field(default_factory=dict)
    received_at: float = field(default_factory=time.monotonic)


class N10Monitor:
    def __init__(self, esp_ip: str, display_range_mm: int) -> None:
        self.esp_ip = esp_ip
        self.display_range_mm = display_range_mm
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("0.0.0.0", UDP_PORT))
        self.socket.setblocking(False)
        self.pending: dict[int, PendingScan] = {}
        self.latest_points: list[tuple[float, int, int]] = []
        self.scan_count = 0
        self.last_subscribe = 0.0

        self.root = tk.Tk()
        self.root.title("N10 LiDAR 点云监控")
        self.root.geometry("840x900")
        self.canvas = tk.Canvas(self.root, background="#10151c", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.status = tk.StringVar(value="正在订阅 ESP32-C3…")
        tk.Label(self.root, textvariable=self.status, anchor="w", padx=10).pack(fill=tk.X)
        self.root.bind("<Escape>", lambda _event: self.root.destroy())

    def subscribe(self) -> None:
        now = time.monotonic()
        if now - self.last_subscribe >= 2.0:
            self.socket.sendto(b"N10_SUBSCRIBE", (self.esp_ip, UDP_PORT))
            self.last_subscribe = now

    def receive_packets(self) -> None:
        while True:
            try:
                packet, _source = self.socket.recvfrom(2048)
            except BlockingIOError:
                return
            if packet == b"N10_OK":
                continue
            if len(packet) < HEADER_SIZE:
                continue

            magic, scan_id, chunk_index, chunk_count, frame_count = struct.unpack_from(
                HEADER_FORMAT, packet
            )
            payload = packet[HEADER_SIZE:]
            if (
                magic != b"N10S"
                or chunk_count == 0
                or chunk_index >= chunk_count
                or len(payload) != frame_count * FRAME_SIZE
            ):
                continue

            scan = self.pending.get(scan_id)
            if scan is None or scan.chunk_count != chunk_count:
                scan = PendingScan(chunk_count)
                self.pending[scan_id] = scan
            scan.chunks[chunk_index] = payload
            if len(scan.chunks) == scan.chunk_count:
                frames = b"".join(scan.chunks[index] for index in range(scan.chunk_count))
                self.latest_points = [
                    point
                    for offset in range(0, len(frames), FRAME_SIZE)
                    if valid_n10_frame(frames[offset : offset + FRAME_SIZE])
                    for point in decode_points(frames[offset : offset + FRAME_SIZE])
                ]
                self.scan_count += 1
                del self.pending[scan_id]

    def draw(self) -> None:
        self.canvas.delete("all")
        width = max(self.canvas.winfo_width(), 1)
        height = max(self.canvas.winfo_height(), 1)
        center_x, center_y = width / 2, height / 2
        radius = min(width, height) * 0.44

        for fraction in (0.25, 0.5, 0.75, 1.0):
            r = radius * fraction
            self.canvas.create_oval(center_x - r, center_y - r, center_x + r, center_y + r,
                                    outline="#334155")
            self.canvas.create_text(center_x + 4, center_y - r - 8,
                                    text=f"{int(self.display_range_mm * fraction)} mm",
                                    fill="#94a3b8", anchor="w")
        self.canvas.create_line(center_x, center_y - radius, center_x, center_y + radius,
                                fill="#334155")
        self.canvas.create_line(center_x - radius, center_y, center_x + radius, center_y,
                                fill="#334155")
        self.canvas.create_oval(center_x - 4, center_y - 4, center_x + 4, center_y + 4,
                                fill="#f97316", outline="")

        visible_points = 0
        for angle, distance, intensity in self.latest_points:
            if distance > self.display_range_mm:
                continue
            radians = math.radians(angle)
            point_radius = distance / self.display_range_mm * radius
            x = center_x + point_radius * math.sin(radians)
            y = center_y - point_radius * math.cos(radians)
            brightness = 90 + min(165, intensity * 2)
            color = f"#00{brightness:02x}{min(255, brightness + 35):02x}"
            self.canvas.create_rectangle(x, y, x + 2, y + 2, fill=color, outline="")
            visible_points += 1

        self.status.set(
            f"ESP32 {self.esp_ip}:{UDP_PORT}  |  扫描 {self.scan_count} 圈  |  "
            f"显示 {visible_points}/{len(self.latest_points)} 点  |  量程 {self.display_range_mm} mm"
        )

    def tick(self) -> None:
        self.subscribe()
        self.receive_packets()
        stale_ids = [
            scan_id
            for scan_id, scan in self.pending.items()
            if time.monotonic() - scan.received_at > 1.0
        ]
        for scan_id in stale_ids:
            del self.pending[scan_id]
        self.draw()
        self.root.after(30, self.tick)

    def run(self) -> None:
        self.tick()
        self.root.mainloop()


def main() -> None:
    parser = argparse.ArgumentParser(description="Monitor N10 point clouds sent by ESP32-C3")
    parser.add_argument("--esp", default=DEFAULT_ESP_IP, help="ESP32-C3 IP address")
    parser.add_argument("--range-mm", type=int, default=6000, help="display radius in millimetres")
    arguments = parser.parse_args()
    if arguments.range_mm <= 0:
        parser.error("--range-mm must be positive")
    N10Monitor(arguments.esp, arguments.range_mm).run()


if __name__ == "__main__":
    main()
