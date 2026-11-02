import math
import socket
import struct
import unittest
from types import SimpleNamespace

from n10_wifi import Cloud, decode_packet, demo_packet, map_cloud_points, polar_points, render


class ProtocolTests(unittest.TestCase):
    def test_fields_and_wrap(self):
        frame = decode_packet(demo_packet(23))
        self.assertEqual(frame.period_us, 4166)
        self.assertEqual((frame.start, frame.end), (345, 0))
        points = list(polar_points(frame))
        self.assertEqual(len(points), 16)
        self.assertEqual(points[-1][0], 0)
        self.assertEqual(points[0][2], 180)

    def test_corruption_and_lengths(self):
        packet = demo_packet(0)
        for bad in (packet[:-1], packet + b'\x00', b'X' + packet[1:]):
            with self.assertRaises(ValueError): decode_packet(bad)
        bad = bytearray(packet)
        bad[20] ^= 1
        with self.assertRaises(ValueError): decode_packet(bad)
        bad = bytearray(packet)
        struct.pack_into('>H', bad, 13, 36000)
        bad[-1] = sum(bad[8:-1]) & 255
        with self.assertRaises(ValueError): decode_packet(bad)

    def test_loss_reorder_rollover_restart_and_expiry(self):
        cloud = Cloud()
        for seq, now in [(0xfffffffe, 1), (0xffffffff, 1.01), (0, 1.02), (3, 1.03)]:
            self.assertTrue(cloud.accept(decode_packet(demo_packet(seq)), now))
        self.assertEqual(cloud.lost, 2)
        self.assertFalse(cloud.accept(decode_packet(demo_packet(2)), 1.04))
        self.assertFalse(cloud.accept(decode_packet(demo_packet(3)), 1.05))
        self.assertTrue(cloud.points(1.1))
        self.assertFalse(cloud.points(2))
        self.assertTrue(cloud.accept(decode_packet(demo_packet(0)), 4))

    def test_zero_returns_clear_old_sector(self):
        cloud = Cloud()
        cloud.accept(decode_packet(demo_packet(0)), 1)
        frame = decode_packet(demo_packet(24))
        frame.samples = [(0, 0)] * 16
        cloud.accept(frame, 1.1)
        self.assertFalse(cloud.points(1.1))

    def test_default_projection_uses_car_start_cell_center(self):
        for zone, expected in [(1, (2125, 450)), (2, (2125, 1985))]:
            point = map_cloud_points([(0, 100, 180, 1)], zone)[0]
            self.assertAlmostEqual(point['x_mm'], expected[0])
            self.assertAlmostEqual(point['y_mm'], expected[1])

    def test_udp_roundtrip(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as esp, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as pc:
            esp.bind(('127.0.0.1', 0))
            esp.settimeout(1)
            pc.settimeout(1)
            pc.sendto(b'N10_SUB', esp.getsockname())
            request, peer = esp.recvfrom(32)
            self.assertEqual(request, b'N10_SUB')
            esp.sendto(demo_packet(42), peer)
            data, _ = pc.recvfrom(2048)
            self.assertEqual(decode_packet(data).sequence, 42)

    def test_render_views(self):
        import cv2
        from pathlib import Path
        cloud = Cloud()
        for i in range(24): cloud.accept(decode_packet(demo_packet(i)), 1)
        args = SimpleNamespace(range_mm=3000, demo=True)
        out = Path(__file__).parent / 'preview'
        out.mkdir(exist_ok=True)
        for view, zone in [('radar', 1), ('map', 1), ('map', 2)]:
            image = render(cloud, 1.1, args, view, zone, 0)
            self.assertEqual(image.shape, (980, 900, 3))
            self.assertTrue(cv2.imwrite(str(out / f'{view}{zone}.png'), image))


if __name__ == '__main__':
    unittest.main()
