import math
from types import SimpleNamespace
import unittest
import zlib
from n10_grid import FIXED_BLOCKED, cell_at
from n10_planner import RoutePlayer, plan_mission, center
from n10_robot import (FullScan, RobotLink, initial_pose, route_steps, upload_frames,
                       scan_pose, car_pose, radar_pose)
from n10_wifi import Frame, polar_points


class Recording:
    def __init__(self):
        self.reset()

    def reset(self):
        self.snapshots = []
        self.blocked = set(FIXED_BLOCKED)

    def capture(self, points, metadata):
        self.snapshots.append(dict(points=points, metadata=metadata))


class RobotTests(unittest.TestCase):
    def test_start_confirmation_and_selection(self):
        robot = RobotLink('unused', 1, Recording(), RoutePlayer())
        robot.line('PC_READY,2', 0.)
        self.assertNotIn('confirmed', robot.status)
        robot.line('PC_READY,1', 0.)
        self.assertIn('Start 1 confirmed', robot.status)
        robot.set_zone(2)
        self.assertEqual(robot.pose, initial_pose(2))
        robot.line('PC_READY,2', 0.)
        self.assertIn('Start 2 confirmed', robot.status)

    def test_tcp_fragmentation_and_short_writes(self):
        class Peer:
            def __init__(self):
                self.parts = [b'SCAN_RE', None, b'ADY,42,1,2125,398,0,100\n', None]
                self.written = bytearray()
            def send(self, data):
                self.written.extend(data[:4])
                return min(4, len(data))
            def recv(self, count):
                part = self.parts.pop(0) if self.parts else None
                if part is None:
                    raise BlockingIOError()
                return part
        robot = RobotLink('unused', 1, Recording(), RoutePlayer())
        robot.sock = peer = Peer()
        robot.poll(10.)
        self.assertIsNone(robot.collector)
        robot.poll(10.01)
        self.assertEqual(robot.scan_key, (42, 1))
        for i in range(10):
            robot.poll(10.02+i*.01)
        self.assertEqual(peer.written, b'PC_HELLO,1\n')

    def test_both_routes_connect_scan_point_and_return_original_start(self):
        for zone in (1, 2):
            plan = plan_mission(set(), zone, start_heading=3)
            pose = car_pose(scan_pose(zone, 2))
            steps = route_steps(plan, pose)
            self.assertEqual({s.leg for s in steps}, set(range(1, 9)))
            self.assertLess(len(steps), 128)
            self.assertLess(math.dist(steps[-1].pose[:2], initial_pose(zone)[:2]), .01)
            self.assertEqual(steps[-1].pose[2], initial_pose(zone)[2])
            if zone == 1:
                self.assertEqual((steps[0].kind, steps[0].direction, steps[0].amount), ('MOVE', 'L', 35))
            else:
                self.assertEqual(steps[0].kind, 'TURN')  # 车中心已在右上格中心，无接驳平移。
            self.assertAlmostEqual(steps[0].pose[0], 2125.)
            for i, action in enumerate(plan.actions):
                if action.kind == 'VISIT':
                    continue
                step = next(s for s in steps if s.plan_index == i and s.kind == ('TURN' if action.kind in ('L', 'R') else 'MOVE') and s.direction == action.kind)
                self.assertLess(math.dist(step.pose[:2], center(*action.end[:2])), .01)
                self.assertEqual(step.pose[2], action.end[2]*90.)

    def test_crc_covers_start_and_exact_step_lines(self):
        plan = plan_mission(set(), 1, start_heading=3)
        steps = route_steps(plan, car_pose(scan_pose(1, 2)))
        lines = upload_frames(42, 1, steps)
        fields = lines[0].strip().split(',')
        crc = zlib.crc32((f'42,1,{len(steps)}\n'+''.join(lines[1:-1])).encode('ascii'))
        self.assertEqual(int(fields[-1], 16), crc)
        self.assertTrue(all(len(line) < 128 for line in lines))

    def test_full_scan_discards_old_partial_and_reordered_data(self):
        scan = FullScan(20)
        self.assertIsNone(scan.feed(SimpleNamespace(sequence=19, start=350), []))
        result = None
        seq = 20
        for cycle in range(3):
            for start in range(0, 360, 10):
                frame = Frame(seq, 0, float(start), float(start+9), [(6000, 10)]*16)
                value = scan.feed(frame, list(polar_points(frame)))
                self.assertIsNone(scan.feed(frame, list(polar_points(frame))))
                if value is not None:
                    result = value
                    break
                seq += 1
            if result is not None:
                break
        self.assertIsNotNone(result)
        self.assertEqual(len(result), 36*16)

    def test_missing_sectors_do_not_complete_scan(self):
        scan = FullScan(0)
        result = None
        seq = 0
        for cycle in range(3):
            for angle in range(0, 360, 10):
                if angle in (50, 60, 70):
                    continue
                result = scan.feed(SimpleNamespace(sequence=seq, start=angle), [(angle, 1000, 1)])
                seq += 1
                self.assertIsNone(result)

    def test_two_scans_auto_plan_upload_ack_and_robot_progress(self):
        for zone in (1, 2):
            recorder, player = Recording(), RoutePlayer()
            robot = RobotLink('unused', zone, recorder, player)
            seq = 100
            for number, heading in ((1, 0 if zone == 1 else 180), (2, 270)):
                x, y, _ = scan_pose(zone, number)
                request = f'SCAN_READY,42,{number},{x},{y},{heading},{seq}'
                robot.line(request, 1.)
                with self.assertRaises(ValueError):
                    robot.set_zone(3-zone)
                for i in range(73):
                    start = (i % 36)*10
                    frame = Frame(seq, 0, start, start+9, [(6000, 10)]*16)
                    robot.feed_scan(frame, list(polar_points(frame)), 2.)
                    seq += 1
                self.assertEqual(len(recorder.snapshots), number)
                robot.line(request, 3.)  # 重复请求只重发SCAN_OK。
                self.assertEqual(len(recorder.snapshots), number)
                self.assertIn(f'SCAN_OK,42,{number}\n'.encode(), robot.tx)
            self.assertEqual(recorder.snapshots[-1]['metadata']['radar_pose_mm_deg'], [2112., y, 270.])
            self.assertEqual(recorder.snapshots[-1]['metadata']['car_pose_mm_deg'], [2125., y, 270.])
            self.assertTrue(robot.pending.startswith('NAV_BEGIN'))
            robot.line('NAV_ACK,42,999', 4.)
            self.assertTrue(robot.pending.startswith('NAV_BEGIN'))
            robot.line('NAV_ACK,42,BEGIN', 4.)
            for i in range(len(robot.steps)):
                self.assertTrue(robot.pending.startswith(f'NAV_STEP,42,{i},'))
                robot.line(f'NAV_ACK,42,{i}', 4.)
            self.assertEqual(robot.pending, 'NAV_END,42\n')
            robot.line('NAV_READY,42,OK', 4.)
            self.assertIsNone(robot.pending)
            before = robot.pose
            player.update(10000.)
            self.assertEqual(robot.pose, before)
            index = len(robot.steps)-1
            robot.line(f'NAV_STATE,42,8,{index},DONE', 5.)
            self.assertEqual(robot.pose, robot.steps[-1].pose)
            robot.line('AUTO,42,DONE,8,3,24,24,S2', 5.)
            self.assertFalse(robot.active)

    def test_measured_radar_positions_and_fixed_rotation_center(self):
        expected = {1: [(2125., 398., 0.), (2112., 385., 270.)],
                    2: [(2125., 2072., 180.), (2112., 2085., 270.)]}
        for zone, scans in expected.items():
            car_centers = []
            for number, pose in enumerate(scans, 1):
                self.assertEqual(scan_pose(zone, number), pose)
                car = car_pose(pose)
                self.assertLess(math.dist(radar_pose(car)[:2], pose[:2]), .001)
                car_centers.append(car[:2])
            self.assertLess(math.dist(*car_centers), .001)
            self.assertEqual(car_centers[0], (2125., 385. if zone == 1 else 2085.))

    def test_old_scan_coordinates_rejected(self):
        for old_y in (315, 248):
            robot = RobotLink('unused', 1, Recording(), RoutePlayer())
            with self.assertRaises(ValueError):
                robot.line(f'SCAN_READY,42,1,2125,{old_y},0,100', 1.)

    def test_both_scans_project_same_map_point(self):
        for zone in (1, 2):
            recorder = Recording()
            robot = RobotLink('unused', zone, recorder, RoutePlayer())
            # 固定地图点，反算两次扫描各自的距离/角度；投影应回到同一点。
            target = (1100., 1200.)
            for number in (1, 2):
                x, y, h = scan_pose(zone, number)
                robot.line(f'SCAN_READY,42,{number},{x},{y},{h},100', 1.)
                x, y, h = robot.radar_pose
                angle = math.radians(h)
                dx, dy = target[0]-x, target[1]-y
                forward = math.sin(angle)*dx+math.cos(angle)*dy
                left = -math.cos(angle)*dx+math.sin(angle)*dy
                point = (math.degrees(math.atan2(left, forward)) % 360, math.hypot(dx, dy), 100)
                robot.collector = SimpleNamespace(floor=100, feed=lambda *_: [point])
                robot.feed_scan(None, [], 2.)
                mapped = recorder.snapshots[-1]['points'][0]
                self.assertAlmostEqual(mapped['x_mm'], target[0])
                self.assertAlmostEqual(mapped['y_mm'], target[1])

    def test_upper_first_scan_position_calibration(self):
        recorder = Recording()
        robot = RobotLink('unused', 2, recorder, RoutePlayer())
        robot.line('SCAN_READY,42,1,2125,2072,180,100', 1.)
        self.assertEqual(robot.pose, (2125., 2085., 180.))
        self.assertEqual(robot.radar_pose, (2125., 1847., 180.))
        # 相同原始回波：首扫投影沿地图Y减225mm，X保持不变。
        x, y = center(3, 3)
        forward, left = 2072-y, x-2125
        point = (math.degrees(math.atan2(left, forward)) % 360,
                 math.hypot(forward, left), 100)
        robot.collector = SimpleNamespace(floor=100, feed=lambda *_: [point])
        robot.feed_scan(None, [], 2.)
        mapped = recorder.snapshots[0]['points'][0]
        self.assertAlmostEqual(mapped['x_mm'], x)
        self.assertAlmostEqual(mapped['y_mm'], y-225.)
        self.assertEqual(cell_at(mapped['x_mm'], mapped['y_mm']), (4, 3))
        metadata = recorder.snapshots[0]['metadata']
        self.assertEqual(metadata['reported_radar_pose_mm_deg'], [2125., 2072., 180.])
        self.assertEqual(metadata['radar_pose_mm_deg'], [2125., 1847., 180.])
        # 重发首扫请求不能重复累加修正；第二扫和导航使用原有位置。
        robot.line('SCAN_READY,42,1,2125,2072,180,100', 2.1)
        self.assertEqual(robot.radar_pose, (2125., 1847., 180.))
        robot.line('SCAN_READY,42,2,2112,2085,270,200', 3.)
        self.assertEqual(robot.radar_pose, (2112., 2085., 270.))
        self.assertEqual(robot.pose, (2125., 2085., 270.))

    def test_lower_first_scan_has_no_upper_start_correction(self):
        robot = RobotLink('unused', 1, Recording(), RoutePlayer())
        robot.line('SCAN_READY,42,1,2125,398,0,100', 1.)
        self.assertEqual(robot.radar_pose, (2125., 398., 0.))


if __name__ == '__main__':
    unittest.main()
