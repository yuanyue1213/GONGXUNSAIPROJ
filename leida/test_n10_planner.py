import heapq
import math
from pathlib import Path
from types import SimpleNamespace
import unittest

from n10_grid import FIXED_BLOCKED, START_CELLS, TwoSnapshots
from n10_planner import (HEADINGS, RoutePlayer, center, handle_control, mission,
                         plan_mission, vehicle_pose, lidar_to_pose)
from n10_plan_ui import BUTTONS, control_at


class PlannerTests(unittest.TestCase):
    def check_plan(self, plan, extra=()):
        forbidden = set(FIXED_BLOCKED) | set(extra)
        current = plan.start
        visits = []
        for index, action in enumerate(plan.actions):
            self.assertEqual(current, action.start)
            self.assertNotIn(action.start[:2], forbidden)
            self.assertNotIn(action.end[:2], forbidden)
            if action.kind in ('F', 'B'):
                self.assertEqual(action.start[2], action.end[2])
                dr, dc = HEADINGS[action.start[2]]
                sign = 1 if action.kind == 'F' else -1
                for a, b in zip(action.cells, action.cells[1:]):
                    self.assertEqual((b[0]-a[0], b[1]-a[1]), (sign*dr, sign*dc))
                    self.assertNotIn(b, forbidden)
                self.assertEqual(action.cells[0], action.start[:2])
                self.assertEqual(action.cells[-1], action.end[:2])
                a, b = center(*action.start[:2]), center(*action.end[:2])
                self.assertEqual(action.distance, abs(a[0]-b[0])+abs(a[1]-b[1]))
                if index:
                    self.assertNotEqual(action.kind, plan.actions[index-1].kind)
            elif action.kind in ('L', 'R'):
                self.assertEqual(action.start[:2], action.end[:2])
                self.assertEqual((action.end[2]-action.start[2]) % 4, 3 if action.kind == 'L' else 1)
            else:
                self.assertEqual(action.kind, 'VISIT')
                self.assertEqual(action.start, action.end)
                visits.append(action)
            current = action.end
        self.assertEqual([a.task for a in visits], [m[0] for m in mission(plan.start_zone)])
        self.assertEqual(visits[0].end, (3, 5, 0))
        expected = {(3, 5): 0, (1, 3): 3, (5, 3): 1, (3, 1): 2}
        for visit in visits[:-1]:
            self.assertEqual(visit.end[2], expected[visit.end[:2]])
        self.assertEqual(current[:2], START_CELLS[plan.start_zone])
        self.assertAlmostEqual(sum(a.duration for a in plan.actions if a.kind != 'VISIT'), plan.cost)

    def test_full_mission_both_starts(self):
        for zone in (1, 2):
            plan = plan_mission(set(), zone)
            self.assertEqual(plan.start, (*START_CELLS[zone], 0 if zone == 1 else 2))
            self.check_plan(plan)
            self.assertTrue(any(len(a.cells) >= 3 for a in plan.actions))

    def test_detour_and_unreachable(self):
        extra = {(3, 3), (2, 5)}
        self.check_plan(plan_mission(extra, 1), extra)
        with self.assertRaisesRegex(ValueError, 'QR'):
            plan_mission({(3, 5)}, 1)
        with self.assertRaisesRegex(ValueError, 'no open route'):
            plan_mission({(4, 5), (5, 4)}, 1)
        with self.assertRaisesRegex(ValueError, 'Start cell'):
            plan_mission({(5, 5)}, 1)

    def test_cost_matches_independent_waypoint_dynamic_program(self):
        # Independent Dijkstra for each possible arrival orientation at each task.
        zone = 2
        frontier = {(*START_CELLS[zone], 2): 0.}
        for _, goal, required in mission(zone):
            distances = dict(frontier)
            queue = [(v, k) for k, v in frontier.items()]
            heapq.heapify(queue)
            while queue:
                cost, pose = heapq.heappop(queue)
                if cost != distances[pose]: continue
                r, c, h = pose
                successors = [((r, c, (h-1) % 4), .8), ((r, c, (h+1) % 4), .8)]
                dr, dc = HEADINGS[h]
                for sign in (1, -1):
                    nr, nc = r+sign*dr, c+sign*dc
                    if 1 <= nr <= 5 and 1 <= nc <= 5 and (nr, nc) not in FIXED_BLOCKED:
                        a, b = center(r, c), center(nr, nc)
                        successors.append(((nr, nc, h), (abs(a[0]-b[0])+abs(a[1]-b[1]))/400))
                for dest, increment in successors:
                    new_cost = cost+increment
                    if new_cost < distances.get(dest, math.inf):
                        distances[dest] = new_cost
                        heapq.heappush(queue, (new_cost, dest))
            frontier = {p: cost for p, cost in distances.items()
                        if p[:2] == goal and (required is None or p[2] == required)}
        self.assertAlmostEqual(plan_mission(set(), zone).cost, min(frontier.values()))

    def test_animation_centerlines_turns_pause_and_finish(self):
        plan = plan_mission(set(), 2)
        for action in plan.actions:
            player = RoutePlayer(speed=1)
            player.load(plan, 0)
            player.index = plan.actions.index(action)
            player.update(action.duration/2)
            x, y, h = player.pose()
            a, b = center(*action.start[:2]), center(*action.end[:2])
            self.assertAlmostEqual(x, (a[0]+b[0])/2)
            self.assertAlmostEqual(y, (a[1]+b[1])/2)
            expected = action.start[2]*90
            if action.kind in ('L', 'R'): expected += -45 if action.kind == 'L' else 45
            self.assertAlmostEqual(h, expected % 360)
        player = RoutePlayer(speed=1)
        player.load(plan, 0)
        player.toggle_pause(.2)
        pose = player.pose()
        player.update(100)
        self.assertEqual(player.pose(), pose)
        player.toggle_pause(100)
        player.update(10000)
        self.assertTrue(player.finished)
        self.assertEqual(player.pose()[:2], center(*START_CELLS[2]))
        self.assertIn('DONE', player.status())

    def test_buttons_readiness_and_replay(self):
        recorder = SimpleNamespace(snapshots=[], blocked=set(FIXED_BLOCKED))
        player = RoutePlayer()
        with self.assertRaisesRegex(ValueError, 'two point-cloud'):
            handle_control('plan', recorder, player, 1, 0)
        self.assertIsNone(player.plan)
        recorder.snapshots = [{}, {}]
        zone = handle_control('top', recorder, player, 1, 0)
        self.assertEqual(zone, 2)
        handle_control('plan', recorder, player, zone, 0)
        player.update(1)
        handle_control('pause', recorder, player, zone, 1)
        self.assertTrue(player.paused)
        handle_control('replay', recorder, player, zone, 2)
        self.assertEqual(player.index, 0)
        self.assertFalse(player.paused)
        for key, (x0, y0, x1, y1), _ in BUTTONS:
            self.assertEqual(control_at((x0+x1)//2, (y0+y1)//2), key)
        self.assertIsNone(control_at(400, 400))

    def test_right_side_faces_each_task_boundary(self):
        # Expected outward boundary normal in map (x right, y up).
        normals = {(3, 5): (1, 0), (1, 3): (0, 1), (5, 3): (0, -1), (3, 1): (-1, 0)}
        for zone in (1, 2):
            plan = plan_mission(set(), zone)
            for action in plan.actions:
                if action.kind == 'VISIT' and action.task != 'RETURN':
                    heading = math.radians(action.end[2]*90)
                    right = (math.cos(heading), -math.sin(heading))
                    normal = normals[action.end[:2]]
                    self.assertAlmostEqual(sum(a*b for a, b in zip(right, normal)), 1.)

    def test_radar_and_car_share_translation_and_rotation(self):
        from n10_wifi import map_cloud_points
        points = [(0., 100., 180, 1), (90., 100., 180, 1)]
        expected = {
            0: [(1000, 1300), (900, 1200)],
            90: [(1100, 1200), (1000, 1300)],
            180: [(1000, 1100), (1100, 1200)],
            270: [(900, 1200), (1000, 1100)],
        }
        for heading, targets in expected.items():
            pose = (1000., 1200., float(heading))
            self.assertEqual(lidar_to_pose(0, 0, pose), pose[:2])
            projected = map_cloud_points(points, 1, pose)
            for point, target in zip(projected, targets):
                self.assertAlmostEqual(point['x_mm'], target[0])
                self.assertAlmostEqual(point['y_mm'], target[1])
        player = RoutePlayer(speed=1)
        player.load(plan_mission(set(), 2), 0)
        first_projection = map_cloud_points(points, 2, vehicle_pose(player, 2))
        player.update(.5)
        pose = vehicle_pose(player, 2)
        self.assertEqual(pose, player.pose())
        projection = map_cloud_points(points, 2, pose)
        self.assertNotEqual(first_projection, projection)
        self.assertAlmostEqual(math.hypot(projection[0]['x_mm']-pose[0], projection[0]['y_mm']-pose[1]), 100)
        # Projecting new live points cannot drag an earlier world-coordinate snapshot.
        self.assertNotEqual(first_projection[0]['y_mm'], projection[0]['y_mm'])

    def test_animation_preview_images(self):
        import cv2
        from n10_wifi import Cloud, render
        from n10_grid import START_CELLS
        from tempfile import TemporaryDirectory
        out = Path(__file__).parent / 'preview'
        out.mkdir(exist_ok=True)
        with TemporaryDirectory() as directory:
            recorder = TwoSnapshots(directory, demo=True)
            for zone in (1, 2):
                x, y = center(*START_CELLS[zone])
                recorder.capture([dict(x_mm=x, y_mm=y)], dict(zone=zone))
            player = RoutePlayer(speed=1)
            player.load(plan_mission(recorder.blocked, 2), 0)
            for i, t in enumerate((0, 1, 4, 12, 1000)):
                player.update(t)
                image = render(Cloud(), t, SimpleNamespace(demo=True, range_mm=6000),
                               'map', 2, 0, recording=recorder, player=player, start_zone=2)
                self.assertTrue(cv2.imwrite(str(out / f'route-{i}.png'), image))

    def test_real_window_mouse_callback_starts_animation_without_capturing(self):
        # Run the actual event loop, replacing only native window calls.
        import itertools
        from contextlib import ExitStack
        from unittest.mock import Mock, patch
        import cv2
        import n10_wifi
        recorder = SimpleNamespace(snapshots=[{'points': []}, {'points': []}],
                                   blocked=set(FIXED_BLOCKED), capture=Mock())
        callback, poses, plans, bound_poses, zones = [], [], [], [], []
        real_render = n10_wifi.render
        def show(*args, **kwargs):
            player = args[-2]
            plans.append(player.plan)
            poses.append(player.pose())
            bound_poses.append(vehicle_pose(player, args[-1]))
            zones.append((args[4], args[-1]))
            return real_render(*args, **kwargs)
        events = iter(('plan', None, 'pause', 'pause', 'replay', 'top', 'bottom', 'keytop', 'quit'))
        def wait(delay):
            event = next(events)
            if event == 'quit': return ord('q')
            if event == 'keytop': return ord('2')
            if event:
                _, rect, _ = next(b for b in BUTTONS if b[0] == event)
                x0, y0, x1, y1 = rect
                callback[0](cv2.EVENT_LBUTTONDOWN, (x0+x1)//2, (y0+y1)//2, 0, None)
            return 255
        with ExitStack() as stack:
            for name in ('namedWindow', 'imshow', 'destroyAllWindows'):
                stack.enter_context(patch.object(cv2, name))
            stack.enter_context(patch.object(cv2, 'setMouseCallback', side_effect=lambda _, cb: callback.append(cb)))
            stack.enter_context(patch.object(cv2, 'getWindowProperty', return_value=1))
            stack.enter_context(patch.object(cv2, 'waitKey', side_effect=wait))
            stack.enter_context(patch.object(n10_wifi, 'render', side_effect=show))
            stack.enter_context(patch.object(n10_wifi, 'TwoSnapshots', return_value=recorder))
            stack.enter_context(patch('sys.argv', ['n10_wifi.py', '--demo']))
            ticks = itertools.count()
            stack.enter_context(patch.object(n10_wifi.time, 'monotonic', side_effect=lambda: 1000+next(ticks)*.02))
            n10_wifi.main()
        self.assertIsNone(plans[0])
        self.assertIsNotNone(plans[1])
        self.assertNotEqual(poses[1], poses[2])
        self.assertTrue(all(zone == start_zone for zone, start_zone in zones))
        self.assertEqual(bound_poses[6], (*center(1, 5), 180.))
        self.assertEqual(bound_poses[7], (*center(5, 5), 0.))
        self.assertEqual(bound_poses[8], (*center(1, 5), 180.))
        recorder.capture.assert_not_called()


if __name__ == '__main__':
    unittest.main()
