import json
import math
import os
import tempfile
import unittest

import numpy as np

from cavlidar.backend.actors.spawn_plan import DEFAULT_ROUTE_SPEED, load_plan
from cavlidar.backend.maps import routes
from cavlidar.core.enums import Role, RouteEnd
from cavlidar.core.errors import ConfigError, RouteError
from cavlidar.editor.model import PlannedVehicle, ScenarioPlan

TOWN03 = "D:/CARLA/CARLA_0.9.12/WindowsNoEditor/CarlaUE4/Content/Carla/Maps/OpenDrive/Town03.xodr"
CAV_0 = {"x": 5.21, "y": -138.16, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": -178.8}  # Figure 2, heading west


def straight(length, step=2.0):
    xs = np.arange(0.0, length + 1e-9, step)
    return routes.Route(np.stack([xs, np.zeros_like(xs), np.zeros_like(xs)], axis=1))


def write_plan(tmp, vehicle):
    path = os.path.join(tmp, "plan.json")
    with open(path, "w") as fh:
        json.dump({"map": "Town03", "vehicles": [dict({"role": "cav", "index": 0, "pose": CAV_0}, **vehicle)]}, fh)
    return path


class TestSpeedProfile(unittest.TestCase):

    def test_straight_route_cruises_then_brakes_to_the_end(self):
        profile = routes.speed_profile(straight(100), 10.0)
        self.assertEqual(profile[0], 10.0)
        self.assertEqual(profile[-1], 0.0)
        s = straight(100).distances
        braking = 10.0 ** 2 / (2 * routes.DECEL)   # ~16.7 m before the end
        self.assertTrue(np.all(profile[s < 100 - braking - 2] == 10.0))
        self.assertTrue(np.all(np.diff(profile) <= 1e-9))

    def test_keep_driving_does_not_brake(self):
        self.assertTrue(np.all(routes.speed_profile(straight(50), 10.0, stop_at_end=False) == 10.0))

    def test_turn_is_slower(self):
        # quarter circle of radius 10 m: v <= sqrt(a_lat * r) = 5 m/s
        angles = np.linspace(0, math.pi / 2, 9)
        arc = np.stack([10 * np.sin(angles), 10 - 10 * np.cos(angles), np.zeros_like(angles)], axis=1)
        line = np.stack([np.arange(-40.0, 0.0, 2.0), np.zeros(20), np.zeros(20)], axis=1)
        profile = routes.speed_profile(routes.Route(np.vstack([line, arc])), 15.0, stop_at_end=False)
        self.assertLess(profile[24], math.sqrt(routes.LATERAL_ACCEL * 10) + 0.5)
        self.assertEqual(profile[0], 15.0)


@unittest.skipUnless(os.path.isfile(TOWN03), "CARLA 0.9.12 OpenDRIVE files not found")
class TestTraceRoute(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        from cavlidar.backend.maps.road_map import RoadMap
        cls.road_map = RoadMap.from_file("Town03", TOWN03)

    def test_straight_ahead(self):
        route = routes.trace_route(self.road_map.map, CAV_0, [[-59.99, -139.56]])
        self.assertAlmostEqual(route.length, 65, delta=2)
        self.assertLess(np.hypot(*(route.points[-1, :2] - [-59.99, -139.56])), 0.1)   # ends on the point
        self.assertTrue(np.all(np.diff(route.points[:, 0]) < 0))                      # always westwards

    def test_up_the_sloped_leg(self):
        start = {"x": 9.06, "y": -102.81, "z": 0.0, "yaw": -88.6}
        route = routes.trace_route(self.road_map.map, start, [[59.99, -133.49]])
        self.assertGreater(route.points[-1, 2], 6.0)   # follows the road up to z ~ 7 m
        self.assertLess(route.length, 120)

    def test_errors(self):
        with self.assertRaises(RouteError):
            routes.trace_route(self.road_map.map, CAV_0, [])


class TestPlanFiles(unittest.TestCase):

    def test_driving_keys_are_read(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = write_plan(tmp, {"route": [[-60, -139.5]], "route_end": "autopilot"})
            entry = load_plan(path, "Town03", 1, 0)[Role.CAV][0]
        self.assertEqual(entry.route, [[-60.0, -139.5]])
        self.assertEqual(entry.speed_kmh, DEFAULT_ROUTE_SPEED)   # a route without speed gets the default
        self.assertEqual(entry.route_end, RouteEnd.AUTOPILOT)

    def test_bad_driving_keys(self):
        for bad in ({"speed_kmh": 0}, {"speed_kmh": "fast"}, {"route": [[1]]}, {"route": "north"},
                    {"route": [[0, 0]], "route_end": "park"}):
            with tempfile.TemporaryDirectory() as tmp, self.assertRaises(ConfigError, msg=str(bad)):
                load_plan(write_plan(tmp, bad), "Town03", 1, 0)

    def test_editor_roundtrip_and_undo(self):
        plan = ScenarioPlan("Town03", [PlannedVehicle(Role.CAV, dict(CAV_0)), PlannedVehicle(Role.TRAFFIC, dict(CAV_0))])
        plan.set_driving(0, route=[[-60.0, -139.5]], speed_kmh=40.0, route_end=RouteEnd.AUTOPILOT)
        plan.set_driving(1, speed_kmh=20.0)
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "plan.json")
            plan.save(path)
            again = ScenarioPlan.load(path)
            entries = load_plan(path, "Town03", 1, 1)
        self.assertEqual((again.vehicles[0].route, again.vehicles[0].speed_kmh, again.vehicles[0].route_end),
                         ([[-60.0, -139.5]], 40.0, RouteEnd.AUTOPILOT))
        self.assertEqual((again.vehicles[1].route, again.vehicles[1].speed_kmh), ([], 20.0))
        self.assertEqual(entries[Role.TRAFFIC][0].route, None)
        plan.undo()
        plan.undo()
        self.assertEqual((plan.vehicles[0].route, plan.vehicles[0].speed_kmh), ([], None))


if __name__ == "__main__":
    unittest.main()
