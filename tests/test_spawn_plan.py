import json
import os
import tempfile
import unittest

from cavlidar.backend.actors.spawn_plan import load_plan
from cavlidar.core.enums import Role
from cavlidar.core.errors import ConfigError

FIGURE2 = "config/spawn_plans/figure2_town03.json"
PAPER_BBOX = [-100, 100, -200, 200]


class TestSpawnPlan(unittest.TestCase):

    def test_figure2_plan(self):
        poses = load_plan(FIGURE2, "Town03", 4, 11, PAPER_BBOX)
        self.assertEqual(len(poses[Role.CAV]), 4)
        self.assertEqual(len(poses[Role.TRAFFIC]), 11)
        for pose in poses[Role.CAV] + poses[Role.TRAFFIC]:
            self.assertLess(abs(pose["x"] - 2.9), 60)   # all around junction 1221
            self.assertLess(abs(pose["y"] + 136.5), 60)

    def test_plan_smaller_than_config_is_allowed(self):
        poses = load_plan(FIGURE2, "Town03", 10, 40)
        self.assertEqual(len(poses[Role.CAV]), 4)

    def test_plan_rejections(self):
        with self.assertRaises(ConfigError):
            load_plan(FIGURE2, "Town03", 4, 5)              # more traffic in the plan than asked
        with self.assertRaises(ConfigError):
            load_plan(FIGURE2, "Town01", 4, 11)             # other map
        with self.assertRaises(ConfigError):
            load_plan(FIGURE2, "Town03", 4, 11, [0, 1, 0, 1])  # outside spawn_bbox
        with self.assertRaises(ConfigError):
            load_plan("config/spawn_plans/missing.json", "Town03", 4, 11)

    def test_bad_role(self):
        plan = {"map": "Town03", "vehicles": [{"role": "rsu", "index": 0,
                                               "pose": {"x": 0, "y": 0, "z": 0, "yaw": 0}}]}
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "plan.json")
            with open(path, "w") as fh:
                json.dump(plan, fh)
            with self.assertRaises(ConfigError):
                load_plan(path, "Town03", 4, 20)


if __name__ == "__main__":
    unittest.main()
