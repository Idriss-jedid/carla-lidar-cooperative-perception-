"""Metrics of the LiDAR sensitivity analysis (tools/eval_ablation.py). No CARLA needed."""
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools"))

from cavlidar.backend.processing.bev_grid import bev_from_points  # noqa: E402
import eval_ablation as ea  # noqa: E402
import make_sensitivity_scenes as msc  # noqa: E402
import run_ablation as ra  # noqa: E402


def vehicle(vid, x, y, yaw=0.0):
    return {"id": vid, "name": "v%d" % vid, "pose": {"x": x, "y": y, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": yaw},
            "extent": [2.4, 1.0, 0.75], "center_offset": [0.0, 0.0, 0.75]}


class TestAblationMetrics(unittest.TestCase):

    def test_bev_cells_match_bev_grid(self):
        pts = np.random.RandomState(0).uniform(-70, 70, (3000, 4)).astype(np.float32)
        cells = ea.bev_cells(pts[:, :2])
        grid = bev_from_points(pts, ea.BEV_RES, (-ea.BEV_HALF, ea.BEV_HALF, -ea.BEV_HALF, ea.BEV_HALF))
        self.assertEqual(set(np.unique(cells[cells >= 0]).tolist()), set(np.flatnonzero(grid.occupancy.ravel()).tolist()))

    def test_false_cells(self):
        cells = np.array([5, 5, 7, 7, 9, -1])
        ground = np.array([True, True, True, False, True, True])
        # cell 5: only ground -> false; cell 7: ground + obstacle -> real; cell 9: only ground -> false
        self.assertEqual(ea.false_cells(cells, ground), 2)
        self.assertEqual(ea.false_cells(np.array([-1]), np.array([True])), 0)

    def test_coverage_near_far_and_window(self):
        pose = {"x": 0.0, "y": 0.0, "z": 2.4, "roll": 0.0, "pitch": 0.0, "yaw": 90.0}  # facing world +y
        vehicles = [vehicle(1, 0.0, 0.0), vehicle(2, 0.0, 20.0), vehicle(3, 0.0, 50.0), vehicle(4, 0.0, 70.0),
                    vehicle(5, -10.0, 5.0)]
        kept = {2: 50, 3: 4, 5: 10}
        cov = ea.coverage(pose, vehicles, own_id=1, body_kept_by_vehicle=kept)
        self.assertEqual(cov["near"], [2, 2])   # v2 (20 m, 50 pts) and v5 (11 m, exactly 10 pts)
        self.assertEqual(cov["far"], [0, 1])    # v3 at 50 m has 4 pts; v4 at 70 m is outside the window

    def test_one_change_per_run(self):
        names = set()
        for factor, (key, values) in ra.FACTORS.items():
            self.assertIn(ra.BASELINE[factor], values)
            for value in values:
                names.add(ra.run_name("flat", factor, value))
                over = ra.overrides("flat", factor, value)
                self.assertEqual(len(over), 0 if value == ra.BASELINE[factor] else 1)
        # baseline shared by every factor: 1 + noise 3 + height 3 + channels 2 + range 2 + drop-off 4 + pps 2
        self.assertEqual(len(names), 17)
        self.assertEqual(ra.run_name("flat", "noise", 0.05), "abl_flat_noise_0.05")
        self.assertEqual(ra.run_name("slope", "channels", 32), "abl_slope_base")
        self.assertEqual(ra.run_name("flat2", "dropoff", 0.0), "abl_flat2_dropoff_0")
        self.assertEqual(ra.run_name("flat2", "pps", 2000000), "abl_flat2_pps_2000000")
        self.assertEqual(ra.run_name("flat2", "range", 50.0), "abl_flat2_range_50")
        self.assertEqual(ra.run_name("flat2", "repeat", None), "abl_flat2_repeat")
        self.assertEqual(ra.overrides("flat2", "invalid_mount", None), ["lidar.mount_z=1.2"])

    def test_population_runs_change_roles_or_traffic_only(self):
        variants = {"c{}".format(k): {"spawn_plan": "p_c{}.json".format(k), "num_cavs": k, "num_traffic": 34 - k}
                    for k in (4, 6, 8, 10)}
        variants.update({"t{}".format(n): {"spawn_plan": "p_t{}.json".format(n), "num_cavs": 4, "num_traffic": n}
                         for n in (20, 30, 40)})
        saved = dict(ra.SCENES)
        ra.SCENES["popx"] = {"config": "x.yaml", "terrain": "junction", "kind": "population", "variants": variants}
        try:
            self.assertEqual(ra.run_name("popx", "cavs", 4), "abl_popx_base")
            self.assertEqual(ra.run_name("popx", "traffic", 30), "abl_popx_base")
            self.assertEqual(ra.overrides("popx", "cavs", 4), [])
            self.assertEqual(ra.overrides("popx", "cavs", 8), ["scenario.spawn_plan=p_c8.json",
                                                               "scenario.num_cavs=8", "scenario.num_traffic=26"])
            self.assertEqual(ra.overrides("popx", "traffic", 40)[1:], ["scenario.num_cavs=4", "scenario.num_traffic=40"])
            runs = ra.planned_runs("popx", ["cavs", "traffic", "noise"], [])
            self.assertEqual(len(runs), 6)   # base + cavs 6/8/10 + traffic 20/40; sensor factors do not apply
        finally:
            ra.SCENES.clear()
            ra.SCENES.update(saved)


    def test_moving_scenes_name_every_value_and_record_no_single_runs(self):
        saved = dict(ra.SCENES)
        ra.SCENES["movx"] = {"config": "x.yaml", "terrain": "moving", "kind": "moving"}
        try:
            # each factor is one simulation with its own baseline: the baseline value is named, not shared
            self.assertEqual(ra.run_name("movx", "dropoff", 0.45), "abl_movx_dropoff_0.45")
            self.assertEqual(ra.run_name("movx", "pps", 1000000), "abl_movx_pps_1000000")
            self.assertEqual(ra.run_name("movx", "repeat", None), "abl_movx_repeat")
        finally:
            ra.SCENES.clear()
            ra.SCENES.update(saved)


class TestSceneGenerator(unittest.TestCase):

    def slot(self, x, y, yaw=0.0):
        return {"x": x, "y": y, "yaw": yaw}

    def test_conflicts_allow_neighbouring_lanes_only(self):
        a = self.slot(0.0, 0.0)
        self.assertFalse(msc.conflicts(self.slot(0.0, 3.5), [a]))    # side by side, next lane
        self.assertTrue(msc.conflicts(self.slot(5.0, 0.0), [a]))     # same lane, 5 m: overlap
        self.assertFalse(msc.conflicts(self.slot(7.0, 0.0), [a]))    # same lane, 7 m
        self.assertTrue(msc.conflicts(self.slot(0.0, 2.0, 90.0), [a]))  # crossing at 2 m

    def test_pick_is_seeded_and_conflict_free(self):
        slots = [self.slot(float(x), 0.0) for x in range(0, 200)]
        import random
        first = msc.pick(slots, random.Random(5), 10, [], lambda s: True)
        again = msc.pick(slots, random.Random(5), 10, [], lambda s: True)
        self.assertEqual(first, again)
        for i, s in enumerate(first):
            self.assertFalse(msc.conflicts(s, first[:i] + first[i + 1:]))


class TestAggregation(unittest.TestCase):

    def test_coop_frame_union(self):
        vehicles = [vehicle(1, 2.9, -136.5), vehicle(2, 20.0, -136.5), vehicle(3, 200.0, -136.5)]
        scans = [{"_body_kept": {2: 6}, "_kept_xy": np.array([[20.1, -136.4], [5.2, -130.1]]),
                  "_kept_ground": np.array([False, True]), "_road_xy": np.array([[20.1, -136.4], [5.2, -130.1]])},
                 {"_body_kept": {2: 5, 1: 3}, "_kept_xy": np.array([[20.2, -136.3]]),
                  "_kept_ground": np.array([False]), "_road_xy": np.array([[20.2, -136.3], [30.0, -136.0]])}]
        c = ea.coop_frame(scans, vehicles)
        self.assertEqual(c["coop_targets"], 2)          # vehicle 3 is outside the 60 m disc
        self.assertEqual(c["coop_covered"], 1)          # vehicle 2: 6 + 5 >= 10 points; vehicle 1: 3
        self.assertEqual(c["coop_cells"], 2)
        self.assertEqual(c["coop_false_cells"], 1)
        self.assertEqual(c["coop_observed_cells"], 3)   # two cells of the first CAV + one new of the second

    def test_paired_differences_across_scenes(self):
        def row(value, base, kept):
            return {"value": value, "baseline": base, "kept": kept, "points": 10.0}
        results = {"a": {"terrain": "flat", "kind": "sensor", "factors": {"pps": [row(1e6, True, 100.0),
                                                                                  row(2e6, False, 180.0)]}},
                   "b": {"terrain": "flat", "kind": "sensor", "factors": {"pps": [row(1e6, True, 50.0),
                                                                                  row(2e6, False, 110.0)]}}}
        rows = ea.aggregate(results)["flat"]["factors"]["pps"]
        self.assertEqual([r["n"] for r in rows], [2, 2])
        self.assertAlmostEqual(rows[1]["mean"]["kept"], 145.0)
        self.assertAlmostEqual(rows[1]["diff_mean"]["kept"], 70.0)
        self.assertAlmostEqual(rows[1]["diff_sd"]["kept"], np.std([80.0, 60.0], ddof=1))
        self.assertTrue(rows[0]["baseline"])


if __name__ == "__main__":
    unittest.main()
