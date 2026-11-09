"""
Ground-truth box vs LiDAR returns (tools/label_alignment.py, the label check of tools/eval_ablation.py and the
analysis of tools/diagnose_label_alignment.py). Regression tests for the moving-vehicle "offset" of the first
label check, which came from its estimator, not from the recorded boxes. No CARLA needed.
"""
import json
import os
import shutil
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools"))

from cavlidar.backend.geometry.transforms import pose_to_matrix  # noqa: E402
import diagnose_label_alignment as dla  # noqa: E402
import eval_ablation as ea  # noqa: E402
import label_alignment as la  # noqa: E402

EXTENT = [2.4, 1.0, 0.75]


def car(vid=1, x=0.0, y=0.0, yaw=0.0, pitch=0.0):
    return {"id": vid, "pose": {"x": x, "y": y, "z": 0.0, "roll": 0.0, "pitch": pitch, "yaw": yaw},
            "extent": list(EXTENT), "center_offset": [0.0, 0.0, EXTENT[2]]}


def surface(vehicle, faces, shift=0.0, n=200, seed=0):
    """
    World points on the box's rear / front / left side faces (heights 0.4 .. 1.4 m), all moved `shift` m along
    the heading: a box `k` ticks late is ahead of its returns by k * v * dt, i.e. shift = -k * v * dt.
    """
    rng = np.random.RandomState(seed)
    e = vehicle["extent"]
    pts = []
    for face in faces:
        h = rng.uniform(0.4, 1.4, n)
        if face == "rear":
            local = np.column_stack([np.full(n, -e[0]), rng.uniform(-e[1], e[1], n), h])
        elif face == "front":
            local = np.column_stack([np.full(n, e[0]), rng.uniform(-e[1], e[1], n), h])
        else:
            local = np.column_stack([rng.uniform(-e[0], e[0], n), np.full(n, e[1]), h])
        pts.append(local)
    local = np.vstack(pts)
    local[:, 0] += shift
    xyz1 = np.hstack([local, np.ones((len(local), 1))])
    return (xyz1 @ pose_to_matrix(vehicle["pose"]).T)[:, :3]


def offset(vehicle, points, sensor):
    view = la.view_of(sensor, vehicle)
    return la.face_offset(la.box_frame(points, vehicle)[0], vehicle["extent"][0], view), view


class TestVisibleEndOffset(unittest.TestCase):

    def test_view_from_sensor_position(self):
        v = car(yaw=90.0)                                   # heading world +y
        self.assertEqual(la.view_of((0.0, -15.0, 2.4), v), "rear")
        self.assertEqual(la.view_of((0.0, 15.0, 2.4), v), "front")
        self.assertEqual(la.view_of((8.0, 0.5, 2.4), v), "side")

    def test_aligned_box_gives_zero_from_every_side(self):
        v = car(x=10.0, y=-5.0, yaw=30.0, pitch=8.0)        # rotated and on a slope
        for faces, sensor in ((["rear"], (-10.0, -16.5, 2.4)), (["front"], (30.0, 6.5, 2.4)),
                              (["rear", "side", "front"], (8.0, 3.0, 2.4))):
            off, view = offset(v, surface(v, faces), sensor)
            self.assertAlmostEqual(off, 0.0, places=6, msg=view)

    def test_box_one_tick_late_is_seen_from_every_side(self):
        v = car(yaw=-45.0)
        lag = -18.0 * 0.05                                  # 18 m/s, one 0.05 s tick: returns 0.9 m behind the box
        h = np.radians(-45.0)
        for faces, dist in ((["rear"], -20.0), (["front"], 20.0)):
            sensor = (dist * np.cos(h), dist * np.sin(h), 2.4)
            off, view = offset(v, surface(v, faces, shift=lag), sensor)
            self.assertAlmostEqual(off, lag, places=6, msg=view)

    def test_sampling_gap_has_opposite_signs_at_the_two_ends(self):
        v = car()
        inside = 0.05                                       # extreme returns 5 cm inside both ends
        rear = surface(v, ["rear"], shift=inside)
        front = surface(v, ["front"], shift=-inside)
        self.assertGreater(offset(v, rear, (-20.0, 0.0, 2.4))[0], 0.0)
        self.assertLess(offset(v, front, (20.0, 0.0, 2.4))[0], 0.0)

    def test_quantile_ignores_a_few_stray_returns(self):
        v = car()
        pts = surface(v, ["rear"])
        stray = np.array([[-3.5, 0.0, 1.0], [-3.6, 0.2, 1.0]])   # e.g. a pole behind the car
        a = la.box_frame(np.vstack([pts, stray]), v)[0]
        self.assertLess(la.face_offset(a, EXTENT[0], "rear"), -1.0)
        self.assertAlmostEqual(la.face_offset(a, EXTENT[0], "rear", q=0.02), 0.0, places=3)


class TestFirstEstimatorBias(unittest.TestCase):
    """The first check's estimate depends on where the sensor is, not on the alignment."""

    def test_end_views_of_an_aligned_box_are_biased_by_half_the_search(self):
        v = car()
        est_rear, lo, hi = la.tie_shift(surface(v, ["rear"]), v)
        self.assertAlmostEqual(est_rear, -0.95, places=2)   # ties from -2.0 to +0.1 m
        self.assertAlmostEqual(hi, 0.1, places=6)           # only the bound on the visible side is informative
        est_front = la.tie_shift(surface(v, ["front"]), v)[0]
        self.assertAlmostEqual(est_front, 0.95, places=2)
        self.assertAlmostEqual(la.tie_shift(surface(v, ["rear", "side", "front"]), v)[0], 0.0, places=6)


class TestConsistencyHelpers(unittest.TestCase):

    def ticks(self):
        out = []
        for i in range(3):
            frame, t = 100 + i, 5.0 + 0.05 * i
            m = {"frame": frame, "timestamp": t}
            out.append({"frame": frame, "dt": 0.05, "snapshot": {"frame": frame, "elapsed": t, "delta": 0.05},
                        "received": [(frame, t)], "sensors": {"cav_0": {"ray": dict(m), "sem": dict(m)}}})
        return out

    def test_frame_checks_pass_on_matching_frames(self):
        c = la.frame_checks(self.ticks())
        self.assertEqual({k: v for k, v in c.items() if k != "ticks" and v}, {})

    def test_frame_checks_catch_a_one_tick_mismatch(self):
        t = self.ticks()
        t[1]["sensors"]["cav_0"]["ray"]["frame"] = 100       # a measurement of the previous tick
        t[2]["snapshot"]["frame"] = 103                      # a snapshot of the next tick
        t[2]["received"] = []                                # no snapshot reached the client
        c = la.frame_checks(t)
        self.assertEqual((c["sensor_frame"], c["snapshot_frame"], c["snapshots_received"]), (1, 1, 1))

    def test_sensor_pose_from_parent_and_mount(self):
        parent = {"x": 3.0, "y": 4.0, "z": 0.1, "roll": 0.0, "pitch": 0.0, "yaw": 90.0}
        pose = la.compose(parent, (1.0, 0.0, 2.4))
        self.assertAlmostEqual(pose["x"], 3.0, places=6)
        self.assertAlmostEqual(pose["y"], 5.0, places=6)    # 1 m forward = world +y
        self.assertAlmostEqual(pose["z"], 2.5, places=6)
        self.assertEqual(la.pose_difference(dict(parent, yaw=179.0), dict(parent, yaw=-179.0)), (0.0, 2.0))

    def test_subset_share_exact_and_with_tolerance(self):
        big = np.random.RandomState(1).uniform(-50, 50, (1000, 3)).astype(np.float32)
        self.assertEqual(la.subset_share(big[::3], big), 1.0)
        moved = big[:10] + np.float32(0.0004)
        self.assertEqual(la.subset_share(moved, big), 0.0)
        self.assertEqual(la.subset_share(moved, big, tol=0.001), 1.0)

    def test_near_box_keeps_the_vehicle_only(self):
        v, other = car(1), car(2, x=6.0)
        pts = np.vstack([surface(v, ["rear", "front"]), surface(other, ["rear"]),
                         [[0.0, 0.0, 0.1], [1.0, 0.5, 0.05]]])            # + two ground returns
        keep = la.near_box(pts, v, [v, other])
        self.assertEqual(int(keep.sum()), 400)


class TestRecordedRunCheck(unittest.TestCase):
    """eval_ablation.label_views on a synthetic recorded run (same layout as the recorder's)."""

    def setUp(self):
        self.run = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.run)

    def write_run(self, lag):
        target = dict(car(7, x=20.0, y=0.0), type_id="vehicle.test", speed_mps=18.0)
        observer = dict(car(1, x=0.0, y=0.0), type_id="vehicle.tesla.model3", speed_mps=0.0)
        sensor = {"x": 0.0, "y": 0.0, "z": 2.4, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        world = surface(target, ["rear"], shift=lag)
        local = (np.hstack([world, np.ones((len(world), 1))]) @ np.linalg.inv(pose_to_matrix(sensor)).T)[:, :3]
        frame = os.path.join(self.run, "frames", "000100")
        os.makedirs(frame)
        np.save(os.path.join(frame, "cav0.npy"), np.hstack([local, np.ones((len(local), 1))]).astype(np.float32))
        meta = {"frame": 100, "cavs": {"cav0": {"vehicle_id": 1, "sensor_pose": sensor}}, "vehicles": [observer, target]}
        with open(os.path.join(frame, "meta.json"), "w") as fh:
            json.dump(meta, fh)

    def test_aligned_moving_vehicle_seen_from_behind(self):
        self.write_run(lag=0.0)
        (row,) = ea.label_views(self.run)
        self.assertEqual(row["view"], "rear")
        self.assertAlmostEqual(row["offset"], 0.0, places=3)
        self.assertAlmostEqual(row["tie_shift"], -0.95, places=2)   # what the first check reported

    def test_box_one_tick_late_is_detected(self):
        self.write_run(lag=-0.9)
        (row,) = ea.label_views(self.run)
        self.assertAlmostEqual(row["offset"], -0.9, places=3)


class TestDiagnosticAnalysis(unittest.TestCase):

    def rows(self, tau, seed=0):
        rng = np.random.RandomState(seed)
        gap = {"rear": 0.03, "side": 0.0, "front": -0.03}
        out = []
        for vid in range(12):
            v = [0.0, 3.0, 6.0, 9.0, 13.0, 18.0][vid % 6]
            for view in ("rear", "front", "side"):
                out.append({"case": "c", "id": vid, "view": view, "v_along": v,
                            "offset_k+0": tau * v + gap[view] + rng.normal(0, 0.01)})
        return out

    def test_fit_recovers_a_lag_and_its_absence(self):
        late = dla._fit(self.rows(-0.05))
        self.assertAlmostEqual(late["tau_s"], -0.05, places=2)
        self.assertLess(late["tau_ci95_s"][1], -0.04)
        aligned = dla._fit(self.rows(0.0))
        self.assertLess(abs(aligned["tau_s"]), 0.005)
        self.assertLess(aligned["tau_ci95_s"][0], 0.0)
        self.assertGreater(aligned["tau_ci95_s"][1], 0.0)

    def test_moving_share(self):
        ticks = [{"actors": {"1": {"velocity": [2.0, 0, 0]}, "2": {"velocity": [0.2, 0, 0]}}}]
        self.assertEqual(dla.moving_share(ticks), 0.5)


class TestNormalisedFalseCells(unittest.TestCase):

    def test_false_cells_per_observed_road(self):
        row = ea.add_derived({"bev_false_cells": 50.0, "observed_cells": 2000.0,
                              "coop_false_cells": 300.0, "coop_observed_cells": 10000.0})
        self.assertAlmostEqual(row["false_per_1000_observed"], 25.0)
        self.assertAlmostEqual(row["coop_false_per_1000_observed"], 30.0)
        self.assertNotIn("coop_false_per_1000_observed", ea.add_derived({"bev_false_cells": 1.0, "observed_cells": 0}))


if __name__ == "__main__":
    unittest.main()
