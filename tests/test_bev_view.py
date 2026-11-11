"""From a recorded run to the BEV shown by the editor: process_recording, run listing, world-frame cells, figure."""
import json
import math
import os
import tempfile
import unittest

import numpy as np

from cavlidar import api
from cavlidar.backend.visualization import pipeline
from cavlidar.editor import results


def write_run(root, name="2026-11-09_101500_test", frames=(42, 62), yaw=90.0):
    """A synthetic run: flat ground ring + a box-shaped obstacle 10 m ahead of the CAV, one other vehicle."""
    rng = np.random.RandomState(0)
    r = rng.uniform(3.0, 40.0, 6000)
    a = rng.uniform(-math.pi, math.pi, 6000)
    ground = np.column_stack((r * np.cos(a), r * np.sin(a), np.full(6000, -2.4) + rng.normal(0, 0.02, 6000),
                              np.full(6000, 0.5)))
    obstacle = np.column_stack((rng.uniform(9.0, 13.0, 800), rng.uniform(-1.0, 1.0, 800),
                                rng.uniform(-2.0, -0.9, 800), np.full(800, 0.9)))
    points = np.vstack((ground, obstacle)).astype(np.float32)
    run = os.path.join(root, name)
    pose = {"x": 100.0, "y": 50.0, "z": 2.4, "roll": 0.0, "pitch": 0.0, "yaw": yaw}
    other = {"id": 7, "type_id": "vehicle.audi.tt", "role": "traffic", "name": "traffic_0",
             "pose": {"x": 100.0, "y": 61.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": yaw},
             "extent": [2.0, 1.0, 0.7], "center_offset": [0.0, 0.0, 0.7], "speed_mps": 0.0}
    for frame in frames:
        d = os.path.join(run, "frames", "{:06d}".format(frame))
        os.makedirs(d)
        np.save(os.path.join(d, "cav0.npy"), points)
        with open(os.path.join(d, "meta.json"), "w") as fh:
            json.dump({"frame": frame, "town": "Town03", "cavs": {"cav0": {"vehicle_id": 1, "sensor_pose": pose,
                                                                           "mount_z": 2.4}},
                       "vehicles": [other]}, fh)
    with open(os.path.join(run, "config.yaml"), "w") as fh:
        fh.write("server:\n  town: Town03\nlidar:\n  range: 120.0\n")
    return run, points


class TestCellsInWorld(unittest.TestCase):

    def test_cell_corners_follow_the_sensor_pose(self):
        occupancy = np.zeros((4, 4), dtype=np.uint8)
        occupancy[0, 3] = 1                       # x in [0.5, 1), y in [0.5, 1): ahead and to the right
        pose = {"x": 10.0, "y": 5.0, "z": 2.0, "roll": 0.0, "pitch": 0.0, "yaw": 90.0}
        cells = pipeline.bev_cells_world(occupancy, (-1, 1, -1, 1), pose)
        self.assertEqual(cells.shape, (1, 4, 2))
        # yaw 90: forward = world +y, right = world -x
        np.testing.assert_allclose(cells[0].mean(axis=0), [9.25, 5.75], atol=1e-9)
        sides = np.linalg.norm(np.diff(np.vstack([cells[0], cells[0][:1]]), axis=0), axis=1)
        np.testing.assert_allclose(sides, 0.5, atol=1e-9)

    def test_empty_grid(self):
        cells = pipeline.bev_cells_world(np.zeros((8, 8)), (-2, 2, -2, 2), {"x": 0, "y": 0, "z": 0, "roll": 0,
                                                                            "pitch": 0, "yaw": 0})
        self.assertEqual(cells.shape, (0, 4, 2))


class TestRunToBev(unittest.TestCase):

    def test_process_list_overlay_figure(self):
        with tempfile.TemporaryDirectory() as root:
            run, points = write_run(root)
            self.assertEqual(results.list_runs(root)[0].processed, [])
            with self.assertRaises(ValueError):
                api.process_recording(run, frames=[999], road_filter=False)
            out = api.process_recording(run, preview="none", road_filter=False)  # synthetic: no map
            self.assertEqual([os.path.basename(o) for o in out], ["000042", "000062"])

            info = results.list_runs(root)[0]
            self.assertEqual((info.town, info.frames, info.processed), ("Town03", [42, 62], [42, 62]))
            self.assertIn("BEV ready", info.label)

            overlay = results.bev_overlay(run, 62)
            self.assertEqual([c.name for c in overlay.cavs], ["cav0"])
            cells = overlay.cavs[0].cells
            self.assertGreater(len(cells), 0)
            # the obstacle 9-13 m ahead of a CAV facing world +y at (100, 50) is at y 59-63
            centres = cells.mean(axis=1)
            self.assertTrue(np.all(np.abs(centres[:, 0] - 100.0) < 2.0))
            self.assertTrue(np.all((centres[:, 1] > 58.5) & (centres[:, 1] < 63.5)))
            x0, x1, y0, y1 = overlay.bounds()
            self.assertTrue(x0 < 99 and x1 > 101 and y0 < 49 and y1 > 63)

            stages = pipeline.stages_from_processed(run, 62, "cav0")
            parts = sum(len(stages[k]) for k in ("ego", "offroad", "ground", "non_ground"))
            self.assertEqual(parts, len(points))                   # every return is in exactly one stage
            self.assertEqual(int(stages["occupancy"].sum()), len(cells))
            self.assertEqual(len(stages["sectors"]), 4)
            self.assertEqual([name for name, _ in stages["boxes"]], ["traffic_0"])

            path = results.make_figure(run, 62, "cav0")
            self.assertTrue(os.path.isfile(path) and os.path.getsize(path) > 10000)

    def test_every_cav_before_and_after_filtering(self):
        with tempfile.TemporaryDirectory() as root:
            run, points = write_run(root, frames=(42,))
            api.process_recording(run, preview="none", road_filter=False)
            rows = pipeline.vehicle_returns(run, 42)
            self.assertEqual([r["name"] for r in rows], ["traffic_0"])
            car = rows[0]["cavs"]["cav0"]
            # the 800 obstacle returns lie on traffic_0's body; nothing of it is lost by the filtering
            self.assertEqual(car["raw"], 800)
            self.assertEqual(car["road"], 800)                      # road filter off in this synthetic run
            self.assertGreaterEqual(car["non_ground"], 790)
            self.assertGreater(car["cells"], 0)
            self.assertAlmostEqual(car["dist"], 11.0, places=6)
            self.assertEqual(rows[0]["seen_by"], ["cav0"])
            stages = pipeline.all_stages(run, 42)
            summary = pipeline.cav_summaries(stages, rows)["cav0"]
            self.assertEqual(summary["raw"], len(points))
            self.assertEqual(summary["ego"] + summary["offroad"] + summary["road"], summary["raw"])
            self.assertEqual((summary["seen_raw"], summary["seen_after"]), (1, 1))
            for which in pipeline.VIEWS:                          # each figure on its own
                path = results.make_view(run, 42, which)
                self.assertEqual(os.path.basename(path), which + ".png")
                self.assertGreater(os.path.getsize(path), 10000)
            path = results.make_view(run, 42, "cav", "cav0")
            self.assertEqual(os.path.basename(path), "cav0_before_after.png")
            with self.assertRaises(ValueError):
                results.make_view(run, 42, "nothing")
            out = pipeline.compare(run, 42)
            self.assertEqual(sorted(out["figures"]), sorted(list(pipeline.VIEWS) + ["cav0"]))
            self.assertTrue(os.path.isfile(out["summary"]))
            with open(out["vehicles"]) as fh:
                lines = fh.read().splitlines()
            self.assertEqual(lines[0].split(",")[:4], ["vehicle", "model", "role", "cav"])
            self.assertEqual(lines[1].split(",")[0], "traffic_0")

    def test_body_points_ignore_the_road_under_the_car(self):
        v = {"pose": {"x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 30.0},
             "extent": [2.0, 1.0, 0.75], "center_offset": [0.0, 0.0, 0.75]}
        pts = np.array([[0.0, 0.0, 0.05],      # road under the car: not the body
                        [0.0, 0.0, 1.0],       # body
                        [1.9 * math.cos(math.radians(30)), 1.9 * math.sin(math.radians(30)), 1.2],   # front, rotated
                        [5.0, 0.0, 1.0]])      # elsewhere
        self.assertEqual(pipeline.on_body(pts, v).tolist(), [False, True, True, False])

    def test_unprocessed_frame_has_no_overlay(self):
        with tempfile.TemporaryDirectory() as root:
            run, _ = write_run(root, frames=(5,))
            with self.assertRaises(ValueError):
                results.bev_overlay(run, 5)
            self.assertIn("not processed", results.run_info(run).label)

    def test_runs_newest_first_and_missing_folder(self):
        with tempfile.TemporaryDirectory() as root:
            write_run(root, "2026-11-09_101500_a", frames=(1,))
            write_run(root, "2026-11-09_111500_b", frames=(1,))
            os.makedirs(os.path.join(root, "not_a_run"))
            self.assertEqual([r.name for r in results.list_runs(root)],
                             ["2026-11-09_111500_b", "2026-11-09_101500_a"])
            self.assertEqual(results.list_runs(os.path.join(root, "missing")), [])


if __name__ == "__main__":
    unittest.main()
