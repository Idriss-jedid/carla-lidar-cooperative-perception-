"""Offline unit tests for processing Milestone 01 (no CARLA needed)."""
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from cavlidar.backend.processing import bev_from_points, patchwise_ground_mask
from cavlidar.backend.processing.__main__ import process_frame


class TestGroundCleaning(unittest.TestCase):
    @staticmethod
    def make_slope(seed=1):
        rng = np.random.RandomState(seed)
        x = rng.uniform(-18, 18, 2500)
        y = rng.uniform(-15, 15, 2500)
        z = 0.13 * x + 0.02 * y - 2.4 + rng.normal(0, 0.025, len(x))
        ground = np.column_stack((x, y, z, np.ones(len(x)))).astype(np.float32)
        ox, oy = rng.uniform(2, 7, 250), rng.uniform(2, 7, 250)
        oz = 0.13 * ox + 0.02 * oy - 2.4 + rng.uniform(0.7, 1.8, len(ox))
        obstacle = np.column_stack((ox, oy, oz, np.full(len(ox), 0.6))).astype(np.float32)
        return np.vstack((ground, obstacle)), len(ground)

    def test_retains_obstacle_removes_sloped_ground(self):
        pts, n_ground = self.make_slope()
        mask = patchwise_ground_mask(pts, patch_size=8, iterations=90, seed=7)
        self.assertGreater(mask[:n_ground].mean(), 0.90)
        self.assertLess(mask[n_ground:].mean(), 0.10)

    def test_deterministic(self):
        pts, _ = self.make_slope()
        a = patchwise_ground_mask(pts, seed=8)
        b = patchwise_ground_mask(pts, seed=8)
        np.testing.assert_array_equal(a, b)

    def test_invalid_and_empty(self):
        self.assertEqual(patchwise_ground_mask(np.empty((0, 4))).shape, (0,))
        with self.assertRaises(ValueError):
            patchwise_ground_mask(np.ones((4, 2)))
        with self.assertRaises(ValueError):
            patchwise_ground_mask(np.zeros((5, 4)), patch_size=0)

    def test_sparse_patch_preserves_points(self):
        cloud = np.array([[2, 3, -2, 0.3], [2, 4, -1, 0.4]])
        self.assertEqual(int(patchwise_ground_mask(cloud).sum()), 0)


class TestBev(unittest.TestCase):
    def test_occupancy_intensity_average_and_orientation(self):
        pts = np.array([[1.1, -1.1, 1, 0.2], [1.2, -1.2, 2, 0.8],
                        [-1.2, 1.2, 1, 0.7]], dtype=np.float32)
        grid = bev_from_points(pts, resolution=1., xy_bounds=(-2, 2, -2, 2))
        self.assertEqual(grid.occupancy.shape, (4, 4))
        self.assertEqual(grid.counts[0, 0], 2)  # +x up, -y left
        self.assertEqual(grid.counts[3, 3], 1)  # -x down, +y right
        self.assertAlmostEqual(float(grid.mean_intensity[0, 0]), 0.5, 5)
        self.assertEqual(grid.occupancy.sum(), 2)

    def test_excludes_nan_and_bounds(self):
        pts = np.array([[0, 0, 0, .3], [np.nan, 0, 0, .3],
                        [1, 1, 0, .2]], dtype=np.float32)
        grid = bev_from_points(pts, resolution=1, xy_bounds=(-1, 1, -1, 1))
        self.assertEqual(int(grid.counts.sum()), 1)
        with self.assertRaises(ValueError):
            bev_from_points(pts, resolution=.3, xy_bounds=(-1, 1, -1, 1))


class TestProcessFrame(unittest.TestCase):
    def test_recorded_frame_writes_outputs_without_mutating_input(self):
        pts, _ = TestGroundCleaning.make_slope()
        with tempfile.TemporaryDirectory() as d:
            frame = Path(d) / "frames" / "000042"
            frame.mkdir(parents=True)
            np.save(str(frame / "cav0.npy"), pts)
            with open(str(frame / "meta.json"), "w") as fh:
                json.dump({"frame": 42, "cavs": {"cav0": {}}}, fh)
            output = process_frame(d, preview=False)
            self.assertTrue((output / "cav0_bev_occupancy.npy").is_file())
            self.assertEqual(np.load(str(output / "cav0_ground.npy")).shape[1], 4)
            self.assertEqual(np.load(str(frame / "cav0.npy")).shape, pts.shape)
            with open(str(output / "summary.json")) as fh:
                result = json.load(fh)
            self.assertEqual(result["frame"], 42)
            self.assertEqual(result["cavs"]["cav0"]["raw_points"], len(pts))
            self.assertEqual(result["cavs"]["cav0"]["raw_points"],
                             result["cavs"]["cav0"]["ground_points"] +
                             result["cavs"]["cav0"]["non_ground_points"])


if __name__ == "__main__":
    unittest.main()
