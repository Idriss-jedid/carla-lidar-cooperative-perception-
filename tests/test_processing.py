"""Offline unit tests for processing Milestone 01 (no CARLA needed)."""
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from cavlidar.backend.processing import bev_from_points, patchwise_ground_mask
from cavlidar.backend.processing.__main__ import process_frame

DATA = Path(__file__).parent / "data"


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


def lidar_scene(ground_z, cars=(), rings=40, per_ring=900, max_range=60.0, seed=3):
    """
    LiDAR-like scan in the sensor frame: ground returns on rings around the
    sensor at height ground_z(x, y), and for each car (x0, x1, y0, y1, height)
    returns on its roof and sides, which also shadow the ground behind them.
    Returns (points, is_car).
    """
    rng = np.random.RandomState(seed)
    radii = np.geomspace(3.0, max_range, rings)
    angles = rng.uniform(-np.pi, np.pi, (rings, per_ring))
    x = (radii[:, None] * np.cos(angles)).ravel()
    y = (radii[:, None] * np.sin(angles)).ravel()
    ground = np.column_stack((x, y, ground_z(x, y)))
    keep = np.ones(len(ground), bool)
    car_pts = []
    for x0, x1, y0, y1, height in cars:
        cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        near = np.hypot(cx, cy) - 1.0
        shadow = (np.abs(np.arctan2(y, x) - np.arctan2(cy, cx)) < 0.35) & (np.hypot(x, y) > near)
        keep &= ~shadow
        base = ground_z(np.array([cx]), np.array([cy]))[0]
        roof = np.column_stack((rng.uniform(x0, x1, 400), rng.uniform(y0, y1, 400), np.full(400, base + height)))
        side = np.column_stack((rng.uniform(x0, x1, 300), np.full(300, y0 if abs(y0) < abs(y1) else y1),
                                base + rng.uniform(0.35, height, 300)))
        car_pts += [roof, side]
    cars_xyz = np.vstack(car_pts) if car_pts else np.empty((0, 3))
    xyz = np.vstack((ground[keep], cars_xyz))
    points = np.column_stack((xyz, np.full(len(xyz), 0.5))).astype(np.float32)
    is_car = np.r_[np.zeros(keep.sum(), bool), np.ones(len(cars_xyz), bool)]
    return points, is_car


def flat(x, y):
    return np.full(len(x), -2.4)


def slope_ahead(x, y):
    """
    Like Town03's sloped leg: flat for x < 10 m, the grade grows smoothly to
    12 degrees at x = 30 m (roads bend gradually, OpenDRIVE elevation is cubic), then stays.
    """
    g = np.tan(np.radians(12.0))
    u = np.clip(x - 10.0, 0.0, 20.0)
    return -2.4 + g * u ** 2 / 40.0 + g * np.clip(x - 30.0, 0.0, None)


class TestHeightPrior(unittest.TestCase):

    def test_seed_plane_gives_sensor_height(self):
        pts, _ = lidar_scene(flat)
        a, b, c = patchwise_ground_mask.__globals__["estimate_seed_plane"](pts)
        self.assertAlmostEqual(-c, 2.4, delta=0.05)
        self.assertLess(np.hypot(a, b), 0.01)

    def test_close_vehicle_is_kept_real_scan(self):
        # Real CARLA scan (routes_demo, cav1, last frame, cropped to 20 m): cav_2 waits right next
        # to cav1. Before the height prior, 58 % of its body points were labelled ground.
        # `vehicle` comes from CARLA's boxes and is only used to score the result.
        data = np.load(str(DATA / "close_pair_cav1.npz"))
        pts, vehicle = data["points"], data["vehicle"]
        mask = patchwise_ground_mask(pts)
        self.assertLess(mask[vehicle].mean(), 0.02)
        road = ~vehicle & (pts[:, 2] < -2.2)   # the flat junction, 2.4 m below the sensor
        self.assertGreater(mask[road].mean(), 0.97)

    def test_slope_is_ground_and_car_on_it_is_kept(self):
        pts, is_car = lidar_scene(slope_ahead, cars=[(24.0, 28.8, -1.0, 1.0, 1.45)])
        mask = patchwise_ground_mask(pts)
        on_slope = ~is_car & (pts[:, 0] > 12.0)
        self.assertGreater(mask[on_slope].mean(), 0.95)
        self.assertLess(mask[is_car].mean(), 0.05)

    def test_sparse_patches(self):
        pts, _ = lidar_scene(flat, max_range=30.0)
        near_known = np.array([[33.0, 1.0, -2.4, 0.5], [33.5, 2.0, -2.45, 0.5], [34.0, 1.5, -1.0, 0.5]])
        isolated = np.array([[100.0, 100.0, -2.4, 0.5], [101.0, 100.5, -2.4, 0.5]])
        mask = patchwise_ground_mask(np.vstack((pts, near_known, isolated)).astype(np.float32))
        self.assertTrue(mask[-5] and mask[-4])   # on the ground predicted by the neighbour patches
        self.assertFalse(mask[-3])               # 1.4 m above it
        self.assertFalse(mask[-2:].any())        # nothing tells where the ground is: kept


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
