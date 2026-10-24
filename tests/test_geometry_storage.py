import os
import tempfile
import unittest

import numpy as np

from cavlidar.backend.geometry.transforms import box_corners_2d, pose_to_matrix, sensor_to_world
from cavlidar.backend.recording import storage
from cavlidar.core.enums import PointFormat, Role


class TestGeometry(unittest.TestCase):

    def test_identity_pose(self):
        pose = dict(x=0, y=0, z=0, roll=0, pitch=0, yaw=0)
        np.testing.assert_allclose(pose_to_matrix(pose), np.eye(4), atol=1e-12)

    def test_yaw_90_rotates_x_to_y(self):
        pose = dict(x=10, y=20, z=1, roll=0, pitch=0, yaw=90)
        pts = np.array([[1.0, 0.0, 0.0, 0.5]], dtype=np.float32)
        out = sensor_to_world(pts, pose)
        np.testing.assert_allclose(out[0, :3], [10, 21, 1], atol=1e-5)
        self.assertAlmostEqual(float(out[0, 3]), 0.5)  # intensity untouched

    def test_box_corners(self):
        pose = dict(x=0, y=0, z=0, roll=0, pitch=0, yaw=0)
        corners = box_corners_2d(pose, [2.0, 1.0, 0.7])
        self.assertEqual(corners.shape, (4, 2))
        np.testing.assert_allclose(sorted(corners[:, 0]), [-2, -2, 2, 2])


class TestStorage(unittest.TestCase):

    def test_ply_and_npy_roundtrip(self):
        pts = np.random.RandomState(0).rand(1000, 4).astype(np.float32)
        with tempfile.TemporaryDirectory() as tmp:
            storage.save_points(tmp, "cav0", pts, [PointFormat.NPY, PointFormat.PLY])
            np.testing.assert_array_equal(storage.load_points(os.path.join(tmp, "cav0.npy")), pts)
            np.testing.assert_array_equal(storage.load_points(os.path.join(tmp, "cav0.ply")), pts)

    def test_frame_roundtrip(self):
        pts = np.ones((10, 4), dtype=np.float32)
        with tempfile.TemporaryDirectory() as tmp:
            run = storage.create_run_dir(tmp, "test")
            d = storage.frame_dir(run, 42)
            storage.save_points(d, "cav0", pts, ["npy"])
            storage.save_json(os.path.join(d, "meta.json"), {"frame": 42, "cavs": {"cav0": {}}})
            self.assertEqual(storage.list_frames(run), [42])
            meta, clouds = storage.load_frame(run, 42)
        self.assertEqual(meta["frame"], 42)
        self.assertEqual(clouds["cav0"].shape, (10, 4))


class TestEnums(unittest.TestCase):

    def test_role_names(self):
        self.assertEqual(Role.CAV.name_of(3), "cav_3")
        self.assertEqual(Role.of("traffic_12"), Role.TRAFFIC)
        self.assertEqual(Role.of("hero"), Role.OTHER)

    def test_parse_is_case_insensitive(self):
        self.assertEqual(PointFormat.parse("PLY"), PointFormat.PLY)
        with self.assertRaises(ValueError):
            PointFormat.parse("las")


if __name__ == "__main__":
    unittest.main()
