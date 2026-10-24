import os
import tempfile
import unittest

from cavlidar.api import DEFAULT_CONFIG
from cavlidar.core.config import load_config, save_config, validate
from cavlidar.core.enums import CameraMode, PointFormat
from cavlidar.core.errors import ConfigError

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class TestConfig(unittest.TestCase):

    def test_default_yaml_matches_paper_lidar(self):
        cfg = load_config(DEFAULT_CONFIG)
        self.assertEqual(cfg.lidar.channels, 32)
        self.assertEqual(cfg.lidar.range, 120.0)
        self.assertEqual(cfg.lidar.points_per_second, 1000000)
        self.assertEqual(cfg.recording.camera, CameraMode.FOLLOW)
        self.assertEqual(cfg.recording.formats, [PointFormat.NPY])

    def test_every_experiment_file_loads(self):
        exp_root = os.path.join(ROOT, "config", "experiments")
        files = [os.path.join(d, f) for d, _, fs in os.walk(exp_root) for f in fs if f.endswith(".yaml")]
        self.assertGreater(len(files), 0)
        for path in files:
            with self.subTest(path=path):
                load_config(DEFAULT_CONFIG, path)

    def test_override_is_typed(self):
        cfg = load_config(DEFAULT_CONFIG, overrides=["lidar.channels=64", "scenario.spawn_center=[1, 2]",
                                                     "recording.camera=top"])
        self.assertEqual(cfg.lidar.channels, 64)
        self.assertEqual(cfg.scenario.spawn_center, [1, 2])
        self.assertEqual(cfg.recording.camera, CameraMode.TOP)

    def test_unknown_key_is_rejected(self):
        with self.assertRaises(ConfigError):
            load_config(DEFAULT_CONFIG, overrides=["lidar.chanels=64"])

    def test_invalid_enum_is_rejected(self):
        with self.assertRaises(ConfigError):
            load_config(DEFAULT_CONFIG, overrides=["recording.camera=sky"])

    def test_invalid_value_is_rejected(self):
        with self.assertRaises(ConfigError):
            load_config(DEFAULT_CONFIG, overrides=["lidar.lower_fov=5"])

    def test_partial_sweep_warning(self):
        cfg = load_config(DEFAULT_CONFIG, overrides=["simulation.fixed_delta_seconds=0.01"])
        warnings = validate(cfg)
        self.assertEqual(len(warnings), 1)
        self.assertIn("72 degrees", warnings[0])

    def test_save_and_reload_roundtrip(self):
        cfg = load_config(DEFAULT_CONFIG, overrides=["recording.formats=[npy, ply]"])
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "config.yaml")
            save_config(cfg, path)
            again = load_config(path)
        self.assertEqual(again, cfg)


if __name__ == "__main__":
    unittest.main()
