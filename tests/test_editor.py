import os
import tempfile
import unittest

from cavlidar.api import DEFAULT_CONFIG
from cavlidar.backend.maps import catalog
from cavlidar.backend.maps.raster import Georef
from cavlidar.backend.maps.road_map import wrap_degrees
from cavlidar.backend.simulator import startup_map
from cavlidar.core.config import load_config
from cavlidar.core.enums import Role
from cavlidar.editor import export
from cavlidar.editor.model import PlannedVehicle, ScenarioPlan, overlap


def pose(x, y, yaw=0.0, z=0.0):
    return {"x": x, "y": y, "z": z, "roll": 0.0, "pitch": 0.0, "yaw": yaw}


def plan_with(*items):
    plan = ScenarioPlan("Town03")
    for role, p in items:
        plan.add(PlannedVehicle(role, p))
    return plan


class TestModel(unittest.TestCase):

    def test_overlap(self):
        self.assertTrue(overlap(pose(0, 0), pose(3, 0)))            # same lane, 3 m apart
        self.assertFalse(overlap(pose(0, 0), pose(6, 0)))           # same lane, 6 m apart
        self.assertFalse(overlap(pose(0, 0), pose(0, 3.5)))         # next lane
        self.assertTrue(overlap(pose(0, 0), pose(2, 2, yaw=90)))    # crossing
        self.assertFalse(overlap(pose(0, 0), pose(0, 0, z=8)))      # bridge above

    def test_names_follow_order_within_role(self):
        plan = plan_with((Role.CAV, pose(0, 0)), (Role.TRAFFIC, pose(10, 0)), (Role.CAV, pose(20, 0)))
        self.assertEqual([plan.name(i) for i in range(3)], ["cav_0", "traffic_0", "cav_1"])
        plan.remove(0)
        self.assertEqual([plan.name(i) for i in range(2)], ["traffic_0", "cav_0"])
        plan.set_role(0, Role.CAV)
        self.assertEqual(plan.counts(), {Role.CAV: 2, Role.TRAFFIC: 0})

    def test_undo_redo(self):
        plan = plan_with((Role.CAV, pose(0, 0)))
        plan.begin_change()
        plan.replace(0, PlannedVehicle(Role.CAV, pose(5, 0)), record=False)
        plan.replace(0, PlannedVehicle(Role.CAV, pose(9, 0)), record=False)   # one drag = one undo step
        self.assertTrue(plan.undo())
        self.assertEqual(plan.vehicles[0].pose["x"], 0)
        self.assertTrue(plan.redo())
        self.assertEqual(plan.vehicles[0].pose["x"], 9)
        self.assertTrue(plan.undo() and plan.undo())
        self.assertEqual(plan.vehicles, [])
        self.assertFalse(plan.undo())

    def test_queries(self):
        plan = plan_with((Role.CAV, pose(0, 0)), (Role.TRAFFIC, pose(150, 0)))
        self.assertEqual(plan.outside([-100, 100, -200, 200]), [1])
        self.assertEqual(plan.hit(1.5, 0.5), 0)
        self.assertIsNone(plan.hit(50, 50))
        self.assertEqual(plan.collisions(pose(2, 0)), [0])
        self.assertEqual(plan.centre(), (0, 0))

    def test_save_load_roundtrip(self):
        plan = plan_with((Role.TRAFFIC, pose(10, 0)), (Role.CAV, pose(0, 0, yaw=90)))
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "plan.json")
            plan.save(path)
            again = ScenarioPlan.load(path)
        self.assertEqual(again.town, "Town03")
        self.assertEqual(again.counts(), plan.counts())
        self.assertEqual(again.vehicles[0].role, Role.CAV)   # CAVs first after loading

    def test_figure2_plan_opens(self):
        plan = ScenarioPlan.load("config/spawn_plans/figure2_town03.json")
        self.assertEqual(plan.counts(), {Role.CAV: 4, Role.TRAFFIC: 11})
        self.assertTrue(all(-180 < v.pose["yaw"] <= 180 for v in plan.vehicles))


class TestExport(unittest.TestCase):

    def test_export_gives_a_valid_experiment(self):
        plan = plan_with((Role.CAV, pose(0, -130)), (Role.CAV, pose(0, -110)), (Role.TRAFFIC, pose(20, -130)))
        options = export.ExportOptions("my test!", extra_traffic=4, autopilot=False,
                                       spawn_bbox=[-100, 100, -200, 200])
        with tempfile.TemporaryDirectory() as tmp:
            plan_rel, yaml_rel = export.export(plan, options, root=tmp)
            self.assertTrue(os.path.isfile(os.path.join(tmp, plan_rel)))
            cfg = load_config(DEFAULT_CONFIG, os.path.join(tmp, yaml_rel))
        self.assertEqual(os.path.basename(yaml_rel), "my_test.yaml")
        self.assertEqual((cfg.scenario.num_cavs, cfg.scenario.num_traffic), (2, 5))
        self.assertFalse(cfg.scenario.autopilot)
        self.assertEqual(cfg.scenario.spawn_center, [0.0, -120.0])

    def test_export_needs_a_cav(self):
        with tempfile.TemporaryDirectory() as tmp, self.assertRaises(ValueError):
            export.export(plan_with((Role.TRAFFIC, pose(0, 0))), export.ExportOptions("x"), root=tmp)
        with self.assertRaises(ValueError):
            export.safe_name("  ?? ")


class TestMaps(unittest.TestCase):

    def test_catalog(self):
        with tempfile.TemporaryDirectory() as root:
            xodr = os.path.join(root, catalog.OPENDRIVE_SUBDIR)
            os.makedirs(xodr)
            for name in ("Town01", "Town01_Opt", "Town06"):
                open(os.path.join(xodr, name + ".xodr"), "w").close()
            open(os.path.join(root, catalog.MAPS_SUBDIR, "Town01.umap"), "w").close()
            maps = catalog.list_maps(catalog.find_carla_root(None, root))
        self.assertEqual([(m.name, m.installed) for m in maps], [("Town01", True), ("Town06", False)])
        self.assertIn("edit only", maps[1].label)

    def test_startup_map(self):
        with tempfile.TemporaryDirectory() as root:
            ini = os.path.join(root, startup_map.INI)
            os.makedirs(os.path.dirname(ini))
            with open(ini, "w") as fh:
                fh.write("[/Script/EngineSettings.GameMapsSettings]\n"
                         "GameDefaultMap=/Game/Carla/Maps/Town03.Town03\n"
                         "ServerDefaultMap=/Game/Carla/Maps/Town03.Town03\nOther=1\n")
            self.assertEqual(startup_map.get_startup_map(root), "Town03")
            startup_map.set_startup_map(root, "Town05")
            self.assertEqual(startup_map.get_startup_map(root), "Town05")
            with open(ini) as fh:
                self.assertIn("ServerDefaultMap=/Game/Carla/Maps/Town05.Town05", fh.read())
            with open(ini + ".bak") as fh:
                self.assertIn("Town03.Town03", fh.read())

    def test_georef(self):
        g = Georef.for_bounds((-150, 250, -210, 210))
        self.assertEqual(g.ppm, 6.0)
        self.assertEqual((g.width, g.height), (2400, 2520))
        x, y = g.to_world(*g.to_px(12.5, -40.0))
        self.assertAlmostEqual(x, 12.5)
        self.assertAlmostEqual(y, -40.0)
        big = Georef.for_bounds((-500, 500, -400, 450))      # large maps get fewer pixels per metre
        self.assertLessEqual(max(big.width, big.height), 4200)

    def test_wrap_degrees(self):
        self.assertAlmostEqual(wrap_degrees(-268.6), 91.4)
        self.assertEqual(wrap_degrees(180.0), 180.0)
        self.assertEqual(wrap_degrees(-180.0), 180.0)
        self.assertAlmostEqual(wrap_degrees(358.9), -1.1)


if __name__ == "__main__":
    unittest.main()
