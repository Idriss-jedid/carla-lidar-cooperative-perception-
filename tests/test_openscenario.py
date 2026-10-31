import math
import os
import tempfile
import unittest
from types import SimpleNamespace

from cavlidar.backend.interop import openscenario as osc
from cavlidar.core.enums import Role
from cavlidar.editor.model import ScenarioPlan
from cavlidar.editor.openscenario_bridge import plan_from_xosc, xosc_from_plan

# Shaped like the output of CARLA's QGIS Traffic Generation Editor (OpenDRIVE / ENU frame)
QGIS_FILE = """<?xml version="1.0"?>
<OpenSCENARIO>
  <FileHeader revMajor="1" revMinor="0" date="2026-10-31T12:00:00" description="Generated OpenSCENARIO File"
              author="QGIS OSCGenerator Plugin"/>
  <ParameterDeclarations><ParameterDeclaration name="speed" type="double" value="8.0"/></ParameterDeclarations>
  <CatalogLocations/>
  <RoadNetwork><LogicFile filepath="Town03"/><SceneGraphFile filepath=""/></RoadNetwork>
  <Entities>
    <ScenarioObject name="Ego_1"><Vehicle name="vehicle.tesla.model3" vehicleCategory="car">
      <Properties><Property name="type" value="ego_vehicle"/></Properties></Vehicle></ScenarioObject>
    <ScenarioObject name="Vehicle_1"><Vehicle name="vehicle.audi.a2" vehicleCategory="car">
      <Properties><Property name="type" value="simulation"/></Properties></Vehicle></ScenarioObject>
    <ScenarioObject name="Vehicle_2"><Vehicle name="vehicle.gazelle.omafiets" vehicleCategory="bicycle">
      <Properties/></Vehicle></ScenarioObject>
    <ScenarioObject name="Pedestrian_1"><Pedestrian name="walker" model="walker" mass="90"
      pedestrianCategory="pedestrian"/></ScenarioObject>
    <ScenarioObject name="Vehicle_3"><Vehicle name="vehicle.audi.tt" vehicleCategory="car">
      <Properties/></Vehicle></ScenarioObject>
  </Entities>
  <Storyboard><Init><Actions>
    <Private entityRef="Ego_1">
      <PrivateAction><TeleportAction><Position>
        <WorldPosition x="5.2" y="138.2" z="0.2" h="3.1206"/></Position></TeleportAction></PrivateAction>
      <PrivateAction><LongitudinalAction><SpeedAction>
        <SpeedActionDynamics dynamicsShape="step" value="0.1" dynamicsDimension="distance"/>
        <SpeedActionTarget><AbsoluteTargetSpeed value="$speed"/></SpeedActionTarget>
      </SpeedAction></LongitudinalAction></PrivateAction>
    </Private>
    <Private entityRef="Vehicle_1">
      <PrivateAction><TeleportAction><Position>
        <WorldPosition x="9.1" y="102.8" z="0.2" h="1.5464"/></Position></TeleportAction></PrivateAction>
    </Private>
    <Private entityRef="Vehicle_2">
      <PrivateAction><TeleportAction><Position>
        <WorldPosition x="0" y="0" z="0" h="0"/></Position></TeleportAction></PrivateAction>
    </Private>
    <Private entityRef="Vehicle_3">
      <PrivateAction><TeleportAction><Position>
        <LanePosition roadId="28" laneId="-2" s="10.0" offset="0.5"/></Position></TeleportAction></PrivateAction>
    </Private>
  </Actions></Init>
  <Story name="s"><Act name="a"><ManeuverGroup maximumExecutionCount="1" name="m">
    <Actors selectTriggeringEntities="false"/><Maneuver name="follow"/></ManeuverGroup>
    <StartTrigger/></Act></Story>
  <StopTrigger/></Storyboard>
</OpenSCENARIO>
"""


def write(text):
    handle, path = tempfile.mkstemp(suffix=".xosc")
    with os.fdopen(handle, "w", encoding="utf-8") as fh:
        fh.write(text)
    return path


class TestRead(unittest.TestCase):

    def setUp(self):
        self.path = write(QGIS_FILE)
        self.scenario = osc.read_xosc(self.path)

    def tearDown(self):
        os.remove(self.path)

    def test_map_roles_and_skipped_objects(self):
        s = self.scenario
        self.assertEqual(s.town, "Town03")
        self.assertEqual([(v.name, v.role) for v in s.vehicles],
                         [("Ego_1", Role.CAV), ("Vehicle_1", Role.TRAFFIC), ("Vehicle_3", Role.TRAFFIC)])
        text = " ".join(s.warnings)
        for expected in ("Vehicle_2 (bicycle)", "Pedestrian_1 (Pedestrian)", "Maneuvers", "Initial speeds"):
            self.assertIn(expected, text)

    def test_opendrive_frame_is_flipped_like_scenario_runner(self):
        ego = self.scenario.vehicles[0]
        self.assertAlmostEqual(ego.pose["x"], 5.2)
        self.assertAlmostEqual(ego.pose["y"], -138.2)          # y -> -y
        self.assertAlmostEqual(ego.pose["yaw"], -178.8, 1)     # yaw -> -degrees(h)
        self.assertEqual(ego.speed_mps, 8.0)                   # $speed parameter resolved

    def test_carla_frame_header_is_not_flipped(self):
        path = write(QGIS_FILE.replace('description="Generated', 'description="CARLA:Generated'))
        try:
            ego = osc.read_xosc(path).vehicles[0]
        finally:
            os.remove(path)
        self.assertAlmostEqual(ego.pose["y"], 138.2)
        self.assertAlmostEqual(ego.pose["yaw"], 178.8, 1)

    def test_lane_position_kept_for_the_map(self):
        lp = self.scenario.vehicles[2].lane_position
        self.assertIsNone(self.scenario.vehicles[2].pose)
        self.assertEqual((lp["road_id"], lp["lane_id"], lp["s"], lp["offset"]), (28, -2, 10.0, 0.5))

    def test_lane_position_offset_like_scenario_runner(self):
        wp = SimpleNamespace(transform=SimpleNamespace(location=SimpleNamespace(x=10.0, y=20.0, z=1.0),
                                                       rotation=SimpleNamespace(yaw=0.0, pitch=0.0, roll=0.0)))
        road_map = SimpleNamespace(map=SimpleNamespace(get_waypoint_xodr=lambda r, l, s: wp))
        lp = {"road_id": 1, "lane_id": -1, "s": 0.0, "offset": 2.0, "dyaw": 90.0, "dpitch": 0.0, "droll": 0.0}
        pose = osc.lane_position_to_pose(road_map, lp)
        # heading +y after the 90 deg turn; the offset goes along (-fy, fx) = (-1, 0)
        self.assertAlmostEqual(pose["x"], 8.0)
        self.assertAlmostEqual(pose["y"], 20.0)
        self.assertAlmostEqual(pose["yaw"], 90.0)


class TestWrite(unittest.TestCase):

    def test_plan_roundtrip(self):
        plan = ScenarioPlan.load("config/spawn_plans/figure2_town03.json")
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "figure2.xosc")
            xosc_from_plan(plan, path)
            with open(path, encoding="utf-8") as fh:
                text = fh.read()
            back, warnings = plan_from_xosc(path)
        self.assertIn('name="Ego_1"', text)
        self.assertIn('name="Vehicle_11"', text)
        self.assertIn('value="ego_vehicle"', text)
        self.assertEqual(warnings, [])
        self.assertEqual(back.counts(), plan.counts())
        for a, b in zip(plan.vehicles, back.vehicles):
            self.assertEqual(a.role, b.role)
            for key in ("x", "y"):
                self.assertAlmostEqual(a.pose[key], b.pose[key], 2)
            self.assertAlmostEqual(math.cos(math.radians(a.pose["yaw"] - b.pose["yaw"])), 1.0, 5)

    def test_frames(self):
        pose = {"x": 1.0, "y": -2.0, "z": 0.5, "roll": 0.0, "pitch": 3.0, "yaw": 30.0}
        back = osc.to_carla(**{k: v for k, v in osc.from_carla(pose).items()})
        for key in pose:
            self.assertAlmostEqual(back[key], pose[key])
        self.assertEqual(osc._town("/Game/Carla/Maps/Town05"), "Town05")
        self.assertEqual(osc._town("C:\\maps\\Town04.xodr"), "Town04")


if __name__ == "__main__":
    unittest.main()
