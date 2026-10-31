"""
OpenSCENARIO 1.0 (.xosc) files: read the vehicles' start poses, write a plan.

Compatible with the CARLA tools:
  * Traffic Generation Editor (QGIS plugin, carla-simulator/traffic-generation-editor):
    entities Ego_<n> (Property type=ego_vehicle) and Vehicle_<n>, start pose in a
    TeleportAction / WorldPosition, initial speed in an AbsoluteTargetSpeed.
  * ScenarioRunner 0.9.12 (srunner/tools/openscenario_parser.py).

Coordinates. OpenSCENARIO uses the right-handed OpenDRIVE frame (y to the left,
heading counter-clockwise, radians); CARLA is left-handed (y to the right, yaw
clockwise, degrees). Like ScenarioRunner: CARLA y = -y and yaw = -degrees(h),
except when the FileHeader description starts with "CARLA:", which marks a file
already written in CARLA coordinates.
"""
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from datetime import datetime
from typing import List, Optional

from ...core.enums import Role

CARLA_FRAME_TAG = "CARLA:"
EGO_TYPE = "ego_vehicle"
TWO_WHEELERS = ("bicycle", "motorbike")


@dataclass
class XoscVehicle:
    name: str          # entity name in the file, e.g. Ego_1
    role: Role         # CAV for ego vehicles, TRAFFIC otherwise
    model: str         # CARLA blueprint, e.g. vehicle.tesla.model3
    pose: Optional[dict]  # CARLA frame: x, y, z, roll, pitch, yaw (degrees); None if given on a lane
    speed_mps: float = 0.0
    lane_position: Optional[dict] = None  # road_id, lane_id, s, offset, dyaw/dpitch/droll (CARLA degrees)


def lane_position_to_pose(road_map, lp):
    """
    Pose of an OpenSCENARIO LanePosition, computed like ScenarioRunner 0.9.12:
    waypoint (road, lane, s) + relative orientation + `offset` metres sideways.
    Returns None if that lane position does not exist on the map.
    """
    wp = road_map.map.get_waypoint_xodr(lp["road_id"], lp["lane_id"], lp["s"])
    if wp is None:
        return None
    t = wp.transform
    yaw = t.rotation.yaw + lp["dyaw"]
    pitch = t.rotation.pitch + lp["dpitch"]
    roll = t.rotation.roll + lp["droll"]
    fx, fy = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    return {"x": t.location.x - lp["offset"] * fy, "y": t.location.y + lp["offset"] * fx,
            "z": t.location.z, "roll": roll, "pitch": pitch, "yaw": yaw}


@dataclass
class XoscScenario:
    town: str
    vehicles: List[XoscVehicle]
    warnings: List[str] = field(default_factory=list)


# ---------------------------------------------------------------------- frames

def to_carla(x, y, z, h, p=0.0, r=0.0, carla_frame=False):
    """OpenSCENARIO WorldPosition -> CARLA pose dict (same rule as ScenarioRunner)."""
    yaw, pitch, roll = math.degrees(h), math.degrees(p), math.degrees(r)
    if not carla_frame:
        y, yaw = -y, -yaw
    return {"x": x, "y": y, "z": z, "roll": roll, "pitch": pitch, "yaw": yaw}


def from_carla(pose):
    """CARLA pose dict -> OpenSCENARIO WorldPosition attributes (OpenDRIVE frame)."""
    return {"x": pose["x"], "y": -pose["y"], "z": pose.get("z", 0.0), "h": -math.radians(pose["yaw"]),
            "p": math.radians(pose.get("pitch", 0.0)), "r": math.radians(pose.get("roll", 0.0))}


# ---------------------------------------------------------------------- reading

def read_xosc(path):
    """Vehicles and their start poses (CARLA frame). Unsupported content is listed in .warnings."""
    root = ET.parse(path).getroot()
    params = {p.get("name"): p.get("value") for p in root.iter("ParameterDeclaration")}
    value = lambda text, default=0.0: _number(text, params, default)  # noqa: E731
    warnings = []

    header = root.find("FileHeader")
    carla_frame = header is not None and header.get("description", "").startswith(CARLA_FRAME_TAG)
    logic = root.find("RoadNetwork/LogicFile")
    town = _town(logic.get("filepath", "") if logic is not None else "")
    if not town:
        warnings.append("No map in RoadNetwork/LogicFile")

    inits = {p.get("entityRef"): p for p in root.findall("Storyboard/Init/Actions/Private")}
    vehicles = []
    for obj in root.iter("ScenarioObject"):
        name = obj.get("name")
        vehicle = obj.find("Vehicle")
        if vehicle is None:
            kind = next((c.tag for c in obj), "object")
            warnings.append("{} ({}) skipped: only vehicles are imported".format(name, kind))
            continue
        if vehicle.get("vehicleCategory") in TWO_WHEELERS:
            warnings.append("{} ({}) skipped: traffic uses 4-wheel vehicles only".format(
                name, vehicle.get("vehicleCategory")))
            continue
        init = inits.get(name)
        position = init.find(".//TeleportAction/Position") if init is not None else None
        world = position.find("WorldPosition") if position is not None else None
        lane = position.find("LanePosition") if position is not None else None
        road = position.find("RoadPosition") if position is not None else None
        pose = lane_position = None
        if world is not None:
            pose = to_carla(value(world.get("x")), value(world.get("y")), value(world.get("z")),
                            value(world.get("h")), value(world.get("p")), value(world.get("r")), carla_frame)
        elif lane is not None or road is not None:
            node = lane if lane is not None else road
            orientation = node.find("Orientation")
            angle = lambda key: math.degrees(value(orientation.get(key))) if orientation is not None else 0.0  # noqa
            if lane is not None:
                lane_id, offset = int(value(lane.get("laneId"))), value(lane.get("offset"))
            else:  # RoadPosition: reference line (lane 0), t > 0 to the left in OpenDRIVE
                lane_id, offset = 0, value(road.get("t")) * (1.0 if carla_frame else -1.0)
            lane_position = {"road_id": int(value(node.get("roadId"))), "lane_id": lane_id,
                             "s": value(node.get("s")), "offset": offset,
                             "dyaw": angle("h") * (1.0 if carla_frame else -1.0),
                             "dpitch": angle("p"), "droll": angle("r")}
        else:
            warnings.append("{} skipped: start position must be a World-, Lane- or RoadPosition".format(name))
            continue
        speed = init.find(".//AbsoluteTargetSpeed")
        is_ego = any(p.get("name") == "type" and p.get("value") == EGO_TYPE
                     for p in vehicle.iter("Property")) or name.lower().startswith(("ego", "hero"))
        vehicles.append(XoscVehicle(name, Role.CAV if is_ego else Role.TRAFFIC, vehicle.get("name", ""), pose,
                                    value(speed.get("value")) if speed is not None else 0.0, lane_position))

    if root.find(".//Maneuver") is not None or root.find(".//Route") is not None:
        warnings.append("Maneuvers / routes are not imported: vehicles drive with the Traffic Manager")
    if any(v.speed_mps for v in vehicles):
        warnings.append("Initial speeds are not imported: vehicles start at rest")
    return XoscScenario(town, vehicles, warnings)


def _number(text, params, default):
    if text is None:
        return default
    if text.startswith("$"):
        text = params.get(text[1:], default)
    return float(text)


def _town(filepath):
    """'Town03' from 'Town03', 'Town03.xodr', '/Game/Carla/Maps/Town03' or a path."""
    name = filepath.replace("\\", "/").rsplit("/", 1)[-1]
    return name[:-5] if name.lower().endswith(".xodr") else name


# ---------------------------------------------------------------------- writing

def write_xosc(path, town, vehicles, description="Scenario from carla-lidar-cooperative-perception",
               duration=60.0, controller="npc_vehicle_control"):
    """
    Write an OpenSCENARIO 1.0 file that the QGIS editor can open and ScenarioRunner can run.
    `vehicles`: list of XoscVehicle (poses in the CARLA frame). The scenario stops after `duration` s.
    """
    root = ET.Element("OpenSCENARIO")
    ET.SubElement(root, "FileHeader", revMajor="1", revMinor="0",
                  date=datetime.now().strftime("%Y-%m-%dT%H:%M:%S"), description=description,
                  author="carla-lidar-cooperative-perception")
    ET.SubElement(root, "ParameterDeclarations")
    ET.SubElement(root, "CatalogLocations")
    network = ET.SubElement(root, "RoadNetwork")
    ET.SubElement(network, "LogicFile", filepath=town)
    ET.SubElement(network, "SceneGraphFile", filepath="")

    entities = ET.SubElement(root, "Entities")
    names = _entity_names(vehicles)
    for v, name in zip(vehicles, names):
        _vehicle_entity(entities, name, v)

    story = ET.SubElement(root, "Storyboard")
    actions = ET.SubElement(ET.SubElement(story, "Init"), "Actions")
    for v, name in zip(vehicles, names):
        private = ET.SubElement(actions, "Private", entityRef=name)
        teleport = ET.SubElement(ET.SubElement(private, "PrivateAction"), "TeleportAction")
        ET.SubElement(ET.SubElement(teleport, "Position"), "WorldPosition",
                      **{k: _fmt(val) for k, val in from_carla(v.pose).items()})
        _controller(private, name, controller)
        if v.speed_mps > 0:
            speed = ET.SubElement(ET.SubElement(ET.SubElement(private, "PrivateAction"), "LongitudinalAction"),
                                  "SpeedAction")
            ET.SubElement(speed, "SpeedActionDynamics", dynamicsShape="step", value="0.1",
                          dynamicsDimension="distance")
            ET.SubElement(ET.SubElement(speed, "SpeedActionTarget"), "AbsoluteTargetSpeed",
                          value=_fmt(v.speed_mps))

    act = ET.SubElement(ET.SubElement(story, "Story", name="Story"), "Act", name="Act")
    group = ET.SubElement(act, "ManeuverGroup", maximumExecutionCount="1", name="Vehicles")
    ET.SubElement(group, "Actors", selectTriggeringEntities="false")
    _time_trigger(ET.SubElement(act, "StartTrigger"), "Start", 0.0)
    _time_trigger(ET.SubElement(story, "StopTrigger"), "End", duration)

    _indent(root)
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)
    return path


def _entity_names(vehicles):
    """Ego_<n> for CAVs and Vehicle_<n> for traffic, numbered from 1 like the QGIS editor."""
    counts = {Role.CAV: 0, Role.TRAFFIC: 0}
    names = []
    for v in vehicles:
        counts[v.role] += 1
        names.append("{}_{}".format("Ego" if v.role == Role.CAV else "Vehicle", counts[v.role]))
    return names


def _vehicle_entity(entities, name, v):
    vehicle = ET.SubElement(ET.SubElement(entities, "ScenarioObject", name=name), "Vehicle",
                            name=v.model, vehicleCategory="car")
    box = ET.SubElement(vehicle, "BoundingBox")
    ET.SubElement(box, "Center", x="1.5", y="0.0", z="0.9")
    ET.SubElement(box, "Dimensions", width="2.1", length="4.5", height="1.8")
    ET.SubElement(vehicle, "Performance", maxSpeed="69.444", maxAcceleration="200", maxDeceleration="10.0")
    axles = ET.SubElement(vehicle, "Axles")
    for tag, steer, x in (("FrontAxle", "0.5", "3.1"), ("RearAxle", "0.0", "0.0")):
        ET.SubElement(axles, tag, maxSteering=steer, wheelDiameter="0.6", trackWidth="1.8",
                      positionX=x, positionZ="0.3")
    ET.SubElement(ET.SubElement(vehicle, "Properties"), "Property", name="type",
                  value=EGO_TYPE if v.role == Role.CAV else "simulation")


def _controller(private, name, module):
    action = ET.SubElement(ET.SubElement(private, "PrivateAction"), "ControllerAction")
    controller = ET.SubElement(ET.SubElement(action, "AssignControllerAction"), "Controller",
                               name="Agent_" + name)
    ET.SubElement(ET.SubElement(controller, "Properties"), "Property", name="module", value=module)
    override = ET.SubElement(action, "OverrideControllerValueAction")
    for tag in ("Throttle", "Brake", "Clutch", "ParkingBrake", "SteeringWheel"):
        ET.SubElement(override, tag, value="0", active="false")
    ET.SubElement(override, "Gear", number="0", active="false")


def _time_trigger(trigger, name, seconds):
    condition = ET.SubElement(ET.SubElement(trigger, "ConditionGroup"), "Condition", name=name, delay="0",
                              conditionEdge="rising")
    ET.SubElement(ET.SubElement(condition, "ByValueCondition"), "SimulationTimeCondition",
                  value=_fmt(seconds), rule="greaterThan")


def _fmt(number):
    return "{:.6g}".format(number) if abs(number) < 1e5 else "{:.2f}".format(number)


def _indent(elem, level=0):
    """Pretty-print (ElementTree.indent only exists from Python 3.9)."""
    pad = "\n" + "  " * level
    if len(elem):
        if not (elem.text or "").strip():
            elem.text = pad + "  "
        for child in elem:
            _indent(child, level + 1)
        if not (child.tail or "").strip():
            child.tail = pad
    if level and not (elem.tail or "").strip():
        elem.tail = pad
