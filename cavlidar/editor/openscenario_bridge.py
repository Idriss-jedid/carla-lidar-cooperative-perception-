"""
OpenSCENARIO <-> ScenarioPlan.

Import keeps each vehicle's x, y and heading from the file and takes the height
and slope from the road under it (files often store z=0, which would put a car
inside a sloped road). Vehicles that are off the lanes, or face against their
lane, are kept but reported.
"""
from ..backend.interop import openscenario
from ..backend.maps.road_map import wrap_degrees
from ..core.enums import Role
from .model import PlannedVehicle, ScenarioPlan

MAX_LANE_DISTANCE = 2.5   # metres: farther than this from a lane centre is reported
MAX_HEADING_ERROR = 45.0  # degrees between the vehicle and its lane


def plan_from_xosc(path, road_map=None):
    """(ScenarioPlan, warnings) from an .xosc file. `road_map` (optional) fixes heights."""
    scenario = openscenario.read_xosc(path)
    warnings = list(scenario.warnings)
    if road_map is not None and road_map.name != scenario.town:
        raise ValueError("The file is for {}, the road map given is {}".format(scenario.town, road_map.name))
    vehicles = []
    for v in scenario.vehicles:
        pose = v.pose
        if pose is None:  # given as road / lane / s
            pose = openscenario.lane_position_to_pose(road_map, v.lane_position) if road_map else None
            if pose is None:
                warnings.append("{} skipped: its LanePosition {} {}".format(
                    v.name, v.lane_position, "is not on this map" if road_map else "needs the map's OpenDRIVE"))
                continue
        pose = {k: round(val, 3) for k, val in pose.items()}
        pose["yaw"] = round(wrap_degrees(pose["yaw"]), 2)
        road_id = lane_id = 0
        in_junction = False
        if road_map is not None:
            lane = road_map.snap(pose["x"], pose["y"], max_distance=MAX_LANE_DISTANCE)
            if lane is None:
                warnings.append("{}: more than {} m from a driving lane".format(v.name, MAX_LANE_DISTANCE))
                pose["z"] = round(road_map.road_height(pose["x"], pose["y"]), 2)
            else:
                pose["z"], pose["pitch"] = lane.pose["z"], lane.pose["pitch"]
                road_id, lane_id, in_junction = lane.road_id, lane.lane_id, lane.in_junction
                if abs(wrap_degrees(pose["yaw"] - lane.pose["yaw"])) > MAX_HEADING_ERROR:
                    warnings.append("{}: heading {:.0f} deg but its lane goes {:.0f} deg (wrong way?)".format(
                        v.name, pose["yaw"], lane.pose["yaw"]))
        vehicles.append(PlannedVehicle(v.role, pose, road_id, lane_id, in_junction))
    # CAVs first, in file order, so Ego_1 becomes cav_0
    vehicles.sort(key=lambda pv: pv.role != Role.CAV)
    plan = ScenarioPlan(scenario.town, vehicles, source="Imported from OpenSCENARIO: {}".format(path))
    return plan, warnings


def xosc_from_plan(plan, path, cav_model="vehicle.tesla.model3", traffic_model="vehicle.*", duration=60.0):
    """Write the plan as an .xosc file (QGIS Traffic Generation Editor / ScenarioRunner)."""
    vehicles = [openscenario.XoscVehicle(plan.name(i), v.role,
                                         cav_model if v.role == Role.CAV else traffic_model, v.pose)
                for i, v in enumerate(plan.vehicles)]
    return openscenario.write_xosc(path, plan.town, vehicles, duration=duration)
