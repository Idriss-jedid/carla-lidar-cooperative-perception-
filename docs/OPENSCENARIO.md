# OpenSCENARIO (.xosc) import and export

Scenarios can be exchanged with the other CARLA tools through OpenSCENARIO 1.0 files:

| Tool | Direction |
|---|---|
| [Traffic Generation Editor](https://github.com/carla-simulator/traffic-generation-editor) (QGIS plugin) | build a scene in QGIS → *Export OpenSCENARIO* → **import here** |
| [ScenarioRunner 0.9.12](https://github.com/carla-simulator/scenario_runner/tree/v0.9.12) | **export here** → run with `scenario_runner.py --openscenario`; its example files can be imported |
| This repository's [editor](EDITOR.md) | File → Import / Export OpenSCENARIO |

```bash
# .xosc -> spawn plan + experiment, then record as usual
python main.py xosc-import figure2_qgis_trial.xosc --name qgis_trial --extra-traffic 9
python main.py record --config config/experiments/custom/qgis_trial.yaml --name qgis_trial

# spawn plan -> .xosc
python main.py xosc-export config/spawn_plans/figure2_town03.json figure2.xosc
```

Only the LiDAR backend of this repository records data; ScenarioRunner is not needed. Do not run ScenarioRunner and `main.py record` on the same CARLA world at the same time: both control actors and the simulation clock.

## What is imported

| In the file | Here |
|---|---|
| `RoadNetwork/LogicFile` (`Town03`, `Town03.xodr`, `/Game/Carla/Maps/Town03`) | `server.town` of the experiment |
| Vehicle with `Property type="ego_vehicle"`, or named `Ego…` / `hero…` | CAV (`cav_0`, `cav_1`, … in file order) |
| Any other vehicle | traffic |
| Start pose: `WorldPosition` | used as is (frame conversion below) |
| Start pose: `LanePosition`, `RoadPosition` | computed on the map like ScenarioRunner (waypoint of road/lane/s, offset sideways) |
| Height | taken from the road under the vehicle (files often store z = 0; on Town03's slope that would put the car inside the road) |

Reported as warnings, not imported: pedestrians, bicycles / motorbikes, props (`MiscObject`), catalog references, relative positions, routes and maneuvers (draw routes in the editor instead). Initial speeds (`AbsoluteTargetSpeed` in `Init`) become the vehicle's planned speed, and planned speeds are exported as initial speeds; routes are not exported. Vehicles more than 2.5 m from a driving lane, or facing more than 45° away from their lane, are imported and reported.

## Coordinate frames

OpenSCENARIO uses the right-handed OpenDRIVE frame (y to the left, heading counter-clockwise, radians); CARLA is left-handed (y to the right, yaw clockwise, degrees). As in ScenarioRunner 0.9.12 (`OpenScenarioParser.convert_position_to_transform`):

```
CARLA x = x      CARLA y = -y      CARLA yaw = -degrees(h)
```

unless the `FileHeader` description starts with `CARLA:`, which marks a file already in CARLA coordinates (all of ScenarioRunner's examples). The QGIS editor writes OpenDRIVE (ENU) coordinates. Checked on Town03's OpenDRIVE: road 3 starts at (y 124.9, hdg −89.8°) in the file and (y −124.9, yaw +89.8°) in CARLA.

Exported files use the OpenDRIVE frame, `Ego_<n>` / `Vehicle_<n>` names and the vehicle / controller layout of the QGIS editor, so they open there and validate against ScenarioRunner's `OpenSCENARIO.xsd`. Every vehicle gets the `npc_vehicle_control` controller and the story stops after `--duration` seconds.

## Using the QGIS editor on this PC

The QGIS plugin needs the `ad_map_access` Python package, which is only published for Linux (no Windows build of any version on PyPI). On Windows it therefore has to run inside WSL (Ubuntu 22.04: QGIS 3 + `ad_map_access` 3.0.0 for Python 3.10), or on a Linux machine. The `.xosc` files it exports are imported here as described above. Without QGIS, the [scenario editor](EDITOR.md) of this repository covers the same placement task on Windows.
