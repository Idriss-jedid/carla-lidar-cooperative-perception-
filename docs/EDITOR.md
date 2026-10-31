# Scenario editor

A window to place CAVs and traffic vehicles by hand, on **any CARLA map**, then record the scene.

```bash
python main.py editor                                            # last map used
python main.py editor --map Town05
python main.py editor --plan config/spawn_plans/figure2_town03.json
```

![Editor on Town03 with the aerial photo and the paper's Figure 2 plan](images/editor_town03_aerial.png)

## How it works

| Part | What it does |
|---|---|
| **Map list** | Every map that has an OpenDRIVE file in the CARLA folder (`CarlaUE4/Content/Carla/Maps/OpenDrive/*.xodr`). The road network is read on the client (`carla.Map(name, xodr)`), so **any map can be edited while CARLA runs another one, or is closed**. Maps whose level is not installed (Town06 and Town07 in the base 0.9.12 package) are marked *edit only*. |
| **Roads background** | Lanes, sidewalks, lane markings and driving-direction arrows, drawn from OpenDRIVE. Rendered once per map into `data/maps/<Town>_roads.png`. |
| **Aerial photo background** | A near-orthographic photo taken from CARLA (camera 1000 m up, narrow field of view, tiles stitched). Needs that map loaded in CARLA; taken once and stored in `data/maps/<Town>_aerial.png`. Aligned with the roads image to the pixel. |
| **Placing** | A click snaps to the nearest driving lane (≤ 6 m) and takes the lane's heading, height and slope. A dashed outline under the cursor shows where the vehicle would go: green = allowed, red = not allowed (reason in the status bar). |
| **Speed and route** | For the selected vehicle: an own speed, a route drawn by clicking points on the roads, and what it does at the last point (stop, or keep driving with the Traffic Manager). See [Routes and speeds](#routes-and-speeds). |
| **Rules** | Vehicles never overlap (4.8 × 2.1 m footprint + 0.6 m gap; different road levels are ignored). Optional spawn area (default: the paper's box on Town03). Optional ban on junctions. |
| **CARLA tab** | *Show the plan in the CARLA window* (boxes, arrows and names drawn for 60 s, camera moved above). *Test spawn* (spawns every vehicle, reports blocked ones, removes them). *Aerial photo*. *Start CARLA on this map next time* (writes `DefaultEngine.ini`, keeps a backup; restart CARLA afterwards). |
| **Export / Record** | Writes `config/spawn_plans/<name>.json` and `config/experiments/custom/<name>.yaml` (map, counts, spawn plan, spawn area, autopilot, frames), checks the YAML like the recorder does, and can start `python main.py record` in a new console. *Extra random traffic* adds vehicles at random CARLA spawn points on top of the planned ones. |

## Controls

| Action | Mouse / key |
|---|---|
| Add a vehicle | left click on a road (role: **1** = CAV, **2** = traffic) |
| Select / move | left click on a vehicle / drag it (it slides along the lanes) |
| Switch CAV ↔ traffic | **R** |
| Delete | **Del** |
| Draw a route for the selected vehicle | **T**, then left clicks on the roads in driving order; **Enter** / **Esc** ends, **Backspace** removes the last point |
| Zoom / pan | mouse wheel / right or middle drag |
| Fit vehicles | **F** |
| Undo / redo | **Ctrl+Z** / **Ctrl+Y** (a whole drag is one step) |
| New / open / save plan | **Ctrl+N** / **Ctrl+O** / **Ctrl+S** |
| OpenSCENARIO | File → Import / Export OpenSCENARIO (.xosc), see [OPENSCENARIO.md](OPENSCENARIO.md) |

## Routes and speeds

CARLA 0.9.12's Traffic Manager can neither follow a given path nor hold a given speed, so the project drives routed vehicles itself (`backend/actors/route_driver.py`).

| Vehicle has | How it drives |
|---|---|
| a route (+ speed, 30 km/h by default) | Follows the path through the clicked points along the lanes (any branch at junctions, lane changes where the markings allow). Holds the speed exactly (CARLA constant-velocity mode), slower in sharp turns (≤ 2.5 m/s² sideways), speeds up at 2.5 m/s², brakes at 3 m/s² for a red light on its route (stops 1 m before the line), for a vehicle on its path ahead (3 m gap) and at the last point. **Stop**: it stays at the last point. **Keep driving**: the Traffic Manager takes over there. |
| only a speed | The Traffic Manager drives it, choosing its own way, at about that speed (0.9.12 only takes a percentage of the road's limit, so the speed is exact on the start road). Drives even when autopilot is off. |
| neither | The Traffic Manager drives it if autopilot is on, else it is parked. |

![Routes on the Figure 2 junction: cav_0 west (stop), cav_1 right turn up the slope (keeps driving), cav_2 south; cav_3 speed only](images/editor_routes.png)

**Timing.** Routed vehicles hold their planned pose during the recorder's warm-up and start, at their planned speed (lower if a red light or the end is close), when the recording starts: the first saved frame is the scene you drew, already moving. Traffic Manager vehicles drive during the warm-up as before.

**In the editor.** Each click snaps to the lane under the cursor; the green circle and arrow show where and in which direction. A point that cannot be reached from the previous one along the lanes (other side of the road, one-way street) is refused with the reason in the status bar. The traced path is drawn in the vehicle's colour (thick when selected); a red dot marks a stop, an arrow marks "keep driving". The table's *km/h, route* column lists speed and route length; the box under the table gives the length and the driving time without stops. Export refuses a plan with a route that cannot be driven. *Show the plan in the CARLA window* also draws the routes.

**In the plan file** (optional keys of a vehicle):

```json
{"role": "cav", "index": 0, "pose": {...},
 "speed_kmh": 30, "route": [[-59.99, -139.56]], "route_end": "stop"}
```

`route_end` is `stop` or `autopilot`. The route is traced again on the server's map when recording; a route that cannot be driven stops the run before anything is spawned.

**Checked in CARLA 0.9.12 (Town03, junction 1221):** speed within 0.6 km/h of the set speed while cruising; on the path within 0.05 m on straight roads and 0.44 m in a 90° junction turn; stop at the last point 0.0 m from it; waits at red (stop line 1 m ahead of the bumper) and goes on green; climbs the sloped +x leg at the set 40 km/h; brakes for a Traffic Manager vehicle crossing its path; hands over to the Traffic Manager at the end of a "keep driving" route.

## Recording on another map

1. Pick the map in the editor and place the vehicles (CARLA can keep running Town03).
2. CARLA tab → *Start CARLA on this map next time*, then close and reopen CARLA.
3. Optional: *Take aerial photo of this map* (once).
4. *Test spawn*, then Export / Record → *Export and record now*.

The exported YAML sets `server.town`, so `python main.py record` refuses to run if CARLA has another map loaded.

![Editor on Town05 (roads background)](images/editor_town05.png)

## Files

| Path | Content |
|---|---|
| `config/spawn_plans/*.json` | Spawn plans (`map`, `vehicles[role, index, pose, road_id, lane_id, speed_kmh?, route?, route_end?]`), the format read by `scenario.spawn_plan` |
| `config/experiments/custom/*.yaml` | Experiments exported by the editor |
| `data/maps/` | Cached road images, aerial photos and OpenDRIVE files fetched from the server (not in git) |
| `data/editor_settings.json` | CARLA folder and last map (not in git) |

The CARLA folder is taken from `server.carla_root`, then from `data/editor_settings.json`, then `$CARLA_ROOT`; otherwise the editor asks for it once (File → CARLA folder...). Without it, the editor still works on the map currently loaded in CARLA (its OpenDRIVE is fetched from the server).
