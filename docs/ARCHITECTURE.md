# Architecture

The code is split in three layers. Each layer only calls the one below it.

| Layer | Folder | Who uses it | Depends on CARLA? |
|---|---|---|---|
| **Front** | `main.py`, `cavlidar/api.py`, `cavlidar/editor/` | you | no (imports backend lazily) |
| **Core** | `cavlidar/core/` | everything | no |
| **Backend** | `cavlidar/backend/` | `api.py` only | `simulator/`, `actors/`, `recording/recorder.py` yes; the rest no |

Design rules:

- **One place for parameters.** Every tunable value is a field in `core/config.py`, set from YAML. No magic numbers inside functions.
- **Enums instead of free text.** Options with a fixed set of values (`camera`, `formats`, roles, plot views) are enums, so a typo fails immediately with the list of valid choices.
- **Fail early, explain why.** Invalid parameters raise `ConfigError` before CARLA is touched; CARLA problems raise `ServerUnavailable`, `MapMismatch`, `SpawnError` or `SensorTimeout` with a message saying what to do.
- **Always clean up.** `SimulationSession` is a context manager: whatever happens (error, Ctrl+C), sensors and vehicles are destroyed and CARLA goes back to asynchronous mode.
- **Offline parts don't need CARLA.** `geometry`, `storage` and `visualization` only use numpy/matplotlib, so recorded data can be processed in another environment (e.g. the PyTorch one).

---

## Front: `cavlidar/api.py`

| Function | What it does |
|---|---|
| `load_config(*experiment_files, overrides=None)` | Loads `config/default.yaml`, then each experiment file, then overrides like `"lidar.channels=64"`. Validates everything and logs warnings (e.g. partial sweeps). Returns a `Config`. |
| `check_connection(cfg)` | Connects, compares client/server versions, checks the map, counts spawn points, warns if the world was left in synchronous mode. Returns a dict. |
| `drive_cars(cfg, seconds=40)` | Spawns the scenario's CAVs and traffic with autopilot and paces the simulation to real time so you can watch. Saves nothing. |
| `record_lidar(cfg, name="run")` | Creates `data/runs/<date>_<time>_<name>/`, saves `config.yaml`, spawns vehicles, mounts one LiDAR per CAV, records frames, saves previews. Returns the run folder. |
| `show_frame(run_dir, frame=None, view="both")` | Plots one recorded frame (default: the last). `view`: `sensor`, `world` or `both`. Works without CARLA. |
| `open_editor(cfg, town=None, plan=None)` | Opens the scenario editor window (see [EDITOR.md](EDITOR.md)). |
| `import_openscenario(cfg, xosc, name, ...)` | `.xosc` → spawn plan + experiment YAML; returns the paths and the warnings (see [OPENSCENARIO.md](OPENSCENARIO.md)). |
| `export_openscenario(cfg, plan, xosc)` | Spawn plan → `.xosc` for the QGIS editor / ScenarioRunner. |

`main.py` is a thin command-line wrapper around these functions.

## Front: `cavlidar/editor/`

| Module | Role |
|---|---|
| `model.py` | `ScenarioPlan`: the vehicles being edited, names (`cav_0`...), overlap test, spawn-area check, undo/redo, load/save. No window, no CARLA: unit-tested. |
| `map_view.py` | `MapView`: Tk canvas showing a georeferenced image with zoom around the cursor and pan; redraws the image only when the view changes. |
| `export.py` | Plan + experiment YAML files, command line for recording. |
| `openscenario_bridge.py` | `.xosc` ↔ `ScenarioPlan`: heights from the road, LanePosition / RoadPosition on the map, warnings for off-lane or wrong-way vehicles. |
| `settings.py` | CARLA folder and map cache shared by the window and the API. |
| `app.py` | The window: toolbar, map, Vehicles / Rules / CARLA / Export tabs. CARLA calls run on a background thread so the window never freezes. |

## Core: `cavlidar/core/`

### `config.py`
| Item | Role |
|---|---|
| `ServerConfig`, `SimulationConfig`, `ScenarioConfig`, `LidarConfig`, `RecordingConfig` | One dataclass per section of the YAML. Defaults = paper settings. |
| `Config` | Groups the five sections; `to_dict()` gives plain YAML-ready data. |
| `load_config(*paths, overrides)` | Merge defaults ← YAML files ← overrides, reject unknown keys, convert enums, validate. |
| `validate(cfg)` | Raises `ConfigError` for impossible values; returns warnings for risky ones. |
| `save_config(cfg, path)` | Writes the exact configuration of a run next to its data. |

### `enums.py`
| Enum | Values | Used for |
|---|---|---|
| `CameraMode` | `follow`, `top`, `none` | `recording.camera` |
| `PointFormat` | `npy`, `ply` | `recording.formats` |
| `Role` | `cav`, `traffic`, `other` | role of each vehicle (`Role.CAV.name_of(2)` → `"cav_2"`) |
| `ViewMode` | `sensor`, `world`, `both` | which plots `show_frame` makes |

All enums have `parse(value)`, which accepts text in any case and gives a clear error otherwise.

### `constants.py`
Fixed names that are not parameters: LiDAR blueprint id, run folder layout (`frames/`, `meta.json`, `config.yaml`), file name of CAV clouds (`cav0`), sensor timeout.

### `errors.py`
`CavLidarError` (base) → `ConfigError`, `ServerUnavailable`, `MapMismatch`, `SpawnError`, `SensorTimeout`. `main.py` catches the base class and prints a one-line message instead of a long traceback.

## Backend: `cavlidar/backend/`

### `simulator/connection.py`
| Function | What it does |
|---|---|
| `connect(host, port, timeout)` | Creates a `carla.Client` and makes one request, so a missing server fails right here with `ServerUnavailable`. |
| `versions(client)` | `(client_version, server_version)`. |
| `map_name(world)` | `"Town03"` from `"Carla/Maps/Town03"`. |
| `ensure_town(client, town, allow_map_change)` | Returns the world if the right map is loaded; otherwise loads it only if allowed, else raises `MapMismatch`. |

### `simulator/sync.py`
`SyncMode` — context manager that switches the world to synchronous mode with `fixed_delta_seconds`, puts the Traffic Manager in sync mode too, and restores the original settings on exit. `tick()` advances one step and returns the frame number.

**Why it matters:** with `fixed_delta_seconds = 1 / rotation_frequency`, one tick = one full LiDAR rotation. In asynchronous mode the server runs at its own FPS and each measurement is only a slice (see `e09`).

### `simulator/camera.py`
| Function | What it does |
|---|---|
| `top_view(world, x, y, z, height)` | Puts the CARLA window's camera above a point, looking down. |
| `follow(world, actor, height)` | Same, above an actor; called every tick to follow it. |

### `simulator/session.py`
`SimulationSession(cfg)` — owns everything created in CARLA.

| Method | What it does |
|---|---|
| `__enter__` | connect → check map → enter `SyncMode` → seed the Traffic Manager. |
| `spawn_fleet()` | Spawns the vehicles of `spawn_plan` (if any) at their exact poses, then the rest at spawn points (inside `spawn_bbox`, around `spawn_center` if set), enables autopilot, ticks once. Raises `SpawnError` unless every CAV and traffic vehicle was spawned. |
| `attach_lidars()` | One `Lidar` per CAV, named `cav0`, `cav1`, ... |
| `read_scans(frame)` | The `LidarScan` of every CAV for that frame. |
| `role_names()` | `{actor_id: "cav_0" / "traffic_3"}` for the ground truth. |
| `update_camera()` | Applies `recording.camera`. |
| `__exit__` | Destroys sensors and vehicles, then restores asynchronous mode — even after an error. |

### `actors/vehicles.py`
| Function | What it does |
|---|---|
| `candidate_spawn_points(world, rng, center, radius, bbox)` | Spawn points to try, in order: points outside `bbox` are dropped; then inside the radius first (shuffled), then the nearest outside. Without center: shuffled. |
| `spawn_vehicles(world, count, pattern, candidates, rng, role)` | Spawns up to `count` 4-wheel vehicles matching `pattern`, consuming `candidates` so CAVs and traffic never share a spot. Sets `role_name` and a random colour. |
| `enable_autopilot(vehicles, tm, speed_range, lights_range, rng)` | Autopilot with a random speed offset and red-light behaviour per vehicle. |
| `spawn_at_poses(world, poses, pattern, rng, role)` | Spawns one vehicle per planned pose (used for `spawn_plan`). |
| `destroy(client, actors)` | Removes actors in one batch. |

### `actors/spawn_plan.py`
`load_plan(path, town, num_cavs, num_traffic, bbox)` — reads a spawn plan (JSON) and checks it against the config: map, roles, counts, `spawn_bbox`, speeds, routes. Returns a `PlannedSpawn` (pose, speed, route, route end) per vehicle and role. No CARLA needed. Relative paths are looked up in the current folder, then in the project folder.

### `actors/route_driver.py`
`RouteDriver` drives one vehicle along its traced route, once per tick (called by `SimulationSession.tick()`): pure pursuit steering, speed held with CARLA's constant-velocity mode, lowered by the route's speed profile, the next red light on the route (`route_lights`) and vehicles on the path ahead. Holds the vehicle until `start()`; at the end it stops or hands over to the Traffic Manager.

### `actors/lidar.py`
| Item | What it does |
|---|---|
| `build_blueprint(world, cfg)` | Copies every `LidarConfig` field to the CARLA ray-cast LiDAR attributes. |
| `to_numpy(measurement)` | Raw bytes → `(N, 4)` float32 array `x, y, z, intensity`. |
| `Lidar(world, vehicle, cfg, name)` | Spawns the sensor at `(mount_x, mount_y, mount_z)` on the vehicle and queues its measurements. |
| `Lidar.get(frame)` | Returns the scan of exactly that frame, dropping older ones; raises `SensorTimeout` if nothing comes. |
| `LidarScan` | `name, frame, timestamp, points, sensor_pose`. |

### `actors/ground_truth.py`
`collect_vehicles(world, role_names)` — for every vehicle: id, model, role, pose, box half-sizes, box centre offset, speed. This is what detections will be compared to for AP@0.5.

### `recording/recorder.py`
`record(session, run_dir)` — saves the initial poses (`spawn.json`), runs warm-up ticks (nothing saved), then saves one frame every `save_every` ticks until `num_frames`: point clouds + `meta.json`. Returns a `RecordSummary` (frames, mean points per scan, mean speed per CAV) and warns if every CAV stood still.

### `recording/storage.py`
| Function | What it does |
|---|---|
| `create_run_dir`, `frame_dir` | Create the run / frame folders. |
| `save_points(dir, name, points, formats)` | Writes `.npy` and/or binary `.ply`. |
| `write_ply`, `read_ply` | Binary PLY writer; reader for binary and ASCII (CARLA's own `save_to_disk`). |
| `load_points`, `save_json`, `load_json` | Small helpers. |
| `list_frames(run_dir)`, `load_frame(run_dir, frame)` | Read a run back: `(meta, {"cav0": points, ...})`. |

### `maps/` (no running simulator needed, except `aerial.py`)
| Module | What it does |
|---|---|
| `catalog.py` | Lists maps from the CARLA folder: OpenDRIVE file (editable) and level `.umap` (simulable). |
| `routes.py` | `trace_route(carla_map, start, points)`: A* over the lanes (driving direction, junction branches, allowed lane changes) → `Route` (points every 2 m + their lanes); `speed_profile` (set speed, slower in turns, braking to the end). Same code in the editor and in the simulation. |
| `road_map.py` | `RoadMap`: `carla.Map` built on the client from OpenDRIVE; `snap(x, y)` → legal lane pose (heading, height, slope); `bounds()`. |
| `raster.py` | `Georef` (world ↔ pixels) and the road image (lanes, sidewalks, markings, direction arrows), cached in `data/maps/`. |
| `aerial.py` | Aerial photo of the loaded map: camera 1000 m up, tiles stitched with the same `Georef` as the road image. |

### `interop/openscenario.py` (no CARLA needed)
`read_xosc(path)` → map, vehicles (role, model, CARLA pose or lane position, speed), warnings. `write_xosc(path, town, vehicles)` → OpenSCENARIO 1.0 in the QGIS editor's layout. `to_carla` / `from_carla`: OpenDRIVE ↔ CARLA frame, same rule as ScenarioRunner.

### `simulator/live.py`, `simulator/startup_map.py`
`draw_plan` / `test_spawn` check a plan in the running simulator. `set_startup_map` writes the map CARLA opens at start (`DefaultEngine.ini`, backup kept).

### `actors/spawn_plan.py` (save side)
`read_plan`, `save_plan` — used by the editor.

### `geometry/transforms.py`
| Function | What it does |
|---|---|
| `transform_to_dict(t)` | `carla.Transform` → `{x, y, z, roll, pitch, yaw}`. |
| `pose_to_matrix(pose)` | 4×4 sensor→world matrix, same formula as CARLA. |
| `sensor_to_world(points, pose)` | Moves a cloud into the world frame — the core operation of early (point-level) fusion. |
| `in_bbox(x, y, bbox)` | Is a point inside `[x_min, x_max, y_min, y_max]`. |
| `box_corners_2d(pose, extent, offset)` | 4 corners of a vehicle box seen from above. |

CARLA uses a **left-handed** frame (x forward, y right, z up); all plots invert the y axis so they look like a top view.

### `visualization/bev.py`
| Function | What it does |
|---|---|
| `plot_sensor_frame(meta, clouds, out)` | One panel per CAV in its own frame, coloured by height. |
| `plot_world_frame(meta, clouds, out)` | All CAVs moved to the world frame on one image (point-level fusion) with ground-truth boxes. Walls seen by two CAVs must overlap — a quick check that poses are right. |

## Adding something new

| You want to… | Do this |
|---|---|
| add a parameter | add a field to the right dataclass in `core/config.py`, add it to `config/default.yaml`, document it in `PARAMETERS.md`, use `cfg.<section>.<field>` in the backend |
| add a fixed choice | add an enum in `core/enums.py`, convert it in `config._from_dict` |
| add a sensor (camera, radar) | new module in `backend/actors/`, attach it in `SimulationSession`, read it in `recorder.record` |
| add a processing step (ground removal, BEV…) | new folder `backend/processing/` using `storage.load_frame` — it does not need CARLA |
| expose it to users | one new function in `api.py` and one sub-command in `main.py` |
