# Parameters

All parameters live in `config/default.yaml`. Change them in three ways:

```bash
# 1. edit config/default.yaml (changes every run)
# 2. an experiment file that lists only what changes
python main.py record --config config/experiments/lidar/e03_lidar_64ch.yaml
# 3. one-off override from the command line (repeatable)
python main.py record --set lidar.range=80 --set scenario.num_cavs=6
```

Priority: defaults < `default.yaml` < experiment files (in order) < `--set`. The exact values of every run are saved in `data/runs/<run>/config.yaml`.

Useful numbers for the LiDAR (paper settings):

- rays per sweep = `points_per_second / rotation_frequency` = 1,000,000 / 20 = **50,000**
- vertical resolution = `(upper_fov − lower_fov) / (channels − 1)` = 27 / 31 = **0.87°**
- horizontal resolution = `360 × rotation_frequency × channels / points_per_second` = **0.23°**
- points actually returned ≈ 50,000 × (rays that hit something) × (1 − dropoff) ≈ **26,500** measured

---

## `server`

| Key | Default | Meaning | If you change it |
|---|---|---|---|
| `host` | `127.0.0.1` | Machine running `CarlaUE4.exe`. | Another PC's IP to run the simulator remotely. |
| `port` | `2000` | CARLA RPC port. | Must match `-carla-rpc-port=` of the server. Use another port to run two servers. |
| `tm_port` | `8000` | Traffic Manager port. | Change if two scripts run at once. |
| `timeout` | `60.0` | Seconds before a request fails. | Increase if the map is slow to load; decrease to fail faster. |
| `town` | `Town03` | Map the scripts expect (the paper uses Town03). | `Town01`…`Town10HD`. The server must have that map loaded. |
| `allow_map_change` | `false` | Call `load_world()` if another map is loaded. | `true` lets scripts change the map, but this crashed CARLA on the 4 GB GPU. Safer: edit `GameDefaultMap`/`ServerDefaultMap` in `CarlaUE4/Config/DefaultEngine.ini` and restart CARLA. |

## `simulation`

| Key | Default | Meaning | If you change it |
|---|---|---|---|
| `fixed_delta_seconds` | `0.05` | Simulated seconds per tick. | **Keep it = 1 / `lidar.rotation_frequency`.** Smaller (e.g. 0.01) → each scan is only a slice of the circle (72° at 0.01 s, see `e09`). Larger than 1/f → more than one turn is merged. Must be ≤ 0.1 (CARLA physics). |
| `no_rendering` | `false` | Turn off graphics. | `true`: faster and frees GPU memory; the CARLA window freezes but LiDAR still works. Good for long recordings. |
| `seed` | `7` | Random seed for vehicle choice, spawn points, colours, traffic behaviour. | Same seed + same config = same scene (reproducible). Change it to get a new scene. |

## `scenario`

| Key | Default | Meaning | If you change it |
|---|---|---|---|
| `num_cavs` | `4` | Vehicles with a LiDAR. | Paper: 4, 6, 8, 10. More CAVs = more coverage and more data to share, slower ticks. |
| `num_traffic` | `20` | Vehicles without sensors. | Paper: 20–40. More = more occlusions and more objects to detect. |
| `cav_blueprint` | `vehicle.tesla.model3` | CARLA model of the CAVs. | Any `vehicle.*` id, e.g. `vehicle.audi.tt`. A taller vehicle changes the LiDAR height above ground. |
| `traffic_blueprint` | `vehicle.*` | Pattern for traffic models (only 4-wheel ones are used). | `vehicle.tesla.*` for one brand, `vehicle.carlamotors.carlacola` for trucks (big occluders). |
| `spawn_center` | `null` | `[x, y]` to concentrate vehicles around a place. | `[83.5, -134.7]` = the sloped four-leg junction (paper-like). `null` = anywhere in the map. |
| `spawn_radius` | `80.0` | Metres around `spawn_center` (a preference, not a limit). | Smaller = denser scene, more overlap between CAVs (more redundancy). If not enough points fit, the nearest outside points are used. |
| `spawn_bbox` | `null` | `[x_min, x_max, y_min, y_max]` in world coordinates: spawn points outside are never used. | Paper Table 1: `[-100, 100, -200, 200]` (used by `e01`, `e02`). Too small a box → `SpawnError` (the run stops if it cannot spawn **every** CAV and traffic vehicle). |
| `autopilot` | `true` | Vehicles drive themselves. | `false` = parked vehicles (static scene, every frame almost identical). |
| `speed_difference_range` | `[-20, 20]` | Per-vehicle % **below** the speed limit (CARLA convention: +20 = 20 % slower, −20 = 20 % faster). | Paper: −20 %…+20 %. `[-50, -50]` = everyone 50 % over the limit. |
| `ignore_lights_range` | `[0, 0]` | Per-vehicle % chance to run red lights. | Paper: 0–100 %. `[0, 100]` = mixed behaviour, more crossings in the junction. |

## `lidar`

| Key | Default | Meaning | If you change it |
|---|---|---|---|
| `channels` | `32` | Number of laser rings. | 64 → twice the vertical resolution (raise `points_per_second` too, otherwise each ring gets fewer points). 16 → sparse, cheap sensor. |
| `range` | `120.0` | Maximum distance (m). | 50 → far vehicles vanish, cooperation matters more. 200 → more points far away, more data. |
| `points_per_second` | `1000000` | Rays per second over all channels. | Higher = denser rings (more points per object, bigger files). |
| `rotation_frequency` | `20.0` | Rotations per second (Hz). | If you change it, set `simulation.fixed_delta_seconds = 1 / rotation_frequency`. 10 Hz → twice the points per sweep. |
| `upper_fov` | `2.0` | Angle of the highest laser (degrees). | Higher (e.g. 15) → more points on buildings/trees, fewer on the road. |
| `lower_fov` | `-25.0` | Angle of the lowest laser. | Closer to 0 → the blind circle around the car grows. |
| `mount_x`, `mount_y`, `mount_z` | `0, 0, 2.4` | Sensor position relative to the vehicle centre (m). | Lower `mount_z` (1.6) → nearby cars hide more of the scene. `mount_x=2` → sensor on the front bumper. |
| `dropoff_general_rate` | `0.45` | Share of points CARLA drops at random (CARLA default). | `0` → every hit is returned (≈ 2× points, see `e05`). Higher → sparser, noisier sensor. |
| `dropoff_intensity_limit` | `0.8` | Points with intensity above this are never dropped. | `1.0` with the others at `0` = no drop-off at all. |
| `dropoff_zero_intensity` | `0.4` | Drop probability for zero-intensity points. | `0` keeps every far/weak return. |
| `noise_stddev` | `0.0` | Gaussian noise along each ray (m). | `0.02`–`0.05` = realistic. Makes ground removal and clustering harder (see `e06`). |

## `recording`

| Key | Default | Meaning | If you change it |
|---|---|---|---|
| `output_dir` | `data/runs` | Where run folders are created. | Any path, e.g. `D:/datasets/cav`. |
| `warmup_ticks` | `100` | Ticks before saving (5 s at 0.05 s/tick). | The Traffic Manager needs ~3 s to start the cars: below 60 ticks the first frames show parked cars (a warning is printed). 200 → vehicles spread out more. |
| `num_frames` | `10` | Frames saved per run. | The paper uses one random timestamp per scenario; use more frames for a dataset. |
| `save_every` | `1` | Save one tick out of N. | `20` → one frame per simulated second (less redundant frames). |
| `formats` | `[npy]` | `npy` and/or `ply`. | Add `ply` to open clouds in MeshLab / CloudCompare (bigger, slower). |
| `save_ground_truth` | `true` | Store all vehicle poses and boxes in `meta.json`. | Needed for AP@0.5; `false` only for quick tests. |
| `camera` | `follow` | `follow` (above CAV 0), `top` (above `spawn_center`), `none`. | `top` needs `scenario.spawn_center`. |
| `camera_height` | `40.0` | Height of the camera (m). | Higher to see the whole junction. |
| `preview` | `true` | Save `preview_sensor.png` and `preview_world.png` of the last frame. | `false` to save time in long runs. |
