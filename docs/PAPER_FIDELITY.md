# Fidelity to the paper

How close this repository is to the setup of the paper (Section III, Table 1). This repository covers **data collection only**; it is not yet a reproduction of the paper's results.

## Matches Table 1

| Parameter | Paper | Here |
|---|---|---|
| Map | CARLA Town 3 | `server.town: Town03` |
| Intersection | four legs, one sloped | `spawn_center: [83.5, -134.7]` (junction at z = 8 m; the leg towards y = −200 slopes down to z = 0) |
| CAVs per scenario | 4, 6, 8, 10 | `scenario.num_cavs` |
| Non-CAVs per scenario | 20–40 | `scenario.num_traffic` |
| LiDAR | 32 channels, 120 m, 1,000,000 pts/s, 20 Hz | `lidar.*` defaults |
| Speed adjustment | −20 % to +20 % | `speed_difference_range: [-20, 20]` |
| Spawn area | x ∈ [−100, 100], y ∈ [−200, 200] | `spawn_bbox: [-100, 100, -200, 200]` in `e01`, `e02` |

## Differs from the paper

| Item | Paper | Here | Status |
|---|---|---|---|
| Scenario framework | CARLA + OpenCDA (scenario management, annotations) | CARLA Python API + Traffic Manager; ground truth read from CARLA actors | Same information, different tool. OpenCDA not integrated. |
| Traffic-light compliance | 0–100 % ignored | `[0, 0]` by default, `[0, 100]` in `e08` | Must vary across the scenario set. |
| Max speed / tailgate speed | 20–50 km/h / 30–60 km/h | not controlled; only the % offset to the speed limit | Not implemented. |
| Random yaw | −180° to 180° | heading of CARLA's road spawn point | Not implemented: a random heading under autopilot spawns cars against the lane. Paper's exact policy unknown. |
| Number of scenarios | 57 per CAV count, 228 in total | one scenario per run | Scenario generator still to write. |
| Processing | ground removal, BEV grids, sectorization, ILP / greedy-swap, PIXOR, AP@0.5 | `bev.py` only draws point clouds | Next phases (see README roadmap). |

## Not specified in the paper (our choices)

LiDAR vertical field of view (+2° / −25°), mount height (2.4 m), drop-off rates (CARLA defaults), noise (0), warm-up length, which timestamp is saved per scenario, and the random seeds. Whether the spawn box is in world coordinates (used here) or relative to the intersection is also not stated: in world coordinates the east leg of the junction (x > 100) gets no spawned vehicles.

## Fixed after checking the recorded data (2026-10-30)

| Problem | Cause | Fix |
|---|---|---|
| All CAVs at ~0 km/h in the first `e01` run | Traffic Manager leaves every car still for ~2 s after spawning; the run saved frames at 1.05–1.55 s | `warmup_ticks` 20 → 100 (5 s); warning if the warm-up is under 3 s; `Mean CAV speed` logged and a warning if every CAV stood still |
| 2 of 4 CAVs and 11 of 20 vehicles spawned outside the paper's spawn area | radius around the junction, with fallback to farther points | `scenario.spawn_bbox` (hard limit) |
| Fewer traffic vehicles than requested only gave a warning | — | `SpawnError` unless every vehicle is spawned |
| Initial poses not saved | — | `spawn.json` in every run |
