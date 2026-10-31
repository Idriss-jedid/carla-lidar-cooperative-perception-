# Fidelity to the paper

How close this repository is to the setup of the paper (Section III, Table 1). This repository covers **data collection only**; it is not yet a reproduction of the paper's results.

## Matches Table 1

| Parameter | Paper | Here |
|---|---|---|
| Map | CARLA Town 3 | `server.town: Town03` |
| Intersection | four legs, one sloped (Figure 2) | Junction 1221, `spawn_center: [2.9, -136.5]`: same buildings as Figure 2; its +x leg climbs from z = 0 to 8 m within 60 m |
| Figure 2 scene | 4 CAVs + 11 vehicles | `config/spawn_plans/figure2_town03.json` (`e11`, `e12`), positions within 1.2 m of the figure |
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

LiDAR vertical field of view (+2° / −25°), mount height (2.4 m), drop-off rates (CARLA defaults), noise (0), warm-up length, which timestamp is saved per scenario, and the random seeds. The paper does not say whether the spawn box is in world coordinates or relative to the intersection. World coordinates are used here: the Figure 2 junction (2.9, −136.5) sits in the middle of x ∈ [−100, 100], y ∈ [−200, 200], which supports this reading.

## Fixed after checking the recorded data (2026-10-30)

| Problem | Cause | Fix |
|---|---|---|
| All CAVs at ~0 km/h in the first `e01` run | Traffic Manager leaves every car still for ~2 s after spawning; the run saved frames at 1.05–1.55 s | `warmup_ticks` 20 → 100 (5 s); warning if the warm-up is under 3 s; `Mean CAV speed` logged and a warning if every CAV stood still |
| 2 of 4 CAVs and 11 of 20 vehicles spawned outside the paper's spawn area | radius around the junction, with fallback to farther points | `scenario.spawn_bbox` (hard limit) |
| Fewer traffic vehicles than requested only gave a warning | — | `SpawnError` unless every vehicle is spawned |
| Initial poses not saved | — | `spawn.json` in every run |

## Finding the Figure 2 junction (2026-10-31)

Earlier runs used junction 356 (83.5, −134.7), which is the *top* of the sloped leg, not the junction of Figure 2. A top-down photo of each of the 25 Town03 junctions was compared with Figure 2: junction 1221 matches (brick houses top-left, beige apartment block top-right, pink plaza bottom-right, curved façade bottom-left, same orientation: figure up = +x). The vehicle boxes drawn on the figure were converted to world coordinates with the junction as scale reference and snapped to lanes with the matching direction (`tools/make_figure2_plan.py`). All experiments now use this junction.
