# Experiment log

Every experiment of the CARLA → BEV part of the project, in the order it was done: what was tested, on which
recordings, what it showed, and where the details are. The ready-made configurations themselves are described in
[EXPERIMENTS.md](EXPERIMENTS.md).

All runs use CARLA 0.9.12 in synchronous mode (0.05 s step = one full LiDAR sweep per tick, except where stated) on
Town03. Ground truth is used only to score results, never by the processing. Raw recordings are in `data/runs/` (264
folders) and `data/diagnostics/`, not in git; metrics and plots are in `results/` and `docs/images/`.

## Index

| # | Date | Experiment | Recordings | Main outcome | Details |
|---|---|---|---|---|---|
| 1 | Oct 22 | First captures: asynchronous vs synchronous LiDAR | Phase 1 scripts, e09 | an asynchronous capture gives ~50–72° slices, not full sweeps → always synchronous mode | §1 |
| 2 | Oct 22–30 | Baseline recordings e01 / e02 | e01, e01_fixed, e02_fixed | CAVs standing still and vehicles outside the paper's area → 5 s warm-up, strict spawn box | §1 |
| 3 | Oct 22 | Recording options e09 / e10 | e09_partial_sweep_demo, e10_export_ply | partial sweep reproduced (72°, 5 327 points); PLY export identical | §1 |
| 4 | Oct 31 | Finding the paper's Figure 2 junction | 25 Town03 junctions | junction 1221 matches; Figure 2 re-created within 1.2 m | §1 |
| 5 | Oct 31 | Figure 2 scenes | e11_figure2, e12_figure2_driving | e12 became the main 4-CAV dataset | §1 |
| 6 | Oct 31 | Scenario editor, OpenSCENARIO, routes, slope | qgis_e2e, routes_demo, slope_check, slope_check_parked | parked cars rolled down the 12° slope → brakes at spawn | §1 |
| 7 | Oct 31 | Ground removal: first prototype vs height prior | 200 scans, 5 runs | vehicle points removed 11.0 % → 0.26 %; close pairs 22–37 % → 0 % | §2 |
| 8 | Oct 31 | Whole cleaning step (road filter + ground removal) | 260 scans, 6 runs | 0.20 % vehicle points removed, 0.77 % ground left | §2 |
| 9 | Oct 31 | Sector metadata on e12 | e12 | ~6.5 k points per sector, S̃ 0–0.04 | §2 |
| 10 | Nov 6 | Sensitivity, first study | 2 scenes, 22 runs | LiDAR hits its own car below ~2 m → ego-body mask | §3.1 |
| 11 | Nov 6 | Sensitivity pilot (protocol check) | flat2, slope2, 12 runs | positions 0.0000 m, repeats bit-identical, invalid mount excluded | §3.2 |
| 12 | Nov 6 | Reproducibility of CARLA | repeats of parked and moving scenes | parked: bit-identical across sessions; moving: diverges up to 3.56 m | §3.3 |
| 13 | Nov 6 | Sensitivity campaign | 15 scenes, 240 scored runs | point density dominates; noise → false cells; no channel optimum; range = truncation | §3.4 |
| 14 | Nov 6 | Audits: sensor height, false cells, min points | 10 parked scenes | 70–100 % of false cells from the sparse-patch rule; min points 8 not adopted (would be tuning) | §3.5 |
| 15 | Nov 6 | First label-timing diagnostics | diag_targets_moving, diag_cavs_moving, …_camera | apparent box offset of moving vehicles (later shown to be the estimator) | §4.1 |
| 16 | Nov 7 | Controlled label-alignment diagnostic | 7 cases (+1 rejected), 8 020 views | boxes frame-matched: τ = +0.0005 s vs −0.05 s for one tick | §4.2 |
| 17 | Nov 7 | Corrected label check on the benchmark | 17 recordings | rear +0.02…+0.07 m, front −0.03…−0.06 m (sampling gap) | §4.3 |
| 18 | Nov 7 | False cells per observed road (CAV count) | 3 population scenes | union rate 25.7 → 33.7 per 1 000 cells (4 → 10 CAVs), raw count ×2.1 | §3.6 |
| 19 | Nov 7 | Processing time re-measured | e12, 40 scans | ~0.1 s per CAV (99 ms), not 472 ms; not real-time | §5 |

## 1. Data collection

**Asynchronous vs synchronous capture (Oct 22).** In asynchronous mode the server ran at ~140 FPS and each LiDAR
measurement was a ~50° slice of a sweep. Every capture since uses synchronous mode with `fixed_delta_seconds` equal to
1 / rotation frequency. `e09_partial_sweep_demo` reproduces the effect on purpose: a 0.01 s tick with a 20 Hz LiDAR
gives 72° per scan (5 327 points, 1/5 of a sweep), with a warning.

**e01, first baseline (Oct 22)**, 4 CAVs + 20 vehicles: every CAV at 0 km/h (the Traffic Manager holds cars ~2 s after
spawning, frames were saved at 1.05–1.55 s) and 11 of 24 vehicles outside the paper's spawn area. Fixes: 100 warm-up
ticks (5 s), a speed check with a warning, a hard `spawn_bbox`, an error if fewer vehicles spawn than requested,
`spawn.json` saved. **e01_fixed (Oct 30)**: 3/4 CAVs moving, 4/4 inside the area.

**e02_fixed (Oct 30)**, 10 CAVs + 40 vehicles, the paper's densest setting: 10/10 CAVs inside the area; 0.146 s per
tick (0.059 s with 4 CAVs + 20). e01 and e02 were recorded at junction
356, before the Figure 2 junction was found.

**e10_export_ply**: `.ply` written next to `.npy`, same point counts.

**Figure 2 junction (Oct 31).** A top-down photo of each of Town03's 25 junctions was compared with the paper's Figure 2:
junction 1221 (2.9, −136.5) matches (buildings, orientation, the 12° sloped leg). The vehicles drawn in the figure were
converted to world positions and snapped to lanes (`tools/make_figure2_plan.py`): within 1.2 m of the figure.
**e11_figure2** = that scene parked; **e12_figure2_driving** = the same scene plus 9 vehicles, all driving, one frame per
second. e12 is the main 4-CAV dataset of the processing work.

**Editor and custom scenes (Oct 31).** `qgis_e2e` (a scene made in the scenario editor, exchanged as OpenSCENARIO),
`routes_demo` (vehicles on planned routes at planned speeds, our route driver), `slope_check` (CAVs on and around the
sloped leg, for the ground removal). In `slope_check` the parked vehicles rolled down the slope; parked vehicles now
get the brake and hand brake at spawn, and `slope_check_parked` (same scene) shows them moving ≤ 0.03 m in 15 frames.

**Not recorded as single runs:** e03, e04, e06, e07 (64 channels, 50 m range, noise, low mount) are covered in a
controlled way by the sensitivity benchmark (§3); e05 (no drop-off) is covered by its drop-off factor; e08 (chaotic
traffic) was not recorded.

## 2. Processing (per CAV, up to the BEV)

Details: [PROCESSING.md](PROCESSING.md).

| Test | Data | Result |
|---|---|---|
| Ground removal, first prototype → slope-aware height prior | every frame of e11, e12, qgis_e2e, routes_demo, slope_check (200 scans) | vehicle points removed 11.0 % → **0.26 %**; ground left 2.6 % → 1.5 %; vehicles on the slope 13.2 % → 0.05 %; slope ground left 1.8 % → 0.5 % |
| Close pairs (vehicles a few metres apart) | 5 pairs in routes_demo, e12, qgis_e2e, slope_check | 22–37 % of the neighbour's body removed → **0 %** in all 5 |
| Whole cleaning step (road filter + ground removal) | 6 runs incl. slope_check_parked, 260 scans | vehicle points removed **0.20 %** (none by the road filter); ground left 0.77 %; the road filter removes 42 % of the returns |
| Known limit | qgis_e2e, routes_demo | 15 % of the far slope ground (93–119 m away) is kept on purpose: unknown ground is kept, not guessed |
| Sector metadata | e12, 4 static sectors | ~6.5 k points (0.10–0.11 MB) per sector, S̃ from 0 to 0.04; a car right behind cav2: 2 750 non-ground points, 217 after 0.2 m averaging |

## 3. LiDAR sensitivity benchmark

Details: [SENSITIVITY.md](SENSITIVITY.md). Results: `results/sensitivity/`.

### 3.1 First study (2 scenes, 22 runs)

Scenes flat (Figure 2) and slope; one factor per run: noise, mount height, channels, range. Found that below ~2 m the
LiDAR hits its own car (9 244 returns per scan at 1.6 m, 35 % of the scan; 917 at 1.9 m) and nothing removed them →
the **ego-body mask** was added. Re-scored before / after: own points in the cloud 9 244 → 0 and 917 → 0; the roof had
inflated the sector quality S̃ by 12 % (flat) and 26 % (slope); every other run identical.

### 3.2 Pilot (Stage 1: flat2, slope2, 12 runs)

Base, mount 1.6 m, drop-off 0, 2M pts/s, a repeat and an invalid mount per scene, to check the protocol before the
campaign:

- every planned vehicle spawned, positions identical to 0.0000 m / 0.0000°;
- the repeat is bit-identical in 20/20 scans;
- the evaluation equals the processing CLI scan by scan;
- the ego mask removes 0 points of other vehicles;
- the 1.2 m mount (inside the body, ~2 800 returns from inside) is flagged and excluded;
- drop-off 0 gives 1.82× the points, 2M pts/s 2.00×.

### 3.3 Reproducibility of CARLA

- Parked scenes: recordings are bit-identical between runs and across CARLA sessions → one recording per setting.
- Moving (Traffic Manager) traffic: two recordings of the same configuration diverge by up to 3.56 m (moving1) and
  0.63 m (moving2). Hence the moving design: every value of a factor is mounted on the same CAVs in one simulation.
- Rule fixed before any metric: a moving simulation counts only if ≥ 50 % of the vehicle states move. 2 of 12
  simulations (traffic at red lights) were recorded again; the rejected ones are kept.

### 3.4 Campaign (15 scenes, 240 scored runs)

| Scenes | Kind | Runs |
|---|---|---|
| flat, flat2–flat5 | parked, flat lanes | 17 each |
| slope, slope2–slope5 | parked, 3–4 CAVs on the 12° leg | 17 each |
| pop1–pop3 | 44 parked Tesla positions: CAV count 4 / 6 / 8 / 10, traffic 20 / 30 / 40 | 6 each |
| moving1, moving2 | Traffic Manager, 4 CAVs + 20 vehicles | 22 LiDAR settings each |

Factors: noise 0 / 0.02 / 0.05 / 0.10 m, mount 1.6 / 1.9 / 2.4 / 2.8 m, channels 16 / 32 / 64, range 50 / 80 / 120 m,
drop-off 0 / 0.2 / 0.3 / 0.45 / 0.6, points/s 0.5M / 1M / 2M. `data/runs/` holds 250 `abl_*` folders: the 240 scored
runs (incl. 6 repeats and 2 invalid mounts), 2 moving baselines of the reproducibility test and 8 folders of the 2
rejected moving simulations.

Main effects (paired change to each scene's baseline, mean ± sd, [scenes agreeing]):

- 0.5M pts/s: −21.4 ± 4.9 points of vehicle visibility on flat scenes [5/5]; 2M pts/s or drop-off 0: +12 to +14 [5/5],
  ~28 fewer false cells, +33 to +42 ms.
- Noise 2 cm: +16 false cells per scan [5/5]; visibility does not move consistently.
- Mount 1.6 m: −35 % observed road [5/5]; 2.8 m: fewer false cells [5/5].
- Channels: observed road −40 % (16 ch) / +37 % (64 ch) [5/5]; no consistent best for false cells.
- Range: no difference within the common 0–50 m region (truncation only).
- CAVs 4 → 10: road observed together +58 %, union false cells ×2 [3/3]; vehicle visibility already 97.6 % with 4.
- Traffic 20 → 30: per-CAV visibility −5.3 points [3/3] (a scene change, not a sensor effect).
- Moving traffic: same directions as parked [2/2].

### 3.5 Audits (10 parked scenes)

- **Sensor height** is inferred from the points: error ≤ 5.4 mm on flat ground, up to 0.29 m at slope transitions.
- **False cells** are real road surface left by the sparse-patch rule (70–100 % of them for 32 and 64 channels), not a
  metric artefact. The 32-channel "peak" of the first study was scene-specific.
- **Min points 18 vs 8:** 8 cuts false cells in every scene but raises vehicle loss in some; choosing it on these
  scenes would be tuning, so 18 is kept.

### 3.6 False cells per observed road (Nov 7)

The union false cells normalised by the road observed together: 25.7 ± 4.0 (4 CAVs), 25.2 ± 1.6 (6), 29.0 ± 5.0 (8),
33.7 ± 7.2 (10) per 1 000 cells: +31 % at 10 CAVs [3/3] against ×2.1 for the raw count; each CAV's own rate does not
rise.

### Processing time

Median per scan, CARLA closed, fastest of 3: 77 ± 5 ms (flat baseline); 51 / 77 / 119 ms at 0.5M / 1M / 2M pts/s.

## 4. Ground-truth boxes of moving vehicles

Details: [LABEL_ALIGNMENT.md](LABEL_ALIGNMENT.md). Results: `results/label_alignment/`.

### 4.1 First label-timing diagnostics (Nov 6)

`diag_targets_moving` (targets driving at 30 km/h, CAVs parked), `diag_cavs_moving` (CAVs driving, targets parked),
`diag_targets_moving_camera` (the first with a camera attached: identical result, so the camera has no effect). With
the first check (shift of the box that holds the most returns), moving2 and e12 showed boxes up to ~0.9 m ahead of
their returns, cause unknown.

### 4.2 Controlled diagnostic (Nov 7)

Semantic LiDAR (labels every return with the vehicle it hit) next to the recorder's LiDAR; every tick saved with
CARLA's snapshots.

| Case | Content | Time offset τ (s), 95 % interval |
|---|---|---|
| straight | road 28, targets 0–18 m/s, 4 models | +0.0042 [−0.0019, +0.0158] |
| straight_dt10 | the same with a 0.1 s step | +0.0029 [−0.0035, +0.0134] |
| observer | LiDAR driving at 15 m/s past parked cars | −0.0083 [−0.0273, +0.0012] |
| slope | road 18, targets up and down at 3.5 / 7 m/s | −0.0011 [−0.0088, +0.0026] |
| traffic1 | moving1 configuration | +0.0070 [+0.0006, +0.0271] |
| traffic2 | moving2 configuration (1st attempt rejected: 4 % moving) | +0.0008 [−0.0020, +0.0024] |
| e12 | e12 configuration | +0.0013 [−0.0015, +0.0038] |

A box one tick late would give −0.05 s (−0.10 s at the 0.1 s step).

- **Frames and timestamps:** 0 violations; the recorder's box equals CARLA's snapshot of the LiDAR's frame.
- **Rays:** the ray-cast and semantic LiDARs cast identical rays.
- **Best box:** the box of tick t fits best in 96–100 % of moving views (slow, fast, turning, braking, slope).
- **Cause of the old offset:** the first check's estimator. Seen from behind, every backward shift keeps the rear-face
  returns inside, and the median of the tie is about −1 m whatever the alignment.

### 4.3 Corrected check on the benchmark recordings (Nov 7)

Offset at the end of the box facing the sensor, on moving1 and moving2 (6 simulations each), e12 and the §4.1
diagnostics: rear +0.02 … +0.07 m, front −0.03 … −0.06 m, the same sampling gap as parked cars (one tick late would
give −0.14 … −0.41 m). Remaining: CARLA's boxes are larger than the body for a few models (Jeep Wrangler +0.33 m,
Carlacola +0.12 m). Recorder, metadata and raw data unchanged.

## 5. Processing time re-measured (Nov 7)

e12, all 40 scans, CARLA and other jobs closed: ego mask + road filter + ground removal + BEV take a median of 99 ms per
CAV (p95 153 ms; 89 ms fastest of 3). The 472 ms reported earlier was measured on a busy machine. With one sweep every
50 ms, this is still not real-time.

## Not done yet

- e08 (chaotic traffic) as a run; dynamic sectorization.
- Outside this repository: the sharing optimization (sector IoU, ILP, Greedy–Swap), the simulated fusion and the
  detection (PIXOR, AP@0.5) build on the BEV and the sector metadata produced here.
- The paper's 57 scenarios per CAV count; other junctions or towns.
