# Results of the LiDAR sensitivity benchmark

Outputs behind [docs/SENSITIVITY.md](../../docs/SENSITIVITY.md). The raw recordings are in `data/runs/` (not in git);
the scene definitions, seeds and spawn plans are in `config/experiments/ablation/` and `config/spawn_plans/sensitivity/`.

| Path | Content | Made by |
|---|---|---|
| `benchmark/results.json` | every scored run per scene and factor (incl. false cells per 1000 observed road cells), aggregates across scenes (mean, sd, paired changes, sign consistency, per-scene values), excluded runs | `tools/eval_ablation.py --timing-repeats 3` |
| `benchmark/summary.md` | the same as tables: aggregates, effects, per-scene tables, protocol checks | same |
| `benchmark/runs/<run>.json` | one run: all metrics, plus per-CAV metrics | same (cache) |
| `benchmark/*.png` | sensitivity (parked flat / slope), population (CAVs, traffic), moving | same |
| `benchmark/processing_check.json` | evaluation vs the processing CLI, scan by scan | `--check-processing` |
| `checks/<scene>.json` | geometry check of a recorded scene: vehicles, positions, rotations, motion, repeat identity | `tools/run_ablation.py` |
| `checks/moving*_<factor>.json` | one moving-traffic simulation: identical vehicles across the factor's values, share of moving vehicles, attempts | `tools/record_moving_sensors.py` |
| (moved) `../label_alignment/` | ground-truth boxes vs returns of moving vehicles: controlled diagnostic, corrected check, the first check's output | `tools/diagnose_label_alignment.py`, `--check-labels` |
| `audit/audit.json` | sensor-height inference, false-cell causes per channel count, min-points causal test (10 parked scenes) | `tools/audit_sensitivity.py` |
| `first_study/before`, `after` | the first 2-scene analysis with the ego-body mask off / on | `tools/eval_ablation.py --scenes flat slope [--ego-mask off]` (version of commit 0eaa996) |
