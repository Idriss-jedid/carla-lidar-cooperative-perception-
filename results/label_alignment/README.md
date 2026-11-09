# Ground-truth box alignment of moving vehicles

Outputs behind [docs/LABEL_ALIGNMENT.md](../../docs/LABEL_ALIGNMENT.md). The raw diagnostic data (every tick: snapshots,
semantic and ray-cast returns, ~420 MB) is in `data/diagnostics/label_alignment/<case>/` (not in git).

| Path | Content | Made by |
|---|---|---|
| `cases/<case>.json` | settings of each diagnostic simulation: step, physics substepping, seed, LiDAR, vehicles (model, box extent, centre offset, box rotation), target speeds, observers, CARLA version; `traffic2_rejected_1` is the attempt rejected for too little motion | `tools/diagnose_label_alignment.py record` |
| `diagnostic.json` | per case: frame / time / pose consistency checks, motion classes with offsets to the boxes of t−2 … t+2 and the best box tick, lag fits (target speed, observer speed) with 95 % intervals; pooled over the 0.05 s cases; fit per step; per-model offsets; ray-cast vs semantic measurement | `... analyse` |
| `views_<case>.csv` | one row per (tick, observer, vehicle with ≥ 30 semantic returns): view, speed, heading speed, yaw rate, pitch, acceleration, observer speed, offsets and inside shares for the boxes of t−2 … t+2, the same measured on the ray-cast LiDAR, the first estimator | `... analyse` |
| `alignment_ticks.png`, `alignment_overlays.png`, `alignment_before_after.png` | the figures of the doc | `... figures` |
| `benchmark_check/labels_check.json`, `labels_views_<run>.json` | corrected check on the benchmark recordings (moving1/2 × 6, e12, label diagnostics, flat): per viewing side, the offset at the visible end, the first estimator, what one tick late would give | `tools/eval_ablation.py --out data/ablation/labels --check-labels ...` |
| `first_check/labels_check.json` | the first check's output (shift holding the most returns), as committed before the correction | `tools/eval_ablation.py` (commit cd1eb5f) |
