# LiDAR sensitivity analysis (ego mask off)

One parameter changed per run, fixed parked scenes, baseline = 32 ch, 120 m, noise 0, mount 2.4 m,
1 000 000 pts/s, 20 Hz. 4 CAVs x 5 sweeps per run. Ground truth used for scoring only.

## flat — noise (lidar.noise_stddev)

| noise_stddev | points/scan | ground left % | vehicle loss % | BEV cells | false cells | coverage % | far cov. % | est. height m | own pts left | ego cells | mean S~ | ms/scan |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 (base) | 26581 | 0.98 | 0.18 | 236 | 47.2 | 84.3 | 61.9 | 2.40 | 0 | 0.0 | 0.0373 | 90 |
| 0.02 | 26538 | 1.14 | 0.39 | 255 | 64.2 | 84.7 | 62.9 | 2.40 | 0 | 0.0 | 0.0390 | 81 |
| 0.05 | 26538 | 1.27 | 0.44 | 278 | 79.3 | 83.5 | 60.0 | 2.40 | 0 | 0.0 | 0.0421 | 80 |
| 0.1 | 26538 | 1.34 | 0.47 | 308 | 92.5 | 84.3 | 61.9 | 2.40 | 0 | 0.0 | 0.0464 | 81 |

## flat — height (lidar.mount_z)

| mount_z | points/scan | ground left % | vehicle loss % | BEV cells | false cells | coverage % | far cov. % | est. height m | own pts left | ego cells | mean S~ | ms/scan |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1.6 | 26698 | 1.43 | 0.93 | 228 | 29.9 | 78.4 | 61.0 | 1.60 | 9244 | 26.0 | 0.0366 | 82 |
| 1.9 | 26636 | 0.92 | 1.11 | 213 | 22.4 | 81.6 | 64.8 | 1.90 | 917 | 12.0 | 0.0364 | 83 |
| 2.4 (base) | 26581 | 0.98 | 0.18 | 236 | 47.2 | 84.3 | 61.9 | 2.40 | 0 | 0.0 | 0.0373 | 90 |
| 2.8 | 26513 | 1.14 | 0.34 | 219 | 16.4 | 86.3 | 66.7 | 2.80 | 0 | 0.0 | 0.0391 | 80 |

## flat — channels (lidar.channels)

| channels | points/scan | ground left % | vehicle loss % | BEV cells | false cells | coverage % | far cov. % | est. height m | own pts left | ego cells | mean S~ | ms/scan |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 16 | 26488 | 1.58 | 1.46 | 166 | 8.8 | 84.7 | 66.7 | 2.40 | 0 | 0.0 | 0.0308 | 79 |
| 32 (base) | 26581 | 0.98 | 0.18 | 236 | 47.2 | 84.3 | 61.9 | 2.40 | 0 | 0.0 | 0.0373 | 90 |
| 64 | 26628 | 0.78 | 0.01 | 222 | 14.7 | 85.9 | 65.7 | 2.40 | 0 | 0.0 | 0.0393 | 85 |

## flat — range (lidar.range)

| range | points/scan | ground left % | vehicle loss % | BEV cells | false cells | coverage % | far cov. % | est. height m | own pts left | ego cells | mean S~ | ms/scan | ground left % <50 m | vehicle loss % <50 m | false cells <50 m | coverage % <50 m |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 50 | 24749 | 0.11 | 0.19 | 195 | 12.2 | 84.3 | 61.9 | 2.40 | 0 | 0.0 | 0.0334 | 88 | 0.11 | 0.19 | 12.2 | 90.6 |
| 80 | 25533 | 0.35 | 0.17 | 230 | 41.3 | 84.7 | 62.9 | 2.40 | 0 | 0.0 | 0.0344 | 83 | 0.11 | 0.17 | 11.7 | 90.2 |
| 120 (base) | 26581 | 0.98 | 0.18 | 236 | 47.2 | 84.3 | 61.9 | 2.40 | 0 | 0.0 | 0.0373 | 90 | 0.13 | 0.18 | 14.5 | 89.4 |

## slope — noise (lidar.noise_stddev)

| noise_stddev | points/scan | ground left % | vehicle loss % | BEV cells | false cells | coverage % | far cov. % | est. height m | own pts left | ego cells | mean S~ | ms/scan | slope ground left % | slope vehicle loss % |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 (base) | 26682 | 0.89 | 0.50 | 102 | 28.9 | 69.1 | 24.4 | 2.41 | 0 | 0.0 | 0.0170 | 75 | 0.13 | 0.55 |
| 0.02 | 26699 | 1.04 | 0.52 | 119 | 43.8 | 71.8 | 31.1 | 2.40 | 0 | 0.0 | 0.0180 | 84 | 0.34 | 0.56 |
| 0.05 | 26699 | 1.08 | 0.48 | 130 | 52.2 | 71.8 | 31.1 | 2.40 | 0 | 0.0 | 0.0192 | 78 | 0.28 | 0.52 |
| 0.1 | 26699 | 1.23 | 0.47 | 158 | 73.0 | 71.8 | 31.1 | 2.39 | 0 | 0.0 | 0.0216 | 88 | 0.41 | 0.51 |

## slope — height (lidar.mount_z)

| mount_z | points/scan | ground left % | vehicle loss % | BEV cells | false cells | coverage % | far cov. % | est. height m | own pts left | ego cells | mean S~ | ms/scan | slope ground left % | slope vehicle loss % |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1.6 | 26842 | 1.63 | 1.14 | 113 | 26.6 | 66.4 | 17.8 | 1.61 | 9235 | 26.0 | 0.0191 | 77 | 0.37 | 1.23 |
| 1.9 | 26773 | 0.60 | 0.78 | 97 | 12.3 | 69.1 | 28.9 | 1.90 | 911 | 12.0 | 0.0159 | 80 | 0.06 | 0.84 |
| 2.4 (base) | 26682 | 0.89 | 0.50 | 102 | 28.9 | 69.1 | 24.4 | 2.41 | 0 | 0.0 | 0.0170 | 75 | 0.13 | 0.55 |
| 2.8 | 26603 | 0.60 | 0.22 | 91 | 12.9 | 72.7 | 33.3 | 2.79 | 0 | 0.0 | 0.0154 | 75 | 0.07 | 0.24 |

## slope — channels (lidar.channels)

| channels | points/scan | ground left % | vehicle loss % | BEV cells | false cells | coverage % | far cov. % | est. height m | own pts left | ego cells | mean S~ | ms/scan | slope ground left % | slope vehicle loss % |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 16 | 26614 | 0.82 | 0.05 | 69 | 9.6 | 72.7 | 33.3 | 2.40 | 0 | 0.0 | 0.0127 | 74 | 0.05 | 0.05 |
| 32 (base) | 26682 | 0.89 | 0.50 | 102 | 28.9 | 69.1 | 24.4 | 2.41 | 0 | 0.0 | 0.0170 | 75 | 0.13 | 0.55 |
| 64 | 26706 | 0.44 | 0.00 | 86 | 8.1 | 72.7 | 33.3 | 2.39 | 0 | 0.0 | 0.0153 | 75 | 0.02 | 0.00 |

## slope — range (lidar.range)

| range | points/scan | ground left % | vehicle loss % | BEV cells | false cells | coverage % | far cov. % | est. height m | own pts left | ego cells | mean S~ | ms/scan | slope ground left % | slope vehicle loss % | ground left % <50 m | vehicle loss % <50 m | false cells <50 m | coverage % <50 m |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 50 | 25400 | 0.06 | 0.35 | 76 | 5.8 | 71.8 | 31.1 | 2.39 | 0 | 0.0 | 0.0115 | 76 | 0.11 | 0.38 | 0.06 | 0.35 | 5.8 | 71.8 |
| 80 | 26060 | 0.31 | 0.49 | 89 | 15.7 | 70.9 | 28.9 | 2.40 | 0 | 0.0 | 0.0132 | 79 | 0.09 | 0.53 | 0.05 | 0.49 | 5.0 | 70.9 |
| 120 (base) | 26682 | 0.89 | 0.50 | 102 | 28.9 | 69.1 | 24.4 | 2.41 | 0 | 0.0 | 0.0170 | 75 | 0.13 | 0.55 | 0.07 | 0.50 | 8.0 | 69.1 |
