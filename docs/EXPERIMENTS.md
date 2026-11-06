# Experiment protocol

## Automated offline evidence
Run:
```bash
python3 -m radar_navigation.evaluate --seeds 5 --output results/synthetic
python3 -m radar_navigation.simulate --output results/navigation.json
```

The perception benchmark uses a stationary wall, Gaussian endpoint noise,
independent missed detections, and one synthetic dynamic outlier per frame.
Modes: idealized dense LiDAR, sparse single radar, two co-located independent
radar samples, no accumulation, and no Doppler motion distinction. These are
models, not measured sensor performance. Dual mode tests sampling density;
it does not model extrinsics, RF interference or occlusion. Precision/recall
use a 0.2 m endpoint tolerance. Endpoint error includes outliers and is not
physical range error. Report per-seed distributions, not just averages.
CSV, summary JSON and SVG recall plots are emitted.

The navigation simulator performs first-return ray sensing, occupancy updates,
inflated A* replanning and independent ground-truth collision checks. Trials
use 31, 61, or 181 beams over ideal 360-degree coverage with 10% missed returns.
All modes use identical noise/clearing logic; these beam counts must not be
reported as validated radar versus LiDAR configurations. Unknown space is allowed
during planning, with conservative footprint inflation and collision reporting.
Completion time uses an assumed 0.5 s per grid step, not measured robot time.
The simulated robot has no acceleration, slip, localization error or moving obstacles.

## Physical study
Use fixed start/goal poses for straight corridor, obstacle detour, narrow passage,
and moving pedestrian exclusion-zone tasks. Include normal lighting, darkness,
and visual occlusion conditions; do not assume darkness degrades LiDAR. Repeat
at least ten runs per condition and sensor configuration, randomizing order.
Compare LiDAR, single radar, dual radar, and ablations for accumulation,
Doppler analysis, and fusion. Keep robot speed, footprint, routes and localization
method fixed, and report any use of LiDAR for localization separately.

Record:
- Surveyed reflector locations and range error with confidence intervals.
- Annotated obstacle detections and one-to-one precision/recall matching.
- Surveyed occupancy reference: occupied IoU, free-space false negatives and
  fraction unknown. Do not exclude unknown cells without reporting coverage.
- Receipt-to-output latency, sensor frame gaps and processed frame rate.
- Goal result, collisions, operator interventions, wall-clock completion time,
  travelled odometry distance and shortest safe reference path length.
- Firmware/config hash, transforms, software commit, environment, operator and
  reason for every aborted run.

A collision or intervention is a failed autonomous run. Report failures in
success-rate denominators. No physical trial metrics have yet been collected
in this repository. Automated bag extraction, physical map scoring and range
ground-truth ingestion remain to implement once the recorded topic schema and
ground-truth format are available.
