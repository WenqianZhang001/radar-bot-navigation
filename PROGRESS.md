# Engineering progress

## Current status
The external-driver navigation pipeline was deployed to the physical Jetson at
8077be6 with Wenqian's local driver/viewer commit 9bee692 preserved. The separate
overlay built; 26 offline tests and the required domain-89 cloud-input middleware
test passed. Real radar clouds/processed output run near 10 Hz and occupancy near
5 Hz. Earlier recordings are not a surveyed empty-scene baseline.

The final top-plate configuration now has an operator-surveyed mount of
(0.150, 0.060, 0.190) m from base_footprint, nominal zero angles, translation
uncertainty +/- 1-2 cm and unmeasured visual angle uncertainty. A phased stationary
physical experiment recorded clean empty, foil/marker/boxes present, boxes-only,
and all-added-objects-removed conditions. The two target-area map cells stayed
occupied with boxes retained, then expired to unknown 1.529/1.639 s after their
last fresh evidence when boxes were removed, with acquisition continuing.

Mixed extended-target returns do not establish isolated foil accuracy: stable
forward groups were near 1.061 and 1.233 m versus a 1.150 m recessed center, with
elevation residuals and support/marker ambiguity. Detailed results, plots, limits,
and the explicit mount override are in
[the physical report](docs/TOP_PLATE_EXPERIMENT_2026-11-08.md).

No robot motion was commanded. Mount security, spatial calibration across
multiple positions, physical stopping, full Nav2 actions, dual-radar control and
radar-only localization remain unverified. Secure/recheck the mount and use an
unchanged-support surveyed reflector experiment before movement validation.

## Integration update — 2026-11-08
Both remote main revisions match the project handoff (`facbc10` and Wenqian's
`528bccd`). Source inspection confirms that a topic remap alone cannot connect
her ROS-coordinate cloud to the serial-owning perception node. Proposed path:
retain her deployed acquisition package and measured base_footprint transform,
add a subscriber input mode, and preserve Doppler/quality and hardware metadata.
See [integration findings](docs/INTEGRATION.md). Her driver currently discards
hardware frame number at publication. Driver/viewer exclusive serial ownership
needs inspection on the actual Jetson before deployment changes.

Next physical milestone: stationary real-radar occupancy with recorded surveyed
obstacles. Blocker: this cloud environment has no configured Jetson VPN access;
ROS is also absent locally. Added a bounded read-only Jetson inventory script,
shell-syntax checked only. No physical tests or new ROS integration tests ran.

## Chronological journal
### Prior work (dates not recorded)
- Initially planned Unitree Go2 navigation; switched to TurtleBot3 Waffle Pi to protect fragile radar hardware.
- Assembled TurtleBot3 with Jetson Orin Nano/OpenCR; configured Ubuntu, ROS 2, SSH, and robot bring-up (reported by project team).
- Mounted AWR1843BOOST in a printed fixture and connected power/USB.
- Implemented configuration, serial parsing, and live point-cloud visualization; verified physical detections.
- Implemented an initial unverified ROS 2 publisher.
- Established repository; initial code is commit 3ee7ab2.
- Cloud onboarding verified synthetic parsing, HTTP page and SSE delivery; live capture blocked by absent USB hardware.

### 2026-11-06
Inspected all repository source/configuration/documentation before edits. Preserving the viewer and firmware profile. Implementing a dependency-light core and ROS adapter, with conservative occupancy semantics and repeatable synthetic evaluation.

## Handoff context
TI coordinates are x right, y forward, z up; ROS sensor coordinates are x forward, y left, z up. Doppler is radial range rate, not Cartesian velocity. Current profile is ~10 Hz and aliases velocities outside approximately ±0.65 m/s. Host receipt timestamps are not hardware acquisition timestamps. Do not command physical movement without an operator. Read and update this journal after substantial changes.


### 2026-11-06 — Offline pipeline implementation and validation
- Added bounded SDK 3.x parser with length/count checks, corruption recovery,
  frame/CPU-cycle/subframe metadata, optional quality TLV, and unknown TLV preservation.
  Existing viewer now uses this parser through the compatibility interface.
- Added explicit serial-port acquisition, exclusive access, timeout, configuration
  rejection, sensor-stop cleanup, JSONL recording and paced replay.
- Added ROS PointCloud2 Doppler, metadata, odometry-frame filtered clouds and
  occupancy, sensor namespaces, time-gated fusion adapter, launch and Nav2 overlay.
  Multiple-device auto-discovery now fails rather than pairing arbitrary ports.
- Added range/height/SNR/finite gates, temporal voxel persistence, radial-motion
  distinction, planar reflector calibration, fixed-frame fusion, occupancy and A*.
  These algorithms are initial classical baselines; multipath and ego-motion
  handling require measured-data tuning. ROS currently does not supply ego velocity.
- Executed 15 unittest checks on Python 3.12, all passed. Covers all packet split
  positions, malformed packets/counts, empty frames, quality TLV, recording,
  transforms/Doppler, filtering, calibration, fusion expiry, unknown-space mapping,
  obstacle-aware planning, closed-loop simulation and original HTTP/SSE viewer.
- Synthetic perception evaluation: five seeds per mode; single radar model endpoint
  precision 0.976, recall 1.000; dual independent-sampling model 0.983/1.000;
  without accumulation 0.910/0.727. These optimistic toy-scene results are NOT
  physical sensor metrics. Outputs and methodology are committed under results/
  and docs/EXPERIMENTS.md; latency is local CPU processing only.
- Simplified closed-loop grid experiment: 15/15 goals reached, zero simulated
  collisions across three beam counts/five seeds. Assumes 360-degree coverage,
  ideal localization and simplified motion. It does not execute ROS/Nav2.
- Editable package installation, Python compilation and whitespace validation passed.
  Added CI for Python 3.10/3.12; remote CI execution is not yet verified.
- No repository test failures remained. No physical commands were sent.
  ROS imports/build, Nav2, Jetson performance, real calibration and hardware
  experiments remain unrun because this environment has no ROS or USB devices.
- Installation and deployment commands: docs/DEPLOYMENT.md. The Nav2 overlay
  requires a working robot parameter set; it is not a complete navigation launch.
  Global localization, aging/clearing, watchdog validation, real-map metrics and
  bag-to-ground-truth evaluation remain necessary for a successful demonstration.
- Commit reference: this implementation commit (resolve with git log -- PROGRESS.md).

### 2026-11-06 — Jetson runtime and control integration
- Shifted work from toy experiments to the real Humble deployment path.
  Added docs/JETSON_DEMO.md with minimum hardware actions and quantitative logging.
- Corrected mapping to use configured filter evidence. Accumulated points no
  longer refresh map evidence indefinitely. A 1.5 s TTL resets cells to unknown
  on a 5 Hz map timer, even if acquisition stops.
- Replaced raw-cloud Nav2 local perception with the processed occupancy grid.
  Humble StaticLayer receives full transient-local map updates and inflation.
  Unknown is unmarked in the local costmap, not asserted physically free.
  The global planner uses a prior static map; live LiDAR is retained only for
  AMCL in the proposed first goal-navigation demonstration.
- Added radar_control: 20 Hz output, 0.3 s perception/request/enable deadlines,
  invalid/empty input stopping, obstacle clearance checks, forward/turn caps and
  reverse rejection. Defaults to a dry topic and disarmed. Reactive policy
  generates slow forward/turn requests through the same gate.
  Base/OpenCR command timeout is still mandatory if the gate/host dies.
- Added localization gating for Nav2 (fresh map → odom, allowing AMCL future
  dating), measured mount launch, Nav2 parameter merge/launch and radar_probe
  for rates, host-receipt latency, sequence gaps and map counts.
- Actual Humble testing exposed shutdown-context errors; fixed signal handling
  so sensorStop and final zero commands occur before destroying the ROS context.
  A later test also exposed interruption inside a TF C-extension callback;
  signal handlers now set an event and the executor exits between callbacks.
  Short-window probe validation initially failed; the two-second integration
  window now records both raw and processed messages with preserved logs.
  Added bounded measurement-time TF retries for delayed wheel odometry; no
  silent identity/latest-transform substitution.
- Built the package with colcon on Ubuntu 22.04/Python 3.10 in a Humble container.
  Passed 22 core tests plus three real ROS middleware integration tests:
  serial acquisition through mapping and command stopping; missing/stale
  localization rejection; actual Nav2 StaticLayer marking and obstacle removal.
  The serial test uses 50 ms delayed odometry and verifies Doppler, obstacle stop,
  enable-release stop, sensor-dropout stop, map expiry and sensorStop cleanup.
- Built seven required Nav2 packages from official Humble source commit
  3c3db59d6969d8ecee8e68468693d006397f4a0c; validated the shipped costmap overlay.
  Network policy blocked packages.ros.org, so the official Nav2 dependency image
  and source build were used with TLS/checksum verification preserved.
- Nav2 parameter merge passed against the official default file. A separate
  attempt to build only nav2_bringup failed because its complete navigation
  dependency graph was not built. Full AMCL/controller/action execution remains
  unrun; costmap runtime validation is not a claim of full Nav2 readiness.
- Added Humble integration CI; remote CI results are not yet verified.
- No new physical results or movement commands. Hardware acceptance must measure
  actual stopping distance, blind spots, mounting/TF accuracy and interference.
  Preserve the existing physical history; do not label PTY middleware results as
  real sensor trials.
- Commit reference: this runtime-integration commit (git log -- PROGRESS.md).
- Recorded serial/ROS probe sample: 20 raw and 20 processed frames in 2.024 s
  (~9.88 Hz), zero sequence gaps, receipt-to-probe p50 71.8 ms/p95 84.6 ms
  with deliberately 50 ms delayed odometry; stale-output stop 0.300 s after
  pausing serial traffic. These are PTY/container timing results, not hardware
  latency or physical braking measurements.
- Final verification after the shutdown and TF fixes: all 22 core and all three
  Humble/Nav2 integration tests passed, with no skipped tests. A second probe run
  measured ~9.87 Hz and 84.4 ms p95; dropout output stopped in 0.250 s.
  Recorded representative runtime evidence is results/humble_runtime.json.

### 2026-11-08 — External driver integration
Jetson inventory reports local commit 9bee692 with a ROS subscriber viewer,
resolving the earlier exclusive-port concern. Preserve that commit: GitHub still
reports 528bccd. No radar USB or active ROS graph was observed during inventory.
Added subscriber input to the perception node, stationary cloud launch without
serial/TF/control ownership, schema validation, coordinate/Doppler conversion,
quality metadata and explicit unavailable hardware-counter reporting. Positive
SNR thresholds reject unknown quality. Added deployment instructions in
docs/JETSON_INTEGRATION.md. All 26 offline tests pass; compilation and diff checks
pass. Added a real ROS middleware test to the existing Humble CI suite; it has
not run locally because this cloud host lacks ROS. Physical validation remains
for Jetson deployment after XDS110 is available.

### 2026-11-08 — Physical Jetson integration and top-plate experiment
- Preserved deployed driver 9bee692 and all earlier artifacts; deployed navigation
  8077be6 to a separate overlay. Build, 26 offline tests and required middleware
  test passed. The earlier input-stop measurement gave stale health at 0.547 s
  and all-map expiry at 1.688 s; it was not physical reflector removal.
- After the radar moved to the third plate, obtained new operator measurements
  (0.150,0.060,0.190) m with nominal zero angles. Preserved old calibration and
  supplied a separate mount_file to the existing driver; one mount TF publisher
  verified. Plate security for movement was explicitly not assumed.
- Honored the operator's absent-scene correction before a fresh 30 s baseline.
  Added boxes/foil/attached marker afterward, then removed foil+marker together,
  retained boxes for a separate phase, and finally removed boxes. Entire session
  recorded without motor/control launches or movement commands.
- Stable source/processed output ~10 Hz, maps ~5 Hz, no stale physical-phase
  health. ROI returns: 0/301 baseline frames, 301/301 full-assembly frames,
  370/370 boxes-only frames, 0/394 final absent frames. Two map cells remained
  occupied in every boxes-only map; after final clearing actual bag values became
  unknown 1.529/1.639 s after last fresh evidence, with no later recurrence.
- Two spatial modes near (1.061,0.060,0.092) and (1.233,0.060,0.061) m. Their
  recessed-center residuals are not point-target sensor error. Do not interpret
  the between-modes pooled median as millimeter accuracy. Supports, attached
  marker, extended foil surface, survey/tilt uncertainty and multipath remain
  confounders. Exact removal seconds were not observed; report evidence-to-expiry.
- Saved finalized 285.9 MiB / 1,236.054 s bag, full metrics/logs and plots under
  /home/tejas/radar_integration_artifacts/top_plate_20261108T1715. Added compact
  results, mount reference and reproducible analysis scripts to this repository.
  All experiment processes stopped; no movement validation claimed.
