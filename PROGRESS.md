# Engineering progress

## Current status
The original serial acquisition and browser viewer were verified on physical hardware by the project team. The initial ROS publisher was unverified and omitted Doppler. Implemented and offline-tested the reusable parser, recording/replay, classical filtering, calibration/fusion, occupancy, planner, and simplified synthetic experiments. Added ROS adapters, launch, and Nav2 overlay; these are not runtime-verified. No autonomous physical navigation claim is supported.
Immediate priorities: build on Humble, validate timestamps/TF and stationary clouds, collect calibrated single/dual recordings, integrate a stale-data motor watchdog and global localization, then run supervised navigation trials. This development machine runs Debian/Python 3.12, without ROS 2 or radar USB devices; physical navigation cannot be validated here.

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
