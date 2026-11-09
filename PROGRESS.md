# Engineering progress

## Current status
One canonical installable ROS package now contains Wenqian's verified acquisition
and subscriber viewer, cloud perception, occupancy, calibration and launch. On
the Jetson use ~/radar_integration_ws and system.launch.py (or the foreground
start_stationary.sh helper); the old driver workspace is archival, not sourced.
The default top-plate TF is (0.150,0.060,0.190) m, nominal zero angles, one publisher.

Current base integration adds guarded native TurtleBot3 bring-up, real 20 Hz wheel
odometry, unique odom/base/radar TF edges, and odom-frame radar perception. Dry
reactive control/dropout checks pass; corrected offline suite 33/33 passes.
Supported-wheel stop tests passed: explicit zero stop 0.212 s, disable 0.258 s,
command producer death 0.429 s, guard/native-child death/OpenCR timeout 0.600 s.
CRITICAL: visual observation found forward wheel tops moving away from the
radar-facing observer, while left-turn directions matched. Encoder signs alone
did not validate physical forward direction. Floor testing is blocked pending
chassis/radar frame diagnosis and correction; no blind command sign flip.
No floor motion or ground-truth odometry accuracy has been validated. Default
drive launch is dry/disarmed; stationary sensing-only launch remains available.
See docs/BASE_INTEGRATION.md and docs/BASE_TEST_2026-11-08.md.

Physical November 8 extended-target results and limits remain below and in
[the report](docs/TOP_PLATE_EXPERIMENT_2026-11-08.md): support boxes sustained
target-area occupancy after foil/marker removal; all-added-object removal yielded
unknown-cell expiry at 1.529/1.639 s from fresh evidence. Spatial return modes and
survey/tilt/support ambiguity prevent a precision foil-accuracy claim.
Nav2's independent teardown investigation remains outside this hardware change.
Next: resolve physical forward/frame mismatch, then explicitly approved low-speed
floor radar stopping/avoidance and measured odometry drift/traction trials.

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

### 2026-11-08 — Canonical-system consolidation: inventory and design
- Reinspected clean Jetson checkouts: canonical 79417f0 and Wenqian local 9bee692;
  GitHub heads remain 79417f0 and 528bccd respectively. XDS110 if00/if03 are
  present and unowned; no radar/robot processes active. No motion authorized.
- Chose one ROS package and one primary workspace: radar_navigation in
  ~/radar_integration_ws. Integrating the physically verified hardware driver
  into radar_navigation/driver and the 9bee692 ROS-subscriber viewer. Keep the
  proven /radar/points interface and the external-cloud perception pipeline.
- Preserve Wenqian's exact commit ancestry as a second merge parent, not just
  an unattributed source copy. Selected files are imported into canonical layout;
  her historical root tree is not installed as a second project. Original checkout
  and experiment artifacts remain archival references, not runtime dependencies.
- Default mount becomes (0.150,0.060,0.190) m, nominal zero angles. Plan one
  stationary system.launch.py owning acquisition, mount TF and cloud perception,
  with optional subscriber-only viewer. Existing motion/replay launches remain
  experimental; stationary launch starts no controller or command publisher.
- Build/test must use /opt/ros/humble plus this package only, excluding the old
  awr1843_radar overlay. Packaging/install closure and a complete-launch PTY test
  are required before real radar smoke validation. Results pending at this stage.

### 2026-11-08 — Canonical-system consolidation: implementation
- Imported Wenqian's hardware driver and 9bee692 viewer into one package.
  Retained packet framing and source x/y/z/velocity/snr/noise schema; driver now
  resolves its configuration from radar_navigation. Live perception consumes
  that cloud only; removed its alternate serial acquisition worker. JSONL replay
  and standalone offline recording remain explicit research tools.
- Added system.launch.py with the default measured top-plate mount, one TF, one
  serial owner, filtering/occupancy/health, optional ROS-only viewer, and shutdown
  of the complete launch when a required process exits. radar.launch.py is an
  alias. demo.launch.py now composes the same acquisition path for future dry
  control research rather than another serial/TF implementation.
- Added foreground start_stationary.sh (Ctrl+C stop), installed all launches,
  config/RViz, docs and analysis scripts, declared YAML/analysis/ROS dependencies.
  Moved the radar profile to one canonical config path; removed its root duplicate.
- Fixed imported driver's shutdown-context warning using the existing signal-event
  lifecycle: stop radar before destroying ROS. Constructor failures release ports;
  rejected config aborts the launch; multiple automatic XDS110 matches require
  explicit ports. These are integration hardening changes, pending tests.
- Viewer now defaults to ROS subscription; direct serial access requires explicit
  legacy --serial. Retired radar_test.py's standalone ROS publisher. README leads
  with the current architecture and launch; old design/deployment docs are marked
  historical instead of presented as current instructions. Added contribution
  provenance and preserved the unlicensed-upstream/TODO license fact.
- Added meaningful hardware-parser regressions and a complete installed-system PTY
  test: new default mount, conversion, occupancy expiry/stale health, web subscriber
  viewer, installation closure, clean sensorStop and rejected-profile teardown.
  Updated the older delayed-odometry/control test to use the canonical hardware
  driver instead of the removed alternate serial path. Verification pending.

### 2026-11-08 — Consolidation verification: first failures and corrections
- Clean /opt-only build passed in 4.00 s; all 29 offline tests passed in 1.634 s.
  Complete stationary PTY/default mount/viewer/expiry and rejected-profile tests
  passed initially, as did cloud-input/localization/costmap marking assertions.
- Full middleware run exposed a fixture regression: adapting delayed odometry to
  base_footprint left the control test's base_link disconnected. The gate correctly
  refused to move its dry output; added the real-model 1 cm fixed base joint to
  the test fixture, not to the production stationary TF tree.
- Optional viewer via a nested `ros2 run` process did not receive launch SIGINT;
  ROS launch escalated it to SIGTERM. Changed to the installed viewer executable
  directly and added an assertion forbidding shutdown escalation. Updated the
  imported viewer startup message to the canonical launch.
- Nav2 1.1.20 costmap marking/unknown propagation assertions passed, but its
  destructor emitted a segmentation fault after cleanup. Strengthened the test
  to assert native exit status and added explicit lifecycle shutdown after cleanup;
  diagnosis/retest pending. This optional Nav2 server is absent from the working
  stationary launch and was never connected to robot commands.
- Git push dry-run failed: no HTTPS credentials on Jetson. Connected GitHub app
  can publish canonical files but cannot preserve arbitrary original commit
  author/timestamp objects. Revised provenance approach: publish a 365 KiB Git
  bundle containing Wenqian's exact 9bee692 ancestry, plus imported source and
  attribution; no new runtime dependency or parallel working project. Local
  pre-consolidation bundles/build outputs are preserved in the artifact directory.
  Earlier planned direct second-parent import is superseded for GitHub publication.

### 2026-11-08 — Consolidation verification: diagnosis and supported boundary
- Delayed-odometry/watchdog regression passed after restoring its base joint;
  measured dry perception-dropout stop was 0.299 s in a PTY run, not physical
  braking. The complete installed stationary system and rejection tests passed
  in 9.615 s; direct viewer executable now stops cleanly without escalation.
- GDB confirmed the optional Nav2 1.1.20 crash in
  rclcpp::CallbackGroup::~CallbackGroup during NodeBase/lifecycle destruction.
  Explicit shutdown transition did not fix it; a compiled diagnostic destroying
  ROS objects before context shutdown still exited -11. No ROS libraries modified.
  Retained exact logs/diagnostic in consolidation artifacts. This is an unresolved
  native Nav2 limitation outside the stationary launch, not an all-passing suite.
- Strengthened costmap test checks exit code instead of silently accepting SIGSEGV.
  Moved its cleanup assertions after Python node/context cleanup to prevent the
  failed native test contaminating subsequent tests. Documented required
  stationary tests separately from broader experimental regressions.
- Added a repeatable operator-authorized real smoke script that checks idle USB
  ownership/ROS graph, runs the canonical launch plus ROS viewer, records a bag,
  measures settled source/processed/map rates, unique default mount and health,
  and verifies complete clean stop without command publishers. Physical results
  pending; this is not a surveyed spatial experiment.

### 2026-11-08 — Consolidation: final installed and physical verification
- Final clean-underlay symlink build: 4.06 s. Offline: 29/29 pass in 1.639 s.
  Broader middleware discovery on domain 89: 5 pass, 1 failure, 0 errors in
  27.995 s. The one failure is the documented optional Nav2 native teardown;
  all three required stationary middleware tests pass, as do delayed-odometry/
  watchdog and localization-loss tests. No skip or ignored native return code
  conceals the limitation. Startup cloud queue warnings now identify `Full`
  explicitly: bounded backlog while discovery/TF catches up, not an unknown error.
- Normal non-symlink install: built in 3.00 s, two complete-system tests pass in
  9.584 s from /tmp outside the checkout. Installed imports resolve under the
  normal-install prefix. Additional /opt + local_setup-only import check excludes
  both primary workspace and legacy overlay prefixes. Installed launch/config,
  driver, viewer, map RViz settings and analysis scripts are present.
- Fresh-terminal start helper --show-args succeeds without inherited ROS setup.
  Canonical system/default mount tested. Removed exact orphan viewer PID 6771,
  port 60847, left by the earlier nested-ros2-run test; the direct-executable fix
  subsequently stopped all children without SIGTERM escalation.
- Real stationary smoke, domain 30, no legacy awr1843_radar prefix: settled
  duration 15.0163615 s; 150 source, 150 raw and 150 processed messages; 75 maps.
  Measured first-to-last rates: source 10.006272 Hz, raw 9.999479 Hz, processed
  10.000377 Hz, occupancy 5.000005 Hz. Last map had 16 occupied room cells;
  this unsurveyed room smoke is not another reflector-accuracy experiment.
- Source schema x/y/z/velocity/snr/noise, 24 bytes per point, radar_link; raw and
  processed x/y/z/doppler, 16 bytes; processed frame base_footprint. One radar_tf
  publisher and one (0.150,0.060,0.190) identity-rotation transform observed.
  No stale status during settled window; no /cmd_vel publisher; viewer HTTP 200.
  System and recorder exit 0, sensorStop logged, no shutdown escalation/traceback.
- Raw smoke bag and complete logs in consolidation_20261108T1900/physical_smoke;
  compact results and runtime source hashes in results/consolidation_20261108.
  Earlier bundles, build/install/log trees, failed attempts and native diagnostic
  are preserved externally. No large raw bag or generated build tree is committed.
- Canonical source and contribution archive now include all working Jetson-only
  driver/viewer code from 9bee692. No required production source remains solely
  in the old checkout. Authenticated GitHub app publication is used because Git
  HTTPS push is not configured; full original exact ancestry is recoverable from
  the committed source-history bundle. Local temporary merge ancestry is not
  misrepresented as a published Git merge parent.
- Remaining limits: mount is approximate and not movement-qualified; isolated
  foil/spatial-sign calibration, real odometry/command timeout/stopping distance,
  full Nav2 actions/native teardown, dual-radar operation and radar-only
  localization remain open. Stationary consolidation does not authorize motion.

### 2026-11-08 — Physical-method retrospective and publication audit
This entry consolidates earlier measurement details for course-report use; it
records no additional physical trial or motion.
- Initial inventory (~15:06 EST): ubuntu/aarch64, Humble domain 30, no active
  radar/viewer/odom graph, only OpenCR enumerated. XDS110 reappeared before the
  16:22 deployment. Absence was not diagnosed as a hardware fault. Host process/
  serial checks required inspection outside the sandbox's hidden PID/device view.
  The first passive ROS observer used the wrong global executor with an explicit
  Context; corrected to SingleThreadedExecutor(context=ctx). Inventory was read-only.
- Runtime 8077be6 used the preserved 9bee692 driver and original mount
  (0.036,-0.065,0.185) m. First bag: 178.199 s, 37.6 MiB, 7,594 messages.
  Required 30.0188 s probe: raw 278/9.2609 Hz, processed 277/9.2275 Hz including
  subscriber discovery, receipt-to-probe p50 19.486 ms/p95 21.271 ms. Settled
  intervals were ~10 Hz. These ages are not hardware acquisition latency;
  hardware frame gaps/resets remain null because the driver discards its counter.
- After repositioning to the third/top plate, the operator measured antenna
  center from the floor projection of the drive-wheel midpoint: x=+0.150,
  y=+0.060, z=+0.190 m, +/- 1-2 cm ruler uncertainty. Angles were visually zero,
  not inclinometer measurements. Robot stationary; plate security for movement
  explicitly unverified. Current default retains these qualifications.
- Before the new baseline, operator corrected that earlier data may contain the
  reflector. Waited a logged 5.007 s absent interval and recorded a fresh 30.147 s
  baseline; earlier bags were preserved, never relabeled clean ground truth.
- Survey: deepest foil-cavity center (1.150,0.060,~0.190) m. The confirmed 1.00 m
  antenna distance was to that recessed point, not the rim. Opening 0.30 x 0.21 m,
  depth 0.16 m, front rim x~0.990 m. Conservative foil box x=[0.99,1.15],
  y=[-0.09,0.21], z=[0.085,0.295] m, with target uncertainty +/- 1-2 cm.
- Full-assembly 30.140 s phase included newly added stacked boxes and an
  unsurveyed fiducial/cardboard behind/above the foil. They were absent from the
  baseline. Foil and marker/cardboard were attached and removed together, not as
  separate factors. Boxes-only phase lasted 36.969 s; final all-added-objects-
  absent phase 39.376 s. 1,236.054 s / 285.9 MiB / 67,780-message continuous bag
  retains operator transitions, which are excluded from stable phase comparisons.
- ROI was the physical box padded 0.12 m forward and 0.20 m lateral/vertical,
  an association window rather than an accuracy tolerance. Baseline-contrast
  0.1 m voxels required >=20% present-frame occupancy and >=20 percentage-point
  increase; target-area map cells required >=50% present maps and <=5% baseline.
- Present-mode coordinates (1.061,0.060,0.092) and (1.233,0.060,0.061) m give
  recessed-center residuals (-8.9,0,-9.8) and (+8.3,0,-12.9) cm, 3D magnitudes
  13.2/15.3 cm. These are not isolated reflector sensor errors. One mode fits the
  foil envelope; the other is ~8.6 cm outside it. Pooled x median 1.147 m lies
  between modes and must not be advertised as 3 mm accuracy. Centered placement
  cannot establish lateral sign correctness; heights/tilt/supports remain confounded.
- Cells 20110/20112 centered at (1.05,0.05)/(1.25,0.05) m stayed occupied in
  100% of boxes-only maps. After final clearing, actual map header-time expiry
  from replayed fresh evidence was 1.638813337 / 1.528897564 s, respectively,
  with zero recurrence in the analyzed window. Both unknown by 22:39:23.982500 UTC.
  Exact action timing was not observed: foil/marker request-to-ack interval
  130.51 s; boxes 58.57 s. Cells were unknown ~26 s before acknowledgement, so
  the first post-ack map (~0.1 s) is not physical removal latency. Unknown != free.
- Spatial report generation briefly failed serializing NumPy int64 voxel keys;
  converted keys to ordinary integers and reran all analysis. Acquisition/bags
  were unaffected. Numeric processed-point/mount consistency and raw quality
  preservation were verified separately from physical spatial accuracy.
- Consolidated implementation was published as a952910cf5ff1a4759fbcf9cd6bc832963e54e08;
  canonical GitHub main and the clean tracking Jetson main matched. Publication
  tree matched the reviewed/staged tree exactly, and runtime hashes matched the
  physically smoke-tested code. This subsequent journal-only audit preserves
  the quantitative reconstruction without changing the tested runtime.

### 2026-11-08 — Base/odometry development: inventory and preparation
- Starting from clean tracking main 9f6d201; canonical GitHub main matches.
  No parallel cloud-agent commits/branches were present at initial check.
- Jetson ubuntu/aarch64/Humble domain 30. OpenCR by-id resolves ttyACM0; radar
  XDS110 command/data ttyACM1/ttyACM2. All unowned, no robot/radar processes active.
  Installed turtlebot3_node 2.3.6 and Waffle Pi model/parameters available.
- Stock robot.launch starts LiDAR as well as the base and state publisher. Plan
  a canonical base-only composition, with the real odometry node as sole
  odom->base_footprint publisher and URDF sole base_footprint->base_link publisher.
  Hardware mount remains sole radar_tf child of base_footprint.
- Humble-specific Waffle Pi parameters select unstamped Twist (root non-Humble
  YAML selects stamped messages); use the correct Humble interface explicitly.
  Wheel radius 0.033 m/separation 0.287 m are installed defaults, not newly
  surveyed wheel geometry. Stock odometry fuses IMU heading; lifted differential
  wheel tests need wheel-only odometry because the chassis will not yaw.
- Requested operator preparation before bring-up: secure top plate/cables, stable
  support with both wheels clear, normal motor battery/power connected, and
  operator able to cut power. No motor commands or base bring-up yet. Physical
  floor motion requires later explicit approval; Nav2 is outside this workstream.

### 2026-11-08 — OpenCR power/reset diagnosis and watchdog design
- Read-only DXL protocol 2 / ID 200 probe: model 0x5000, model information 2,
  firmware register 5, supply 12.02 V. Motor device status -1, connected=false,
  torque=false, all motor telemetry zero. USB logic worked, motor initialization
  had failed before power was attached. No commands or firmware flashing used.
- First reported reset did not reboot OpenCR: uptime ~20,090,463 ms (~5.6 h).
  Requested the labeled RESET specifically, avoiding SW1/SW2 motor-test buttons.
  Second reset verified uptime 65,599 ms, 12.04 V, device status 0, both motors
  connected, torque=true, zero velocity and encoder positions 1772/3737 ticks.
  Proper power followed by an actual reset resolved the latched connection state.
- Upstream ROBOTIS source inspection: OpenCR has a 500 ms ROS-node heartbeat
  timeout; base node sends heartbeat every 100 ms even if cmd_vel stops. Velocity
  write callbacks have no command-age watchdog in the inspected source. This
  cannot yet be claimed as behavior of this specific board; physical tests pending.
- Added separate base_watchdog using stock turtlebot3_ros as its serial-owning
  child. Command lease 0.25 s, enable lease 0.3 s, odometry/hardware freshness
  0.3 s, dry by default, speed caps 0.05 m/s and 0.2 rad/s. Child subscribes only
  /base/driver_cmd_vel; conventional /cmd_vel is not wired directly to motors.
  Guard requires fresh healthy motor torque/voltage telemetry as well as odometry.
- Linux PR_SET_PDEATHSIG=SIGKILL links base child lifetime to watchdog death,
  preventing a surviving heartbeat process from keeping a latched command alive
  when its parent watchdog dies. Child spawned before rclpy/DDS threads start;
  fork-race parent-PID check included. Pure Linux parent-death regression passed.
- Added base.launch.py (base + stock Waffle Pi URDF, no LiDAR) and drive.launch.py
  composing real odom, existing radar with fixed_frame=odom and dry reactive
  output. Wheel-only odometry selected for lifted tests; IMU mode remains an
  explicit option because chassis yaw is fixed on the support.
- Sandbox offline run passed new lease/PDEATHSIG tests but existing viewer socket
  test was denied by sandbox networking. This is an environment failure; rerun
  full tests on the host before treating build/test status as passed.

### 2026-11-08 — Real base/odom and dry radar-control validation
- Built initial integration in 4.01 s; host offline suite 32/32 passed in 1.807 s.
  Started base with actuate=false, recorded telemetry and stock Waffle Pi URDF.
  OpenCR gyro calibration completes in 5 s; actual wheel/sensor/IMU/odom data
  publishes ~20 Hz. Both motors connected, torque=true, supply ~12.04 V.
  Verified real odom->base_footprint and URDF's 0.01 m base_link joint.
- Started radar with fixed_frame=odom and reactive dry controller capped at
  0.05 m/s / 0.2 rad/s. Observed enabled dry forward output 0.05 m/s; all actual
  base-command outputs remained zero because the independent guard was dry.
- Controlled SIGSTOP/SIGCONT of only owned driver/native-base processes tested
  sensor and odometry dropouts while operator lease stayed alive. Independent
  observer measured disable-to-zero 0.031 s, radar-pause-to-zero ~0.300 s, and
  odometry-pause-to-zero ~0.269 s; health became stale and processing stopped
  without invented/latest odometry. Initial validation script reported false
  timing failures because it serviced fewer callbacks than incoming topic rate;
  removed its artificial sleep and rate-limited enable publication independently.
- Independent evidence also found a real problem: 13 source frames in 0.25 s
  after reader resume, buffered before/through the pause but host-stamped fresh.
  Changed acquisition to flush UART + partial parser state after >0.2 s polling
  stall and retain only the newest complete frame from an ordinary read batch.
  Hardware acquisition timestamps remain unavailable; this prevents stale UART
  replay, not a claim of measured hardware latency. Physical retry pending.
- Guard parameter validation now rejects nonfinite/nonpositive leases and limits
  beyond 0.10 m/s / 0.35 rad/s absolute bounds. Unknown status ages serialize as
  JSON null rather than Infinity. No powered wheel motion has yet been commanded.

### 2026-11-08 — First powered supported-wheel pulses and measurement correction
- Operator confirmed secured top plate, stable support with both wheels clear,
  and 12 V / 5 A supply. No floor motion authorized or commanded.
- Short 0.04 m/s forward and +0.2 rad/s left-turn commands produced expected
  encoder signs: both positive forward, left negative/right positive turning.
- First harness aborted before disable/death tests because its activity threshold
  assumed JointState velocity in rad/s. Installed driver reports ~0.039 m/s
  during 0.04 m/s forward and ~0.028 m/s during 0.2 rad/s turn, consistent
  with wheel surface velocity. Changed activity threshold to 0.01 m/s and stop
  threshold to 0.002 m/s, requiring three consecutive samples. The initial
  0.039 s explicit-zero result used the wrong threshold and is NOT accepted.
- Failure cleanup disabled the lease and killed the guard/native child. Preserved
  all data in base_development_20261108T1940/lifted_motor_test; corrected retry
  uses a new directory. No emergency power action was needed.

### 2026-11-08 — Accepted supported-wheel control and independent timeout results
- Corrected reader/dropout retry passed: disable output zero 0.014665 s, stale
  radar/odom stopped dry output, no processed frames >1 s into odometry pause,
  base commands always zero in dry mode, resume only 2 source frames/0.25 s.
- Corrected physical harness passed forward/turn encoder signs; measured wheel
  stop 0.211581 s explicit zero, 0.257795 s enable false, 0.429201 s producer
  SIGKILL with enable alive (guard reason stale_command). All motion on support.
- SIGKILL watchdog killed native serial child via Linux PDEATHSIG; port released.
  Read-only OpenCR polls measured stop at 0.599646 s, three consecutive samples
  <=2 raw motor units. Latched linear register remained 4 (0.04 m/s), heartbeat
  register 195; no new heartbeat sent. No emergency stop needed. This establishes
  independent heartbeat expiry on this board, not cmd_vel timeout with heartbeat.
- Stop criterion for ROS telemetry: <=0.002 m/s for three consecutive ~20 Hz
  samples; timestamp first sample. Single-trial measurements, not worst-case bounds.
- Forward two-second window encoder distance 0.070946 m, turn yaw 0.356466 rad.
  Median surface velocities +0.038777/+0.038777 m/s forward and -0.027698/
  +0.027698 m/s turn. Wheel geometry remains stock .033/.287 m, not surveyed.
  Chassis stationary throughout; these are NOT ground-truth body displacement.
- Operator did not watch first runs. Requested and executed separately announced
  visual two-second forward / three-second stop / two-second left-turn repeat.
  Awaiting independent visual confirmation; no floor movement authorized.
- Final offline rerun found /proc process-exit race in test cleanup, fixed by
  handling disappearance; 33/33 then passed in 1.756 s. Build 4.12 s. Earlier
  33/33 run passed in 1.875 s; no Nav2 tests/code changed in this hardware task.
- Full real composition produced ~20 Hz odom, ~10 Hz source/processed and 5 Hz
  map in odom, base always zero while dry. Smoke observer initially used a
  two-argument callback unsupported by this Humble rclpy; fixed observer only.
  Then corrected an assertion which wrongly excluded RSP's legitimate /tf wheel
  joint updates. RSP and diff_drive_controller on /tf are expected, not duplicate
  odom publishers. Final quantitative retry follows.
- Artifacts: /home/tejas/radar_integration_artifacts/base_development_20261108T1940;
  hardware_bag, observations.jsonl, failed first and accepted retry motor runs,
  logs, probes, phase events and visual_repeat preserved. Compact summary/plot
  in results/base_development_20261108; analyze_lifted_test.py reproduces them.
- Documented guard-freeze limitation: PDEATHSIG handles death, not a hung guard
  while native heartbeat continues. No certification or universal fail-safe claim.

### 2026-11-08 — Complete launch smoke and operating workflow
- Final drive.launch.py smoke passed, 20.036974 s observer interval: 401 odom
  (19.999 Hz), 201 source (9.996 Hz), 201 processed (10.029 Hz), 101 occupancy
  (5.004 Hz). Processed/map frames odom; source radar_link. All 401 observed
  base outputs zero while dry, despite enabled reactive output reaching .05 m/s.
- Graph: sole /odom diff_drive_controller; sole source awr1843_driver; sole
  /radar/cmd_vel_safe radar_control; sole /base/driver_cmd_vel base_watchdog;
  /cmd_vel has no publishers. /tf legitimately contains diff_drive_controller
  odom and robot_state_publisher joint updates. /tf_static contains only RSP
  robot static links and radar_tf mount. Relevant chain: odom->base_footprint,
  base_footprint->base_link, base_footprint->radar_link. No synthetic/static odom.
- Added start_drive.sh foreground command (Ctrl+C stop), base operation and
  physical test reports, bounded supported-wheel/visual scripts, read-only
  OpenCR diagnostic, reproducible motor analysis and compact measured JSON/plot.
  All launches/config/scripts/docs included through existing ROS install globs.
  Added runtime dependencies for stock TurtleBot3, URDF/xacro and DXL SDK.
- Preserved previous stationary launch and all prior experiments/workspaces.
  New raw evidence remains outside Git; compact results only committed.
  Main remained 9f6d201 on pre-publication check; no Nav2 branch changes overwritten.

### 2026-11-08 — Visual direction mismatch blocks floor testing
- Published tested software/stop evidence as 29948a2080e6c3bac1fad3c59cc6f31d9ea9d763
  on canonical main. Both runtime/source trees verified identical to GitHub tree
  5d220812516f7e32a08e852b79c58e95c81c7eea; clean tracked checkouts.
  Published install rebuild passed in 4.07 s. Parallel Nav2 branch
  fix/humble-nav2-teardown at 4d3fb7d was left untouched.
- Finalized original hardware bag: 2778.455724 s, 584651 messages, ~637 MiB.
  Final read-only stopped probe: both raw wheel velocities zero, motors connected,
  torque true, 12.04 V. All launch nodes stopped cleanly; source hash manifest
  saved in artifact root. Raw data includes long idle intervals and multiple
  restarts; use phase events, not whole-bag rates, for performance claims.
- Operator watched first visual repeat but could not establish absolute forward
  direction. Clarified forward tire tops toward a person facing antenna/front;
  left turn observer-right wheel away / observer-left wheel toward. Executed
  separately announced second supported-wheel repeat, recorded visual_repeat2
  and visual_repeat2_bag. Encoders +/+ forward, -/+ turn; final velocities zero.
- Operator reports BOTH tops actually AWAY during commanded forward, but correct
  observer-left toward / observer-right away for left turn. Thus physical forward
  convention is NOT verified despite positive encoder/odom signs. No floor motion
  was attempted. Stopped all test base processes and recording; floor tests blocked.
- A 180-degree difference between native chassis forward and assumed radar-facing
  forward would reverse translation while preserving yaw sign, consistent with
  report, but this is a hypothesis, not established hardware diagnosis. Requested
  antenna/observer direction, original ruler axes and board/chassis FRONT marking.
  Must correct mount/TF and command/odom conventions consistently, or physically
  face radar toward native base forward; do not merely invert cmd_vel.

### 2026-11-08 — Trace direction without changing motor or TF signs
- Operator confirms observer stood directly in radar scan direction, also used
  as +x in original mount survey. Chassis/OpenCR FRONT marking not yet checked.
  Requested visible FRONT and motor ID labels, or readiness for isolated-wheel
  pulses if inaccessible. No floor movement; no command/odometry/TF sign changes.
- Reviewed own watchdog: nonnegative linear command passes unchanged; base child
  remap changes topic only. Official TurtleBot3 2.3.6 turtlebot3.cpp callback
  writes int(linear.x*100) and int(angular.z*100), no sign flip. Board register
  after previous physical guard-kill test retained +4, agreeing with +.04 m/s.
- Official OpenCR source uses left=v-w*b/2, right=v+w*b/2, motor IDs 1/2; source
  may not exactly match flashed board. Wheel telemetry and observer report fit
  multiple physical conventions, so do not declare a verified 180-degree frame
  reversal or motor assembly fault. Pure rotation preserves yaw sign under a
  180-degree body-axis rotation, hence correct-looking turn does not rule it out.
- Official 2.3.6 joint_state.cpp confirms RPM_TO_MS velocities, TICK_TO_RAD positions;
  this independently supports the measurement-threshold correction above.
  Saved upstream references in artifact root; no vendor firmware/code rewritten.
