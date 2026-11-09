# Stationary system operation

The canonical project is `jelsayyid/radar-navigation`, installed as one
`radar_navigation` ROS package. The Jetson workspace is
`/home/tejas/radar_integration_ws`; its historical name does not imply another
integration repository is required. The old `/home/tejas/radar-bot-navigation`
checkout and its `ros2_ws` are preserved history and are no longer sourced.

## Build and launch

From a clean Humble terminal:

```bash
source /opt/ros/humble/setup.bash
cd ~/radar_integration_ws
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --symlink-install --packages-select radar_navigation
source install/setup.bash
ros2 launch radar_navigation system.launch.py viewer:=true
```

On the existing Jetson, the equivalent single command from a fresh terminal is:

```bash
~/radar_integration_ws/src/radar_navigation/scripts/start_stationary.sh viewer:=true
```

Stop with **Ctrl+C in that terminal**. Launch stops its children; the driver sends
`sensorStop` and releases both serial ports before destroying ROS. No `pkill`,
background service or separate TF command is required. A required child failure
shuts down the complete launch. Optional viewer failure does not stop acquisition.
`radar.launch.py` is a compatibility alias for this same stationary system.

Only `awr1843_driver` opens hardware. It discovers one XDS110 command/data pair;
OpenCR is never selected. Explicit `cli_port:=... data_port:=...` is supported
for diagnostics/multiple-device disambiguation. Check that another program does
not own the radar before launching. Two concurrent launches are unsupported;
exclusive serial access rejects the second and its transient TF must not be
used as a second system. Operate one launch per robot/domain.

## Configuration and interfaces

- Installed `config/radar_mount.yaml`: `(0.150,0.060,0.190)` m from floor-level
  `base_footprint` to antenna-centered `radar_link`, zero nominal RPY. One
  `radar_tf` process publishes it; the cloud node and viewer publish no mount.
  Translation uncertainty +/- 1-2 cm, angles visually estimated. This is the
  current third/top-plate assembly, not a precision calibration or movement
  qualification. Old `(0.036,-0.065,0.185)` is preserved in imported Git history.
- Installed `config/awr1843_profile.cfg`: physically verified ~10 Hz SDK 3.x
  profile, character-paced CLI at 115200 baud and data at 921600 baud.
- `/radar/points`: `radar_link`, x-forward/y-left/z-up, FLOAT32
  x/y/z/velocity/snr/noise, reliable/volatile. Timestamp is host receipt; hardware
  counter is unavailable. The pipeline uses a best-effort subscriber compatible
  with that publisher and retains quality in metadata.
- `/radar_front/points_raw`: source-frame x/y/z/doppler.
- `/radar_front/points`: filtered/history x/y/z/doppler in `base_footprint`.
- `/radar_front/occupancy`: transient-local 200x200, 0.1 m cells, 5 Hz.
  Fresh evidence only refreshes cells; 1.5 s TTL + map timer returns old evidence
  to unknown (-1). Unknown is not asserted free. Static zero-Doppler obstacles
  remain eligible; point history lasts 0.5 s.
- `/radar_front/status`: JSON stale flag after about 0.5 s without accepted input.
- `/radar_front/metadata`: source geometry/quality, subscriber sequence and
  explicit `hardware_frame_number_available: false`.

No controller, odometry publisher or `/cmd_vel` publisher is started. The default
fixed frame is appropriate only while stationary. Future moving tests require
real measurement-time `odom` TF and `fixed_frame:=odom`, plus separately validated
base watchdog and stopping behavior. Never invent static odometry. Current
perception is planar; the launch rejects roll/pitch beyond its supported range.

## View, inspect and record

Viewer subscribes to ROS by default; the launch always passes `--ros`. It does
not configure or stop the radar. Forward TCP port 8765 over your existing SSH
connection (`-L 8765:localhost:8765`), then open `http://localhost:8765` locally.
The page uses radar-style right/forward display axes; ROS messages remain
forward/left. The web viewer shows source points; use RViz for processed mapping:

```bash
rviz2 -d "$(ros2 pkg prefix radar_navigation)/share/radar_navigation/config/stationary.rviz"
ros2 topic hz /radar/points
ros2 topic hz /radar_front/points
ros2 topic info /radar_front/occupancy --verbose
ros2 topic echo /radar_front/status
ros2 run tf2_ros tf2_echo base_footprint radar_link
ros2 run radar_navigation radar_probe --seconds 30 --output /tmp/radar_metrics.json
ros2 bag record -o /home/tejas/radar_integration_artifacts/new_trial /radar/points /radar_front/points_raw /radar_front/points /radar_front/metadata /radar_front/status /radar_front/occupancy /tf_static /tf
```

Use a new bag directory every time. Stop the recorder separately with Ctrl+C
before disconnecting. For RViz without a local Jetson display, use a ROS-enabled
remote workstation on domain 30 with working DDS networking, or an established
desktop connection; SSH port forwarding alone does not carry DDS.

## Recordings and analysis

Raw recordings stay outside Git:

- `/home/tejas/radar_integration_artifacts/8077be6_20261108T1622`: first deployment,
  probe and input-stop checks; not a clean surveyed empty-scene baseline.
- `/home/tejas/radar_integration_artifacts/top_plate_20261108T1715`: phased
  physical experiment; `experiment_bag/`, `observations.jsonl`, `events.jsonl`,
  survey, source revisions, graphs, metrics, plots and checksums.

Compact November 8 results/plots are committed under `results/top_plate_20261108`.
See [the physical experiment](TOP_PLATE_EXPERIMENT_2026-11-08.md) for surveyed
geometry, confounders and actual evidence-expiry measurements. Reproduce on the
Jetson with the analysis directory unchanged:

```bash
share="$(ros2 pkg prefix radar_navigation)/share/radar_navigation"
python3 "$share/scripts/analyze_spatial_experiment.py" /home/tejas/radar_integration_artifacts/top_plate_20261108T1715
python3 "$share/scripts/analyze_spatial_expiry.py" /home/tejas/radar_integration_artifacts/top_plate_20261108T1715
```

These scripts consume phased observer JSONL plus the finalized bag; they are not
generic bag-only evaluators. NumPy/matplotlib are declared analysis dependencies.

## Development

```bash
python3 -m unittest discover -s tests -v
ROS_DOMAIN_ID=89 python3 -m unittest discover -s tests_ros -p test_cloud_input.py -v
ROS_DOMAIN_ID=89 python3 -m unittest discover -s tests_ros -p test_system.py -v
# Broader experimental regression suite:
ROS_DOMAIN_ID=89 python3 -m unittest discover -s tests_ros -v
```

Run middleware tests in an isolated domain, not the physical robot's domain 30.
The observed Jetson has an [optional Nav2 native teardown failure](NAV2_NATIVE_TEARDOWN.md);
full discovery intentionally reports it instead of ignoring a segmentation fault.
They use pseudo-terminals and synthetic topics. Control tests publish only dry
test topics and never open OpenCR. `test_system.py` checks the installed default
mount, actual canonical driver, source conversion, occupancy/expiry, subscriber
viewer, installation contents, clean stop and profile-rejection cleanup.

The ROS perception node has one live input path: the hardware driver's cloud.
Recorded JSONL replay remains available through `replay:=... input_topic:=''` and
an appropriate frame/TF. `radar record` and the core parser remain offline tools;
they are not a second production ROS driver. Standalone `radar_test.py --ros` is
retired. `demo.launch.py` and `navigation.launch.py` are experimental dry-output
control paths requiring odometry/localization; they are not part of this task's
physical stationary validation.

Update `PROGRESS.md` after each implementation, test, failure or experiment.
Record source revisions, method, measured values, evidence paths and limitations;
distinguish physical hardware from PTY/container/simulation results.

An authorized operator can repeat the real stationary smoke check (it opens the
radar, records a bag, and stops only its own launch):

```bash
python3 "$(ros2 pkg prefix radar_navigation)/share/radar_navigation/scripts/smoke_stationary.py" --seconds 15 --output /home/tejas/radar_integration_artifacts/new_smoke
```
