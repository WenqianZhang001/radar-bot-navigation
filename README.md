# Radar-guided TurtleBot3 navigation

The canonical working project for Yale ECE 9500: TI AWR1843BOOST radar on a
Jetson Orin Nano / TurtleBot3 Waffle Pi, ROS 2 Humble. Real USB acquisition,
processed point clouds, occupancy marking and obstacle-evidence expiry have
been verified while stationary. Robot motion is the next development stage.

```text
XDS110 USB -> awr1843_driver -> /radar/points -> radar_pipeline
                                |                 |-- processed points
                                |                 |-- aging occupancy + health
                                +-> optional browser viewer (ROS subscriber)
radar_tf: base_footprint -> radar_link (one publisher)
```

Everything required for the stationary system is in this repository and one
installable ROS package, `radar_navigation`. Wenqian's physically verified driver
and ROS viewer are integrated with their original history and attribution;
no separate hardware-driver repository/overlay is required.

## Build

On Ubuntu 22.04 / ROS 2 Humble:

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/radar_integration_ws/src
cd ~/radar_integration_ws/src
git clone https://github.com/jelsayyid/radar-navigation.git radar_navigation
cd ..
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --symlink-install --packages-select radar_navigation
```

The lab Jetson already uses this workspace. Check XDS110 power/USB enumeration
and `dialout` membership before operating. OpenCR is not a radar interface.

## Start and stop

From a fresh Jetson terminal:

```bash
~/radar_integration_ws/src/radar_navigation/scripts/start_stationary.sh viewer:=true
```

Or, after `source ~/radar_integration_ws/install/setup.bash`:

```bash
ros2 launch radar_navigation system.launch.py viewer:=true
```

**Ctrl+C in the launch terminal stops the system and sends `sensorStop`.** Omit
`viewer:=true` for headless acquisition/mapping. The launch starts no motor or
control nodes. Do not run an old driver/viewer serial session alongside it.

The current top-plate mount defaults to **(0.150, 0.060, 0.190) m**, nominal zero
rotation, from `base_footprint` to `radar_link`. No mount override or separate TF
command is needed. The ruler survey is accurate only to approximately 1-2 cm;
angles were visually estimated. Secure and recheck the mount before movement.

## View and inspect

Forward port 8765 through SSH and open `http://localhost:8765` for source points.
For the processed occupancy map in a graphical ROS session:

```bash
rviz2 -d "$(ros2 pkg prefix radar_navigation)/share/radar_navigation/config/stationary.rviz"
ros2 topic hz /radar/points
ros2 topic hz /radar_front/points
ros2 topic echo /radar_front/status
```

Source/processed clouds run near 10 Hz; occupancy near 5 Hz. Source fields are
x/y/z/velocity/snr/noise; processed fields are x/y/z/doppler, with quality retained
in metadata. Default mapping frame `base_footprint` is **stationary only**. Moving
mapping requires real measurement-time odometry, never a static odom substitute.

## Layout and evidence

| Path | Purpose |
| --- | --- |
| `radar_navigation/driver/` | Integrated Wenqian hardware driver and packet framing |
| `radar_navigation/` | Cloud conversion, filtering, occupancy, replay and control research |
| `launch/system.launch.py` | Canonical stationary launch; one TF and one serial owner |
| `config/` | Installed default mount, radar profile, RViz and experimental Nav2 settings |
| `radar_viewer.py` | Subscriber-only browser viewer by default |
| `tests/`, `tests_ros/` | Offline and isolated-domain middleware/PTY tests |
| `scripts/`, `docs/`, `results/` | Analysis, operation and compact experiment evidence |

November 8 physical experiment: no target-region returns in clean absent phases;
two persistent groups with the assembly present; cells stayed occupied with
support boxes retained. After all added objects were removed, two cells became
unknown 1.529/1.639 s after last fresh evidence while acquisition continued.
Mixed reflector/marker/support returns do not establish isolated foil accuracy.
Raw bags are preserved outside Git at `/home/tejas/radar_integration_artifacts/`.

[Operation, recording, analysis and development](docs/STATIONARY_SYSTEM.md) ·
[Physical measurements and plots](docs/TOP_PLATE_EXPERIMENT_2026-11-08.md) ·
[Engineering journal](PROGRESS.md) · [Contribution history](CONTRIBUTORS.md)

No physical autonomous navigation or stopping-distance validation is claimed.
Next: secure the mount, survey spatial performance with unchanged supports,
validate wheel odometry and the base command timeout, then perform separately
approved low-speed supervised control tests.

The broader Jetson middleware suite exposes an [optional native Nav2 teardown
failure](docs/NAV2_NATIVE_TEARDOWN.md); the stationary system does not run Nav2.
[Consolidation validation](results/consolidation_20261108/validation.json) records
29 offline passes, five middleware passes/one optional failure, install checks
and the subsequent real stationary smoke.
