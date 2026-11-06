# Jetson deployment and validation

## Install
Ubuntu 22.04 with an existing ROS 2 Humble and TurtleBot3 installation is required.
The cloud tests use Python 3.12; the following ROS commands require Jetson's system
Python and are not validated in the cloud.

```bash
source /opt/ros/humble/setup.bash
sudo apt-get install python3-colcon-common-extensions python3-serial ros-humble-sensor-msgs-py ros-humble-tf2-ros ros-humble-navigation2 ros-humble-nav2-bringup
mkdir -p ~/radar_ws/src
cd ~/radar_ws/src
git clone https://github.com/jelsayyid/radar-navigation.git
cd ~/radar_ws
colcon build --symlink-install --packages-select radar_navigation
source install/setup.bash
cd src/radar-navigation
python3 -m unittest discover -s tests -v
```

## Stationary acquisition
Identify both ports belonging to the **same serial number** under
/dev/serial/by-id. Commands end in if00; data end in if03. Set CLI_PORT and
DATA_PORT to these full paths. Never infer dual-radar pairing from enumeration
order. Ensure dialout group access. Set a unique sensor name per device.

```bash
radar record --cli "$CLI_PORT" --data "$DATA_PORT" --cfg awr1843_profile.cfg --sensor radar_front --output front.jsonl
radar replay front.jsonl --speed 1
```

Ctrl+C stops capture and sends sensorStop. JSONL stores TI axes, radial Doppler,
host receipt timestamp, frame number, hardware cycle counter, subframe, quality
TLV (if emitted), and unknown TLV payloads. It does not store rejected packets;
capture raw serial bytes separately when diagnosing transport corruption.
Do not run the viewer and acquisition on the same serial device concurrently.

## ROS stationary validation

```bash
ros2 launch radar_navigation radar.launch.py cli:="$CLI_PORT" data:="$DATA_PORT" cfg:="$PWD/awr1843_profile.cfg"
```

Raw points on /radar_front/points_raw have float32 x,y,z,doppler in ROS sensor
axes (forward, left, up); /radar_front/metadata preserves the complete frame.
Use ros2 topic hz and ros2 topic echo to verify receipt. Replay uses
`replay:=/absolute/path/front.jsonl` with no serial arguments and shifts recorded
timestamps to the current ROS clock. Simulated-time replay is not supported.

Supply measured base_link → radar_front static TF and robot odom → base_link TF.
The static transform describes the already converted ROS sensor axes, not TI axes.
Use the Humble static_transform_publisher named arguments with measured x/y/z/yaw;
roll and pitch must be zero for the current planar adapter. Missing TF causes
mapping to skip measurements rather than substitute an identity transform.

For the second radar, repeat launch with namespace:=radar_rear,
frame_id:=radar_rear and its own explicit serial paths and measured transform.
Then run `ros2 run radar_navigation radar_fusion`.
Fusion consumes timestamped fixed-frame filtered clouds, rejects old input, and
publishes /radar/fused. It does not estimate extrinsics or solve clock alignment.
Use radar_navigation.fusion.calibrate on surveyed matching reflector coordinates;
check the RMS residual and validate against held-out reflector positions.

## Navigation integration gate

config/nav2_radar.yaml is an **overlay** for the robot's working Humble Nav2
parameters, not a complete runnable configuration. Merge its local-costmap
sections; remove rear from observation_sources for single-radar operation.
Nav2 transforms raw clouds using TF. Clearing is disabled because sparse radar
returns do not justify LiDAR-style free-space assumptions. Consequently obstacles
can persist in the local costmap; operator validation of clearing/aging remains
required. Expected update rate is a costmap freshness check, not a motor watchdog.

Global localization/planning still requires a validated map and localization
source. Wheel odometry alone drifts. The provided odometry-frame occupancy grid
is experimental, fixed at 20×20 m, and cannot be represented as a validated
global navigation map. Do not relabel a LiDAR-localized run as radar-only navigation.

Before supervised movement:
1. Verify all TF, topic rates, timestamp age, measured footprint and radar coverage
   with the robot stationary. Confirm a new stationary obstacle is marked.
2. Verify disconnects/stale perception cause a stop through the robot's independent
   command watchdog and emergency stop. This repository does not yet implement
   a validated command safety gate.
3. Have an operator validate low-speed goals in a bounded area, with an emergency
   stop available. No movement command is issued by installation or radar launch.
4. Record odometry, TF, radar clouds/metadata, cmd_vel, Nav2 action outcomes,
   reference sensing, and operator interventions.

ROS compilation/import, actual TF timing, Nav2 behavior, firmware side-info
format, dual-radar RF interference and physical control remain deployment checks.
