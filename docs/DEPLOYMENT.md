# Deployment reference

For the current integrated workflow, use [Jetson demonstration](JETSON_DEMO.md).
The checklist includes filtered-map Nav2 integration, command gating and localization.

# Jetson deployment and validation

## Install
Ubuntu 22.04 with an existing ROS 2 Humble and TurtleBot3 installation is required.
Core tests run on Python 3.12 and Humble integration runs on container Python 3.10.
The Jetson hardware remains unvalidated; use its ROS system Python.

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

## Navigation integration

Use docs/JETSON_DEMO.md for the current gate, aging-map Nav2 configuration and
AMCL localization launch. The old raw-point ObstacleLayer overlay has been
replaced. Do not use observation_persistence as obstacle expiry: it only controls
observation buffering and does not erase already marked cells.

The map uses configured filter results and refreshes occupancy only from current
evidence, not accumulated history. Cells expire to unknown after 1.5 seconds.
Nav2 locally treats unknown as unmarked; the command gate separately stops stale
perception. Validate both behaviors on the robot before supervised motion.
