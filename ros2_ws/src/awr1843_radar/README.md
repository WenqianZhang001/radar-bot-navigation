# awr1843_radar

ROS 2 Humble driver for the robot's TI AWR1843BOOST radar. It configures the radar over USB, publishes its point cloud, and publishes where the radar sits on the robot.

## Build

```bash
cd ~/radar-bot-navigation/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash        # needed in every new terminal
```

## Run

```bash
ros2 launch awr1843_radar radar.launch.py
```

Ctrl+C stops the radar. Only one program can use the radar at a time, so stop `radar_viewer.py` first.

## What it publishes

| Output | Details |
| --- | --- |
| `/radar/points` (`sensor_msgs/PointCloud2`) | About 10 per second, in frame `radar_link`. Fields: `x`, `y`, `z` in meters (x forward, y left, z up); `velocity` in m/s along the line of sight, positive = moving away; `snr` and `noise` in dB. |
| Transform `base_footprint` → `radar_link` | From `config/radar_mount.yaml`. |

Check them with:

```bash
ros2 topic hz /radar/points
ros2 run tf2_ros tf2_echo base_footprint radar_link
```

## Settings

- `config/radar_mount.yaml`: where the radar sits on the robot. Measure it on the robot, then set `measured: true`. With `--symlink-install`, edits take effect at the next launch without rebuilding.
- `config/awr1843_profile.cfg`: radar settings. It starts as a copy of the viewer's; tune it here for navigation.
- Driver parameters: `cli_port` and `data_port` (empty = find the radar automatically), `cfg_file`, `frame_id` (default `radar_link`), and `time_offset` (seconds subtracted from timestamps to account for the radar's delay).
