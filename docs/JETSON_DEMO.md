> Historical integration/design record. For the consolidated working system,
> use [STATIONARY_SYSTEM.md](STATIONARY_SYSTEM.md). Do not source the old driver
> workspace or follow the earlier parallel serial/TF launch procedure.

# Operator checklist: real Jetson demonstration

This procedure uses real radar measurements. No startup command arms motion.
The default command topic is a dry output. The smallest first demonstration is
single front radar, filtered points, occupancy, and supervised reactive motion.
Use the physical emergency stop; the software gate is not a certified safety device.

## 1. Update and build on Ubuntu 22.04 / Humble

With the existing TurtleBot3 bring-up installed:
```bash
source /opt/ros/humble/setup.bash
sudo apt-get install python3-serial python3-colcon-common-extensions   ros-humble-sensor-msgs-py ros-humble-tf2-ros
cd ~/radar_ws/src/radar-navigation
git pull --ff-only origin main
cd ~/radar_ws
colcon build --symlink-install --packages-select radar_navigation
source install/setup.bash
cd src/radar-navigation
python3 -m unittest discover -s tests -v
ROS_DOMAIN_ID=89 python3 -m unittest discover -s tests_ros -v
```

The ROS test uses pseudo-terminals and dry command output; run it with robot
bring-up stopped, in a separate ROS domain, e.g.
`ROS_DOMAIN_ID=89 python3 -m unittest discover -s tests_ros -v`.
It does not access the physical radar or motor topic.

## 2. Stationary real sensor check

Run the robot's existing bring-up to provide wheel odometry and
odom → base_link (possibly via base_footprint). Do not publish a second identity
odom transform on the robot. Measure the level-mounted radar pose in base_link:
x forward, y left, z up, yaw radians. Tilted mounts are currently rejected.

Set CLI_PORT and DATA_PORT to the full by-id paths for the same XDS110 serial
number (if00 and if03). Set X, Y, Z and YAW to those measured mount values.

```bash
ros2 launch radar_navigation demo.launch.py   cli:="$CLI_PORT" data:="$DATA_PORT" x:="$X" y:="$Y" z:="$Z" yaw:="$YAW"
```

In a second sourced terminal:
```bash
ros2 run radar_navigation radar_probe --seconds 30 --output stationary.json
ros2 topic echo /radar/control_status
```

Expect ~10 Hz raw and processed clouds, fields x/y/z/doppler, occupied map cells
when a reflector is present, and disarmed status. A raw rate with zero processed
rate means TF/age validation failed; inspect launch warnings. The probe records
host-receipt-to-probe latency, not hardware acquisition latency.

Place a surveyed reflector at 1 m and 2 m; confirm front-facing coordinates and
Doppler sign. Remove it; its map cells must become unknown within approximately
1.5 s + 0.2 s publishing period. Never infer free space from this expiry.

## 3. Dry-output gate test, then supervised movement

Keep all other command sources stopped. With dry output still selected, publish
the operator enable lease in a dedicated terminal:
```bash
ros2 topic pub -r 10 /radar/enable std_msgs/msg/Bool '{data: true}'
```

Inspect /radar/cmd_vel_safe. Forward output is capped at 0.10 m/s and turn output
at 0.35 rad/s. Empty detections stop motion by default. A frontal detection within
0.65 m blocks forward motion; the reactive policy starts turning below 0.9 m.
A detection within 0.4 m blocks rotation. These starting distances must be
validated against the actual footprint, stopping distance and radar blind zones.

Test each event separately: stop the enable publisher, disconnect the radar USB,
and stop the perception process. Commands must become zero within 0.3 s plus one
50 ms timer period under normal scheduling. Missing/stale TF also blocks output.
Restarting requires both fresh processed input and a current operator lease.

Only after these stationary tests, an operator may restart the demo launch with
`output:=/cmd_vel`. Verify `ros2 topic info /cmd_vel --verbose` shows only the
gate as publisher and that the base uses geometry_msgs/Twist on that topic.
Then repeat the enable command while supervising low-speed motion in a bounded
area. Stopping that command releases the lease. Do not background it or leave it
running unattended. Never simultaneously run the demo gate and navigation gate.

Test the base/OpenCR command timeout with the wheels lifted: killing the gate
must itself stop the motors. The gate cannot publish zero after process/host
failure. Confirm the base watchdog before any floor trial.

## 4. Capture quantitative evidence

```bash
ros2 bag record -o radar_trial /radar_front/points_raw /radar_front/points   /radar_front/metadata /radar_front/occupancy /radar_front/status   /radar/control_status /radar/enable /radar/cmd_vel_request   /radar/cmd_vel_safe /cmd_vel /odom /tf /tf_static
ros2 run radar_navigation radar_probe --seconds 60 --output trial_metrics.json
```

Keep the software commit, mount measurements, reflector locations, observed
stopping distance, interruptions, collisions and emergency interventions with
each bag. All interventions count as failed autonomous trials. The probe does
not estimate navigation success or perception ground truth from unlabelled data.

## 5. Goal-directed Nav2 with explicit localization

First use a surveyed/prior map and the existing LiDAR for AMCL localization.
Wheel odometry supplies odom → base_link; AMCL supplies map → odom.
The navigation gate additionally requires a fresh map → odom transform;
AMCL future dating up to 1.5 s is accepted, past age beyond 0.5 s is rejected.
This is **radar obstacle perception with LiDAR-assisted localization**, not
radar-only localization. Reactive tests need only odometry and do not require AMCL.
Radar-only localization remains an open research task.

```bash
sudo apt-get install ros-humble-navigation2 ros-humble-nav2-bringup python3-yaml
python3 scripts/prepare_nav2.py "$ROBOT_NAV2_PARAMS" /tmp/nav2_radar.yaml
ros2 launch radar_navigation navigation.launch.py   map:="$PRIOR_MAP_YAML" params_file:=/tmp/nav2_radar.yaml
```

Run perception using radar.launch.py and the measured static transform; stop the
demo launch's control process before launching the navigation gate. Keep the
robot's ordinary bring-up and /scan running. In RViz, initialize AMCL pose,
verify localization, and inspect the local costmap before sending a goal.
The supplied launch routes Nav2 output to /radar/cmd_vel_request and leaves the
gate output dry. The same operator lease and output:=/cmd_vel gate apply.

Nav2 local costmap reads **/radar_front/occupancy**, built from fresh configured
filter output, using Humble StaticLayer plus inflation. Full map replacement
propagates expiry; observation_persistence on ObstacleLayer alone would not
erase marked cells. Local unknown cells are treated as unmarked for planning,
not certified free. The separate safety gate remains mandatory.
The global planner uses only the prior static map and inflation; live LiDAR
obstacles are excluded by the merge script, though /scan still feeds AMCL.

Current deployment is single-radar and a bounded 20×20 m odometry map. Dual-radar
occupancy/control arbitration and rolling map recentering are not yet integrated.
Do not drive outside the mapped area. Full Nav2 routing, map alignment and
physical stopping still require the operator acceptance procedure above.
