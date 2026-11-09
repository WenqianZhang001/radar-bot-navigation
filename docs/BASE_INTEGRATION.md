# TurtleBot3 base and radar operation

Canonical checkout/workspace on Jetson: `~/radar_integration_ws/src/radar_navigation`.
ROS 2 Humble, TurtleBot3 Waffle Pi, OpenCR firmware register 5 / model 0x5000.
No firmware was flashed. Install package dependencies with rosdep and build as in README.
The base ownership check also requires `fuser` from Ubuntu package `psmisc`
(already installed on the lab Jetson).

## Start, inspect, stop

```bash
~/radar_integration_ws/src/radar_navigation/scripts/start_drive.sh
# optional viewer:=true; defaults: actuate:=false, odom_use_imu:=false
# Ctrl+C in this terminal stops the complete system.
```

This starts real OpenCR wheel odometry, robot_state_publisher, radar acquisition,
measurement-time odom-frame perception and a dry reactive controller. OpenCR
motor power is required for valid motor telemetry. **Dry mode sends only zero to
motors even if an enable lease exists.** It is appropriate for software checks;
starting the base can enable motor torque. Keep the physical power switch accessible.
Do not run stock robot.launch, teleop, old radar sessions or another base launch
alongside it. ROS_DOMAIN_ID defaults to 30 in the helper.

```bash
source /opt/ros/humble/setup.bash
source ~/radar_integration_ws/install/setup.bash
export ROS_DOMAIN_ID=30
ros2 topic hz /odom
ros2 topic hz /radar/points
ros2 topic echo /base/watchdog_status
ros2 topic echo /radar/control_status
ros2 topic echo /radar_front/status
ros2 run tf2_ros tf2_echo odom radar_link
```

Use RViz Fixed Frame `odom` and add `/radar_front/occupancy` (transient-local)
and `/radar_front/points`. Existing stationary RViz configuration uses
base_footprint; change its fixed frame for the drive launch.
For the original stationary sensing-only mode use `scripts/start_stationary.sh`.
That mode starts no base/controller and uses base_footprint mapping.

## Interfaces and ownership

- OpenCR stable by-id path is default in `launch/base.launch.py`; XDS110 has its
  own independent radar ports. The watchdog's native turtlebot3_ros child alone
  opens OpenCR. Existing owner or competing command publishers cause failure.
- Native base publishes private `/base/native_odom`, wheel/IMU streams, motor
  telemetry and the sole dynamic native_odom -> native_base_footprint TF, ~20 Hz.
- The measured [coordinate adapter](BASE_COORDINATES.md) exposes canonical
  `/odom`, `/joint_states`, and connects the TF chain to the radar-facing
  base_footprint/base_link. It preserves measurement stamps and covariance.
- Waffle Pi URDF uses native_ frame prefix; explicit canonical base_link z=0.010 m.
  Existing radar_tf alone publishes base_footprint -> radar_link
  (0.150,0.060,0.190 m, zero nominal rotation). No static odom is allowed.
- Drive launch sets perception `fixed_frame=odom`. Source receipt stamps are
  transformed at that measurement timestamp with bounded TF retry; the controller
  transforms accumulated fixed-frame points to the current base pose. No ego
  Doppler compensation or hardware acquisition timestamps are claimed.
- Polling stalls >0.2 s flush the radar UART/parser; multiple frames in a regular
  read retain only the newest complete frame. This avoids stale host-stamped replay.
- Reactive controller publishes `/radar/cmd_vel_safe`, never conventional `/cmd_vel`.
  Native base receives only `/base/driver_cmd_vel` from the independent watchdog;
  that private output is in native axes (negative linear means radar-forward).
  Use canonical safe/request interfaces, never publish private native commands.
- Both control and watchdog require a continuously refreshed `/radar/enable` Bool
  true lease (0.3 s). Watchdog additionally checks command age <=0.25 s, odometry
  and healthy torque/10–15 V telemetry age <=0.3 s. Zero on invalid/stale/disarmed.
  Dry is immutable for a running guard; relaunch with `actuate:=true` is required.
  Default limits 0.05 m/s / 0.2 rad/s; reverse/unsupported Twist components rejected.
- Parent watchdog death kills the native serial child via Linux PDEATHSIG, so its
  OpenCR heartbeat also dies. Corrected physical test confirmed board stop in 0.575 s.
  This is not protection against every partial host hang: a frozen watchdog with
  a still-running native child is not covered by parent-death signaling. Hardware
  power cutoff/operator supervision remains necessary; no safety certification.

## Physical testing

Do not publish an enable lease or opt into actuation without an attended, approved
experiment. Floor motion requires explicit operator approval and clear test area.
The bounded test harness is **for secured wheels lifted only**:

```bash
ros2 launch radar_navigation base.launch.py actuate:=true
# In another sourced terminal, without radar_control or any other command source:
python3 scripts/lifted_motor_test.py --wheels-lifted-confirmed --output /path/to/NEW_run
python3 scripts/analyze_lifted_test.py /path/to/NEW_run
```

The harness performs brief 0.04 m/s and +0.2 rad/s pulses and destructive process
lifetime tests, then leaves the base stopped. It must not be used on the floor.
For observed channel diagnosis, `visual_wheel_pulses.py --sequence isolated`
performs two separate native-channel pulses. It accepts `--start-at-unix` within
the next minute so an operator can be told the real start time after process
startup. Always pass `--wheels-lifted-confirmed --output NEW_directory`; do not
use this diagnostic on the floor.

Watchdog defaults never self-arm. Ctrl+C/releasing the enable lease stops output;
physical power switch is the emergency action.

Wheel-only odometry is the tested default. `odom_use_imu:=true` enables stock
IMU fusion but is not physically validated here. Lifted wheels rotate while the
chassis does not move: integrated odom and corresponding maps are not world
position truth. Stop all launches and restart with a fresh map/odom before a floor
experiment. Do not infer floor stopping distances from supported-wheel timeouts.

If USB works but motors are absent, ensure normal 12 V motor supply is on, then
press the actual OpenCR RESET (not motor-test SW1/SW2). Connectivity is initialized
at boot; merely adding 12 V after USB boot did not repair the latched missing state.
With all base processes stopped, `scripts/read_opencr_status.py` provides a
read-only register diagnostic and refuses an already-owned serial port.

## Evidence

[Corrected physical test report](BASE_ALIGNMENT_TEST_2026-11-08.md), compact
measurements/plot in `results/base_alignment_20261108/`; initial native tests
remain in `results/base_development_20261108/`. Raw bag, JSONL, failed/corrected harness runs,
logs and inspected upstream references stay at:
`/home/tejas/radar_integration_artifacts/base_development_20261108T1940/`.
Record `/odom /joint_states /sensor_state /tf /tf_static /radar/points
/radar_front/points /radar_front/occupancy /radar_front/status /radar/cmd_vel_safe
/base/driver_cmd_vel /base/watchdog_status /radar/enable` for future experiments.
