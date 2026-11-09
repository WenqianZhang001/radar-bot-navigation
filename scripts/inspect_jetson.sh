#!/usr/bin/env bash
# Read-only inventory: no serial opens, sensor configuration, or motor commands.
set -uo pipefail
probe() {
  printf '\n## %s\n' "$1"
  shift
  timeout 12s "$@" 2>&1 || printf 'Unavailable or timed out (exit %s)\n' "$?"
}
probe 'Platform' uname -a
for repo in /home/tejas/radar_integration_ws/src/radar_navigation /home/tejas/radar-bot-navigation; do
  if [ -d "$repo" ]; then
    probe "$repo HEAD" git -C "$repo" log -1 --format='%H %s'
    probe "$repo local changes (names only)" git -C "$repo" status --short
  fi
done
probe 'Serial device identities (no device reads)' ls -l /dev/serial/by-id
probe 'Process names (no arguments or environment)' ps -eo pid,ppid,comm
if [ -r /opt/ros/humble/setup.bash ]; then
  set +u
  source /opt/ros/humble/setup.bash
  set -u
fi
if command -v ros2 >/dev/null; then
  probe 'ROS nodes' ros2 node list
  probe 'ROS topics and types' ros2 topic list -t
  probe 'Radar publishers and QoS' ros2 topic info /radar/points --verbose
  probe 'Radar schema and timestamp' ros2 topic echo /radar/points --once --no-arr
  probe 'Static transforms' ros2 topic echo /tf_static --once --qos-durability transient_local
  probe 'Odometry-to-radar transform' ros2 run tf2_ros tf2_echo odom radar_link
else
  printf '\nROS CLI unavailable in this shell.\n'
fi
printf '\nInventory complete. No acquisition or motion processes started.\n'
