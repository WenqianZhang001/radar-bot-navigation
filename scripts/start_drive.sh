#!/usr/bin/env bash
# Foreground dry base + radar + reactive output; actuation requires explicit opt-in.
set -e
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ -f "$project_dir/../../install/setup.bash" ]]; then
    source "$project_dir/../../install/setup.bash"
elif [[ -f "$project_dir/../../local_setup.bash" ]]; then
    source "$project_dir/../../local_setup.bash"
else
    echo "Build radar_navigation with colcon before starting." >&2
    exit 1
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
exec ros2 launch radar_navigation drive.launch.py "$@"
