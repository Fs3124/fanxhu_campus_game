#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
[[ -f "$root/install/local_setup.bash" ]] || { printf '请先运行 ./build.sh\n' >&2; exit 1; }
set +u
source /opt/ros/jazzy/setup.bash
source "$root/install/local_setup.bash"
set -u
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
exec ros2 run ros_tcp_endpoint default_server_endpoint --ros-args \
    -p "ROS_IP:=${TDT_ROS_IP:-127.0.0.1}" -p "ROS_TCP_PORT:=${TDT_ROS_PORT:-10000}" "$@"
