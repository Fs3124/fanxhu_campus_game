#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "$root"
colcon build --base-paths "$root/src" \
    --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
