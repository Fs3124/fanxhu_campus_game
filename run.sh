#!/usr/bin/env bash
set -eo pipefail

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ros_setup="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"

if [[ $# != 1 || ( "$1" != bridge && "$1" != start ) ]]; then
    echo "用法：bash run.sh bridge  # 官方桥接"
    echo "      bash run.sh start   # 按YAML策略执行，开启射击许可"
    exit 2
fi

if [[ ! -f "$ros_setup" || ! -f "$root/campusgame_ros/ros2_auto_framework/install/local_setup.bash" ||
      ! -f "$root/install/local_setup.bash" ]]; then
    echo "缺少ROS环境或编译结果，请先运行：bash build.sh" >&2
    exit 1
fi

export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
unset LD_LIBRARY_PATH PYTHONPATH ROS_PACKAGE_PATH
source "$ros_setup"
source "$root/campusgame_ros/ros2_auto_framework/install/local_setup.bash"

if [[ "$1" == bridge ]]; then
    # 使用与官方start.sh相同的程序入口；官方文件本身保持不改。
    exec ros2 run ros_tcp_endpoint default_server_endpoint --ros-args \
        -p ROS_IP:=127.0.0.1 -p ROS_TCP_PORT:=10000
fi

source "$root/install/local_setup.bash"

exec ros2 run aim_node aim_node --ros-args \
    --params-file "$root/src/aim_bringup/config/strategy.yaml" \
    -p enable_fire:=true
