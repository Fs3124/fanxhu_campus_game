#!/usr/bin/env bash
set -eo pipefail

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
official="$root/campusgame_ros/ros2_auto_framework"
ros_setup="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
python="${ROS_PYTHON:-/usr/bin/python3}"
opencv_prefix="/usr/local/opencv-4.7.0"
opencv_dir="$opencv_prefix/lib/cmake/opencv4"

if [[ $# != 0 ]]; then
    echo "用法：bash build.sh"
    echo "自动编译官方桥接/消息，再编译本工程；不会启动模拟器或下载依赖。"
    exit 2
fi
if [[ ! -f "$ros_setup" || ! -x "$python" || ! -d "$official/src/tdt_interface" ]]; then
    echo "缺少ROS环境、系统Python或官方源码。请先准备依赖。" >&2
    echo "ROS_SETUP可指定ROS环境脚本，ROS_PYTHON可指定对应的Python。" >&2
    exit 1
fi

if [[ ! -f "$opencv_dir/OpenCVConfig.cmake" ]]; then
    echo "缺少比赛机OpenCV 4.7配置：$opencv_dir/OpenCVConfig.cmake" >&2
    echo "本导出包固定使用比赛机的4.7目录，不自动改用4.6或5.1。" >&2
    exit 1
fi

# 不继承其他工程的ROS覆盖层，避免链接到另一份同名包。
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
unset LD_LIBRARY_PATH PYTHONPATH ROS_PACKAGE_PATH
source "$ros_setup"
command -v colcon >/dev/null || { echo "请先安装colcon。" >&2; exit 1; }
export PYTHONDONTWRITEBYTECODE=1
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
cd "$root"

# 官方源码和消息不改；只重新生成当前机器、当前目录的编译产物。
# 清理CMake缓存可消除复制目录后留下的旧源码路径，不删除整个工作空间。
# ROS消息生成器读中文编译路径有兼容问题，官方包的编译缓存单独放英文临时目录。
# 安装结果仍放回工程，运行不依赖临时目录；无需移动或改名你的工程。
official_build="$(mktemp -d /tmp/tdt-plus-official-XXXXXX)"
echo "[1/2] 编译官方桥接和消息"
colcon --log-base "$root/log/official" build \
    --base-paths "$official/src" \
    --build-base "$official_build" --install-base "$official/install" \
    --executor sequential --cmake-clean-cache \
    --cmake-args "-DPython3_EXECUTABLE=$python"
source "$official/install/local_setup.bash"

echo "[2/2] 编译参赛工程"
echo "固定选择OpenCV 4.7.0：$opencv_dir"
colcon --log-base "$root/log" build \
    --base-paths "$root/src" \
    --build-base "$root/build" --install-base "$root/install" \
    --packages-select aim_tools aim_node aim_bringup \
    --executor sequential --cmake-clean-cache \
    --cmake-args "-DPython3_EXECUTABLE=$python" \
        "-DOpenCV_DIR=$opencv_dir" \
        "-DCMAKE_BUILD_RPATH=$opencv_prefix/lib" \
        "-DCMAKE_INSTALL_RPATH=$opencv_prefix/lib"

echo "编译完成。两个终端分别执行：bash run.sh bridge 和 bash run.sh start"
