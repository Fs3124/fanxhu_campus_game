#!/usr/bin/python3
# 装配根：唯一参数文件 → 唯一正式节点，不编排策略或其他控制器。
# ros2 launch调用generate_launch_description；直接运行本文件也可启动。
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node


def generate_launch_description():
    configuration = os.path.join(get_package_share_directory("aim_bringup"), "config", "strategy.yaml")
    return LaunchDescription([Node(package="aim_node", executable="aim_node",
                                   parameters=[configuration], output="screen")])


if __name__ == "__main__":
    service = LaunchService()
    service.include_launch_description(generate_launch_description())
    service.run()
