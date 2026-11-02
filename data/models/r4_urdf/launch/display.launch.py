"""RViz2 快速查看：ros2 launch r4_urdf display.launch.py"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("r4_urdf")
    urdf = os.path.join(share, "urdf", "r4.urdf")
    with open(urdf, encoding="utf-8") as f:
        robot_description = f.read()

    cfg = os.path.join(share, "config", "r4.rviz")
    rviz_args = ["-d", cfg] if os.path.exists(cfg) else []

    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui"),
        Node(package="rviz2", executable="rviz2", arguments=rviz_args),
    ])
