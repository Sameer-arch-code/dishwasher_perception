#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    default_params_file = os.path.join(
        get_package_share_directory("iki_tiago_bringup"),
        "config",
        "slam",
        "slam_toolbox.yaml",
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Full path to the ROS2 parameters file",
    )

    params = LaunchConfiguration("params_file")
    slam_toolbox = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        emulate_tty=True,
        parameters=[params],
    )

    return LaunchDescription([params_file_arg, slam_toolbox])
