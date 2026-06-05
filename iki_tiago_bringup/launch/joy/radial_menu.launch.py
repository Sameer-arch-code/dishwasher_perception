#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg = get_package_share_directory("iki_tiago_bringup")
    joystick_config_dir = os.path.join(bringup_pkg, "config", "joy")

    radial_menu_node = Node(
        package="radial_menu",
        executable="radial_menu_node",
        name="radial_menu_node",
        parameters=[os.path.join(joystick_config_dir, "radial_menu.yaml")],
    )

    return LaunchDescription([radial_menu_node])
