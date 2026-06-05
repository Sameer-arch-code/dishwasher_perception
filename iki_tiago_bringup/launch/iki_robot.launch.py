#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    iki_tiago_bringup = get_package_share_directory("iki_tiago_bringup")
    iki_robot_config = os.path.join(iki_tiago_bringup, "config", "iki_robot")

    if os.getenv("SIM", "false") != "true":
        print("Starting in REAL mode!")
        param_file = os.path.join(iki_robot_config, "real.yaml")
    else:
        print("Starting in SIM mode!")
        param_file = os.path.join(iki_robot_config, "sim.yaml")

    iki_robot = Node(
        package="iki_robot",
        executable="iki_robot_node",
        name="iki_robot_node",
        parameters=[param_file],
    )
    web_commander = Node(
        package="iki_robot_web_commander",
        executable="iki_robot_web_ui",
        name="iki_robot_web_commander",
    )

    return LaunchDescription([iki_robot, web_commander])
