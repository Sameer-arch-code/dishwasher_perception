#!/usr/bin/env python3
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_pkg_share = get_package_share_directory("iki_tiago_bringup")
    description_pkg_share = get_package_share_directory("tiago_description")

    xacro_file = os.path.join(description_pkg_share, "robots", "tiago.urdf.xacro")
    rviz_config = os.path.join(config_pkg_share, "config", "rviz", "urdf_viewer.rviz")

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {"robot_description": doc.toxml()}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace="tiago",
        parameters=[robot_description],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        namespace="tiago",
        output="screen",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz", default_value="True", description="Whether to launch RViz"
            ),
            robot_state_publisher,
            joint_state_publisher,
            rviz_node,
        ]
    )
