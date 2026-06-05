#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # --- Launch arguments ---
    joy_dev = LaunchConfiguration("joy_dev")
    declare_joy_dev = DeclareLaunchArgument(
        "joy_dev",
        default_value="/dev/input/js0",
        description="Path to the joystick device",
    )

    config_joy = LaunchConfiguration("config_joy")
    default_joy = os.path.join(get_package_share_directory('iki_tiago_bringup'), 'config', 'joy', 'steamdeck_joy.yaml')
    declare_joy = DeclareLaunchArgument(
        'config_joy',
        default_value=default_joy,
        description='YAML file for teleop_twist_joy parameters'
    )

    config_joy_teleop = LaunchConfiguration("config_joy_teleop")
    default_joy_teleop = os.path.join(get_package_share_directory('iki_tiago_bringup'), 'config', 'joy', 'steamdeck_joy_teleop.yaml')
    declare_joy_teleop = DeclareLaunchArgument(
        'config_joy_teleop',
        default_value=default_joy_teleop,
        description='YAML file for joy_teleop parameters'
    )


    # --- Nodes ---
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="steamdeck_joy_node",
        parameters=[
            {
                "dev": joy_dev,
                "autorepeat_rate": 50.0,
            }
        ],
        remappings=[
            ('/joy', '/steamdeck_joy'),
        ],
    )

    joy_teleop_node = Node(
        package="joy_teleop",
        executable="joy_teleop",
        name="steamdeck_joy_teleop_node",
        parameters=[config_joy_teleop],
        remappings=[("joy", "/steamdeck_joy")],
    )


    return LaunchDescription([declare_joy_dev, declare_joy, joy_node, declare_joy_teleop, joy_teleop_node])