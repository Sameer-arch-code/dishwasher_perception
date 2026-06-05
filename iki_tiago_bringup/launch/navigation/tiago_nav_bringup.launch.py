# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Changed by Maik Knof for iki :)

import os
from dataclasses import dataclass
import yaml
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_pal.arg_utils import LaunchArgumentsBase, read_launch_argument
from launch_pal.include_utils import include_scoped_launch_py_description
from launch_pal.robot_arguments import CommonArgs
from launch_ros.actions import Node
from tiago_description.launch_arguments import TiagoArgs

def deep_update(dst, src):
    for k, v in src.items():
        if isinstance(v, dict) and isinstance(dst.get(k), dict):
            deep_update(dst[k], v)
        else:
            dst[k] = v
    return dst

def merge_yaml_files(base_path, overlay_path):
    with open(base_path, "r") as f:
        base = yaml.safe_load(f) or {}
    with open(overlay_path, "r") as f:
        overlay = yaml.safe_load(f) or {}

    merged = deep_update(base, overlay)

    tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False)
    yaml.safe_dump(merged, tmp, default_flow_style=False, sort_keys=False)
    tmp.close()
    return tmp.name

@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):

    is_public_sim: DeclareLaunchArgument = CommonArgs.is_public_sim
    base_type: DeclareLaunchArgument = TiagoArgs.base_type
    slam: DeclareLaunchArgument = CommonArgs.slam
    map_path: DeclareLaunchArgument = DeclareLaunchArgument(
        "map_path", default_value="", description="Path to map.yaml"
    )
    launch_nav2_rviz: DeclareLaunchArgument = DeclareLaunchArgument(
        "launch_nav2_rviz", default_value="True", description="Launch nav2 RViz?"
    )


def generate_launch_description():

    # Create the launch description and populate
    ld = LaunchDescription()
    launch_arguments = LaunchArguments()

    launch_arguments.add_to_launch_description(ld)

    declare_actions(ld, launch_arguments)

    return ld


def public_nav_function(context, *args, **kwargs):
    base_type = read_launch_argument("base_type", context)
    map_path = read_launch_argument("map_path", context)
    actions = []
    iki_tiago_bringup = get_package_share_directory("iki_tiago_bringup")
    pmb2_2dnav = get_package_share_directory("pmb2_2dnav")
    default_param_file = os.path.join(pmb2_2dnav, "config", "nav_public_sim.yaml")

    iki_tiago_bringup = get_package_share_directory("iki_tiago_bringup")
    iki_tiago_nav_params = os.path.join(iki_tiago_bringup, "config", "navigation", "sim.yaml")
    merged_param_file = merge_yaml_files(default_param_file, iki_tiago_nav_params)

    rviz_config_file = os.path.join(iki_tiago_bringup, "config", "rviz", "tiago.rviz")

    nav_bringup_launch = include_scoped_launch_py_description(
        pkg_name="nav2_bringup",
        paths=["launch", "navigation_launch.py"],
        #launch_arguments={"params_file": iki_tiago_nav_params, "use_sim_time": "True"},
        launch_arguments={"params_file": merged_param_file, "use_sim_time": "True"},
    )

    rviz_bringup_launch = include_scoped_launch_py_description(
        pkg_name="nav2_bringup",
        paths=["launch", "rviz_launch.py"],
        launch_arguments={"rviz_config": rviz_config_file},
    )

    actions.append(nav_bringup_launch)
    if read_launch_argument("launch_nav2_rviz", context) == "True":
        actions.append(rviz_bringup_launch)
    return actions


def private_nav_function(context, *args, **kwargs):
    base_type = read_launch_argument("base_type", context)
    actions = []
    tiago_2dnav = get_package_share_directory("tiago_2dnav")

    remappings_file = os.path.join(
        tiago_2dnav, "params", "tiago_" + base_type + "_remappings_sim.yaml"
    )

    nav_bringup_launch = include_scoped_launch_py_description(
        pkg_name="pal_nav2_bringup",
        paths=["launch", "nav_bringup.launch.py"],
        launch_arguments={
            "params_pkg": "tiago_2dnav",
            "params_file": "tiago_" + base_type + "_nav.yaml",
            "robot_name": "tiago",
            "remappings_file": remappings_file,
        },
    )

    laser_bringup_launch = include_scoped_launch_py_description(
        pkg_name="pal_nav2_bringup",
        paths=["launch", "nav_bringup.launch.py"],
        launch_arguments={
            "params_pkg": "tiago_laser_sensors",
            "params_file": base_type + "_laser_pipeline_sim.yaml",
            "robot_name": "tiago",
            "remappings_file": remappings_file,
        },
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(
                tiago_2dnav,
                "config",
                "rviz",
                "navigation.rviz",
            ),
        ],
        output="screen",
    )

    actions.append(nav_bringup_launch)
    actions.append(laser_bringup_launch)
    if read_launch_argument("launch_nav2_rviz", context) == "True":
        actions.append(rviz_node)

    return actions


def declare_actions(
    launch_description: LaunchDescription, launch_args: LaunchArguments
):

    launch_description.add_action(
        OpaqueFunction(
            function=public_nav_function,
            condition=IfCondition(LaunchConfiguration("is_public_sim")),
        )
    )

    launch_description.add_action(
        OpaqueFunction(
            function=private_nav_function,
            condition=UnlessCondition(LaunchConfiguration("is_public_sim")),
        )
    )
