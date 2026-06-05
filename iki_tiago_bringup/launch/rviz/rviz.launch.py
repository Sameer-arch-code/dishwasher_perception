import os
from dataclasses import dataclass

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_pal.arg_utils import LaunchArgumentsBase, read_launch_argument
from launch_pal.robot_arguments import CommonArgs
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from tiago_description.launch_arguments import TiagoArgs
from tiago_description.tiago_launch_utils import get_tiago_hw_suffix


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):
    base_type: DeclareLaunchArgument = TiagoArgs.base_type
    arm_type: DeclareLaunchArgument = TiagoArgs.arm_type
    end_effector: DeclareLaunchArgument = TiagoArgs.end_effector
    ft_sensor: DeclareLaunchArgument = TiagoArgs.ft_sensor

    wrist_model: DeclareLaunchArgument = DeclareLaunchArgument(
        "wrist_model", default_value="wrist-2010", description="Wrist model to use"
    )

    camera_model: DeclareLaunchArgument = TiagoArgs.camera_model
    laser_model: DeclareLaunchArgument = TiagoArgs.laser_model

    use_sim_time: DeclareLaunchArgument = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation time"
    )

    use_sensor_manager: DeclareLaunchArgument = CommonArgs.use_sensor_manager

    rviz_config: DeclareLaunchArgument = DeclareLaunchArgument(
        "rviz_config",
        default_value="",
        description="Absolute path to RViz config file. If empty, default package config is used",
    )


def generate_launch_description():
    ld = LaunchDescription()
    launch_arguments = LaunchArguments()
    launch_arguments.add_to_launch_description(ld)
    declare_actions(ld, launch_arguments)
    return ld


def declare_actions(launch_description: LaunchDescription, launch_args: LaunchArguments):
    launch_description.add_action(OpaqueFunction(function=start_rviz))


def start_rviz(context, *args, **kwargs):

    base_type = read_launch_argument("base_type", context)
    arm_type = read_launch_argument("arm_type", context)
    end_effector = read_launch_argument("end_effector", context)
    ft_sensor = read_launch_argument("ft_sensor", context)

    use_sim_time_str = read_launch_argument("use_sim_time", context)
    use_sim_time = str(use_sim_time_str).lower() in ("true", "1", "yes")

    rviz_config_arg = read_launch_argument("rviz_config", context)

    hw_suffix = get_tiago_hw_suffix(
        arm=arm_type,
        end_effector=end_effector,
    )

    srdf_file_path = os.path.join(
        get_package_share_directory("tiago_moveit_config"),
        "config",
        "srdf",
        "tiago.srdf.xacro",
    )

    srdf_input_args = {
        "arm_type": arm_type,
        "end_effector": end_effector,
        "ft_sensor": ft_sensor,
        "base_type": base_type,
    }

    moveit_simple_controllers_path = f"config/controllers/controllers{hw_suffix}.yaml"

    robot_description_kinematics = "config/kinematics_kdl.yaml"
    joint_limits = "config/joint_limits.yaml"
    pilz_cartesian_limits = "config/pilz_cartesian_limits.yaml"

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": True,
    }

    moveit_config = (
        MoveItConfigsBuilder("tiago")
        .robot_description_semantic(file_path=srdf_file_path, mappings=srdf_input_args)
        .robot_description_kinematics(file_path=robot_description_kinematics)
        .trajectory_execution(moveit_simple_controllers_path)
        .planning_scene_monitor(planning_scene_monitor_parameters)
        .joint_limits(file_path=joint_limits)
        .planning_pipelines(
            pipelines=["ompl", "chomp"], default_planning_pipeline="ompl"
        )
        .pilz_cartesian_limits(file_path=pilz_cartesian_limits)
        .to_moveit_configs()
    )

    moveit_config.robot_description.update({"use_sim_time": use_sim_time})

    default_rviz_dir = os.path.join(
        get_package_share_directory("iki_tiago_bringup"),
        "config",
        "rviz",
    )

    default_rviz_file = os.path.join(default_rviz_dir, "tiago.rviz")

    if rviz_config_arg and os.path.isabs(rviz_config_arg):
        rviz_file = rviz_config_arg
    else:
        rviz_file = default_rviz_file

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=[
            "-d",
            rviz_file,
            "--stylesheet",
            os.path.join(default_rviz_dir, "dark.qss"),
        ],
        emulate_tty=True,
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {
                "rviz_config_panel_params": os.path.join(
                    default_rviz_dir, "config_panel.yaml"
                ),
                "use_sim_time": use_sim_time,
            },
        ],
    )

    return [rviz_node]
