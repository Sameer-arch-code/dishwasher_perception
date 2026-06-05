import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    default_param_path = PathJoinSubstitution(
        [
            FindPackageShare("iki_tiago_bringup"),
            "config",
            "manipulation",
            "moveit_wrapper.yaml",
        ]
    )

    # Arguments
    param_file_arg = DeclareLaunchArgument(
        "param_file",
        default_value=default_param_path,
        description="Path to the YAML parameter file",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    srdf_file_path = os.path.join(
        get_package_share_directory("tiago_moveit_config"),
        "config", "srdf",
        "tiago.srdf.xacro",
    )

    srdf_input_args = {
        "arm_type": "tiago-arm",
        "end_effector": "pal-gripper",
        "ft_sensor": "schunk-ft",
        "base_type": "pmb2",
    }

    builder = (
        MoveItConfigsBuilder("tiago", package_name="tiago_moveit_config")
        .robot_description(
            file_path="haha_you_have_no_robot_model_bitch"
        )
        .robot_description_semantic(file_path=srdf_file_path, mappings=srdf_input_args)
        .robot_description_kinematics(file_path="config/kinematics_trac_ik.yaml")
        .trajectory_execution(file_path="config/controllers/controllers_pal-gripper.yaml")
        .planning_scene_monitor(
            publish_planning_scene=False,
            publish_geometry_updates=False,
            publish_state_updates=False,
            publish_transforms_updates=False,
            publish_robot_description=False,
            publish_robot_description_semantic=False
        )
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"], default_planning_pipeline="ompl"
        )
        .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
        .sensors_3d(file_path="config/sensors_3d.yaml")
        .to_moveit_configs()
    )
    builder.move_group_capabilities["capabilities"] = "move_group/ExecuteTaskSolutionCapability"

    container = ComposableNodeContainer(
        name="moveit_wrapper_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        composable_node_descriptions=[
            ComposableNode(
                package="moveit_wrapper",
                plugin="moveit_wrapper::MoveitWrapper",
                name="moveit_wrapper",
                namespace="",
                parameters=[
                    LaunchConfiguration("param_file"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
            ),
            ComposableNode(
                package="moveit_wrapper_task_constructor",
                plugin="moveit_wrapper_mtc::MoveitWrapperMTC",
                name="moveit_wrapper_task_constructor",
                namespace="",
                parameters=[
                    LaunchConfiguration("param_file"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                    {"ompl.fix_start_state":True},
                    {"robot_description_timeout": 60.0},
                    {"robot_description_semantic_timeout": 60.0},
                    builder.to_dict(),
                ],
            )
        ],
    )

    return LaunchDescription(
        [
            param_file_arg,
            use_sim_time_arg,
            container,
        ]
    )
