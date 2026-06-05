import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory("iki_tiago_bringup"),
        "config",
        "environment_manager.yaml",
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Full path to the ROS2 parameters file",
    )

    params = LaunchConfiguration("params_file")

    actions = [params_file_arg]

    tf_component = ComposableNode(
        package="waypoint_server",
        plugin="TfContext",
        name="tf_context",
        namespace="",
    )

    container = ComposableNodeContainer(
        name="environment_manager_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        output="screen",
        emulate_tty=True,
        composable_node_descriptions=[tf_component],
    )
    actions.append(container)

    bounds_node = Node(
        name="bounds_manager",
        namespace="",
        package="bounds_manager",
        executable="bounds_manager_node",
        output="screen",
        parameters=[{"path": ""}],
    )
    actions.append(bounds_node)

    env_manager_node = Node(
        name="environment_manager",
        namespace="",
        package="environment_manager",
        executable="environment_manager_node",
        output="screen",
        parameters=[params],
    )
    actions.append(env_manager_node)

    return LaunchDescription(actions)
