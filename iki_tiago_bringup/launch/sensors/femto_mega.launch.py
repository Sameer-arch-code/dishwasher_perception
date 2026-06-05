
import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def load_yaml(file_path):
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)

def convert_value(value):
    if isinstance(value, str):
        try:
            return int(value)
        except ValueError:
            pass
        try:
            return float(value)
        except ValueError:
            pass
        if value.lower() == 'true':
            return True
        elif value.lower() == 'false':
            return False
    return value


def load_parameters(context):
    params_file_path = LaunchConfiguration('params_file').perform(context)
    default_params = load_yaml(params_file_path)
    skip_convert = {'params_file_path', 'usb_port', 'serial_number'}
    return {
        key: (value if key in skip_convert else convert_value(value))
        for key, value in default_params.items()
    }


def generate_launch_description():

    # Point to a default params YAML (you can also set default_value to an absolute path)
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(get_package_share_directory('iki_tiago_bringup'), "config", "sensors", "femto_mega.yaml"),
        description="Full path to the YAML file with node parameters."
    )

    def launch_setup(context, *args, **kwargs):

        params = load_parameters(context)
        params = params.get("/**", {}).get("ros__parameters", {})
        # Just pass the YAML file straight into the node
        compose_node = ComposableNode(
            package="orbbec_camera",
            plugin="orbbec_camera::OBCameraNodeDriver",
            name=params.get("camera_name", "femto_mega"),
            namespace=params.get("namespace", ""),
            parameters=[LaunchConfiguration('params_file')],
            extra_arguments=[{'use_intra_process_comms': False}],
            remappings=[
                ('/color/camera_info', '~/color/camera_info'),
                ('/color/image_raw', '~/color/image_raw'),
                ('/color/image_raw/compressed', '~/color/image_raw/compressed'),
                ('/color/image_raw/compressedDepth', '~/color/image_raw/compressedDepth'),
                ('/color/image_raw/theora', '~/color/image_raw/theora'),
                ('/depth/camera_info', '~/depth/camera_info'),
                ('/depth/image_raw', '~/depth/image_raw'),
                ('/depth/image_raw/compressed', '~/depth/image_raw/compressed'),
                ('/depth/image_raw/compressedDepth', '~/depth/image_raw/compressedDepth'),
                ('/depth/image_raw/theora', '~/depth/image_raw/theora'),
                ('/depth/points', '~/depth/points'),
                ('/depth_filter_status', '~/depth_filter_status'),
                ('/depth_registered/points', '~/depth_registered/points'),
                ('/depth_to_color', '~/depth_to_color'),
            ]
        )

        container = ComposableNodeContainer(
            name=PythonExpression(["'", params.get("camera_name", "femto_mega"), "'", " + '_container'"]),
            namespace=params.get("namespace", ""),
            package="rclcpp_components",
            executable="component_container_mt",
            composable_node_descriptions=[compose_node],
            output="screen",
        )
        return [GroupAction([container])]

    return LaunchDescription([
        params_file_arg,
        OpaqueFunction(function=launch_setup),
    ])
