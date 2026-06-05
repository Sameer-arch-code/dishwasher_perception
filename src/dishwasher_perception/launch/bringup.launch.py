from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    debug      = LaunchConfiguration('debug')
    publish_pc = LaunchConfiguration('cpc_param')
    markers    = LaunchConfiguration('markers')

    return LaunchDescription([
        DeclareLaunchArgument('debug',     default_value='False'),
        DeclareLaunchArgument('cpc_param', default_value='False'),
        DeclareLaunchArgument('markers',   default_value='False'),

        Node(
            package='dishwasher_perception',
            executable='dishwasher_perception',
            name='dishwasher_node',
            output='screen',
            parameters=[{
                'debug':     debug,
                'cpc_param': publish_pc,
                'markers':   markers,
            }],
        ),
    ])