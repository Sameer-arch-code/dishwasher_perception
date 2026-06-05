from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_dir = get_package_share_directory('iki_tiago_bringup')
    launch_file_dir = os.path.join(package_dir, 'launch', 'sensors')

    # Declare enable arguments for each camera
    enable_astra2_arg = DeclareLaunchArgument(
        'enable_astra2', default_value='true',
        description='Enable Astra 2 camera'
    )
    enable_femto_arg = DeclareLaunchArgument(
        'enable_femto', default_value='true',
        description='Enable Femto Bolt camera'
    )
    enable_gemini_arg = DeclareLaunchArgument(
        'enable_gemini', default_value='true',
        description='Enable Gemini 335/336 camera'
    )

    enable_femto_mega_arg = DeclareLaunchArgument(
        'enable_femto_mega', default_value='false',
        description='Enable Femto Mega camera'
    )

    # Function to conditionally include launch files
    def launch_setup(context, *args, **kwargs):
        ld_actions = []

        if context.launch_configurations.get('enable_femto', 'true').lower() == 'true':
            femto_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_file_dir, 'femto_bolt.launch.py')),
            )
            ld_actions.append(GroupAction([femto_launch]))

        if context.launch_configurations.get('enable_femto_mega', 'false').lower() == 'true':
            femto_mega_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_file_dir, 'femto_mega.launch.py')),
            )
            ld_actions.append(GroupAction([femto_mega_launch]))

        if context.launch_configurations.get('enable_astra2', 'true').lower() == 'true':
            astra2_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_file_dir, 'astra2.launch.py'))
            )
            ld_actions.append(GroupAction([astra2_launch]))

        if context.launch_configurations.get('enable_gemini', 'true').lower() == 'true':
            gemini_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_file_dir, 'gemini_330_series.launch.py')),
            )
            ld_actions.append(GroupAction([gemini_launch]))

        return ld_actions

    # Return the LaunchDescription
    return LaunchDescription([
        enable_astra2_arg,
        enable_femto_arg,
        enable_gemini_arg,
        enable_femto_mega_arg,
        OpaqueFunction(function=launch_setup),
    ])
