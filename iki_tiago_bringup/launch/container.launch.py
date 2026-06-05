from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            OpaqueFunction)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer


def _launch_setup(context, *args, **kwargs):
    ns = LaunchConfiguration("namespace").perform(context)
    isolated = LaunchConfiguration("isolated").perform(context).lower() in (
        "true",
        "1",
        "yes",
        "on",
    )
    threads = int(LaunchConfiguration("threads").perform(context))
    params = []
    args = []

    if not isolated:
        if threads > 1:
            executable = "component_container_mt"
            params.append({"thread_num": threads})
        else:
            executable = "component_container"
    else:
        executable = "component_container_isolated"
        if threads > 1:
            args.append("--use_multi_threaded_executor")
            params.append(
                {"thread_num": threads}
            )  # dont know if isolated container supports this

    container = ComposableNodeContainer(
        name="container",
        namespace=ns,
        package="rclcpp_components",
        executable=executable,
        parameters=params,
        arguments=args,
        output="screen",
    )
    actions = [container]

    auto_start = LaunchConfiguration("auto_start").perform(context).lower() in (
        "true",
        "1",
        "yes",
        "on",
    )

    sim = LaunchConfiguration("sim").perform(context).lower() in (
        "true",
        "1",
        "yes",
        "on",
    )
        

    if auto_start:
        # cut of dumper/
        sub_namespace = ns.split("/")[-1]
        selection = f"{sub_namespace}/*"
        cmd=[
            "ros2",
            "iki_tiago_bringup",
            "up",
            selection,
            "--timeout_sec",
            "10.0",
        ]
        if sim:
            cmd.append("--sim")
        actions.append(
            ExecuteProcess(
                cmd=cmd,
                output="screen",
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                description="Namespace for the container",
            ),
            DeclareLaunchArgument(
                "isolated",
                default_value="false",
                description="Use component_container_isolated when true",
            ),
            DeclareLaunchArgument(
                "threads",
                default_value="1",
                description="Number of threads; >1 enables MT (or MT flag for isolated)",
            ),
            DeclareLaunchArgument(
                "auto_start",
                default_value="false",
                description="If true, call ros2 iki_tiago_bringup up <namespace>",
            ),
            DeclareLaunchArgument(
                "sim",
                default_value="false",
                description="If sim is used",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
