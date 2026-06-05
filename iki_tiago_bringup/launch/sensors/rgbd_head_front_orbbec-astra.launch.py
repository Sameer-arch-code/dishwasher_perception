# Copyright (c) 2025 PAL Robotics S.L. All rights reserved.
#
# Unauthorized copying of this file, via any medium is strictly prohibited,
# unless it was supplied under the terms of a license agreement or
# nondisclosure agreement with PAL Robotics SL. In this case it may not be
# copied or disclosed except in accordance with the terms of that agreement.

import rclpy
from rclpy.logging import get_logger
from rclpy.node import Node as RclpyNode
from launch import LaunchDescription
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode

from launch_pal import get_pal_configuration


def generate_launch_description():

    ld = LaunchDescription()
    camera_name = 'head_front_camera'
    astra_camera_driver_node = 'astra_camera_driver'
    point_cloud_xyz_node = 'point_cloud_xyz'
    point_cloud_xyzrgb_node = 'point_cloud_xyzrgb'
    point_cloud_to_laser_scan_node = 'point_cloud_to_laser_scan'
    astra_camera_driver_config = get_pal_configuration(
        pkg='astra_camera_cfg',
        node=astra_camera_driver_node,
        ld=ld,
        cmdline_args=['camera_name', 'serial_number']
    )
    point_cloud_xyz_config = get_pal_configuration(
        pkg='astra_camera_cfg',
        node=point_cloud_xyz_node,
        ld=ld,
        cmdline_args=False
    )
    point_cloud_xyzrgb_config = get_pal_configuration(
        pkg='astra_camera_cfg',
        node=point_cloud_xyzrgb_node,
        ld=ld,
        cmdline_args=False
    )

    # Due to a bug in rclcpp (https://github.com/ros2/rclcpp/issues/2404)
    # the ROS Container has to be created by the Navigation as it loads global
    # and local costmap parameters in the container itself as workaround
    # (https://github.com/ros-navigation/navigation2/issues/4011)
    rclpy.init()
    node = RclpyNode('node_checker')
    rclpy.spin_once(node, timeout_sec=1.0)
    while 'rgbd_container' not in node.get_node_names():
        get_logger('rgbd_head_orbbec-astra').warn(
            'rgbd_container not available. Waiting...'
        )
        rclpy.spin_once(node, timeout_sec=1.0)
    rclpy.shutdown()

    astra_component = LoadComposableNodes(
        target_container='rgbd_container',
        composable_node_descriptions=[
            # Camera Driver
            ComposableNode(
                package='astra_camera',
                plugin='astra_camera::OBCameraNodeFactory',
                name=astra_camera_driver_node,
                namespace=camera_name,
                parameters=astra_camera_driver_config['parameters'],
                remappings=astra_camera_driver_config['remappings'],
            ),
            ComposableNode(
                package='astra_camera',
                plugin='astra_camera::PointCloudXyzNode',
                name=point_cloud_xyz_node,
                namespace=camera_name,
                parameters=point_cloud_xyz_config['parameters'],
                remappings=point_cloud_xyz_config['remappings'],
            ),
            ComposableNode(
                package='astra_camera',
                plugin='astra_camera::PointCloudXyzrgbNode',
                name=point_cloud_xyzrgb_node,
                namespace=camera_name,
                parameters=point_cloud_xyzrgb_config['parameters'],
                remappings=point_cloud_xyzrgb_config['remappings'],
            ),
            ComposableNode(
                package="pointcloud_to_laserscan",
                plugin="pointcloud_to_laserscan::PointCloudToLaserScanNode",
                name=point_cloud_to_laser_scan_node,
                namespace=camera_name,
                remappings=[
                    ("cloud_in", "/head_front_camera/depth/points"),
                    ("scan", "/head_front_camera/depth/scan"),
                ],
                parameters=[{
                    "target_frame": "base_footprint",
                    "transform_tolerance": 0.3,
                    "min_height": 0.03,
                    "max_height": 2.0,
                    "angle_min": -1.5708,
                    "angle_max": 1.5708,
                    "angle_increment": 0.0087,
                    "scan_time": 0.3333,
                    "range_min": 0.15,
                    "range_max": 4.0,
                    "use_inf": True,
                    "inf_epsilon": 1.0,
                }],
            ),
        ],
    )
    ld.add_action(astra_component)
    return ld
