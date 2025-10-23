#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('pointcloud_compressor')
    config_file = os.path.join(pkg_dir, 'config', 'pointcloud_compressor_params.yaml')

    pointcloud_compressor_node = Node(
        package='pointcloud_compressor',
        executable='pointcloud_compressor_node',
        name='pointcloud_compressor_node',
        output='screen',
        parameters=[config_file],
    )

    log_info = LogInfo(
        msg=['Launching PointCloud Compressor Node with parameters from ', config_file]
    )

    return LaunchDescription([
        log_info,
        pointcloud_compressor_node,
    ])
