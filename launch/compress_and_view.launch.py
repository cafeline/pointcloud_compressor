#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('pointcloud_compressor')
    
    # Declare launch arguments
    input_file_arg = DeclareLaunchArgument(
        'input_file',
        default_value='/tmp/test.pcd',
        description='Input point cloud file (PCD or PLY)'
    )
    
    voxel_size_arg = DeclareLaunchArgument(
        'voxel_size',
        default_value='0.01',
        description='Voxel size for point cloud compression'
    )
    
    block_size_arg = DeclareLaunchArgument(
        'block_size',
        default_value='8',
        description='Block size for pattern dictionary'
    )
    
    min_points_threshold_arg = DeclareLaunchArgument(
        'min_points_threshold',
        default_value='1',
        description='Minimum points per voxel to be considered occupied'
    )
    
    publish_occupied_voxel_markers_arg = DeclareLaunchArgument(
        'publish_occupied_voxel_markers',
        default_value='false',
        description='Publish occupied voxel markers'
    )
    
    # PointCloud Compressor Node
    compressor_node = Node(
        package='pointcloud_compressor',
        executable='pointcloud_compressor_node',
        name='pointcloud_compressor_node',
        output='screen',
        parameters=[{
            'input_file': LaunchConfiguration('input_file'),
            'voxel_size': LaunchConfiguration('voxel_size'),
            'block_size': LaunchConfiguration('block_size'),
            'min_points_threshold': LaunchConfiguration('min_points_threshold'),
            'publish_occupied_voxel_markers': LaunchConfiguration('publish_occupied_voxel_markers'),
            'publish_once': True,
            'use_8bit_indices': True,
        }]
    )
    
    # Compressed Viewer Node
    viewer_node = Node(
        package='pointcloud_compressor',
        executable='compressed_viewer_node',
        name='compressed_viewer_node',
        output='screen'
    )
    
    return LaunchDescription([
        input_file_arg,
        voxel_size_arg,
        block_size_arg,
        min_points_threshold_arg,
        publish_occupied_voxel_markers_arg,
        compressor_node,
        viewer_node,
    ])