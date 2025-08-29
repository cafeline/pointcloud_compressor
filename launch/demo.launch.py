#!/usr/bin/env python3
"""
Simplified demo launch file for pointcloud compressor
Compress once and visualize
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('pointcloud_compressor')
    viewer_pkg_dir = get_package_share_directory('compressed_viewer')
    
    # Configuration file path - use demo_params.yaml from compressed_viewer package
    config_file = os.path.join(viewer_pkg_dir, 'config', 'demo_params.yaml')
    
    # Declare launch arguments
    input_file_arg = DeclareLaunchArgument(
        'input_file',
        default_value='/tmp/sample.pcd',
        description='Input file path (PCD or PLY)'
    )
    
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )
    
    # Single shot compression node
    compressor_node = Node(
        package='pointcloud_compressor',
        executable='pointcloud_compressor_node',
        name='pointcloud_compressor_node',
        output='screen',
        parameters=[
            config_file,
            {
                'publish_once': True,
                'publish_patterns': True,
                'publish_occupied_voxel_markers': True
            }
        ]
    )
    
    # Viewer node
    viewer_node = Node(
        package='compressed_viewer',
        executable='compressed_viewer_node',
        name='compressed_viewer_node',
        output='screen',
        parameters=[config_file]
    )
    
    # RViz2 configuration
    rviz_config_file = os.path.join(
        viewer_pkg_dir,
        'rviz',
        'compressed_viewer.rviz'
    )
    
    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
        output='screen'
    )
    
    return LaunchDescription([
        input_file_arg,
        launch_rviz_arg,
        compressor_node,
        viewer_node,
        rviz_node
    ])