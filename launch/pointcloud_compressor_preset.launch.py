#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_with_preset(context, *args, **kwargs):
    """Launch function that loads preset configuration"""
    
    # Get package directory
    pkg_dir = get_package_share_directory('pointcloud_compressor')
    
    # Get preset name
    preset = LaunchConfiguration('preset').perform(context)
    
    # Map preset to configuration file
    preset_configs = {
        'high_quality': 'compression_presets.yaml',
        'balanced': 'compression_presets.yaml',
        'high_compression': 'compression_presets.yaml',
        'fast_processing': 'compression_presets.yaml',
        'memory_constrained': 'compression_presets.yaml'
    }
    
    if preset not in preset_configs:
        raise ValueError(f"Unknown preset: {preset}. Available presets: {list(preset_configs.keys())}")
    
    # Configuration file path
    config_file = os.path.join(pkg_dir, 'config', preset_configs[preset])
    
    # Create the node with preset configuration
    pointcloud_compressor_node = Node(
        package='pointcloud_compressor',
        executable='pointcloud_compressor_node',
        name='pointcloud_compressor_node',
        namespace=preset,  # Use preset name as namespace
        output='screen',
        parameters=[
            {preset: config_file},  # Load specific preset section
            {
                'input_pcd_file': LaunchConfiguration('input_pcd_file'),
            }
        ],
        remappings=[
            ('compressed_pointcloud', LaunchConfiguration('output_topic')),
        ]
    )
    
    return [pointcloud_compressor_node]


def generate_launch_description():
    # Declare launch arguments
    input_pcd_file_arg = DeclareLaunchArgument(
        'input_pcd_file',
        default_value='',
        description='Path to input PCD file to compress'
    )
    
    preset_arg = DeclareLaunchArgument(
        'preset',
        default_value='balanced',
        description='Compression preset: high_quality, balanced, high_compression, fast_processing, memory_constrained'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='compressed_pointcloud',
        description='ROS topic name for compressed point cloud output'
    )
    
    # Log information about the preset
    log_preset_info = LogInfo(
        msg=[
            'Starting PointCloud Compressor Node with preset: ', LaunchConfiguration('preset'), '\n',
            'Input PCD file: ', LaunchConfiguration('input_pcd_file'), '\n',
            'Output topic: ', LaunchConfiguration('output_topic')
        ]
    )
    
    return LaunchDescription([
        input_pcd_file_arg,
        preset_arg,
        output_topic_arg,
        log_preset_info,
        OpaqueFunction(function=launch_with_preset)
    ])