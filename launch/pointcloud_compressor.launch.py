#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('pointcloud_compressor')
    
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
    
    voxel_size_arg = DeclareLaunchArgument(
        'voxel_size',
        default_value='0.01',
        description='Voxel size for compression (overrides preset)'
    )
    
    block_size_arg = DeclareLaunchArgument(
        'block_size',
        default_value='8',
        description='Block size for compression (overrides preset)'
    )
    
    use_8bit_indices_arg = DeclareLaunchArgument(
        'use_8bit_indices',
        default_value='true',
        description='Use 8-bit indices if possible (overrides preset)'
    )
    
    publish_once_arg = DeclareLaunchArgument(
        'publish_once',
        default_value='true',
        description='Publish compressed data only once (true) or periodically (false)'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='compressed_pointcloud',
        description='ROS topic name for compressed point cloud output'
    )
    
    # Configuration file path
    config_file = os.path.join(pkg_dir, 'config', 'pointcloud_compressor_params.yaml')
    
    # Create the node
    pointcloud_compressor_node = Node(
        package='pointcloud_compressor',
        executable='pointcloud_compressor_node',
        name='pointcloud_compressor_node',
        output='screen',
        parameters=[
            config_file,
            {
                'input_pcd_file': LaunchConfiguration('input_pcd_file'),
                'voxel_size': LaunchConfiguration('voxel_size'),
                'block_size': LaunchConfiguration('block_size'),
                'use_8bit_indices': LaunchConfiguration('use_8bit_indices'),
                'publish_once': LaunchConfiguration('publish_once'),
                'output_topic': LaunchConfiguration('output_topic'),
            }
        ],
        remappings=[
            ('compressed_pointcloud', LaunchConfiguration('output_topic')),
        ]
    )
    
    # Log information
    log_info = LogInfo(
        msg=[
            'Starting PointCloud Compressor Node with the following settings:\n',
            '  Input PCD file: ', LaunchConfiguration('input_pcd_file'), '\n',
            '  Preset: ', LaunchConfiguration('preset'), '\n',
            '  Voxel size: ', LaunchConfiguration('voxel_size'), '\n',
            '  Block size: ', LaunchConfiguration('block_size'), '\n',
            '  Use 8-bit indices: ', LaunchConfiguration('use_8bit_indices'), '\n',
            '  Publish once: ', LaunchConfiguration('publish_once'), '\n',
            '  Output topic: ', LaunchConfiguration('output_topic')
        ]
    )
    
    return LaunchDescription([
        input_pcd_file_arg,
        preset_arg,
        voxel_size_arg,
        block_size_arg,
        use_8bit_indices_arg,
        publish_once_arg,
        output_topic_arg,
        log_info,
        pointcloud_compressor_node
    ])