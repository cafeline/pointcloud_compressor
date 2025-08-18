#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch.conditions import IfCondition, LaunchConfigurationEquals, UnlessCondition
from launch.substitutions import LaunchConfiguration, FindExecutable, PathJoinSubstitution, PythonExpression, AndSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('pointcloud_compressor')
    
    # Declare launch arguments for demo
    demo_mode_arg = DeclareLaunchArgument(
        'demo_mode',
        default_value='single_shot',
        description='Demo mode: single_shot, continuous, comparison'
    )
    
    create_sample_arg = DeclareLaunchArgument(
        'create_sample',
        default_value='true',
        description='Create a sample PCD file for demonstration'
    )
    
    sample_size_arg = DeclareLaunchArgument(
        'sample_size',
        default_value='1000',
        description='Number of points in the sample PCD file'
    )
    
    input_file_arg = DeclareLaunchArgument(
        'input_file',
        default_value='',  # Empty means use YAML value
        description='Input file path (PCD or PLY) - leave empty to use YAML config'
    )
    
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )
    
    # Configuration file path - use demo_params.yaml from compressed_viewer package
    config_file = os.path.join(get_package_share_directory('compressed_viewer'), 'config', 'demo_params.yaml')
    
    # Sample PCD file path
    sample_pcd_path = '/tmp/pointcloud_compressor_sample.pcd'
    
    # Create sample PCD file using the CLI tool
    create_sample_pcd = ExecuteProcess(
        cmd=[
            FindExecutable(name='python3'),
            '-c',
            [
                'import numpy as np; import open3d as o3d; ',
                'n_points = ', LaunchConfiguration('sample_size'), '; ',
                'points = np.random.rand(int(n_points), 3) * 2.0 - 1.0; ',
                'pcd = o3d.geometry.PointCloud(); ',
                'pcd.points = o3d.utility.Vector3dVector(points); ',
                f'o3d.io.write_point_cloud("{sample_pcd_path}", pcd); ',
                'print(f"Created sample PCD file with {{}} points: ', f'{sample_pcd_path}'.format('{n_points}'), '")'
            ]
        ],
        output='screen',
        condition=LaunchConfigurationEquals('create_sample', 'true')
    )
    
    # Single shot compression demo
    single_shot_node = Node(
        package='pointcloud_compressor',
        executable='pointcloud_compressor_node',
        name='pointcloud_compressor_single_shot',
        output='screen',
        parameters=[config_file],  # Use YAML config only
        condition=LaunchConfigurationEquals('demo_mode', 'single_shot')
    )
    
    # Python Viewer node for single shot mode
    single_shot_viewer = Node(
        package='compressed_viewer',
        executable='compressed_viewer_node',
        name='compressed_viewer_single_shot',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory('compressed_viewer'), 'config', 'demo_params.yaml')
        ],
        condition=LaunchConfigurationEquals('demo_mode', 'single_shot')
    )
    
    # Continuous compression demo
    continuous_node = Node(
        package='pointcloud_compressor',
        executable='pointcloud_compressor_node',
        name='pointcloud_compressor_continuous',
        output='screen',
        parameters=[
            config_file,
            {
                'publish_once': False,
                'publish_interval_ms': 3000,  # Every 3 seconds
            }
        ],
        condition=LaunchConfigurationEquals('demo_mode', 'continuous')
    )
    
    # Python Viewer node for continuous mode
    continuous_viewer = Node(
        package='compressed_viewer',
        executable='compressed_viewer_node',
        name='compressed_viewer_continuous',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory('compressed_viewer'), 'config', 'demo_params.yaml')
        ],
        condition=LaunchConfigurationEquals('demo_mode', 'continuous')
    )
    
    # Comparison demo - multiple nodes with different settings
    comparison_high_quality = Node(
        package='pointcloud_compressor',
        executable='pointcloud_compressor_node',
        name='pointcloud_compressor_high_quality',
        namespace='high_quality',
        output='screen',
        parameters=[
            config_file,
            {
                'publish_once': True,
                'voxel_size': 0.005,
                'block_size': 4,
                'use_8bit_indices': False,
            }
        ],
        remappings=[
            ('pattern_dictionary', 'high_quality/pattern_dictionary'),
        ],
        condition=LaunchConfigurationEquals('demo_mode', 'comparison')
    )
    
    comparison_high_compression = Node(
        package='pointcloud_compressor',
        executable='pointcloud_compressor_node',
        name='pointcloud_compressor_high_compression',
        namespace='high_compression',
        output='screen',
        parameters=[
            config_file,
            {
                'publish_once': True,
                'voxel_size': 0.02,
                'block_size': 16,
                'use_8bit_indices': True,
            }
        ],
        remappings=[
            ('pattern_dictionary', 'high_compression/pattern_dictionary'),
        ],
        condition=LaunchConfigurationEquals('demo_mode', 'comparison')
    )
    
    # Python Viewer nodes for comparison mode
    comparison_viewer_high_quality = Node(
        package='compressed_viewer',
        executable='compressed_viewer_node',
        name='compressed_viewer_high_quality',
        namespace='high_quality',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory('compressed_viewer'), 'config', 'demo_params.yaml')
        ],
        remappings=[
            ('pattern_markers', 'high_quality/pattern_markers'),
        ],
        condition=LaunchConfigurationEquals('demo_mode', 'comparison')
    )
    
    comparison_viewer_high_compression = Node(
        package='compressed_viewer',
        executable='compressed_viewer_node',
        name='compressed_viewer_high_compression',
        namespace='high_compression',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory('compressed_viewer'), 'config', 'demo_params.yaml')
        ],
        remappings=[
            ('pattern_markers', 'high_compression/pattern_markers'),
        ],
        condition=LaunchConfigurationEquals('demo_mode', 'comparison')
    )
    
    # Log demo information
    demo_info = LogInfo(
        msg=[
            '\n========================================\n',
            'PointCloud Compressor Demo\n',
            '========================================\n',
            'Demo mode: ', LaunchConfiguration('demo_mode'), '\n',
            'Sample PCD file: ', sample_pcd_path, '\n',
            'Sample size: ', LaunchConfiguration('sample_size'), ' points\n',
            '\nAvailable demo modes:\n',
            '  single_shot  - Compress once and publish result\n',
            '  continuous   - Compress and publish every 3 seconds\n',
            '  comparison   - Run high quality vs high compression side by side\n',
            '\nTo monitor output:\n',
            '  ros2 topic echo /pattern_dictionary\n',
            '  ros2 topic echo /pattern_markers\n',
            '  ros2 topic echo /high_quality/pattern_dictionary\n',
            '  ros2 topic echo /high_quality/pattern_markers\n',
            '  ros2 topic echo /high_compression/pattern_dictionary\n',
            '  ros2 topic echo /high_compression/pattern_markers\n',
            '========================================\n'
        ]
    )
    
    # RViz2 configuration
    rviz_config_file = os.path.join(
        get_package_share_directory('compressed_viewer'),
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
        demo_mode_arg,
        create_sample_arg,
        sample_size_arg,
        input_file_arg,
        launch_rviz_arg,
        demo_info,
        create_sample_pcd,
        single_shot_node,
        single_shot_viewer,
        continuous_node,
        continuous_viewer,
        comparison_high_quality,
        comparison_viewer_high_quality,
        comparison_high_compression,
        comparison_viewer_high_compression,
        rviz_node,
    ])