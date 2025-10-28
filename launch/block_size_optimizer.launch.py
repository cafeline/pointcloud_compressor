import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare('pointcloud_compressor')
    
    # Declare launch arguments
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            pkg_share,
            'config',
            'block_size_optimizer_params.yaml'
        ]),
        description='Path to the parameter configuration file'
    )
    
    input_file_arg = DeclareLaunchArgument(
        'input_file',
        default_value='',
        description='Path to input PCD file (overrides parameter file if provided)'
    )
    
    min_block_size_arg = DeclareLaunchArgument(
        'min_block_size',
        default_value='0',
        description='Minimum block size (0 to use parameter file value)'
    )
    
    max_block_size_arg = DeclareLaunchArgument(
        'max_block_size',
        default_value='0',
        description='Maximum block size (0 to use parameter file value)'
    )
    
    step_size_arg = DeclareLaunchArgument(
        'step_size',
        default_value='0',
        description='Step size for testing (0 to use parameter file value)'
    )
    
    voxel_size_arg = DeclareLaunchArgument(
        'voxel_size',
        default_value='0.0',
        description='Voxel size (0.0 to use parameter file value)'
    )
    
    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Enable verbose output'
    )
    
    run_once_arg = DeclareLaunchArgument(
        'run_once',
        default_value='true',
        description='Run optimization once and exit'
    )
    
    # Create node with conditional parameter overrides
    def create_parameters():
        params = [LaunchConfiguration('params_file')]
        
        # Add command-line overrides if provided
        overrides = {}
        
        input_file = LaunchConfiguration('input_file').perform(None)
        if input_file:
            overrides['input_file'] = input_file
            
        min_bs = int(LaunchConfiguration('min_block_size').perform(None) or '0')
        if min_bs > 0:
            overrides['min_block_size'] = min_bs
            
        max_bs = int(LaunchConfiguration('max_block_size').perform(None) or '0')
        if max_bs > 0:
            overrides['max_block_size'] = max_bs
            
        step = int(LaunchConfiguration('step_size').perform(None) or '0')
        if step > 0:
            overrides['step_size'] = step
            
        voxel = float(LaunchConfiguration('voxel_size').perform(None) or '0.0')
        if voxel > 0.0:
            overrides['voxel_size'] = voxel
            
        if LaunchConfiguration('verbose').perform(None) == 'true':
            overrides['verbose'] = True
            
        if LaunchConfiguration('run_once').perform(None) == 'false':
            overrides['run_once'] = False
            
        if overrides:
            params.append(overrides)
            
        return params
    
    # Block size optimizer node
    optimizer_node = Node(
        package='pointcloud_compressor',
        executable='block_size_optimizer_node',
        name='block_size_optimizer',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[
            # Add any topic remappings if needed
        ]
    )
    
    return LaunchDescription([
        # Arguments
        params_file_arg,
        input_file_arg,
        min_block_size_arg,
        max_block_size_arg,
        step_size_arg,
        voxel_size_arg,
        verbose_arg,
        run_once_arg,
        # Node
        optimizer_node
    ])
