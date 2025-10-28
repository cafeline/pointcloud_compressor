from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('vq_occupancy_compressor')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            pkg_share,
            'config',
            'block_size_optimizer_params.yaml'
        ]),
        description='Path to the parameter configuration file'
    )

    optimizer_node = Node(
        package='vq_occupancy_compressor',
        executable='block_size_optimizer_node',
        name='block_size_optimizer',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[
        ]
    )

    return LaunchDescription([
        params_file_arg,
        optimizer_node
    ])
