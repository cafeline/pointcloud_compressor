import os
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('vq_occupancy_compressor')
    params_file = os.path.join(pkg_dir, 'config', 'vq_occupancy_compressor_params.yaml')

    vq_occupancy_compressor_node = Node(
        package='vq_occupancy_compressor',
        executable='vq_occupancy_compressor_node',
        name='vq_occupancy_compressor_node',
        output='screen',
        parameters=[params_file],
    )

    log_info = LogInfo(
        msg=['Launching VqOccupancy Compressor Node with parameters from ', params_file]
    )

    return LaunchDescription([
        log_info,
        vq_occupancy_compressor_node,
    ])
