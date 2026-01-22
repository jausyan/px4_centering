#!/usr/bin/env python3
"""
Launch file untuk menjalankan hanya control node
(MAVROS harus sudah running terpisah)
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Path ke config file
    config_file = PathJoinSubstitution([
        FindPackageShare('control'),
        'config',
        'config.yaml'
    ])
    
    # Control node (gate centering)
    control_node = Node(
        package='control',
        executable='control_lidar',
        name='drone_controller',
        output='screen',
        parameters=[config_file],
    )
    
    return LaunchDescription([
        control_node,
    ])
