#!/usr/bin/env python3
"""
Launch file untuk menjalankan MAVROS dengan PX4 Flight Controller
dan node control untuk gate centering experiment
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare launch arguments
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='udp://:14540@localhost:14557',
        description='URL untuk koneksi ke FCU PX4 (default: SITL)'
    )
    
    gcs_url_arg = DeclareLaunchArgument(
        'gcs_url',
        default_value='udp://@',
        description='URL untuk Ground Control Station'
    )
    
    tgt_system_arg = DeclareLaunchArgument(
        'tgt_system',
        default_value='1',
        description='Target system ID'
    )
    
    tgt_component_arg = DeclareLaunchArgument(
        'tgt_component',
        default_value='1',
        description='Target component ID'
    )
    
    # Path ke config file
    config_file = PathJoinSubstitution([
        FindPackageShare('control'),
        'config',
        'config.yaml'
    ])
    
    # MAVROS node
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        parameters=[{
            'fcu_url': LaunchConfiguration('fcu_url'),
            'gcs_url': LaunchConfiguration('gcs_url'),
            'target_system_id': LaunchConfiguration('tgt_system'),
            'target_component_id': LaunchConfiguration('tgt_component'),
            'fcu_protocol': 'v2.0',
        }],
        remappings=[
            ('/mavros/setpoint_position/local', '/mavros/setpoint_position/local'),
        ]
    )
    
    # Control node (gate centering)
    control_node = Node(
        package='control',
        executable='control_lidar',
        name='drone_controller',
        output='screen',
        parameters=[config_file],
    )
    
    return LaunchDescription([
        fcu_url_arg,
        gcs_url_arg,
        tgt_system_arg,
        tgt_component_arg,
        mavros_node,
        control_node,
    ])
