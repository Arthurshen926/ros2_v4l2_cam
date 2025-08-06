#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """
    Simple launch file for multi-camera components
    """
    
    # Declare launch arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'num_cameras',
            default_value='2',
            description='Number of cameras to launch'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'container_name',
            default_value='multi_camera_container',
            description='Name of the component container'
        )
    )
    
    # Create composable nodes
    composable_nodes = [
        ComposableNode(
            package='v4l2_camera',
            plugin='v4l2_camera::MultiCameraComponent',
            name='multi_camera_manager',
            parameters=[{
                'num_cameras': LaunchConfiguration('num_cameras'),
                'camera_config_file': '',
                'default_width': 1920,
                'default_height': 1500,
                'default_device_prefix': '/dev/video',
                'default_topic_prefix': '/camera',
            }],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
    ]
    
    # Create container with intra-process communication enabled
    container = ComposableNodeContainer(
        name=LaunchConfiguration('container_name'),
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=composable_nodes,
        output='screen',
    )
    
    return LaunchDescription(declared_arguments + [container])
