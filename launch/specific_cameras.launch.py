#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """
    Launch file for specific camera device groups
    """
    
    # Declare launch arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'camera_group',
            default_value='fisheye',
            description='Camera group to launch: fisheye, surround, front, or all',
            choices=['fisheye', 'surround', 'front', 'all']
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'container_name',
            default_value='multi_camera_container',
            description='Name of the component container'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'image_width',
            default_value='1920',
            description='Image width'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'image_height',
            default_value='1500',
            description='Image height'
        )
    )
    
    # Define camera device groups
    camera_groups = {
        'fisheye': ['/dev/video0', '/dev/video1', '/dev/video2', '/dev/video3'],
        'surround': ['/dev/video8', '/dev/video9', '/dev/video10', '/dev/video11', '/dev/video12', '/dev/video13'],
        'front': ['/dev/video5']
    }
    
    # Create composable nodes for different camera groups
    composable_nodes = []
    
    # Fisheye cameras
    for i, device in enumerate(camera_groups['fisheye']):
        composable_nodes.append(
            ComposableNode(
                package='v4l2_camera',
                plugin='v4l2_camera::CameraLauncherComponent',
                name=f'fisheye_camera_{i}',
                parameters=[{
                    'arg_camera_dev': device,
                    'arg_camera_id': '',  # 设备路径已经完整，不需要ID
                    'arg_publish_topic': f'/fisheye/camera{i}',
                    'arg_resolution_width': LaunchConfiguration('image_width'),
                    'arg_resolution_height': LaunchConfiguration('image_height'),
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        )
    
    # Surround cameras
    for i, device in enumerate(camera_groups['surround']):
        composable_nodes.append(
            ComposableNode(
                package='v4l2_camera',
                plugin='v4l2_camera::CameraLauncherComponent',
                name=f'surround_camera_{i}',
                parameters=[{
                    'arg_camera_dev': device,
                    'arg_camera_id': '',
                    'arg_publish_topic': f'/surround/camera{i}',
                    'arg_resolution_width': LaunchConfiguration('image_width'),
                    'arg_resolution_height': LaunchConfiguration('image_height'),
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        )
    
    # Front cameras
    for i, device in enumerate(camera_groups['front']):
        composable_nodes.append(
            ComposableNode(
                package='v4l2_camera',
                plugin='v4l2_camera::CameraLauncherComponent',
                name=f'front_camera_{i}',
                parameters=[{
                    'arg_camera_dev': device,
                    'arg_camera_id': '',
                    'arg_publish_topic': f'/front/camera{i}',
                    'arg_resolution_width': LaunchConfiguration('image_width'),
                    'arg_resolution_height': LaunchConfiguration('image_height'),
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        )
    
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
