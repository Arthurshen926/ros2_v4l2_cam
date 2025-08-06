#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """
    Launch file for specific camera device groups with proper device handling
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
            default_value='640',  # 降低默认分辨率以减少资源占用
            description='Image width'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'image_height',
            default_value='480',  # 降低默认分辨率以减少资源占用
            description='Image height'
        )
    )
    
    # Define camera device groups
    camera_groups = {
        'fisheye': {
            'devices': ['/dev/video0', '/dev/video1', '/dev/video2', '/dev/video3'],
            'topic_prefix': '/fisheye'
        },
        'surround': {
            'devices': ['/dev/video8', '/dev/video9', '/dev/video10', '/dev/video11', '/dev/video12', '/dev/video13'],
            'topic_prefix': '/surround'
        },
        'front': {
            'devices': ['/dev/video5'],
            'topic_prefix': '/front'
        }
    }
    
    # Create composable nodes based on selected group
    composable_nodes = []
    
    # Get selected group
    selected_group = LaunchConfiguration('camera_group').perform(None)
    if selected_group == 'all':
        groups_to_launch = camera_groups.keys()
    else:
        groups_to_launch = [selected_group] if selected_group in camera_groups else ['fisheye']
    
    node_counter = 0
    for group_name in groups_to_launch:
        group_config = camera_groups[group_name]
        devices = group_config['devices']
        topic_prefix = group_config['topic_prefix']
        
        for i, device in enumerate(devices):
            composable_nodes.append(
                ComposableNode(
                    package='v4l2_camera',
                    plugin='v4l2_camera::CameraLauncherComponent',
                    name=f'{group_name}_camera_{i}',
                    parameters=[{
                        'arg_camera_dev': device,
                        'arg_camera_id': '',  # 设备路径已经完整，不需要ID
                        'arg_publish_topic': f'{topic_prefix}/camera{i}',
                        'arg_resolution_width': LaunchConfiguration('image_width'),
                        'arg_resolution_height': LaunchConfiguration('image_height'),
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}],
                )
            )
            node_counter += 1
    
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
