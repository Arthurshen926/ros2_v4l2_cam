#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """
    Launch multiple camera components with intra-process communication
    """
    
    # Declare launch arguments
    declared_arguments = []
    
    # Number of cameras
    declared_arguments.append(
        DeclareLaunchArgument(
            'num_cameras',
            default_value='2',
            description='Number of cameras to launch'
        )
    )
    
    # Camera configuration file (optional)
    declared_arguments.append(
        DeclareLaunchArgument(
            'camera_config_file',
            default_value='',
            description='Path to camera configuration YAML file'
        )
    )
    
    # Default camera parameters
    declared_arguments.append(
        DeclareLaunchArgument(
            'default_width',
            default_value='1920',
            description='Default camera width'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'default_height',
            default_value='1500',
            description='Default camera height'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'device_prefix',
            default_value='/dev/video',
            description='Device path prefix'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'topic_prefix',
            default_value='/camera',
            description='Topic prefix for camera topics'
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
                'camera_config_file': LaunchConfiguration('camera_config_file'),
                'default_width': LaunchConfiguration('default_width'),
                'default_height': LaunchConfiguration('default_height'),
                'default_device_prefix': LaunchConfiguration('device_prefix'),
                'default_topic_prefix': LaunchConfiguration('topic_prefix'),
            }],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
    ]
    
    # Create container with intra-process communication enabled
    container = ComposableNodeContainer(
        name=LaunchConfiguration('container_name'),
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',  # Multi-threaded container
        composable_node_descriptions=composable_nodes,
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription(declared_arguments + [container])
