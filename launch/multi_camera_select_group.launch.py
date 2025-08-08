#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    """根据enabled_groups动态生成ComposableNode列表"""
    
    # 获取启用的组
    enabled_groups_str = LaunchConfiguration('enabled_groups').perform(context)
    enabled_groups = [group.strip() for group in enabled_groups_str.split(',')]
    
    # 动态创建ComposableNode列表
    composable_nodes = []
    
    # 鱼眼摄像头组
    if 'fisheye' in enabled_groups:
        composable_nodes.append(
            ComposableNode(
                package='v4l2_camera',
                plugin='v4l2_camera::MultiCameraComponent',
                name='fisheye_camera_manager',
                parameters=[{
                    'camera_config_file': LaunchConfiguration('camera_config_file'),
                    'device_group': 'fisheye',
                    'use_image_transport': True,
                    'convert_to_rgb': True,
                    'jpeg_quality': 40,
                }]
            )
        )
    
    # 前视摄像头组
    if 'front' in enabled_groups:
        composable_nodes.append(
            ComposableNode(
                package='v4l2_camera',
                plugin='v4l2_camera::MultiCameraComponent',
                name='front_camera_manager',
                parameters=[{
                    'camera_config_file': LaunchConfiguration('camera_config_file'),
                    'device_group': 'front',
                    'use_image_transport': True,
                    'convert_to_rgb': True,
                    'jpeg_quality': 60,
                }]
            )
        )
    
    # 环视摄像头组
    if 'surround' in enabled_groups:
        composable_nodes.append(
            ComposableNode(
                package='v4l2_camera',
                plugin='v4l2_camera::MultiCameraComponent',
                name='surround_camera_manager',
                parameters=[{
                    'camera_config_file': LaunchConfiguration('camera_config_file'),
                    'device_group': 'surround',
                    'use_image_transport': True,
                    'convert_to_rgb': True,
                    'jpeg_quality': 30,
                }]
            )
        )
    
    return [
        ComposableNodeContainer(
            name='multi_camera_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=composable_nodes,
            output='screen',
            parameters=[{
                'use_intra_process_comms': LaunchConfiguration('use_intra_process_comms')
            }]
        )
    ]


def generate_launch_description():
    """CyberBus 相机驱动启动文件 - 使用 multi_camera_container"""
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_intra_process_comms',
            default_value='true',
            description='Use intra-process communication'
        ),
        
        DeclareLaunchArgument(
            'camera_config_file',
            default_value='/home/cyberbus/test_ws/src/CyberBus_Driver/ros2_v4l2_cam/config/camera_groups_config.yaml',
            description='Camera configuration file path'
        ),
        
        DeclareLaunchArgument(
            'enabled_groups',
            default_value='fisheye,surround,front',
            description='Enabled camera groups (fisheye,surround,front)'
        ),
        
        OpaqueFunction(function=launch_setup)
    ])