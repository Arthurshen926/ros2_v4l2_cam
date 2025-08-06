#!/usr/bin/env python3

"""
multi_camera_standalone.launch.py
Launch文件用于运行multi_camera_standalone节点
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """生成launch描述"""
    
    # 获取包路径
    package_dir = FindPackageShare('v4l2_camera')
    
    # 声明启动参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            package_dir,
            'config',
            'camera_groups_config.yaml'
        ]),
        description='相机组配置文件路径'
    )
    
    device_group_arg = DeclareLaunchArgument(
        'device_group',
        default_value='fisheye',
        description='要启动的设备组名称 (fisheye, surround, front)'
    )
    
    frequency_monitor_arg = DeclareLaunchArgument(
        'frequency_monitor_enabled',
        default_value='true',
        description='是否启用频率监控'
    )
    
    frequency_interval_arg = DeclareLaunchArgument(
        'frequency_interval',
        default_value='2.0',
        description='频率监控打印间隔（秒）'
    )
    
    use_image_transport_arg = DeclareLaunchArgument(
        'use_image_transport',
        default_value='true',
        description='是否使用image_transport（支持压缩）'
    )
    
    convert_to_rgb_arg = DeclareLaunchArgument(
        'convert_to_rgb',
        default_value='false',
        description='是否转换为RGB格式（false=输出原始UYVY）'
    )
    
    # 创建multi_camera_standalone节点
    multi_camera_node = Node(
        package='v4l2_camera',
        executable='multi_camera_standalone',
        name='multi_camera_system',
        parameters=[{
            'camera_config_file': LaunchConfiguration('config_file'),
            'device_group': LaunchConfiguration('device_group'),
            'frequency_monitor_enabled': LaunchConfiguration('frequency_monitor_enabled'),
            'frequency_monitor_interval': LaunchConfiguration('frequency_interval'),
            'use_image_transport': LaunchConfiguration('use_image_transport'),
            'convert_to_rgb': LaunchConfiguration('convert_to_rgb'),
        }],
        output='screen'
    )
    
    # 打印启动信息
    startup_info = LogInfo(msg=[
        '启动Multi-Camera系统',
        '\n配置文件: ', LaunchConfiguration('config_file'),
        '\n设备组: ', LaunchConfiguration('device_group'),
        '\n使用image_transport: ', LaunchConfiguration('use_image_transport'),
        '\n转换为RGB: ', LaunchConfiguration('convert_to_rgb'),
        '\n频率监控: ', LaunchConfiguration('frequency_monitor_enabled')
    ])
    
    return LaunchDescription([
        config_file_arg,
        device_group_arg,
        frequency_monitor_arg,
        frequency_interval_arg,
        use_image_transport_arg,
        convert_to_rgb_arg,
        startup_info,
        multi_camera_node
    ])
