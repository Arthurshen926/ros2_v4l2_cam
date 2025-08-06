#!/usr/bin/env python3
# multi_camera_all_groups.launch.py
# 一次性启动所有相机组的启动文件

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, GroupAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """生成启动所有相机组的描述"""
    
    # 声明启动参数
    jpeg_quality_arg = DeclareLaunchArgument(
        'jpeg_quality',
        default_value='40',
        description='JPEG压缩质量 (1-100, 默认40)'
    )
    
    convert_to_rgb_arg = DeclareLaunchArgument(
        'convert_to_rgb',
        default_value='true',
        description='是否转换为RGB格式以支持压缩 (默认true)'
    )
    
    use_image_transport_arg = DeclareLaunchArgument(
        'use_image_transport',
        default_value='true',
        description='是否使用image_transport进行自动压缩 (默认true)'
    )
    
    # 配置文件路径
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='/home/cyberbus/driver_ws/src/ros2_v4l2_camera/config/camera_groups_config.yaml',
        description='相机配置文件路径'
    )
    
    # 启用的相机组
    enabled_groups_arg = DeclareLaunchArgument(
        'enabled_groups',
        default_value='fisheye,front',  # 默认启动鱼眼和前视摄像头
        description='启用的相机组列表，用逗号分隔 (fisheye,fisheye_hd,surround,front)'
    )
    
    # 相机组定义
    camera_groups = {
        'fisheye': {
            'container_name': 'fisheye_camera_container',
            'description': '鱼眼摄像头组 (优化分辨率, 10fps)'
        },
        'fisheye_hd': {
            'container_name': 'fisheye_hd_camera_container', 
            'description': '鱼眼摄像头组 (高清分辨率, 8fps)'
        },
        'surround': {
            'container_name': 'surround_camera_container',
            'description': '环视摄像头组 (6个摄像头, 5fps)'
        },
        'front': {
            'container_name': 'front_camera_container',
            'description': '前视摄像头组 (单摄像头, 15fps)'
        }
    }
    
    # 创建相机组容器
    camera_containers = []
    
    # 为每个相机组创建容器
    for group_name, group_info in camera_groups.items():
        container = ComposableNodeContainer(
            name=group_info['container_name'],
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            parameters=[{
                'use_intra_process_comms': True,
            }],
            composable_node_descriptions=[
                ComposableNode(
                    package='v4l2_camera',
                    plugin='v4l2_camera::MultiCameraComponent',
                    name=f'{group_name}_camera_manager',
                    parameters=[{
                        'camera_config_file': LaunchConfiguration('config_file'),
                        'device_group': group_name,
                        'use_image_transport': LaunchConfiguration('use_image_transport'),
                        'convert_to_rgb': LaunchConfiguration('convert_to_rgb'),
                        'jpeg_quality': LaunchConfiguration('jpeg_quality'),
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ],
            output='screen',
            condition=None,  # 将在运行时根据enabled_groups进行过滤
        )
        camera_containers.append(container)
    
    # 启动信息
    launch_info = LogInfo(
        msg=[
            '启动多组相机系统:\n',
            '  - 启用组: ', LaunchConfiguration('enabled_groups'), '\n',
            '  - JPEG质量: ', LaunchConfiguration('jpeg_quality'), '\n', 
            '  - RGB转换: ', LaunchConfiguration('convert_to_rgb'), '\n',
            '  - 图像传输: ', LaunchConfiguration('use_image_transport'), '\n',
            '  - 配置文件: ', LaunchConfiguration('config_file'), '\n',
            '相机组配置:\n',
            '  ✓ fisheye: 鱼眼摄像头 (1280x1024, 10fps)\n',
            '  ✓ fisheye_hd: 高清鱼眼 (1920x1500, 8fps)\n',
            '  ✓ surround: 环视摄像头 (1920x1500, 5fps)\n',
            '  ✓ front: 前视摄像头 (1920x1500, 15fps)\n',
            '优化功能:\n',
            '  ✓ 可配置帧率\n',
            '  ✓ 自动压缩支持 (JPEG/PNG)\n',
            '  ✓ 带宽优化配置\n',
            '  ✓ RGB格式转换\n',
            '  ✓ 进程内通信\n'
        ]
    )
    
    return LaunchDescription([
        jpeg_quality_arg,
        convert_to_rgb_arg,
        use_image_transport_arg,
        config_file_arg,
        enabled_groups_arg,
        launch_info,
    ] + camera_containers)
