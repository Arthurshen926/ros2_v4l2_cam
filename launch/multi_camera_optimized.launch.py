#!/usr/bin/env python3
# multi_camera_optimized.launch.py
# 优化的多摄像头启动文件，支持带宽优化配置

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """生成优化的多摄像头启动描述"""
    
    # 声明启动参数
    device_group_arg = DeclareLaunchArgument(
        'device_group',
        default_value='fisheye',
        description='相机设备组名称 (fisheye, fisheye_hd, surround, front)'
    )
    
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
    
    container_name_arg = DeclareLaunchArgument(
        'container_name',
        default_value='multi_camera_optimized_container',
        description='容器名称'
    )
    
    # 配置文件路径
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='/home/cyberbus/driver_ws/src/ros2_v4l2_camera/config/camera_groups_config.yaml',
        description='相机配置文件路径'
    )
    
    # 创建多摄像头组件容器
    multi_camera_container = ComposableNodeContainer(
        name=LaunchConfiguration('container_name'),
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
                name='multi_camera_manager',
                parameters=[{
                    'camera_config_file': LaunchConfiguration('config_file'),
                    'device_group': LaunchConfiguration('device_group'),
                    'use_image_transport': LaunchConfiguration('use_image_transport'),
                    'convert_to_rgb': LaunchConfiguration('convert_to_rgb'),
                    'jpeg_quality': LaunchConfiguration('jpeg_quality'),
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )
    
    # 启动信息
    launch_info = LogInfo(
        msg=[
            '启动优化的多摄像头系统:\n',
            '  - 设备组: ', LaunchConfiguration('device_group'), '\n',
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
        device_group_arg,
        jpeg_quality_arg,
        convert_to_rgb_arg,
        use_image_transport_arg,
        container_name_arg,
        config_file_arg,
        launch_info,
        multi_camera_container,
    ])
