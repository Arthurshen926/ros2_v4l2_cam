#!/usr/bin/env python3

"""
示例launch文件：演示Camera Launcher Component的原始图像输出功能
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """生成launch描述"""
    
    # 声明启动参数
    camera_dev_arg = DeclareLaunchArgument(
        'camera_dev',
        default_value='/dev/video0',
        description='相机设备路径'
    )
    
    camera_id_arg = DeclareLaunchArgument(
        'camera_id',
        default_value='',
        description='相机ID（通常为空）'
    )
    
    publish_topic_arg = DeclareLaunchArgument(
        'publish_topic',
        default_value='/camera',
        description='发布话题基础名称'
    )
    
    convert_to_rgb_arg = DeclareLaunchArgument(
        'convert_to_rgb',
        default_value='false',
        description='是否转换为RGB格式（false=输出原始UYVY）'
    )
    
    use_image_transport_arg = DeclareLaunchArgument(
        'use_image_transport',
        default_value='true',
        description='是否使用image_transport（支持压缩）'
    )
    
    width_arg = DeclareLaunchArgument(
        'width',
        default_value='1920',
        description='图像宽度'
    )
    
    height_arg = DeclareLaunchArgument(
        'height',
        default_value='1500',
        description='图像高度'
    )
    
    # 创建相机节点
    camera_node = Node(
        package='v4l2_camera',
        executable='camera_launcher_node',
        name='camera_launcher',
        parameters=[{
            'arg_camera_dev': LaunchConfiguration('camera_dev'),
            'arg_camera_id': LaunchConfiguration('camera_id'),
            'arg_publish_topic': LaunchConfiguration('publish_topic'),
            'arg_resolution_width': LaunchConfiguration('width'),
            'arg_resolution_height': LaunchConfiguration('height'),
            'convert_to_rgb': LaunchConfiguration('convert_to_rgb'),
            'use_image_transport': LaunchConfiguration('use_image_transport'),
        }],
        output='screen'
    )
    
    return LaunchDescription([
        camera_dev_arg,
        camera_id_arg,
        publish_topic_arg,
        convert_to_rgb_arg,
        use_image_transport_arg,
        width_arg,
        height_arg,
        camera_node
    ])
