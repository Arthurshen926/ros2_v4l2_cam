from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='camera_launcher_node',
            name='camera_launcher_node',
            parameters=[{
                'arg_camera_dev': '/dev/video0',
                'arg_camera_id': '10',
                'arg_publish_topic': '/camera',
                'arg_resolution_width': 1920,
                'arg_resolution_height': 1500
            }],
            output='screen'
        )
    ])
