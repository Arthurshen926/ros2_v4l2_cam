#!/bin/bash

# 测试脚本：验证Camera Launcher Component的原始图像输出功能
# 此脚本演示如何使用新的参数来控制图像输出格式

echo "Camera Launcher Component 原始图像输出测试"
echo "============================================="

echo ""
echo "1. 测试输出原始UYVY格式图像（默认）："
echo "ros2 run v4l2_camera camera_launcher_node --ros-args \\"
echo "  -p arg_camera_dev:=/dev/video0 \\"
echo "  -p arg_camera_id:='' \\"
echo "  -p arg_publish_topic:=/camera \\"
echo "  -p use_image_transport:=true \\"
echo "  -p convert_to_rgb:=false"

echo ""
echo "2. 测试输出RGB转换后的图像："
echo "ros2 run v4l2_camera camera_launcher_node --ros-args \\"
echo "  -p arg_camera_dev:=/dev/video0 \\"
echo "  -p arg_camera_id:='' \\"
echo "  -p arg_publish_topic:=/camera \\"
echo "  -p use_image_transport:=true \\"
echo "  -p convert_to_rgb:=true"

echo ""
echo "3. 测试不使用image_transport（仅标准ROS2发布器）："
echo "ros2 run v4l2_camera camera_launcher_node --ros-args \\"
echo "  -p arg_camera_dev:=/dev/video0 \\"
echo "  -p arg_camera_id:='' \\"
echo "  -p arg_publish_topic:=/camera \\"
echo "  -p use_image_transport:=false \\"
echo "  -p convert_to_rgb:=false"

echo ""
echo "4. 查看可用的话题："
echo "ros2 topic list | grep camera"

echo ""
echo "5. 查看图像信息："
echo "ros2 topic echo /camera/image_raw --once | head -20"

echo ""
echo "6. 查看压缩图像话题（如果使用image_transport）："
echo "ros2 topic list | grep compressed"

echo ""
echo "参数说明："
echo "- use_image_transport: 是否使用image_transport（支持压缩）"
echo "- convert_to_rgb: 是否将UYVY转换为RGB格式"
echo "- arg_camera_dev: 相机设备路径"
echo "- arg_camera_id: 相机ID（通常为空字符串表示使用完整设备路径）"
echo "- arg_publish_topic: 发布话题的基础名称"

echo ""
echo "图像格式："
echo "- convert_to_rgb=false: 输出yuv422_yuy2格式（原始UYVY）"
echo "- convert_to_rgb=true: 输出rgb8格式（转换后的RGB）"

echo ""
echo "image_transport压缩功能："
echo "当use_image_transport=true时，会自动提供以下话题："
echo "- /camera/image_raw - 原始图像"
echo "- /camera/image_raw/compressed - 压缩图像（需要安装image_transport_plugins）"
echo "- /camera/image_raw/theora - Theora视频压缩（需要相应插件）"
