#!/bin/bash

# test_multi_camera.sh
# 测试多相机组件功能的脚本

echo "=== ROS2 多相机组件测试脚本 ==="

# 设置环境
source /opt/ros/foxy/setup.bash
source /home/cyberbus/driver_ws/install/setup.bash

echo "1. 检查可用的组件类型..."
ros2 component types v4l2_camera

echo ""
echo "2. 检查可用的可执行文件..."
ros2 pkg executables v4l2_camera

echo ""
echo "3. 启动组件容器 (后台运行)..."
ros2 run rclcpp_components component_container_mt --ros-args -r __node:=multi_camera_container &
CONTAINER_PID=$!
sleep 2

echo "4. 加载多相机管理器组件..."
ros2 component load /multi_camera_container v4l2_camera v4l2_camera::MultiCameraComponent \
    -p num_cameras:=4 \
    -p default_width:=1920 \
    -p default_height:=1500 \
    --node-name multi_camera_manager

echo ""
echo "5. 等待组件初始化..."
sleep 3

echo "6. 检查运行中的节点..."
ros2 node list

echo ""
echo "7. 检查可用的话题..."
ros2 topic list | grep camera

echo ""
echo "8. 检查组件状态..."
ros2 component list

echo ""
echo "9. 测试完成，清理资源..."
kill $CONTAINER_PID
sleep 1

echo "=== 测试结束 ==="
