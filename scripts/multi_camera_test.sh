#!/bin/bash

# multi_camera_test.sh
# 测试multi_camera_standalone系统

echo "Multi-Camera Standalone 测试脚本"
echo "==============================="

# 设置工作空间
WORKSPACE_DIR="/home/cyberbus/driver_ws"
cd $WORKSPACE_DIR

echo ""
echo "1. 加载环境..."
source install/setup.bash

echo ""
echo "2. 检查可用的相机设备..."
echo "可用的视频设备："
ls -la /dev/video* 2>/dev/null || echo "没有找到视频设备"

echo ""
echo "3. 检查设备权限..."
for device in /dev/video{0..13}; do
    if [ -e "$device" ]; then
        if [ -r "$device" ] && [ -w "$device" ]; then
            echo "✓ $device - 权限正常"
        else
            echo "✗ $device - 权限不足"
            echo "  运行: sudo chmod 666 $device"
        fi
    fi
done

echo ""
echo "4. 验证配置文件..."
CONFIG_FILE="$WORKSPACE_DIR/src/ros2_v4l2_camera/config/camera_groups_config.yaml"
if [ -f "$CONFIG_FILE" ]; then
    echo "✓ 配置文件存在: $CONFIG_FILE"
    echo ""
    echo "配置文件内容："
    cat "$CONFIG_FILE"
else
    echo "✗ 配置文件不存在: $CONFIG_FILE"
    exit 1
fi

echo ""
echo "5. 检查可执行文件..."
if [ -f "install/v4l2_camera/lib/v4l2_camera/multi_camera_standalone" ]; then
    echo "✓ multi_camera_standalone 可执行文件存在"
else
    echo "✗ multi_camera_standalone 可执行文件不存在"
    echo "  请运行编译命令"
    exit 1
fi

echo ""
echo "6. 检查launch文件..."
LAUNCH_FILE="$WORKSPACE_DIR/src/ros2_v4l2_camera/launch/multi_camera_standalone.launch.py"
if [ -f "$LAUNCH_FILE" ]; then
    echo "✓ Launch文件存在: $LAUNCH_FILE"
else
    echo "✗ Launch文件不存在: $LAUNCH_FILE"
fi

echo ""
echo "==============================="
echo "测试完成！"
echo ""
echo "如果一切正常，您可以运行以下命令启动系统："
echo ""
echo "方式1 - 使用launch文件启动fisheye组："
echo "ros2 launch v4l2_camera multi_camera_standalone.launch.py device_group:=fisheye"
echo ""
echo "方式2 - 直接运行可执行文件："
echo "ros2 run v4l2_camera multi_camera_standalone --ros-args \\"
echo "  -p camera_config_file:=$CONFIG_FILE \\"
echo "  -p device_group:=fisheye \\"
echo "  -p frequency_monitor_enabled:=true"
echo ""
echo "运行后，在另一个终端中查看话题："
echo "ros2 topic list | grep image"
