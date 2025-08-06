#!/bin/bash

# 演示multi_camera_standalone系统的运行

echo "Multi-Camera Standalone 演示脚本"
echo "================================"

# 设置颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 工作目录
WORKSPACE_DIR="/home/cyberbus/driver_ws"

echo -e "${BLUE}工作目录: $WORKSPACE_DIR${NC}"

# 检查是否在正确的目录
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo -e "${RED}错误: 工作目录不存在${NC}"
    exit 1
fi

cd $WORKSPACE_DIR

echo ""
echo -e "${YELLOW}1. 设置ROS2环境...${NC}"
source install/setup.bash

echo ""
echo -e "${YELLOW}2. 显示可用的设备组:${NC}"
echo "  - fisheye: 4个相机 (/dev/video0-3)"
echo "  - surround: 6个相机 (/dev/video8-13)" 
echo "  - front: 1个相机 (/dev/video5)"

echo ""
echo -e "${YELLOW}3. 选择要启动的设备组:${NC}"
echo "请选择设备组:"
echo "1) fisheye"
echo "2) surround"
echo "3) front"
echo "4) 自定义命令"
echo "5) 退出"

read -p "请输入选择 (1-5): " choice

case $choice in
    1)
        DEVICE_GROUP="fisheye"
        echo -e "${GREEN}启动fisheye组相机...${NC}"
        ;;
    2)
        DEVICE_GROUP="surround"
        echo -e "${GREEN}启动surround组相机...${NC}"
        ;;
    3)
        DEVICE_GROUP="front"
        echo -e "${GREEN}启动front组相机...${NC}"
        ;;
    4)
        echo -e "${YELLOW}自定义命令模式${NC}"
        echo ""
        echo "可用的启动命令："
        echo ""
        echo "A) 使用launch文件:"
        echo "ros2 launch v4l2_camera multi_camera_standalone.launch.py device_group:=fisheye"
        echo ""
        echo "B) 直接运行:"
        echo "ros2 run v4l2_camera multi_camera_standalone --ros-args \\"
        echo "  -p camera_config_file:=/home/cyberbus/driver_ws/src/ros2_v4l2_camera/config/camera_groups_config.yaml \\"
        echo "  -p device_group:=fisheye"
        echo ""
        echo "C) 查看话题:"
        echo "ros2 topic list | grep image"
        echo ""
        echo "D) 查看话题信息:"
        echo "ros2 topic info /fisheye0/image_raw"
        echo ""
        exit 0
        ;;
    5)
        echo -e "${YELLOW}退出${NC}"
        exit 0
        ;;
    *)
        echo -e "${RED}无效选择${NC}"
        exit 1
        ;;
esac

echo ""
echo -e "${YELLOW}4. 启动参数配置:${NC}"
echo "  - 设备组: $DEVICE_GROUP"
echo "  - 使用image_transport: true"
echo "  - 输出格式: 原始UYVY"
echo "  - 频率监控: 启用"
echo "  - 监控间隔: 2秒"

echo ""
echo -e "${YELLOW}5. 启动系统...${NC}"
echo -e "${BLUE}命令: ros2 launch v4l2_camera multi_camera_standalone.launch.py device_group:=$DEVICE_GROUP${NC}"
echo ""
echo -e "${GREEN}系统即将启动...${NC}"
echo -e "${YELLOW}按 Ctrl+C 停止系统${NC}"
echo ""

# 等待用户确认
read -p "按 Enter 继续启动，或 Ctrl+C 取消..."

# 启动系统
ros2 launch v4l2_camera multi_camera_standalone.launch.py device_group:=$DEVICE_GROUP

echo ""
echo -e "${GREEN}系统已停止${NC}"
