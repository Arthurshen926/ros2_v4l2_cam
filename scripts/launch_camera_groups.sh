#!/bin/bash
# launch_camera_groups.sh
# 便捷的相机组启动脚本

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}======================================${NC}"
echo -e "${BLUE}     ROS2 多相机组启动脚本${NC}"
echo -e "${BLUE}======================================${NC}"

# 检查参数
if [ "$#" -eq 0 ]; then
    echo -e "${YELLOW}使用方法:${NC}"
    echo "  $0 <相机组名> [参数...]"
    echo ""
    echo -e "${YELLOW}可用的相机组:${NC}"
    echo "  • fisheye      - 鱼眼摄像头 (1280x1024, 10fps)"
    echo "  • fisheye_hd   - 高清鱼眼 (1920x1500, 8fps)"  
    echo "  • surround     - 环视摄像头 (1920x1500, 5fps)"
    echo "  • front        - 前视摄像头 (1920x1500, 15fps)"
    echo "  • all          - 启动所有组"
    echo ""
    echo -e "${YELLOW}可选参数:${NC}"
    echo "  jpeg_quality:=<1-100>     JPEG压缩质量 (默认40)"
    echo "  convert_to_rgb:=<true/false>  RGB转换 (默认true)"
    echo ""
    echo -e "${YELLOW}示例:${NC}"
    echo "  $0 fisheye"
    echo "  $0 fisheye_hd jpeg_quality:=60"
    echo "  $0 all convert_to_rgb:=false"
    exit 1
fi

CAMERA_GROUP=$1
shift  # 移除第一个参数

# 进入工作目录
cd /home/cyberbus/driver_ws

# 检查工作空间
if [ ! -f "install/setup.bash" ]; then
    echo -e "${RED}错误: 未找到ROS2工作空间！${NC}"
    echo "请确保在正确的工作空间目录中运行此脚本"
    exit 1
fi

# 设置环境
echo -e "${GREEN}设置ROS2环境...${NC}"
source install/setup.bash

# 根据相机组选择launch文件
case $CAMERA_GROUP in
    "fisheye"|"fisheye_hd"|"surround"|"front")
        echo -e "${GREEN}启动单个相机组: $CAMERA_GROUP${NC}"
        exec ros2 launch src/ros2_v4l2_camera/launch/multi_camera_optimized.launch.py \
            device_group:=$CAMERA_GROUP "$@"
        ;;
    "all")
        echo -e "${GREEN}启动所有相机组...${NC}"
        exec ros2 launch src/ros2_v4l2_camera/launch/multi_camera_all_groups.launch.py \
            enabled_groups:=fisheye,front "$@"
        ;;
    *)
        echo -e "${RED}错误: 未知的相机组 '$CAMERA_GROUP'${NC}"
        echo "可用的相机组: fisheye, fisheye_hd, surround, front, all"
        exit 1
        ;;
esac
