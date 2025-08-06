#!/bin/bash

# cleanup_cameras.sh
# 清理相机设备和容器的脚本

echo "=== 清理相机设备和ROS2容器 ==="

# 1. 停止所有相关的ROS2节点和容器
echo "1. 停止ROS2组件容器..."
pkill -f "component_container"
pkill -f "multi_camera"
sleep 2

# 2. 检查并释放可能被占用的相机设备
echo "2. 检查相机设备使用情况..."
for dev in /dev/video{0..15}; do
    if [ -e "$dev" ]; then
        # 检查设备是否被占用
        lsof_output=$(lsof "$dev" 2>/dev/null)
        if [ ! -z "$lsof_output" ]; then
            echo "设备 $dev 被占用："
            echo "$lsof_output"
            # 可选：强制终止占用进程
            # lsof -t "$dev" | xargs -r kill -9
        fi
    fi
done

# 3. 重新设置设备权限（如果需要）
echo "3. 检查设备权限..."
for dev in /dev/video{0..15}; do
    if [ -e "$dev" ]; then
        ls -l "$dev"
    fi
done

# 4. 清理ROS2相关进程
echo "4. 清理ROS2相关进程..."
ros2 daemon stop 2>/dev/null || true
sleep 1
ros2 daemon start 2>/dev/null || true

echo "清理完成！"
