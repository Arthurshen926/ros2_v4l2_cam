#!/bin/bash

# launch_specific_cameras.sh
# 启动特定设备组的相机

source /opt/ros/foxy/setup.bash
source /home/cyberbus/driver_ws/install/setup.bash

# 设备定义
declare -A CAMERA_GROUPS
CAMERA_GROUPS[fisheye]="/dev/video0 /dev/video1 /dev/video2 /dev/video3"
CAMERA_GROUPS[surround]="/dev/video8 /dev/video9 /dev/video10 /dev/video11 /dev/video12 /dev/video13"
CAMERA_GROUPS[front]="/dev/video5"

function check_device() {
    local device=$1
    if [ ! -e "$device" ]; then
        echo "设备不存在: $device"
        return 1
    fi
    
    if [ ! -r "$device" ] || [ ! -w "$device" ]; then
        echo "设备权限不足: $device"
        return 1
    fi
    
    if lsof "$device" 2>/dev/null >/dev/null; then
        echo "设备被占用: $device"
        return 1
    fi
    
    return 0
}

function launch_camera_group() {
    local group_name=$1
    local container_name="multi_camera_container"
    
    if [ -z "${CAMERA_GROUPS[$group_name]}" ]; then
        echo "未知的相机组: $group_name"
        echo "可用的组: ${!CAMERA_GROUPS[@]}"
        exit 1
    fi
    
    echo "=== 启动 $group_name 相机组 ==="
    
    # 清理现有资源
    echo "清理现有资源..."
    bash /home/cyberbus/driver_ws/src/ros2_v4l2_camera/scripts/container_manager.sh cleanup
    sleep 2
    
    # 检查设备
    echo "检查设备可用性..."
    devices=(${CAMERA_GROUPS[$group_name]})
    available_devices=()
    
    for device in "${devices[@]}"; do
        if check_device "$device"; then
            echo "✓ $device 可用"
            available_devices+=("$device")
        else
            echo "✗ $device 不可用"
        fi
    done
    
    if [ ${#available_devices[@]} -eq 0 ]; then
        echo "没有可用的设备！"
        exit 1
    fi
    
    echo "可用设备: ${available_devices[@]}"
    
    # 启动容器
    echo "启动组件容器..."
    ros2 run rclcpp_components component_container_mt --ros-args -r __node:=$container_name &
    CONTAINER_PID=$!
    sleep 3
    
    # 逐个加载相机组件
    echo "加载相机组件..."
    for i in "${!available_devices[@]}"; do
        device="${available_devices[$i]}"
        component_name="${group_name}_camera_${i}"
        topic_prefix="/${group_name}/camera"
        camera_id="${i}"
        
        echo "加载相机 $component_name (设备: $device, 话题: ${topic_prefix}${camera_id}/compressed)"
        
        ros2 component load /$container_name v4l2_camera v4l2_camera::CameraLauncherComponent \
            --node-name "$component_name" \
            -p "arg_camera_dev:=$device" \
            -p "arg_camera_id:=\"$camera_id\"" \
            -p "arg_publish_topic:=\"$topic_prefix\"" \
            -p "arg_resolution_width:=640" \
            -p "arg_resolution_height:=480"
        
        # 等待组件初始化
        sleep 2
        
        # 检查是否成功
        if ros2 component list | grep -q "$component_name"; then
            echo "✓ $component_name 加载成功"
        else
            echo "✗ $component_name 加载失败"
        fi
    done
    
    echo ""
    echo "=== 启动完成 ==="
    echo "容器PID: $CONTAINER_PID"
    echo ""
    echo "检查状态:"
    echo "组件列表:"
    ros2 component list
    echo ""
    echo "话题列表:"
    ros2 topic list | grep camera
    echo ""
    echo "使用 'bash /home/cyberbus/driver_ws/src/ros2_v4l2_camera/scripts/container_manager.sh stop' 停止所有相机"
}

function show_usage() {
    echo "用法: $0 <camera_group>"
    echo ""
    echo "可用的相机组:"
    for group in "${!CAMERA_GROUPS[@]}"; do
        echo "  $group: ${CAMERA_GROUPS[$group]}"
    done
    echo ""
    echo "示例:"
    echo "  $0 fisheye    # 启动鱼眼相机组"
    echo "  $0 surround   # 启动环视相机组"
    echo "  $0 front      # 启动前置相机组"
}

# 主逻辑
if [ $# -eq 0 ]; then
    show_usage
    exit 1
fi

GROUP_NAME=$1
launch_camera_group "$GROUP_NAME"
