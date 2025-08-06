#!/bin/bash

# check_camera_devices.sh
# 检查相机设备状态

echo "=== 相机设备状态检查 ==="
echo ""

# 设备定义
declare -A CAMERA_GROUPS
CAMERA_GROUPS[fisheye]="/dev/video0 /dev/video1 /dev/video2 /dev/video3"
CAMERA_GROUPS[surround]="/dev/video8 /dev/video9 /dev/video10 /dev/video11 /dev/video12 /dev/video13"
CAMERA_GROUPS[front]="/dev/video5"

function check_device_detailed() {
    local device=$1
    local status="✓"
    local issues=()
    
    # 检查设备存在
    if [ ! -e "$device" ]; then
        status="✗"
        issues+=("不存在")
    else
        # 检查权限
        if [ ! -r "$device" ]; then
            status="⚠"
            issues+=("无读权限")
        fi
        if [ ! -w "$device" ]; then
            status="⚠"
            issues+=("无写权限")
        fi
        
        # 检查是否被占用
        if lsof "$device" 2>/dev/null >/dev/null; then
            status="⚠"
            occupying_process=$(lsof "$device" 2>/dev/null | tail -n +2 | awk '{print $1 "(" $2 ")"}' | head -1)
            issues+=("被占用:$occupying_process")
        fi
        
        # 检查设备信息
        if [ -e "$device" ] && [ -r "$device" ]; then
            v4l2_info=$(v4l2-ctl --device="$device" --info 2>/dev/null | grep "Card type" | cut -d: -f2 | xargs)
            if [ -n "$v4l2_info" ]; then
                issues+=("卡类型:$v4l2_info")
            else
                issues+=("无法获取设备信息")
            fi
        fi
    fi
    
    printf "%-15s %s %s\n" "$device" "$status" "$(IFS=', '; echo "${issues[*]}")"
}

function show_group_status() {
    local group_name=$1
    echo "=== $group_name 相机组 ==="
    printf "%-15s %-8s %s\n" "设备" "状态" "详情"
    printf "%-15s %-8s %s\n" "-----" "----" "-----"
    
    devices=(${CAMERA_GROUPS[$group_name]})
    for device in "${devices[@]}"; do
        check_device_detailed "$device"
    done
    echo ""
}

function show_system_info() {
    echo "=== 系统信息 ==="
    echo "当前用户: $(whoami)"
    echo "用户组: $(groups)"
    echo ""
    
    echo "=== V4L2 设备列表 ==="
    v4l2-ctl --list-devices 2>/dev/null || echo "v4l2-ctl 未安装或无可用设备"
    echo ""
    
    echo "=== 活动的ROS2进程 ==="
    ps aux | grep -E "(ros2|component_container)" | grep -v grep || echo "无活动的ROS2进程"
    echo ""
    
    echo "=== 当前活动的相机话题 ==="
    timeout 3 ros2 topic list 2>/dev/null | grep camera || echo "无ROS2相机话题或ROS2未运行"
    echo ""
}

# 主逻辑
if [ "$1" = "--system" ]; then
    show_system_info
    exit 0
fi

if [ "$1" = "--group" ] && [ -n "$2" ]; then
    if [ -n "${CAMERA_GROUPS[$2]}" ]; then
        show_group_status "$2"
    else
        echo "未知的相机组: $2"
        echo "可用的组: ${!CAMERA_GROUPS[@]}"
        exit 1
    fi
    exit 0
fi

# 默认显示所有组
show_system_info

for group in "${!CAMERA_GROUPS[@]}"; do
    show_group_status "$group"
done

echo "用法:"
echo "  $0                    # 显示所有信息"
echo "  $0 --group fisheye    # 只显示特定组"
echo "  $0 --system          # 只显示系统信息"
