#!/bin/bash

# container_manager.sh
# 管理多相机容器的脚本

CONTAINER_NAME="multi_camera_container"

function cleanup_cameras() {
    echo "正在清理相机资源..."
    
    # 检查是否有相机进程在运行
    if pgrep -f "camera_launcher_node\|v4l2_camera_node\|multi_camera" > /dev/null; then
        echo "发现相机进程，正在终止..."
        pkill -f "camera_launcher_node\|v4l2_camera_node\|multi_camera"
        sleep 2
    fi
    
    # 检查容器是否存在
    if ros2 node list 2>/dev/null | grep -q "$CONTAINER_NAME"; then
        echo "发现容器 $CONTAINER_NAME，正在卸载组件..."
        
        # 获取容器中的组件列表
        COMPONENTS=$(ros2 component list 2>/dev/null | grep -A 20 "/$CONTAINER_NAME" | grep -E "^ *[0-9]+" | awk '{print $2}')
        
        # 逐个卸载组件
        for component in $COMPONENTS; do
            echo "卸载组件: $component"
            ros2 component unload /$CONTAINER_NAME $component 2>/dev/null || true
        done
        
        sleep 1
    fi
    
    # 终止容器进程
    if pgrep -f "component_container.*$CONTAINER_NAME" > /dev/null; then
        echo "终止容器进程..."
        pkill -f "component_container.*$CONTAINER_NAME"
        sleep 2
    fi
    
    # 检查并释放V4L2设备
    echo "检查V4L2设备状态..."
    for device in /dev/video{0..15}; do
        if [ -e "$device" ]; then
            # 检查设备是否被占用
            if lsof "$device" 2>/dev/null | grep -q video; then
                echo "设备 $device 仍被占用，尝试释放..."
                PIDS=$(lsof -t "$device" 2>/dev/null)
                if [ -n "$PIDS" ]; then
                    echo "终止占用 $device 的进程: $PIDS"
                    kill -TERM $PIDS 2>/dev/null || true
                    sleep 1
                    # 如果还在运行，强制终止
                    kill -KILL $PIDS 2>/dev/null || true
                fi
            fi
        fi
    done
    
    echo "清理完成！"
}

function start_container() {
    echo "启动多相机容器..."
    cleanup_cameras
    sleep 1
    
    echo "启动新的容器..."
    ros2 run rclcpp_components component_container_mt --ros-args -r __node:=$CONTAINER_NAME &
    CONTAINER_PID=$!
    
    # 等待容器启动
    echo "等待容器启动..."
    for i in {1..10}; do
        if ros2 node list 2>/dev/null | grep -q "$CONTAINER_NAME"; then
            echo "容器启动成功！"
            break
        fi
        sleep 1
    done
    
    echo "容器PID: $CONTAINER_PID"
}

function stop_container() {
    echo "停止多相机容器..."
    cleanup_cameras
}

function restart_container() {
    echo "重启多相机容器..."
    stop_container
    sleep 2
    start_container
}

function show_status() {
    echo "=== 容器状态 ==="
    echo "节点列表:"
    ros2 node list 2>/dev/null | grep -E "(camera|container)" || echo "未发现相机相关节点"
    
    echo ""
    echo "组件列表:"
    ros2 component list 2>/dev/null || echo "未发现组件容器"
    
    echo ""
    echo "相机话题:"
    ros2 topic list 2>/dev/null | grep camera || echo "未发现相机话题"
    
    echo ""
    echo "V4L2设备状态:"
    for device in /dev/video{0..15}; do
        if [ -e "$device" ]; then
            if lsof "$device" 2>/dev/null >/dev/null; then
                echo "$device: 被占用"
            else
                echo "$device: 可用"
            fi
        fi
    done
}

case "$1" in
    start)
        start_container
        ;;
    stop)
        stop_container
        ;;
    restart)
        restart_container
        ;;
    status)
        show_status
        ;;
    cleanup)
        cleanup_cameras
        ;;
    *)
        echo "用法: $0 {start|stop|restart|status|cleanup}"
        echo ""
        echo "命令说明:"
        echo "  start   - 启动多相机容器"
        echo "  stop    - 停止多相机容器"
        echo "  restart - 重启多相机容器"
        echo "  status  - 显示当前状态"
        echo "  cleanup - 清理所有相机资源"
        exit 1
        ;;
esac
