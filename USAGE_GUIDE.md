# 多相机组件系统使用指南

## 概述

该系统提供了一个完整的多相机组件解决方案，支持设备分组管理、自动清理和进程内通信。

## 系统特性

- **ROS2组件架构**: 使用组件化设计，支持运行时组合
- **设备分组管理**: 支持按功能分组管理相机（如鱼眼、环视、前视相机）
- **进程内通信**: 优化的零拷贝消息传递
- **YAML配置**: 灵活的配置文件支持
- **设备自动清理**: 智能的设备资源管理

## 使用方法

### 1. 预清理设备（推荐）

在启动相机系统前，运行清理脚本：

```bash
cd /home/cyberbus/driver_ws/src/ros2_v4l2_camera
./scripts/cleanup_cameras.sh
```

### 2. 启动特定设备组

#### 启动鱼眼相机组：
```bash
source /home/cyberbus/driver_ws/install/setup.bash
ros2 launch v4l2_camera specific_cameras.launch.py device_group:=fisheye_devs
```

#### 启动环视相机组：
```bash
ros2 launch v4l2_camera specific_cameras.launch.py device_group:=surround_devs
```

#### 启动前视相机组：
```bash
ros2 launch v4l2_camera specific_cameras.launch.py device_group:=front_devs
```

#### 启动所有相机：
```bash
ros2 launch v4l2_camera specific_cameras.launch.py device_group:=all
```

### 3. 验证相机状态

查看相机话题：
```bash
ros2 topic list | grep camera
```

查看组件状态：
```bash
ros2 component list
```

监听特定相机的图像：
```bash
ros2 run rqt_image_view rqt_image_view /fisheye_front/image_raw
```

## 配置文件说明

### camera_groups_config.yaml

该文件定义了不同的设备组和相机配置：

```yaml
device_groups:
  fisheye_devs:
    description: "前后鱼眼相机"
    devices:
      - device_path: "/dev/video0"
        camera_id: "fisheye_front"
        topic_prefix: "fisheye_front"
        width: 1920
        height: 1080
        publish_rate: 30
  
  surround_devs:
    description: "环视相机系统"
    devices:
      - device_path: "/dev/video2"
        camera_id: "surround_left"
        # ... 更多配置
```

### 自定义配置

要添加新的设备组：

1. 编辑 `config/camera_groups_config.yaml`
2. 添加新的设备组和设备列表
3. 重新启动系统

## 故障排除

### 设备忙错误 (Device or resource busy)

如果遇到此错误：

1. **运行清理脚本**：
   ```bash
   ./scripts/cleanup_cameras.sh
   ```

2. **手动检查设备状态**：
   ```bash
   lsof /dev/video*
   ```

3. **重启相机硬件**（如果可用）：
   ```bash
   sudo modprobe -r uvcvideo
   sudo modprobe uvcvideo
   ```

### 没有图像输出

1. **检查设备权限**：
   ```bash
   ls -la /dev/video*
   sudo chmod 666 /dev/video*
   ```

2. **验证设备功能**：
   ```bash
   v4l2-ctl --list-devices
   v4l2-ctl -d /dev/video0 --list-formats-ext
   ```

3. **检查ROS2话题**：
   ```bash
   ros2 topic echo /fisheye_front/image_raw --field header
   ```

### 组件注册失败

1. **检查组件是否正确注册**：
   ```bash
   ros2 component types | grep v4l2_camera
   ```

2. **重新编译包**：
   ```bash
   cd /home/cyberbus/driver_ws
   colcon build --packages-select v4l2_camera
   source install/setup.bash
   ```

## 高级功能

### 动态重配置

在运行时更改相机参数：

```bash
ros2 param set /multi_camera_container camera_fisheye_front.arg_publish_topic new_topic_name
```

### 监控相机状态

订阅相机状态话题：

```bash
ros2 topic echo /multi_camera_status
```

### 性能优化

1. **启用进程内通信**：系统默认启用，减少消息复制开销
2. **调整发布频率**：根据需要在配置文件中调整 `publish_rate`
3. **设置CPU亲和性**：在launch文件中配置线程亲和性

## 架构说明

### 组件层次结构

```
MultiCameraComponent (管理器)
├── CameraLauncherComponent (相机1)
├── CameraLauncherComponent (相机2)
└── CameraLauncherComponent (相机N)
```

### 数据流

```
V4L2设备 → CameraLauncherComponent → ROS2话题
              ↓ (进程内通信)
         MultiCameraComponent (状态监控)
```

## 开发指南

### 添加新的相机组件

1. 继承 `CameraLauncherComponent`
2. 重写 `configure()` 和 `start()` 方法
3. 在CMakeLists.txt中注册组件
4. 更新launch文件

### 扩展配置选项

1. 在 `CameraConfig` 结构体中添加新字段
2. 更新YAML配置解析逻辑
3. 在组件初始化中处理新参数

## 常用命令参考

```bash
# 编译系统
colcon build --packages-select v4l2_camera

# 清理设备
./scripts/cleanup_cameras.sh

# 启动特定设备组
ros2 launch v4l2_camera specific_cameras.launch.py device_group:=fisheye_devs

# 查看组件
ros2 component list

# 监控话题
ros2 topic list | grep camera

# 检查设备
v4l2-ctl --list-devices

# 查看日志
ros2 log info | grep v4l2_camera
```

## 联系支持

如果遇到问题，请：

1. 检查错误日志
2. 运行诊断脚本
3. 查看该文档的故障排除部分
4. 提供完整的错误信息和系统环境
