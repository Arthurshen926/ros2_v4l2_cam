# 多相机组件使用文档

## 概述

此实现为 `camera_launcher_node.cpp` 扩展了一个使用 ROS2 component 组件进行多相机进程内通信的功能。主要包含以下组件：

## 组件架构

### 1. CameraLauncherComponent
- **文件**: `src/camera_launcher_component.cpp`
- **头文件**: `include/v4l2_camera/camera_launcher_component.hpp`
- **功能**: 将原始的 camera_launcher_node 转换为可重用的 ROS2 组件
- **特性**:
  - 支持 V4L2 相机捕获
  - UYVY 到 BGR 颜色空间转换
  - JPEG 压缩图像发布
  - 可配置的相机参数

### 2. MultiCameraComponent
- **文件**: `src/multi_camera_component.cpp`
- **头文件**: `include/v4l2_camera/multi_camera_component.hpp`
- **功能**: 管理多个相机组件的生命周期
- **特性**:
  - 自动创建和管理多个相机实例
  - 支持进程内通信 (intra-process communication)
  - 相机状态监控和自动重启
  - 灵活的配置系统
  - **新增**: 支持YAML配置文件的相机组管理

## 配置文件支持

### YAML配置文件格式

系统现在支持通过YAML配置文件来管理相机组。配置文件示例 (`config/camera_groups_config.yaml`):

```yaml
camera_groups:
  fisheye:
    devices:
      - "/dev/video0"
      - "/dev/video1" 
      - "/dev/video2"
      - "/dev/video3"
    topic_prefix: "/fisheye"
    width: 1920
    height: 1500
    
  surround:
    devices:
      - "/dev/video8"
      - "/dev/video9"
      - "/dev/video10"
      - "/dev/video11"
      - "/dev/video12"
      - "/dev/video13"
    topic_prefix: "/surround"
    width: 1920
    height: 1500

global_settings:
  container_name: "multi_camera_container"
  use_intra_process_comms: true
  publish_rate: 30
```

## 使用方法

### 方法 1: 使用配置文件 (新增推荐方式)

```bash
# 使用默认的fisheye组
ros2 run v4l2_camera multi_camera_standalone

# 指定不同的相机组
ros2 run v4l2_camera multi_camera_standalone --ros-args -p device_group:=surround

# 指定自定义配置文件路径
ros2 run v4l2_camera multi_camera_standalone --ros-args \
    -p camera_config_file:=/path/to/your/config.yaml \
    -p device_group:=fisheye
```

### 方法 2: 使用组件容器

```bash
# 启动组件容器
ros2 run rclcpp_components component_container_mt --ros-args -r __node:=multi_camera_container

# 加载多相机管理器（使用配置文件）
ros2 component load /multi_camera_container v4l2_camera v4l2_camera::MultiCameraComponent \
    -p camera_config_file:=/home/cyberbus/driver_ws/src/ros2_v4l2_camera/config/camera_groups_config.yaml \
    -p device_group:=fisheye \
    --node-name multi_camera_manager
```

### 方法 3: 传统参数方式 (保持兼容)

```bash
# 使用传统参数
ros2 run v4l2_camera multi_camera_standalone --ros-args \
    -p num_cameras:=2 \
    -p default_width:=1920 \
    -p default_height:=1500
```

## 配置参数

### MultiCameraComponent 参数

| 参数名 | 类型 | 默认值 | 描述 |
|--------|------|--------|------|
| `camera_config_file` | string | "" | YAML配置文件路径 |
| `device_group` | string | "fisheye" | 要使用的相机组名称 |
| `num_cameras` | int | 2 | 相机数量（无配置文件时使用） |
| `default_width` | int | 1920 | 默认相机宽度 |
| `default_height` | int | 1500 | 默认相机高度 |
| `default_device_prefix` | string | "/dev/video" | 设备路径前缀 |
| `default_topic_prefix` | string | "/camera" | 话题前缀 |

## 话题输出

基于配置文件的话题输出示例：

**fisheye组**:
- `/fisheye0/compressed` - 鱼眼相机0
- `/fisheye1/compressed` - 鱼眼相机1
- `/fisheye2/compressed` - 鱼眼相机2
- `/fisheye3/compressed` - 鱼眼相机3

**surround组**:
- `/surround0/compressed` - 环视相机0
- `/surround1/compressed` - 环视相机1
- `/surround2/compressed` - 环视相机2
- `/surround3/compressed` - 环视相机3
- `/surround4/compressed` - 环视相机4
- `/surround5/compressed` - 环视相机5

## 进程内通信优势

1. **零拷贝传输**: 同一进程内的组件间可以共享内存，避免数据拷贝
2. **降低延迟**: 减少了序列化/反序列化开销
3. **提高吞吐量**: 特别适合高频率、大数据量的传输
4. **资源效率**: 减少网络和 CPU 开销

## 监控和调试

### 查看运行状态

```bash
# 查看活跃的组件
ros2 component list

# 查看话题
ros2 topic list

# 监控图像数据
ros2 topic echo /camera0/compressed --no-arr

# 查看话题频率
ros2 topic hz /camera0/compressed
```

### 查看节点图

```bash
# 安装 rqt_graph (如果尚未安装)
ros2 run rqt_graph rqt_graph
```

## 故障排除

### 常见问题

1. **相机设备无法打开**:
   - 检查设备权限: `ls -l /dev/video*`
   - 确保设备存在: `v4l2-ctl --list-devices`

2. **无图像输出**:
   - 检查相机是否被其他进程占用
   - 验证分辨率设置是否被相机支持

3. **组件加载失败**:
   - 确保库文件已正确编译和安装
   - 检查 ROS2 环境变量设置

### 调试命令

```bash
# 详细日志输出
ros2 run v4l2_camera multi_camera_standalone --ros-args --log-level debug

# 检查组件注册
ros2 pkg executables v4l2_camera
```

## 扩展功能

### 添加新的相机类型

1. 继承 `CameraLauncherComponent`
2. 重写相机初始化和捕获方法
3. 在 `MultiCameraComponent` 中添加相应的工厂方法

### 配置文件支持

要支持 YAML 配置文件，需要：

1. 在 `CMakeLists.txt` 中添加 `yaml-cpp` 依赖
2. 更新 `loadCameraConfigurations()` 方法
3. 创建对应的 YAML 配置文件

## 性能优化建议

1. **调整发布频率**: 根据需求设置合适的帧率
2. **优化图像压缩**: 调整 JPEG 压缩质量参数
3. **使用多线程执行器**: 确保使用 `MultiThreadedExecutor`
4. **内存预分配**: 在高频应用中预分配图像缓冲区
