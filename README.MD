# ROS2 V4L2 Camera - 多相机组件系统

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-Apache%202.0-orange.svg)](LICENSE)

一个功能完整的 ROS2 多相机驱动系统，基于 Video4Linux2 (V4L2)，支持组件化架构、设备分组管理和进程内通信优化。

## ✨ 主要特性

- 🚀 **高性能**: 基于 ROS2 组件架构，支持进程内零拷贝通信
- 📹 **多相机支持**: 同时管理多个相机设备，支持设备分组
- ⚙️ **灵活配置**: YAML 配置文件支持，易于管理不同相机组合
- 🖼️ **多格式输出**: 支持原始 UYVY 和 RGB 格式，可选压缩传输
- 📊 **实时监控**: 内置频率监控和状态报告
- 🔧 **易于扩展**: 模块化设计，支持自定义相机类型

## 🚀 快速开始

### 1. 编译项目

```bash
cd /home/cyberbus/driver_ws
colcon build --packages-select v4l2_camera --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 2. 检查相机设备

```bash
# 查看可用设备
ls -la /dev/video*

# 设置权限（如需要）
sudo chmod 666 /dev/video*
```

### 3. 启动相机系统

```bash
# 启动所有相机组
ros2 launch v4l2_camera multi_camera_all_groups.launch.py

# 选择性启动特定相机组
ros2 launch v4l2_camera multi_camera_select_group.launch.py enabled_groups:=fisheye

# 启动多个组合
ros2 launch v4l2_camera multi_camera_select_group.launch.py enabled_groups:=fisheye,front

# 启动单个相机进行测试
ros2 launch v4l2_camera single_camera_launcher.launch.py
```

### 4. 验证运行状态

```bash
# 查看相机话题
ros2 topic list | grep image

# 监控图像频率
ros2 topic hz /fisheye/image_raw

# 查看系统状态
ros2 topic echo /multi_camera_status
```

## 📁 项目结构

```
ros2_v4l2_camera/
├── config/
│   └── camera_groups_config.yaml    # 相机组配置文件
├── include/v4l2_camera/
│   ├── camera_launcher_component.hpp # 相机组件头文件
│   ├── multi_camera_component.hpp   # 多相机管理器
│   └── visibility_control.h         # 可见性控制
├── src/
│   ├── camera_launcher_component.cpp # 相机组件实现
│   ├── multi_camera_component.cpp   # 多相机管理器实现
│   ├── simple_multi_camera_example.cpp # 使用示例
│   └── v4l2_camera_node.cpp         # 原始单相机节点
├── launch/
│   ├── multi_camera_all_groups.launch.py    # 启动所有相机组
│   ├── multi_camera_select_group.launch.py  # 选择性启动相机组
│   └── single_camera_launcher.launch.py     # 单相机测试启动
├── scripts/
│   └── cleanup_cameras.sh          # 设备清理脚本
└── docs/                           # 详细文档
```

## ⚙️ 配置文件

### camera_groups_config.yaml

```yaml
device_groups:
  fisheye:
    description: "鱼眼相机组"
    devices:
      - device_path: "/dev/video0"
        camera_id: "fisheye0"
        topic_prefix: "fisheye"
        width: 1920
        height: 1080
        publish_rate: 30
      - device_path: "/dev/video1"
        camera_id: "fisheye1"
        topic_prefix: "fisheye"
        width: 1920
        height: 1080
        publish_rate: 30
  
  surround:
    description: "环视相机系统"
    devices:
      - device_path: "/dev/video8"
        camera_id: "surround0"
        topic_prefix: "surround"
        width: 1920
        height: 1500
        publish_rate: 30
      # ... 更多设备配置
  
  front:
    description: "前视相机组"
    devices:
      - device_path: "/dev/video4"
        camera_id: "front0"
        topic_prefix: "front"
        width: 1920
        height: 1080
        publish_rate: 30
```

## 🎯 使用方式

### 方式 1: Launch 文件（推荐）

```bash
# 启动所有相机组（默认配置）
ros2 launch v4l2_camera multi_camera_all_groups.launch.py

# 选择性启动相机组
ros2 launch v4l2_camera multi_camera_select_group.launch.py \
  enabled_groups:=fisheye,surround

# 自定义配置文件
ros2 launch v4l2_camera multi_camera_select_group.launch.py \
  camera_config_file:=/path/to/custom_config.yaml \
  enabled_groups:=front
```

### 方式 2: 单相机测试

```bash
ros2 launch v4l2_camera single_camera_launcher.launch.py \
  arg_camera_dev:=/dev/video0 \
  arg_camera_id:=test_camera \
  arg_publish_topic:=/test_camera
```

### 方式 3: 组件容器（高级用法）

```bash
# 启动容器
ros2 run rclcpp_components component_container_mt --ros-args -r __node:=multi_camera_container

# 加载组件
ros2 component load /multi_camera_container v4l2_camera v4l2_camera::MultiCameraComponent
```

## 📊 参数配置

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `enabled_groups` | string | fisheye,surround,front | 要启动的设备组名称（逗号分隔） |
| `camera_config_file` | string | 默认路径 | YAML配置文件路径 |
| `use_image_transport` | bool | true | 启用image_transport（支持压缩） |
| `convert_to_rgb` | bool | true | 转换为RGB格式（false=原始UYVY） |
| `jpeg_quality` | int | 40/60/30 | JPEG压缩质量（按组配置） |
| `use_intra_process_comms` | bool | true | 启用进程内通信优化 |

## 🖼️ 图像格式和话题

### 话题命名规则
- 鱼眼相机: `/fisheye/image_raw`, `/fisheye/camera_info`
- 环视相机: `/surround/image_raw`, `/surround/camera_info`
- 前视相机: `/front/image_raw`, `/front/camera_info`

### 原始 UYVY 格式 (convert_to_rgb=false)
- **编码**: `yuv422_yuy2`
- **优势**: 保持相机原始格式，CPU负载低，传输效率高
- **大小**: width × height × 2 bytes

### RGB 格式 (convert_to_rgb=true)
- **编码**: `rgb8`  
- **优势**: 直接可视化，广泛兼容
- **大小**: width × height × 3 bytes

### 压缩传输 (use_image_transport=true)
当启用时，每个相机组会发布：
- `/fisheye/image_raw` - 原始图像
- `/fisheye/image_raw/compressed` - JPEG压缩
- `/fisheye/image_raw/theora` - Theora压缩


## ⏱️ 硬件时间戳支持

### 时间戳特性

本项目使用 V4L2 buffer 机制获取相机硬件时间戳，提供高精度的图像采集时间信息：

- **硬件级精度**: 直接从相机驱动获取时间戳，避免软件延迟
- **多相机同步**: 支持多相机间的精确时间同步
- **ROS2 时间兼容**: 自动转换为 ROS2 时间格式
- **时钟校准**: 支持系统时钟偏移校准

### 时间戳验证

#### 检查相机设备时间戳支持

```bash
# 验证设备时间戳功能
v4l2-ctl --stream-mmap --stream-count=3 -d /dev/video0 --verbose


#### 验证ROS2话题时间戳

```bash
# 检查图像消息的时间戳字段
ros2 topic echo /fisheye/image_raw --field header.stamp

# 监控时间戳连续性
ros2 run rqt_plot rqt_plot /fisheye/image_raw/header/stamp/sec
```

#### 多相机时间同步检查

```bash
# 同时监控多个相机的时间戳
ros2 topic echo /fisheye/image_raw --field header.stamp &
ros2 topic echo /surround/image_raw --field header.stamp &
ros2 topic echo /front/image_raw --field header.stamp &

# 使用时间戳同步工具
ros2 run message_filters approximate_sync_example
```


#### 查看系统时钟偏移

```bash
# 检查系统时钟偏移量
cat /sys/devices/system/clocksource/clocksource0/offset_ns

# 手动设置偏移量（如需要）
echo 1000000 | sudo tee /sys/devices/system/clocksource/clocksource0/offset_ns
```



## 🔍 监控和调试

### 系统状态监控

系统运行时会显示详细状态信息：

```
=== Camera Status Update ===
Camera status: 4/4 cameras running
  Camera 0 (/dev/video0): RUNNING -> /fisheye
  Camera 1 (/dev/video1): RUNNING -> /fisheye
  Camera 2 (/dev/video8): RUNNING -> /surround
  Camera 3 (/dev/video9): RUNNING -> /surround

=== Camera Frequency Statistics ===
  /fisheye/image_raw: 30.0 Hz (target: 30.0 Hz) [NORMAL]
  /surround/image_raw: 29.8 Hz (target: 30.0 Hz) [NORMAL]
Active cameras: 4/4 | Total frequency: 119.8 Hz | Efficiency: 99.8%
```

### 常用调试命令

```bash
# 查看组件状态
ros2 component list

# 监控话题
ros2 topic list | grep camera
ros2 topic hz /fisheye/image_raw

# 查看图像信息
ros2 topic echo /fisheye/image_raw --once | head -20

# 检查设备
v4l2-ctl --list-devices
lsof /dev/video*
```

## 🛠️ 故障排除

### 设备忙错误
```bash
# 运行清理脚本
./scripts/cleanup_cameras.sh

# 手动检查占用
lsof /dev/video*

# 重启相机模块
sudo modprobe -r uvcvideo
sudo modprobe uvcvideo
```

### 权限问题
```bash
# 设置设备权限
sudo chmod 666 /dev/video*

# 添加用户到video组
sudo usermod -a -G video $USER
```

### 没有图像输出
```bash
# 验证设备功能
v4l2-ctl -d /dev/video0 --list-formats-ext

# 检查话题数据
ros2 topic echo /fisheye/image_raw --field header
```

## 🚀 性能优化

### 推荐配置
- **高性能**: `convert_to_rgb=false` + `use_image_transport=true`
- **可视化**: `convert_to_rgb=true` + `use_image_transport=true`  
- **低延迟**: `convert_to_rgb=false` + `use_image_transport=false`

### 不同相机组的质量设置
- **鱼眼相机**: `jpeg_quality=40` (平衡质量和性能)
- **前视相机**: `jpeg_quality=60` (高质量，用于检测)
- **环视相机**: `jpeg_quality=30` (低质量，用于监控)

### 性能特点
- ✅ 进程内零拷贝通信
- ✅ 组件化架构提高效率
- ✅ 可选择性启动减少资源占用
- ✅ 多线程执行器提高并发性能

## 🔧 扩展开发

### 添加新相机组
1. 编辑 `config/camera_groups_config.yaml`
2. 在配置文件中添加新的设备组
3. 在启动文件中添加对应的ComposableNode
4. 重启系统

### 自定义相机组件
1. 继承 `CameraLauncherComponent`
2. 重写 `configure()` 和 `start()` 方法
3. 注册到 CMakeLists.txt

## 📚 文档

- [详细使用指南](USAGE_GUIDE.md) - 完整的使用说明
- [快速启动指南](QUICK_START_GUIDE.md) - 快速上手
- [多相机使用文档](MULTI_CAMERA_USAGE.md) - 多相机配置详解
- [原始图像输出说明](README_RAW_IMAGE.md) - 图像格式说明
- [实现总结](IMPLEMENTATION_SUMMARY.md) - 技术实现详情

## 📋 系统要求

- **ROS2**: Humble 或更高版本
- **操作系统**: Ubuntu 20.04/22.04 LTS
- **依赖包**: 
  - `rclcpp`
  - `rclcpp_components`
  - `sensor_msgs`
  - `image_transport`
  - `opencv`
  - `yaml-cpp`

## 📄 许可证

本项目采用 Apache 2.0 许可证。详见 [LICENSE](LICENSE) 文件。

## 🤝 贡献

欢迎提交 Issue 和 Pull Request 来改进这个项目。

## 📞 支持

如遇问题，请：
1. 查看故障排除部分
2. 检查日志输出
3. 提供完整的错误信息和环境配置
4. 创建 GitHub Issue

---

**快速开始命令**:
```bash
# 一键启动所有相机组
cd /home/cyberbus/driver_ws && source install/setup.bash
ros2 launch v4l2_camera multi_camera_all_groups.launch.py

# 选择性启动特定组
ros2 launch v4l2_camera multi_camera_select_group.launch.py enabled_groups:=fisheye,front
```
