# Multi-Camera Standalone 快速启动指南

## 系统概述

Multi-Camera Standalone 是一个基于ROS2的多相机管理系统，支持：
- ✅ 读取 YAML 配置文件
- ✅ 多相机组管理（fisheye、surround、front）
- ✅ image_transport 支持（原始图像 + 压缩传输）
- ✅ 原始UYVY格式输出
- ✅ 进程内通信优化
- ✅ 实时频率监控

## 快速启动

### 1. 编译系统
```bash
cd /home/cyberbus/driver_ws
colcon build --packages-select v4l2_camera --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 2. 启动相机系统

#### 方式A: 使用launch文件（推荐）
```bash
# 启动fisheye组相机（4个相机）
ros2 launch v4l2_camera multi_camera_standalone.launch.py device_group:=fisheye

# 启动surround组相机（6个相机）
ros2 launch v4l2_camera multi_camera_standalone.launch.py device_group:=surround

# 启动front组相机（1个相机）
ros2 launch v4l2_camera multi_camera_standalone.launch.py device_group:=front
```

#### 方式B: 直接运行
```bash
ros2 run v4l2_camera multi_camera_standalone --ros-args \
  -p camera_config_file:=/home/cyberbus/driver_ws/src/ros2_v4l2_camera/config/camera_groups_config.yaml \
  -p device_group:=fisheye \
  -p frequency_monitor_enabled:=true
```

### 3. 验证系统运行

在另一个终端中：
```bash
# 查看所有话题
ros2 topic list

# 查看图像话题
ros2 topic list | grep image

# 查看特定话题信息
ros2 topic info /fisheye0/image_raw

# 查看话题数据
ros2 topic echo /fisheye0/image_raw --once | head -20
```

## 配置说明

### 相机组配置
- **fisheye**: 4个相机 (`/dev/video0-3`) → 话题 `/fisheye0-3/image_raw`
- **surround**: 6个相机 (`/dev/video8-13`) → 话题 `/surround0-5/image_raw`
- **front**: 1个相机 (`/dev/video5`) → 话题 `/front0/image_raw`

### image_transport 功能
启用 `use_image_transport:=true` 时，每个相机提供：
- `/fisheye0/image_raw` - 原始UYVY图像
- `/fisheye0/image_raw/compressed` - JPEG压缩图像（需要插件）
- `/fisheye0/image_raw/theora` - Theora压缩（需要插件）

### 系统参数
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `device_group` | fisheye | 相机组名称 |
| `use_image_transport` | true | 启用image_transport |
| `convert_to_rgb` | false | 输出原始UYVY格式 |
| `frequency_monitor_enabled` | true | 启用频率监控 |
| `frequency_monitor_interval` | 2.0 | 监控间隔（秒） |

## 系统监控

启动后系统会显示：
```
=== Camera Status Update ===
Camera status: 4/4 cameras running
  Camera 0 (/dev/video0): RUNNING -> /fisheye0
  Camera 1 (/dev/video1): RUNNING -> /fisheye1
  Camera 2 (/dev/video2): RUNNING -> /fisheye2
  Camera 3 (/dev/video3): RUNNING -> /fisheye3

=== Camera Frequency Statistics ===
  /fisheye0/image_raw: 30.0 Hz (target: 30.0 Hz) [NORMAL]
  /fisheye1/image_raw: 29.8 Hz (target: 30.0 Hz) [NORMAL]
  /fisheye2/image_raw: 30.2 Hz (target: 30.0 Hz) [NORMAL]
  /fisheye3/image_raw: 30.1 Hz (target: 30.0 Hz) [NORMAL]
Active cameras: 4/4 | Total frequency: 120.1 Hz | Efficiency: 100.1%
```

## 压缩传输

要启用压缩传输，需要安装 image_transport 插件：
```bash
sudo apt install ros-humble-image-transport-plugins
# 或从源码编译
git clone https://github.com/ros-perception/image_transport_plugins.git
```

## 故障排除

### 设备权限问题
```bash
sudo chmod 666 /dev/video*
```

### 设备被占用
```bash
lsof /dev/video*
```

### 查看详细日志
系统会在终端输出详细的状态信息和错误日志。

## 自定义配置

编辑配置文件 `/home/cyberbus/driver_ws/src/ros2_v4l2_camera/config/camera_groups_config.yaml`：
```yaml
camera_groups:
  custom_group:
    devices:
      - "/dev/video0"
      - "/dev/video1"
    topic_prefix: "/custom"
    width: 1920
    height: 1500
```

然后启动：
```bash
ros2 launch v4l2_camera multi_camera_standalone.launch.py device_group:=custom_group
```

## 性能优化

- ✅ 使用原始UYVY格式减少CPU负载
- ✅ 进程内通信减少延迟
- ✅ image_transport压缩减少网络带宽
- ✅ 多线程执行器提高并发性能

系统已经过全面测试，可以在生产环境中使用！
