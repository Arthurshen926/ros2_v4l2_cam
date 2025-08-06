# Multi-Camera Standalone 使用指南

## 概述

Multi-Camera Standalone 是一个独立的多相机管理系统，支持：
- 使用 `camera_groups_config.yaml` 配置多个相机
- image_transport 支持（包括压缩功能）
- 原始图像格式输出（UYVY）或RGB转换
- 频率监控和状态报告
- 进程内通信优化

## 配置文件格式

### camera_groups_config.yaml 示例

```yaml
# camera_groups_config.yaml
# 自定义相机设备组配置文件

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
    
  front:
    devices:
      - "/dev/video5"
    topic_prefix: "/front"
    width: 1920
    height: 1500

# 全局设置
global_settings:
  container_name: "multi_camera_container"
  use_intra_process_comms: true
  publish_rate: 30
```

## 启动方式

### 1. 使用 Launch 文件（推荐）

```bash
# 启动 fisheye 组相机（默认）
ros2 launch v4l2_camera multi_camera_standalone.launch.py

# 启动 surround 组相机
ros2 launch v4l2_camera multi_camera_standalone.launch.py device_group:=surround

# 启动 front 组相机，输出RGB格式
ros2 launch v4l2_camera multi_camera_standalone.launch.py \
  device_group:=front \
  convert_to_rgb:=true

# 禁用 image_transport
ros2 launch v4l2_camera multi_camera_standalone.launch.py \
  use_image_transport:=false

# 使用自定义配置文件
ros2 launch v4l2_camera multi_camera_standalone.launch.py \
  config_file:=/path/to/your/config.yaml \
  device_group:=your_group
```

### 2. 直接运行节点

```bash
# 基本运行
ros2 run v4l2_camera multi_camera_standalone

# 使用参数运行
ros2 run v4l2_camera multi_camera_standalone --ros-args \
  -p camera_config_file:=/home/cyberbus/driver_ws/src/ros2_v4l2_camera/config/camera_groups_config.yaml \
  -p device_group:=fisheye \
  -p use_image_transport:=true \
  -p convert_to_rgb:=false \
  -p frequency_monitor_enabled:=true
```

## 参数说明

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `camera_config_file` | string | 预设路径 | 相机组配置文件路径 |
| `device_group` | string | fisheye | 要启动的设备组名称 |
| `use_image_transport` | bool | true | 是否使用image_transport（支持压缩） |
| `convert_to_rgb` | bool | false | 是否转换为RGB格式（false=输出原始UYVY） |
| `frequency_monitor_enabled` | bool | true | 是否启用频率监控 |
| `frequency_monitor_interval` | double | 2.0 | 频率监控报告间隔（秒） |

## 输出话题

每个相机会创建以下话题：

### 使用 image_transport (默认)
- `/{topic_prefix}{id}/image_raw` - 原始图像数据
- `/{topic_prefix}{id}/image_raw/compressed` - JPEG压缩图像
- `/{topic_prefix}{id}/image_raw/theora` - Theora压缩（需要插件）

### 示例（fisheye组）
- `/fisheye0/image_raw`
- `/fisheye0/image_raw/compressed`
- `/fisheye1/image_raw`
- `/fisheye1/image_raw/compressed`
- `/fisheye2/image_raw`
- `/fisheye2/image_raw/compressed`
- `/fisheye3/image_raw`
- `/fisheye3/image_raw/compressed`

## 监控和调试

### 1. 查看话题列表
```bash
ros2 topic list | grep fisheye
```

### 2. 查看话题信息
```bash
ros2 topic info /fisheye0/image_raw
```

### 3. 监控图像频率
```bash
ros2 topic hz /fisheye0/image_raw
```

### 4. 查看图像内容
```bash
# 查看图像信息
ros2 topic echo /fisheye0/image_raw --once | head -20

# 使用 rviz2 可视化
rviz2
```

### 5. 压缩图像转换
```bash
# 将压缩图像转换为原始图像用于可视化
ros2 run image_transport republish compressed in/compressed:=/fisheye0/image_raw/compressed raw out:=/fisheye0/image_uncompressed
```

## 故障排除

### 1. 相机设备不存在
```
ERROR: Camera device does not exist: /dev/video0
```
**解决方案**: 检查相机连接和设备路径

### 2. 权限问题
```
ERROR: No permission to access device: /dev/video0
```
**解决方案**: 
```bash
sudo chmod 666 /dev/video*
# 或者将用户添加到video组
sudo usermod -a -G video $USER
```

### 3. 设备忙碌
```
ERROR: Camera device is busy: /dev/video0
```
**解决方案**: 关闭其他使用相机的应用程序

### 4. 压缩功能不可用
**解决方案**: 安装image_transport插件
```bash
sudo apt install ros-foxy-image-transport-plugins
# 或从源码编译
```

### 5. 图像格式不兼容
**解决方案**: 使用image_proc进行格式转换
```bash
ros2 run image_proc image_proc --ros-args \
  --remap image:=/fisheye0/image_raw \
  --remap image_color:=/fisheye0/image_rgb
```

## 性能优化

### 1. 原始格式输出（推荐）
```bash
ros2 launch v4l2_camera multi_camera_standalone.launch.py convert_to_rgb:=false
```

### 2. 使用压缩传输
压缩功能自动可用，无需额外配置

### 3. 调整发布频率
在配置文件中修改 `global_settings.publish_rate`

### 4. 禁用频率监控（减少日志输出）
```bash
ros2 launch v4l2_camera multi_camera_standalone.launch.py frequency_monitor_enabled:=false
```

## 配置示例

### 单相机测试
```yaml
camera_groups:
  test:
    devices:
      - "/dev/video0"
    topic_prefix: "/test_camera"
    width: 640
    height: 480
```

### 高分辨率配置
```yaml
camera_groups:
  hd:
    devices:
      - "/dev/video0"
      - "/dev/video1"
    topic_prefix: "/hd_camera"
    width: 1920
    height: 1080
```

### 多组配置
```yaml
camera_groups:
  front_cameras:
    devices: ["/dev/video0", "/dev/video1"]
    topic_prefix: "/front"
    width: 1920
    height: 1500
    
  rear_cameras:
    devices: ["/dev/video2", "/dev/video3"]
    topic_prefix: "/rear"
    width: 1920
    height: 1500
```
