# 多相机系统帧率配置和多组启动使用指南

## 🎯 新增功能

### ✅ 已完成的更新

1. **配置文件中的帧率控制**
   - 在 `camera_groups_config.yaml` 中为每个相机组添加了 `frame_rate` 参数
   - 支持针对不同相机组设置不同的帧率

2. **多相机组同时启动**
   - 创建了 `multi_camera_all_groups.launch.py` 支持同时启动多个相机组
   - 创建了便捷的启动脚本 `launch_camera_groups.sh`

3. **优化的帧率实现**
   - 摄像头组件自动根据配置的帧率调整发布频率
   - 显示实际的帧率配置信息

## 📁 配置文件格式

### 更新后的 `camera_groups_config.yaml`

```yaml
camera_groups:
  fisheye:
    devices:
      - "/dev/video0"
      - "/dev/video1"
      - "/dev/video2"
      - "/dev/video3"
    topic_prefix: "/fisheye"
    width: 1280
    height: 1024
    frame_rate: 10  # 新增：帧率配置

  fisheye_hd:
    devices:
      - "/dev/video0"
      - "/dev/video1"
      - "/dev/video2"
      - "/dev/video3"
    topic_prefix: "/fisheye"
    width: 1920
    height: 1500
    frame_rate: 8   # 高分辨率使用更低帧率

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
    frame_rate: 5   # 环视摄像头低帧率

  front:
    devices:
      - "/dev/video5"
    topic_prefix: "/front"
    width: 1920
    height: 1500
    frame_rate: 15  # 前视摄像头较高帧率
```

## 🚀 启动方式

### 方法1: 使用便捷脚本 (推荐)

```bash
# 基本使用
./src/ros2_v4l2_camera/scripts/launch_camera_groups.sh <相机组名>

# 启动单个相机组
./src/ros2_v4l2_camera/scripts/launch_camera_groups.sh fisheye
./src/ros2_v4l2_camera/scripts/launch_camera_groups.sh fisheye_hd
./src/ros2_v4l2_camera/scripts/launch_camera_groups.sh surround
./src/ros2_v4l2_camera/scripts/launch_camera_groups.sh front

# 启动所有相机组
./src/ros2_v4l2_camera/scripts/launch_camera_groups.sh all

# 带参数启动
./src/ros2_v4l2_camera/scripts/launch_camera_groups.sh fisheye jpeg_quality:=60
./src/ros2_v4l2_camera/scripts/launch_camera_groups.sh all convert_to_rgb:=false
```

### 方法2: 直接使用launch文件

```bash
# 启动单个相机组
ros2 launch src/ros2_v4l2_camera/launch/multi_camera_optimized.launch.py \
    device_group:=fisheye jpeg_quality:=40 convert_to_rgb:=true

# 启动多个相机组
ros2 launch src/ros2_v4l2_camera/launch/multi_camera_all_groups.launch.py \
    enabled_groups:=fisheye,front jpeg_quality:=40
```

## 📊 相机组配置总览

| 相机组 | 分辨率 | 帧率 | 摄像头数量 | 带宽预估 |
|--------|--------|------|------------|----------|
| fisheye | 1280x1024 | 10fps | 4 | ~135MB/s |
| fisheye_hd | 1920x1500 | 8fps | 4 | ~320MB/s |
| surround | 1920x1500 | 5fps | 6 | ~300MB/s |
| front | 1920x1500 | 15fps | 1 | ~120MB/s |

## 🎛️ 可用参数

### 全局参数
- `jpeg_quality` (1-100): JPEG压缩质量，默认40
- `convert_to_rgb` (true/false): 是否转换为RGB格式，默认true
- `use_image_transport` (true/false): 是否使用image_transport，默认true

### 相机组参数
- `device_group`: 要启动的相机组名称
- `enabled_groups`: 要启动的多个相机组，用逗号分隔

## 🔧 帧率工作原理

1. **配置读取**: 从YAML文件中读取每个相机组的 `frame_rate` 设置
2. **发布控制**: 根据配置的帧率计算 `publish_every_n` 参数
   - 例如：30fps基础频率，设置10fps → `publish_every_n = 3`
3. **实际输出**: 摄像头以设置的帧率发布图像消息

## 📈 性能监控

### 检查话题列表
```bash
ros2 topic list | grep image
```

### 测量带宽
```bash
# 原始图像带宽
ros2 topic bw /fisheye0/image_raw

# 压缩图像带宽  
ros2 topic bw /fisheye0/image_raw/compressed
```

### 检查帧率
```bash
ros2 topic hz /fisheye0/image_raw
```

## 🔍 故障排除

### 常见问题

1. **设备权限错误**
   ```bash
   sudo chmod 666 /dev/video*
   ```

2. **设备占用**
   ```bash
   sudo lsof /dev/video*
   pkill -f camera
   ```

3. **查看日志**
   ```bash
   ros2 node list
   ros2 log info /multi_camera_manager
   ```

## 📝 配置建议

### 带宽优化场景
- 使用 `fisheye` 配置 (低分辨率 + 10fps)
- 启用RGB转换和压缩: `convert_to_rgb:=true`
- 调整JPEG质量: `jpeg_quality:=30-50`

### 高质量场景  
- 使用 `fisheye_hd` 或 `front` 配置
- 较高的JPEG质量: `jpeg_quality:=70-90`
- 根据需要调整帧率

### 多组场景
- 谨慎选择同时启动的相机组
- 监控系统资源使用情况
- 考虑网络带宽限制

## 🎉 功能特性

✅ **可配置帧率**: 每个相机组独立的帧率设置  
✅ **多组启动**: 同时运行多个相机组  
✅ **便捷脚本**: 简化的启动和管理方式  
✅ **带宽优化**: RGB转换 + JPEG压缩  
✅ **实时监控**: 完整的性能监控工具  
✅ **灵活配置**: 丰富的参数选项
