# Camera Launcher Component 原始图像输出说明

## 概述

Camera Launcher Component 现在支持输出原始图像格式，并保持对 image_transport 压缩功能的可选支持。

## 新增参数

### `convert_to_rgb` (bool, 默认: false)
- `false`: 输出原始 UYVY 格式图像 (yuv422_yuy2 编码)
- `true`: 输出 RGB 转换后的图像 (rgb8 编码)

### `use_image_transport` (bool, 默认: true)
- `true`: 使用 image_transport 发布器，支持自动压缩功能
- `false`: 使用标准 ROS2 发布器，仅支持原始图像传输

## 图像格式

### 原始格式 (convert_to_rgb=false)
- **编码**: `yuv422_yuy2`
- **格式**: UYVY 4:2:2
- **优势**: 
  - 保持相机原始数据格式
  - 减少CPU计算负载
  - 更高的传输效率
  - 适合后续处理流水线

### RGB格式 (convert_to_rgb=true)
- **编码**: `rgb8`
- **格式**: RGB 8-bit per channel
- **优势**:
  - 直接可视化
  - 兼容大多数图像处理工具
  - 标准格式

## image_transport 压缩支持

当 `use_image_transport=true` 时，自动提供以下话题：

- `/camera{id}/image_raw` - 原始图像数据
- `/camera{id}/image_raw/compressed` - JPEG 压缩图像
- `/camera{id}/image_raw/compressedDepth` - 深度图压缩（如适用）
- `/camera{id}/image_raw/theora` - Theora 视频压缩

**注意**: 压缩功能需要安装 `image_transport_plugins` 包。

## 使用示例

### 1. 输出原始UYVY格式（推荐）
```bash
ros2 run v4l2_camera camera_launcher_node --ros-args \
  -p arg_camera_dev:=/dev/video0 \
  -p convert_to_rgb:=false \
  -p use_image_transport:=true
```

### 2. 输出RGB格式
```bash
ros2 run v4l2_camera camera_launcher_node --ros-args \
  -p arg_camera_dev:=/dev/video0 \
  -p convert_to_rgb:=true \
  -p use_image_transport:=true
```

### 3. 仅使用标准ROS2发布器
```bash
ros2 run v4l2_camera camera_launcher_node --ros-args \
  -p arg_camera_dev:=/dev/video0 \
  -p convert_to_rgb:=false \
  -p use_image_transport:=false
```

## 性能对比

| 模式 | CPU使用率 | 内存使用率 | 网络带宽 | 兼容性 |
|------|-----------|------------|----------|---------|
| 原始UYVY | 低 | 低 | 中等 | 需要YUV支持 |
| RGB转换 | 高 | 高 | 高 | 广泛兼容 |
| 压缩传输 | 中等 | 中等 | 低 | 需要解压缩 |

## 建议配置

### 高性能应用
- `convert_to_rgb: false`
- `use_image_transport: true`
- 使用压缩传输以减少网络负载

### 简单可视化
- `convert_to_rgb: true`
- `use_image_transport: true`

### 低延迟应用
- `convert_to_rgb: false`
- `use_image_transport: false`

## 兼容性说明

### UYVY格式支持
大多数现代图像处理库都支持YUV格式：
- OpenCV: `cv::COLOR_YUV2RGB_UYVY`
- ROS image_proc: 内置支持
- FFmpeg: 原生支持

### image_transport插件
确保安装以下包以获得完整的压缩支持：
```bash
sudo apt install ros-humble-image-transport-plugins
# 或者从源码编译
git clone https://github.com/ros-perception/image_transport_plugins.git
```

## 故障排除

### 图像格式错误
如果遇到格式不兼容问题，可以使用 `image_proc` 进行格式转换：
```bash
ros2 run image_proc image_proc --ros-args \
  --remap image:=/camera/image_raw \
  --remap image_color:=/camera/image_rgb
```

### 压缩功能不可用
检查是否安装了压缩插件：
```bash
ros2 pkg list | grep image_transport
```

确保相关插件在运行时可用：
```bash
ros2 run image_transport list_transports
```
