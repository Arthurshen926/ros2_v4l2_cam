# Camera Launcher Component 修改总结

## 修改目标

将 Camera Launcher Component 从输出压缩图像修改为输出原始图像，并加入可选的 image_transport 压缩功能。

## 主要修改

### 1. 头文件修改 (`camera_launcher_component.hpp`)

```cpp
// 新增成员变量
bool convert_to_rgb_;  // 控制是否转换为RGB格式

// 修改函数签名，支持可配置编码格式
sensor_msgs::msg::Image cvToRosImage(const cv::Mat& img, const std::string& encoding = "rgb8");

// 新增函数：处理UYVY原始格式
sensor_msgs::msg::Image uyyvToRosImage(const cv::Mat& uyvy_img);
```

### 2. 实现文件修改 (`camera_launcher_component.cpp`)

#### 参数声明
```cpp
// 新增参数：控制是否转换为RGB
this->declare_parameter("convert_to_rgb", false);  // 默认输出原始图像
```

#### 图像处理逻辑 (`processFrame()`)
```cpp
if (convert_to_rgb_) {
    // 转换为RGB格式
    cv::Mat rgb;
    cv::cvtColor(uyvy, rgb, cv::COLOR_YUV2RGB_UYVY);
    img_msg = cvToRosImage(rgb, "rgb8");
} else {
    // 输出原始UYVY格式
    img_msg = uyyvToRosImage(uyvy);
}
```

#### 新增函数实现
```cpp
// 处理UYVY原始格式图像
sensor_msgs::msg::Image CameraLauncherComponent::uyyvToRosImage(const cv::Mat& uyvy_img)
{
    sensor_msgs::msg::Image msg;
    msg.encoding = "yuv422_yuy2";  // 标准ROS图像编码
    msg.step = uyvy_img.cols * 2;  // UYVY格式每像素2字节
    // ... 数据复制逻辑
}
```

## 新增参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `convert_to_rgb` | bool | false | 是否将UYVY转换为RGB格式 |
| `use_image_transport` | bool | true | 是否使用image_transport（已存在） |

## 图像格式对比

### 原始格式 (convert_to_rgb=false)
- **编码**: `yuv422_yuy2`
- **数据大小**: width × height × 2 bytes
- **优势**: 保持原始格式，CPU开销低，传输效率高

### RGB格式 (convert_to_rgb=true)
- **编码**: `rgb8`
- **数据大小**: width × height × 3 bytes
- **优势**: 直接可视化，兼容性好

## 使用示例

### 输出原始UYVY格式（推荐）
```bash
ros2 run v4l2_camera camera_launcher_node --ros-args \
  -p convert_to_rgb:=false \
  -p use_image_transport:=true
```

### 输出RGB格式
```bash
ros2 run v4l2_camera camera_launcher_node --ros-args \
  -p convert_to_rgb:=true \
  -p use_image_transport:=true
```

## image_transport 支持

当 `use_image_transport:=true` 时，自动提供：
- `/camera{id}/image_raw` - 原始数据
- `/camera{id}/image_raw/compressed` - JPEG压缩（需要插件）
- `/camera{id}/image_raw/theora` - Theora压缩（需要插件）

## 性能优势

1. **降低CPU使用率**: 原始格式避免了不必要的颜色空间转换
2. **减少内存占用**: UYVY格式比RGB格式节省33%内存
3. **保持灵活性**: 可选择输出格式，满足不同需求
4. **压缩传输**: 支持image_transport自动压缩，降低网络负载

## 兼容性

- **向后兼容**: 可通过参数选择RGB输出
- **标准格式**: 使用标准ROS图像编码
- **工具支持**: OpenCV、image_proc等都支持YUV格式转换

## 文件列表

### 修改的文件
1. `include/v4l2_camera/camera_launcher_component.hpp`
2. `src/camera_launcher_component.cpp`
3. `package.xml` (修复依赖重复问题)

### 新增的文件
1. `README_RAW_IMAGE.md` - 详细使用说明
2. `scripts/test_raw_image_output.sh` - 测试脚本
3. `launch/raw_image_example.launch.py` - 示例launch文件

## 验证

Camera Launcher Component 可以正常单独编译通过语法检查。现有的 multi_camera_component.cpp 编译错误与此修改无关，是原始代码中的类型不匹配问题。
