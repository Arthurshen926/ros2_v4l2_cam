// camera_launcher_component.cpp
// ROS2 camera launcher component implementation
#include "v4l2_camera/camera_launcher_component.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace {
const int DEFAULT_BUFFERS = 4;
} // namespace

namespace v4l2_camera
{

CameraLauncherComponent::CameraLauncherComponent(const rclcpp::NodeOptions & options)
: Node("camera_launcher_component", options),
  fd_(-1),
  width_(1920),
  height_(1500),
  sys_offset_ns_(0),
  epoch_offset_ns_(0),
  publish_every_n_(3),
  frame_count_(0),
  running_(false),
  shutdown_requested_(false)
{
    RCLCPP_INFO(this->get_logger(), "Camera Launcher Component initialized");
    
    // Declare parameters with defaults
    this->declare_parameter("arg_camera_dev", std::string("/dev/video"));
    this->declare_parameter("arg_camera_id", std::string("0"));
    this->declare_parameter("arg_publish_topic", std::string("/camera"));
    this->declare_parameter("arg_resolution_width", 1920);
    this->declare_parameter("arg_resolution_height", 1500);
    this->declare_parameter("use_image_transport", true);
    this->declare_parameter("convert_to_rgb", false);  // 默认输出原始图像
    this->declare_parameter("frame_rate", 30);  // 添加帧率参数，默认30fps
    
    try {
        // Initialize image transport parameters
        use_image_transport_ = this->get_parameter("use_image_transport").as_bool();
        convert_to_rgb_ = this->get_parameter("convert_to_rgb").as_bool();
        int frame_rate = this->get_parameter("frame_rate").as_int();
        
        // 计算每帧发布间隔
        publish_every_n_ = std::max(1, 30 / frame_rate);  // 假设基础频率为30fps
        
        RCLCPP_INFO(this->get_logger(), 
                   "Camera parameters: use_image_transport=%s, convert_to_rgb=%s, frame_rate=%d fps (publish_every_n=%d)",
                   use_image_transport_ ? "true" : "false", 
                   convert_to_rgb_ ? "true" : "false",
                   frame_rate, publish_every_n_);
        
        // Note: image_transport will be initialized later in start() method
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error initializing camera parameters: %s", e.what());
        // Use default values
        use_image_transport_ = true;
        convert_to_rgb_ = false;
        publish_every_n_ = 3;  // 默认10fps (30/3)
        // Note: image_transport will be initialized later in start() method
    }
    
    // Get epoch offset
    epoch_offset_ns_ = getEpochOffsetNs();
    
    // Read system offset if available
    int64_t sys_offset_ns = 0;
    std::ifstream fin("/sys/devices/system/clocksource/clocksource0/offset_ns");
    if (fin && (fin >> sys_offset_ns)) {
        RCLCPP_INFO(this->get_logger(), "Read sys offset_ns = %ld", sys_offset_ns);
    } else {
        RCLCPP_WARN(this->get_logger(), "offset_ns not found, using zero");
    }
    sys_offset_ns_ = sys_offset_ns;
}

CameraLauncherComponent::~CameraLauncherComponent()
{
    stop();
}

void CameraLauncherComponent::configure(const std::string& camera_dev, 
                                       const std::string& camera_id,
                                       const std::string& pub_topic,
                                       int width, int height)
{
    if (running_) {
        RCLCPP_WARN(this->get_logger(), "Cannot configure while camera is running");
        return;
    }
    
    pub_topic_ = pub_topic;
    camera_id_ = camera_id;
    width_ = width;
    height_ = height;
    
    // Initialize image_transport if needed and not already initialized
    // Note: This should only be called after the object is fully constructed and managed by shared_ptr
    if (use_image_transport_ && !image_transport_) {
        try {
            image_transport_ = std::make_unique<image_transport::ImageTransport>(shared_from_this());
            it_pub_ = image_transport_->advertise(pub_topic_ + "/image_raw", 1);
            RCLCPP_INFO(this->get_logger(), "Image transport initialized for topic: %s/image_raw", pub_topic_.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize image transport: %s", e.what());
            RCLCPP_WARN(this->get_logger(), "Falling back to standard ROS2 publisher");
            use_image_transport_ = false;
        }
    }
    
    // Create publishers
    // 检查image_transport和RGB转换的兼容性
    if (use_image_transport_ && !convert_to_rgb_) {
        RCLCPP_WARN(this->get_logger(), 
                   "image_transport compression works best with RGB format. "
                   "Consider setting convert_to_rgb=true for better compression.");
    }
    
    // Create image publisher if not using image_transport  
    if (!use_image_transport_) {
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(pub_topic_ + "/image_raw", 1);
        RCLCPP_INFO(this->get_logger(), "Standard ROS2 publisher created for topic: %s/image_raw", pub_topic_.c_str());
    }
    
    RCLCPP_INFO(this->get_logger(), "Camera configured: %s%s -> %s (w=%d, h=%d)", 
               camera_dev.c_str(), camera_id.c_str(), pub_topic.c_str(), width, height);
}

bool CameraLauncherComponent::start()
{
    if (running_) {
        RCLCPP_WARN(this->get_logger(), "Camera already running");
        return true;
    }
    
    std::string camera_dev = this->get_parameter("arg_camera_dev").as_string();
    std::string camera_id = this->get_parameter("arg_camera_id").as_string();
    std::string pub_topic = this->get_parameter("arg_publish_topic").as_string();
    int width = this->get_parameter("arg_resolution_width").as_int();
    int height = this->get_parameter("arg_resolution_height").as_int();
    
    configure(camera_dev, camera_id, pub_topic, width, height);
    
    // 如果camera_dev已经是完整路径（如/dev/video0），直接使用
    // 否则拼接camera_dev + camera_id
    std::string device_path;
    if (camera_id.empty() || camera_dev.find("/dev/video") == 0) {
        device_path = camera_dev;  // 已经是完整路径
    } else {
        device_path = camera_dev + camera_id;  // 需要拼接
    }
    
    if (!openCamera(device_path)) {
        return false;
    }
    
    if (!setupFormat()) {
        closeCamera();
        return false;
    }
    
    if (!setupBuffers()) {
        closeCamera();
        return false;
    }
    
    if (!startStreaming()) {
        closeCamera();
        return false;
    }
    
    running_ = true;
    shutdown_requested_ = false;
    
    // Start capture thread
    capture_thread_ = std::thread(&CameraLauncherComponent::captureLoop, this);
    
    RCLCPP_INFO(this->get_logger(), "Camera started successfully: %s", device_path.c_str());
    return true;
}

void CameraLauncherComponent::stop()
{
    if (!running_) {
        return;
    }
    
    shutdown_requested_ = true;
    
    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }
    
    stopStreaming();
    closeCamera();
    running_ = false;
    
    RCLCPP_INFO(this->get_logger(), "Camera stopped");
}

bool CameraLauncherComponent::openCamera(const std::string& device_path)
{
    // 检查设备是否存在
    if (access(device_path.c_str(), F_OK) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Camera device does not exist: %s", device_path.c_str());
        return false;
    }
    
    // 检查设备权限
    if (access(device_path.c_str(), R_OK | W_OK) != 0) {
        RCLCPP_ERROR(this->get_logger(), "No permission to access device: %s", device_path.c_str());
        return false;
    }
    
    // 尝试打开设备
    fd_ = ::open(device_path.c_str(), O_RDWR | O_NONBLOCK, 0);
    if (fd_ < 0) {
        if (errno == EBUSY) {
            RCLCPP_ERROR(this->get_logger(), "Camera device is busy: %s. "
                        "Please close other applications using this camera.", device_path.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Cannot open %s: %s", 
                        device_path.c_str(), strerror(errno));
        }
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Successfully opened camera device: %s", device_path.c_str());
    return true;
}

bool CameraLauncherComponent::setupFormat()
{
    v4l2_format fmt = {};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width_;
    fmt.fmt.pix.height = height_;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    
    if (xioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
        RCLCPP_FATAL(this->get_logger(), "VIDIOC_S_FMT failed: %s", strerror(errno));
        return false;
    }
    
    if (xioctl(fd_, VIDIOC_G_FMT, &fmt) < 0) {
        RCLCPP_WARN(this->get_logger(), "VIDIOC_G_FMT failed: %s", strerror(errno));
    } else {
        char fourcc[] = {(char)(fmt.fmt.pix.pixelformat & 0xFF),
                         (char)((fmt.fmt.pix.pixelformat >> 8) & 0xFF),
                         (char)((fmt.fmt.pix.pixelformat >> 16) & 0xFF),
                         (char)((fmt.fmt.pix.pixelformat >> 24) & 0xFF), 0};
        RCLCPP_INFO(this->get_logger(), "Driver accepted format: %s (%dx%d)", 
                   fourcc, fmt.fmt.pix.width, fmt.fmt.pix.height);
    }
    
    return true;
}

bool CameraLauncherComponent::setupBuffers()
{
    v4l2_requestbuffers req = {};
    req.count = DEFAULT_BUFFERS;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    
    if (xioctl(fd_, VIDIOC_REQBUFS, &req) < 0) {
        RCLCPP_FATAL(this->get_logger(), "VIDIOC_REQBUFS failed: %s", strerror(errno));
        return false;
    }
    
    bufs_.resize(req.count);
    for (uint32_t i = 0; i < req.count; ++i) {
        v4l2_buffer buf = {};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        xioctl(fd_, VIDIOC_QUERYBUF, &buf);
        bufs_[i].length = buf.length;
        bufs_[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, 
                             MAP_SHARED, fd_, buf.m.offset);
        xioctl(fd_, VIDIOC_QBUF, &buf);
    }
    
    return true;
}

bool CameraLauncherComponent::startStreaming()
{
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
        RCLCPP_FATAL(this->get_logger(), "VIDIOC_STREAMON failed: %s", strerror(errno));
        return false;
    }
    return true;
}

void CameraLauncherComponent::stopStreaming()
{
    if (fd_ >= 0) {
        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        xioctl(fd_, VIDIOC_STREAMOFF, &type);
        
        for (auto& b : bufs_) {
            if (b.start != MAP_FAILED) {
                munmap(b.start, b.length);
            }
        }
        bufs_.clear();
    }
}

void CameraLauncherComponent::closeCamera()
{
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

void CameraLauncherComponent::captureLoop()
{
    while (rclcpp::ok() && running_ && !shutdown_requested_) {
        processFrame();
        std::this_thread::sleep_for(std::chrono::milliseconds(33)); // ~30 FPS
    }
}

void CameraLauncherComponent::processFrame()
{
    v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    
    if (xioctl(fd_, VIDIOC_DQBUF, &buf) == 0) {
        frame_count_++;
        if (frame_count_ % publish_every_n_ == 0) {
            frame_count_ = 0;
            
            int64_t mono_ns = buf.timestamp.tv_sec * 1000000000LL + 
                             buf.timestamp.tv_usec * 1000LL - sys_offset_ns_;
            rclcpp::Time stamp(mono_ns);
            
            cv::Mat uyvy(height_, width_, CV_8UC2, bufs_[buf.index].start);
            sensor_msgs::msg::Image img_msg;
            
            if (convert_to_rgb_) {
                // 转换为RGB格式
                cv::Mat rgb;
                cv::cvtColor(uyvy, rgb, cv::COLOR_YUV2RGB_UYVY);
                img_msg = cvToRosImage(rgb, "rgb8");
            } else {
                // 输出原始UYVY格式
                img_msg = uyyvToRosImage(uyvy);
            }
            
            img_msg.header.stamp = stamp;
            img_msg.header.frame_id = pub_topic_ + camera_id_;
            
            if (use_image_transport_) {
                it_pub_.publish(img_msg);
            } else {
                image_pub_->publish(img_msg);
            }
        }
        xioctl(fd_, VIDIOC_QBUF, &buf);
    }
}

int CameraLauncherComponent::xioctl(int fd, unsigned long request, void* arg)
{
    int r;
    do {
        r = ::ioctl(fd, request, arg);
    } while (r == -1 && errno == EINTR);
    return r;
}

sensor_msgs::msg::Image CameraLauncherComponent::cvToRosImage(const cv::Mat& img, const std::string& encoding)
{
    sensor_msgs::msg::Image msg;
    msg.header = std_msgs::msg::Header();
    msg.height = img.rows;
    msg.width = img.cols;
    msg.encoding = encoding;
    msg.is_bigendian = false;
    msg.step = img.cols * img.channels();
    
    size_t size = msg.step * img.rows;
    msg.data.resize(size);
    
    if (img.isContinuous()) {
        memcpy(msg.data.data(), img.data, size);
    } else {
        for (int i = 0; i < img.rows; ++i) {
            memcpy(msg.data.data() + i * msg.step, img.ptr(i), msg.step);
        }
    }
    
    return msg;
}

sensor_msgs::msg::Image CameraLauncherComponent::uyyvToRosImage(const cv::Mat& uyvy_img)
{
    sensor_msgs::msg::Image msg;
    msg.header = std_msgs::msg::Header();
    msg.height = uyvy_img.rows;
    msg.width = uyvy_img.cols;
    msg.encoding = sensor_msgs::image_encodings::YUV422;  // 使用标准的YUV422编码
    msg.is_bigendian = false;
    msg.step = uyvy_img.cols * 2;  // UYVY格式每像素2字节
    
    size_t size = msg.step * uyvy_img.rows;
    msg.data.resize(size);
    
    if (uyvy_img.isContinuous()) {
        memcpy(msg.data.data(), uyvy_img.data, size);
    } else {
        for (int i = 0; i < uyvy_img.rows; ++i) {
            memcpy(msg.data.data() + i * msg.step, uyvy_img.ptr(i), msg.step);
        }
    }
    
    return msg;
}

int64_t CameraLauncherComponent::getEpochOffsetNs()
{
    struct timespec mono{}, real{};
    clock_gettime(CLOCK_MONOTONIC, &mono);
    clock_gettime(CLOCK_REALTIME, &real);
    int64_t mono_ns = mono.tv_sec * 1000000000LL + mono.tv_nsec;
    int64_t real_ns = real.tv_sec * 1000000000LL + real.tv_nsec;
    return real_ns - mono_ns;
}

} // namespace v4l2_camera

RCLCPP_COMPONENTS_REGISTER_NODE(v4l2_camera::CameraLauncherComponent)
