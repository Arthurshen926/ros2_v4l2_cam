// camera_launcher_node.cpp
// ROS2 version of the original camera launcher
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/header.hpp>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <cerrno>
#include <cstdio>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <string>

namespace {
const int DEFAULT_BUFFERS = 4;
int xioctl(int fd, unsigned long request, void* arg) {
    int r;
    do {
        r = ::ioctl(fd, request, arg);
    } while (r == -1 && errno == EINTR);
    return r;
}

sensor_msgs::msg::CompressedImage cvToRos(const cv::Mat& img) {
    sensor_msgs::msg::CompressedImage msg;
    msg.header = std_msgs::msg::Header();
    msg.format = "bgr8; jpeg compressed ";
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 90,
                               cv::IMWRITE_JPEG_PROGRESSIVE, 0,
                               cv::IMWRITE_JPEG_OPTIMIZE, 0};
    cv::imencode(".jpg", img, msg.data, params);
    return msg;
}

int64_t getEpochOffsetNs() {
    struct timespec mono{}, real{};
    clock_gettime(CLOCK_MONOTONIC, &mono);
    clock_gettime(CLOCK_REALTIME, &real);
    int64_t mono_ns = mono.tv_sec * 1000000000LL + mono.tv_nsec;
    int64_t real_ns = real.tv_sec * 1000000000LL + real.tv_nsec;
    return real_ns - mono_ns;
}
} // namespace

class CameraLauncherNode : public rclcpp::Node {
public:
    CameraLauncherNode() : Node("multi_camera_driver") {
        this->declare_parameter("arg_camera_dev", std::string("/dev/video0"));
        this->declare_parameter("arg_camera_id", std::string("0"));
        this->declare_parameter("arg_publish_topic", std::string("/camera"));
        this->declare_parameter("arg_resolution_width", 1920);
        this->declare_parameter("arg_resolution_height", 1500);

        std::string camera_dev = this->get_parameter("arg_camera_dev").as_string();
        std::string camera_id = this->get_parameter("arg_camera_id").as_string();
        std::string pub_topic = this->get_parameter("arg_publish_topic").as_string();
        int width = this->get_parameter("arg_resolution_width").as_int();
        int height = this->get_parameter("arg_resolution_height").as_int();

        pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
            pub_topic + camera_id + "/compressed", 1);

        std::string dev = camera_dev + camera_id;
        fd_ = ::open(dev.c_str(), O_RDWR | O_NONBLOCK, 0);
        if (fd_ < 0) {
            RCLCPP_FATAL(this->get_logger(), "Cannot open %s: %s", dev.c_str(), strerror(errno));
            throw std::runtime_error("Failed to open camera device");
        }

        v4l2_format fmt = {};
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = width;
        fmt.fmt.pix.height = height;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;
        if (xioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
            RCLCPP_FATAL(this->get_logger(), "VIDIOC_S_FMT failed: %s", strerror(errno));
            throw std::runtime_error("Failed to set format");
        }
        if (xioctl(fd_, VIDIOC_G_FMT, &fmt) < 0) {
            RCLCPP_WARN(this->get_logger(), "VIDIOC_G_FMT failed: %s", strerror(errno));
        } else {
            char fourcc[] = {(char)(fmt.fmt.pix.pixelformat & 0xFF),
                             (char)((fmt.fmt.pix.pixelformat >> 8) & 0xFF),
                             (char)((fmt.fmt.pix.pixelformat >> 16) & 0xFF),
                             (char)((fmt.fmt.pix.pixelformat >> 24) & 0xFF), 0};
            RCLCPP_INFO(this->get_logger(), "Driver accepted format: %s (%dx%d)", fourcc, fmt.fmt.pix.width, fmt.fmt.pix.height);
        }

        v4l2_requestbuffers req = {};
        req.count = DEFAULT_BUFFERS;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        if (xioctl(fd_, VIDIOC_REQBUFS, &req) < 0) {
            RCLCPP_FATAL(this->get_logger(), "VIDIOC_REQBUFS failed: %s", strerror(errno));
            throw std::runtime_error("Failed to request buffers");
        }

        bufs_.resize(req.count);
        for (uint32_t i = 0; i < (uint32_t)req.count; ++i) {
            v4l2_buffer buf = {};
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;
            xioctl(fd_, VIDIOC_QUERYBUF, &buf);
            bufs_[i].length = buf.length;
            bufs_[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);
            xioctl(fd_, VIDIOC_QBUF, &buf);
        }

        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        xioctl(fd_, VIDIOC_STREAMON, &type);

        int64_t sys_offset_ns = 0;
        std::ifstream fin("/sys/devices/system/clocksource/clocksource0/offset_ns");
        if (fin && (fin >> sys_offset_ns))
            RCLCPP_INFO(this->get_logger(), "Read sys offset_ns = %ld", sys_offset_ns);
        else
            RCLCPP_WARN(this->get_logger(), "offset_ns not found, using zero");

        epoch_offset_ns_ = getEpochOffsetNs();
        sys_offset_ns_ = sys_offset_ns;
        width_ = width;
        height_ = height;
        pub_topic_ = pub_topic;
        camera_id_ = camera_id;
        publish_every_n_ = 3;
        frame_count_ = 0;

        timer_ = this->create_wall_timer(std::chrono::milliseconds(33), std::bind(&CameraLauncherNode::timerCallback, this));
    }

    ~CameraLauncherNode() {
        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        xioctl(fd_, VIDIOC_STREAMOFF, &type);
        for (auto& b : bufs_) munmap(b.start, b.length);
        ::close(fd_);
    }

private:
    struct Buffer {
        void* start;
        size_t length;
    };
    std::vector<Buffer> bufs_;
    int fd_;
    int width_, height_;
    std::string pub_topic_, camera_id_;
    int64_t sys_offset_ns_ = 0, epoch_offset_ns_ = 0;
    int publish_every_n_ = 3, frame_count_ = 0;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void timerCallback() {
        v4l2_buffer buf = {};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if (xioctl(fd_, VIDIOC_DQBUF, &buf) == 0) {
            frame_count_++;
            if (frame_count_ % publish_every_n_ == 0) {
                frame_count_ = 0;
                int64_t mono_ns = buf.timestamp.tv_sec * 1000000000LL + buf.timestamp.tv_usec * 1000LL - sys_offset_ns_;
                // int64_t epoch_ns = mono_ns + epoch_offset_ns_;
                rclcpp::Time stamp(mono_ns);
                
                cv::Mat uyvy(height_, width_, CV_8UC2, bufs_[buf.index].start);
                cv::Mat bgr;
                cv::cvtColor(uyvy, bgr, cv::COLOR_YUV2BGR_UYVY);
                auto img_msg = cvToRos(bgr);
                img_msg.header.stamp = stamp;
                img_msg.header.frame_id = pub_topic_ + camera_id_;
                pub_->publish(img_msg);
            }
            xioctl(fd_, VIDIOC_QBUF, &buf);
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<CameraLauncherNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        fprintf(stderr, "Exception: %s\n", e.what());
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
