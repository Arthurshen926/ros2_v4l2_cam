// camera_launcher_component.hpp
// ROS2 camera launcher component version
#ifndef V4L2_CAMERA__CAMERA_LAUNCHER_COMPONENT_HPP_
#define V4L2_CAMERA__CAMERA_LAUNCHER_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/header.hpp>
#include <image_transport/image_transport.hpp>
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
#include <memory>
#include <vector>
#include <thread>
#include <atomic>
#include <chrono>
#include "v4l2_camera/visibility_control.h"

namespace v4l2_camera
{

class ROS2_V4L2_CAMERA_PUBLIC CameraLauncherComponent : public rclcpp::Node
{
public:
    explicit CameraLauncherComponent(const rclcpp::NodeOptions & options);
    virtual ~CameraLauncherComponent();

    // Configuration methods
    void configure(const std::string& camera_dev, 
                   const std::string& camera_id,
                   const std::string& pub_topic,
                   int width, int height);
    
    bool start();
    void stop();
    bool is_running() const { return running_; }

private:
    struct Buffer {
        void* start;
        size_t length;
    };

    // V4L2 camera control
    int fd_;
    std::vector<Buffer> bufs_;
    int width_, height_;
    std::string pub_topic_, camera_id_;
    
    // Timing and sync
    int64_t sys_offset_ns_;
    int64_t epoch_offset_ns_;
    int publish_every_n_;
    int frame_count_;
    
    // ROS2 publishing
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    std::unique_ptr<image_transport::ImageTransport> image_transport_;
    image_transport::Publisher it_pub_;
    bool use_image_transport_;
    bool convert_to_rgb_;
    
    // Control thread
    std::thread capture_thread_;
    std::atomic<bool> running_;
    std::atomic<bool> shutdown_requested_;
    
    // Utility functions
    int xioctl(int fd, unsigned long request, void* arg);
    sensor_msgs::msg::Image cvToRosImage(const cv::Mat& img, const std::string& encoding = "rgb8");
    sensor_msgs::msg::Image uyyvToRosImage(const cv::Mat& uyvy_img);
    int64_t getEpochOffsetNs();
    
    // Camera operations
    bool openCamera(const std::string& device_path);
    bool setupFormat();
    bool setupBuffers();
    bool startStreaming();
    void stopStreaming();
    void closeCamera();
    
    // Capture loop
    void captureLoop();
    void processFrame();
};

} // namespace v4l2_camera

#endif // V4L2_CAMERA__CAMERA_LAUNCHER_COMPONENT_HPP_
