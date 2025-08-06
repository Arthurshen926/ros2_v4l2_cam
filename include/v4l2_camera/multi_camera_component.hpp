// multi_camera_component.hpp
// ROS2 multi-camera component for intra-process communication
#ifndef V4L2_CAMERA__MULTI_CAMERA_COMPONENT_HPP_
#define V4L2_CAMERA__MULTI_CAMERA_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/header.hpp>
#include <memory>
#include <vector>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <unordered_map>
#include <chrono>
#include "v4l2_camera/visibility_control.h"
#include "v4l2_camera/camera_launcher_component.hpp"

namespace v4l2_camera
{

struct CameraConfig
{
    std::string device_path;
    std::string camera_id;
    std::string topic_prefix;
    int width;
    int height;
    int publish_rate;
    std::string group_name;  // 新增：相机组名称
};



class ROS2_V4L2_CAMERA_PUBLIC MultiCameraComponent : public rclcpp::Node
{
public:
    explicit MultiCameraComponent(const rclcpp::NodeOptions & options);
    virtual ~MultiCameraComponent();

private:
    void loadCameraConfigurations();
    void initializeCameras();
    void shutdownCameras();
    
    // Camera instances
    std::vector<std::shared_ptr<CameraLauncherComponent>> camera_components_;
    
    // Configuration
    std::vector<CameraConfig> camera_configs_;
    
    // Intra-process communication options
    rclcpp::NodeOptions intra_process_options_;
    
    // Synchronization
    std::mutex cameras_mutex_;
    std::atomic<bool> shutdown_requested_;
};

} // namespace v4l2_camera

#endif // V4L2_CAMERA__MULTI_CAMERA_COMPONENT_HPP_
