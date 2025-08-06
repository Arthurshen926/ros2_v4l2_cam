// multi_camera_component.cpp
// ROS2 multi-camera component implementation
#include "v4l2_camera/multi_camera_component.hpp"
#include "v4l2_camera/camera_launcher_component.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <sstream>

namespace v4l2_camera
{

MultiCameraComponent::MultiCameraComponent(const rclcpp::NodeOptions & options)
: Node("multi_camera_manager", options),
  shutdown_requested_(false)
{
    // Configure for intra-process communication
    intra_process_options_ = rclcpp::NodeOptions(options).use_intra_process_comms(true);
    
    RCLCPP_INFO(this->get_logger(), "Multi-Camera Component Manager initialized");
    
    // Declare parameters with default values
    this->declare_parameter("camera_config_file", std::string("/home/cyberbus/driver_ws/src/ros2_v4l2_camera/config/camera_groups_config.yaml"));
    this->declare_parameter("num_cameras", 2);
    this->declare_parameter("default_width", 1920);
    this->declare_parameter("default_height", 1500);
    this->declare_parameter("default_device_prefix", std::string("/dev/video"));
    this->declare_parameter("default_topic_prefix", std::string("/camera"));
    this->declare_parameter("use_image_transport", true);
    this->declare_parameter("convert_to_rgb", false);
    
    // Check if device_group parameter already exists before declaring
    if (!this->has_parameter("device_group")) {
        this->declare_parameter("device_group", std::string("fisheye"));
    }
    
    // Load camera configurations
    loadCameraConfigurations();
    
    // Initialize cameras
    initializeCameras();
    
    RCLCPP_INFO(this->get_logger(), "Multi-Camera Manager started with %zu cameras", 
               camera_components_.size());
}

MultiCameraComponent::~MultiCameraComponent()
{
    shutdownCameras();
}

void MultiCameraComponent::loadCameraConfigurations()
{
    try {
        // 获取配置文件路径
        std::string config_file = this->get_parameter("camera_config_file").as_string();
        std::string device_group = this->get_parameter("device_group").as_string();
        
        RCLCPP_INFO(this->get_logger(), "Loading camera configurations from: %s", config_file.c_str());
        RCLCPP_INFO(this->get_logger(), "Device group: %s", device_group.c_str());
        
        if (!config_file.empty()) {
            // 从YAML文件加载配置
            YAML::Node config = YAML::LoadFile(config_file);
            
            RCLCPP_INFO(this->get_logger(), "YAML file loaded successfully");
            
            // 检查新的配置文件格式
            if (config["camera_groups"] && config["camera_groups"][device_group]) {
                RCLCPP_INFO(this->get_logger(), "Found camera group: %s", device_group.c_str());
                YAML::Node group_config = config["camera_groups"][device_group];
                YAML::Node devices = group_config["devices"];
                
                // 从数组中读取设备路径
                for (size_t i = 0; i < devices.size(); ++i) {
                    CameraConfig cam_config;
                    cam_config.device_path = devices[i].as<std::string>();
                    cam_config.camera_id = "";  // 设备路径已完整，不需要额外ID
                    cam_config.topic_prefix = group_config["topic_prefix"].as<std::string>() + std::to_string(i);
                    cam_config.width = group_config["width"].as<int>();
                    cam_config.height = group_config["height"].as<int>();
                    cam_config.publish_rate = group_config["frame_rate"] ? 
                                            group_config["frame_rate"].as<int>() : 30;  // 使用frame_rate字段
                    cam_config.group_name = device_group;
                    
                    camera_configs_.push_back(cam_config);
                }
                
                RCLCPP_INFO(this->get_logger(), 
                    "Loaded %zu cameras from group '%s'", 
                    camera_configs_.size(), device_group.c_str());
                return;
            }
            // 检查旧的配置文件格式（保持兼容性）
            else if (config["device_groups"] && config["device_groups"][device_group]) {
                YAML::Node group_config = config["device_groups"][device_group];
                YAML::Node devices = group_config["devices"];
                
                for (const auto& device : devices) {
                    CameraConfig cam_config;
                    cam_config.device_path = device["device_path"].as<std::string>();
                    cam_config.camera_id = device["camera_id"].as<std::string>();
                    cam_config.topic_prefix = device["topic_prefix"].as<std::string>();
                    cam_config.width = device["width"].as<int>();
                    cam_config.height = device["height"].as<int>();
                    cam_config.publish_rate = device["publish_rate"].as<int>();
                    cam_config.group_name = device_group;
                    
                    camera_configs_.push_back(cam_config);
                }
                
                RCLCPP_INFO(this->get_logger(), 
                    "Loaded %zu cameras from legacy group '%s'", 
                    camera_configs_.size(), device_group.c_str());
                return;
            } else {
                RCLCPP_ERROR(this->get_logger(), 
                    "Device group '%s' not found in config file. Available groups: ", device_group.c_str());
                
                // 列出可用的组
                if (config["camera_groups"]) {
                    for (const auto& group : config["camera_groups"]) {
                        RCLCPP_ERROR(this->get_logger(), "  - %s", group.first.as<std::string>().c_str());
                    }
                } else {
                    RCLCPP_ERROR(this->get_logger(), "  No camera_groups found in config file");
                }
            }
        }
        
        // 回退到默认配置（保持向后兼容）
        int num_cameras = this->get_parameter("num_cameras").as_int();
        int default_width = this->get_parameter("default_width").as_int();
        int default_height = this->get_parameter("default_height").as_int();
        std::string device_prefix = this->get_parameter("default_device_prefix").as_string();
        std::string topic_prefix = this->get_parameter("default_topic_prefix").as_string();
        
        for (int i = 0; i < num_cameras; ++i) {
            CameraConfig config;
            config.device_path = device_prefix + std::to_string(i);  // 这里会生成 /dev/video0
            config.camera_id = "";  // 设备路径已完整，不需要额外ID
            config.topic_prefix = topic_prefix + std::to_string(i);
            config.width = default_width;
            config.height = default_height;
            config.publish_rate = 30;
            config.group_name = "default";
            
            camera_configs_.push_back(config);
        }
        
        RCLCPP_INFO(this->get_logger(), "Created %d camera configurations", num_cameras);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), 
            "Failed to load camera configurations: %s", e.what());
        
        // 回退到单个默认相机
        CameraConfig default_config;
        default_config.device_path = "/dev/video0";
        default_config.camera_id = "0";
        default_config.topic_prefix = "camera_0";
        default_config.width = 640;
        default_config.height = 480;
        default_config.publish_rate = 30;
        default_config.group_name = "default";
        
        camera_configs_.push_back(default_config);
    }
}

void MultiCameraComponent::initializeCameras()
{
    std::lock_guard<std::mutex> lock(cameras_mutex_);
    
    for (size_t i = 0; i < camera_configs_.size(); ++i) {
        const auto& config = camera_configs_[i];
        
        try {
            // Create node options for each camera with intra-process communication
            auto camera_options = rclcpp::NodeOptions(intra_process_options_);
            
            // Get global parameters from this node
            bool use_image_transport = true;
            bool convert_to_rgb = false;
            this->get_parameter_or("use_image_transport", use_image_transport, true);
            this->get_parameter_or("convert_to_rgb", convert_to_rgb, false);
            
            // Set parameters for this camera
            camera_options.parameter_overrides({
                {"arg_camera_dev", config.device_path},
                {"arg_camera_id", config.camera_id},
                {"arg_publish_topic", config.topic_prefix},
                {"arg_resolution_width", config.width},
                {"arg_resolution_height", config.height},
                {"frame_rate", config.publish_rate},  // 添加帧率参数
                {"use_image_transport", use_image_transport},
                {"convert_to_rgb", convert_to_rgb}
            });
            
            RCLCPP_INFO(this->get_logger(), 
                       "Initializing camera %zu: %s%s -> %s (use_image_transport=%s, convert_to_rgb=%s)", 
                       i, config.device_path.c_str(), config.camera_id.c_str(), 
                       config.topic_prefix.c_str(),
                       use_image_transport ? "true" : "false",
                       convert_to_rgb ? "true" : "false");
            
            // Create camera component
            auto camera_component = std::make_shared<CameraLauncherComponent>(camera_options);
            
            // Start the camera
            if (camera_component->start()) {
                camera_components_.push_back(camera_component);
                RCLCPP_INFO(this->get_logger(), "Camera %zu initialized successfully: %s%s", 
                           i, config.device_path.c_str(), config.camera_id.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to start camera %zu: %s%s", 
                            i, config.device_path.c_str(), config.camera_id.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception initializing camera %zu (%s%s): %s", 
                        i, config.device_path.c_str(), config.camera_id.c_str(), e.what());
        }
    }
}

void MultiCameraComponent::shutdownCameras()
{
    std::lock_guard<std::mutex> lock(cameras_mutex_);
    shutdown_requested_ = true;
    
    RCLCPP_INFO(this->get_logger(), "Shutting down %zu cameras", camera_components_.size());
    
    for (auto& camera : camera_components_) {
        if (camera) {
            camera->stop();
        }
    }
    
    camera_components_.clear();
    RCLCPP_INFO(this->get_logger(), "All cameras shut down");
}

} // namespace v4l2_camera

RCLCPP_COMPONENTS_REGISTER_NODE(v4l2_camera::MultiCameraComponent)
