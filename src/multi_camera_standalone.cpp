// multi_camera_standalone.cpp
// Standalone application to run multiple cameras with intra-process communication
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include "v4l2_camera/multi_camera_component.hpp"
#include "v4l2_camera/camera_launcher_component.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        // Create executor for managing multiple nodes
        rclcpp::executors::MultiThreadedExecutor executor;
        
        // Configure intra-process communication
        auto intra_process_options = rclcpp::NodeOptions().use_intra_process_comms(true);
        
        // Create multi-camera manager
        auto multi_camera_manager = std::make_shared<v4l2_camera::MultiCameraComponent>(
            intra_process_options);
        
        // Add to executor
        executor.add_node(multi_camera_manager);
        
        // Get parameters for logging
        std::string config_file = multi_camera_manager->get_parameter("camera_config_file").as_string();
        std::string device_group = multi_camera_manager->get_parameter("device_group").as_string();
        bool use_image_transport = multi_camera_manager->get_parameter("use_image_transport").as_bool();
        bool convert_to_rgb = multi_camera_manager->get_parameter("convert_to_rgb").as_bool();
        
        RCLCPP_INFO(rclcpp::get_logger("multi_camera_main"), 
                   "========================================");
        RCLCPP_INFO(rclcpp::get_logger("multi_camera_main"), 
                   "Starting Multi-Camera System");
        RCLCPP_INFO(rclcpp::get_logger("multi_camera_main"), 
                   "Configuration file: %s", config_file.c_str());
        RCLCPP_INFO(rclcpp::get_logger("multi_camera_main"), 
                   "Device group: %s", device_group.c_str());
        RCLCPP_INFO(rclcpp::get_logger("multi_camera_main"), 
                   "Use image_transport: %s", use_image_transport ? "ENABLED" : "DISABLED");
        RCLCPP_INFO(rclcpp::get_logger("multi_camera_main"), 
                   "Convert to RGB: %s", convert_to_rgb ? "ENABLED" : "DISABLED");
        RCLCPP_INFO(rclcpp::get_logger("multi_camera_main"), 
                   "========================================");
        
        // Spin
        executor.spin();
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("multi_camera_main"), 
                    "Exception in multi-camera system: %s", e.what());
        return 1;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("multi_camera_main"), "Multi-camera system shutting down");
    rclcpp::shutdown();
    return 0;
}
