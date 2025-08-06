// simple_multi_camera_example.cpp
// 简单的多相机组件使用示例
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

class CameraListener : public rclcpp::Node
{
public:
    CameraListener() : Node("camera_listener")
    {
        // 订阅两个相机的话题
        camera0_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
            "/camera0/compressed", 10,
            [this](sensor_msgs::msg::CompressedImage::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "Received image from camera0: %zu bytes", 
                          msg->data.size());
            });
            
        camera1_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
            "/camera1/compressed", 10,
            [this](sensor_msgs::msg::CompressedImage::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "Received image from camera1: %zu bytes", 
                          msg->data.size());
            });
            
        RCLCPP_INFO(this->get_logger(), "Camera listener initialized");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr camera0_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr camera1_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto listener = std::make_shared<CameraListener>();
    
    RCLCPP_INFO(rclcpp::get_logger("simple_example"), 
               "开始监听相机数据... 请在另一个终端启动多相机组件:");
    RCLCPP_INFO(rclcpp::get_logger("simple_example"), 
               "ros2 run v4l2_camera multi_camera_standalone");
    
    rclcpp::spin(listener);
    rclcpp::shutdown();
    
    return 0;
}
