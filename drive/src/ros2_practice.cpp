#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
using namespace std;
using std::placeholders::_1;

class SafetyMonitor : public rclcpp::Node
{
    public:
        SafetyMonitor() : Node("Safety_Monitoring_Node")
    {
        image_listener = this->create_subscription<sensor_msgs::msg::Image>("camera_detected", rclcpp::SystemDefaultQoS(), std::bind(&SafetyMonitor::image_callback, this, _1));
        lidar_listener = this->create_subscription<sensor_msgs::msg::LaserScan>("lidar_distance", rclcpp::SystemDefaultQoS(), std::bind(&SafetyMonitor::scan_callback, this, _1));
        string_publisher = this->create_publisher<std_msgs::msg::String>("safety_status", rclcpp::SystemDefaultQoS());
        timer = this->create_wall_timer(100ms, std::bind(&SafetyMonitor::timer_callback, this));
    }

    private:
    int height;
    vector<uint8_t> image_data;

    vector<float> distances;

    char message_to_publish[50];

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_publisher;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_listener;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_listener;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr image_message)
    {
        //auto message_image = sensor_msgs::msg::Image();
        height = image_message->height;
        image_data = image_message->data;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_message)
    {
        distances = scan_message->ranges;
    }

    void timer_callback()
    {
        auto send_message = std_msgs::msg::String();
        send_message.data = "Robot Navigating";
        string_publisher->publish(send_message);
    }
};

int main (int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SafetyMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
