#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class LidarSubscriber : public rclcpp::Node
{
public:
    LidarSubscriber() : Node("lidar_subscriber")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LidarSubscriber::laserCallback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("filtered_lidar", 10)
    }

private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        float start_angle = 0.0;
        float end_angle = 90.0;

        start_angle *= M_PI / 180.0;
        end_angle *= M_PI / 180.0;

        float increment = msg->angle_increment;

        auto filtered_msg = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);
        filtered_msg->ranges.clear();

        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            float angle = (increment) * (i);
            if (angle >= start_angle && angle <= end_angle)
            {
                filtered_msg->ranges.push_back(msg->ranges[i]);
            }
        }
        publisher_->publish(*filtered_msg);

    }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}