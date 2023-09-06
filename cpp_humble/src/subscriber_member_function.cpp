#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1; // bind()의 첫번째 인자

class SubscriberMemberFunction : public rclcpp ::Node
{
public:
    SubscriberMemberFunction()
        : Node("subscriber_member_function")
    {
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
        subscriber_member_function_ = this->create_subscription<std_msgs::msg::String>(
            "topic",
            qos_profile,
            std::bind(&SubscriberMemberFunction::subscriber_callback, this, _1)); // '_1'는 bind 함수의 첫번째 인자
    }

private:
    void subscriber_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Received message: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_member_function_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SubscriberMemberFunction>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
