#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// bind 함수 대체
using std::placeholders::_1;

// rclcpp의 Node 클래스를 상속
class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  // 생성자 호출 및 초기화
  : Node("minimal_subscriber")
  {
    // depth를 10으로 설정
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    // subscribe
    subscriber_ = this->create_subscription<std_msgs::msg::String>(
      "topic",
      qos_profile,
      std::bind(&MinimalSubscriber::subscribe_callback, this, _1));
  }

private:
  // subscribe_callback 함수 선언
  void subscribe_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received message: '%s'", msg->data.c_str());
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
