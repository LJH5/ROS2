#include <chrono>
#include <memory>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// 시간을 가식성을 높인 문자로 표현
using namespace std::chrono_literals;

// rclcpp의 Node 클래스를 상속
class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  // 생성자 호출 및 초기화
  : Node("minimal_publisher"), count_(0)
  {
    // depth를 10으로 설정
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    // publish, depth를 10
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", qos_profile);
    // timer_callback 함수 1초마다 실행
    timer_ = this->create_wall_timer(
      1s, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  // timer_callback 함수 선언
  void timer_callback()
  {
    // msg 타입은 String
    auto msg = std_msgs::msg::String();
    // msg 데이터에 값 넣기
    msg.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
    // msg publish하기
    publisher_->publish(msg);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
