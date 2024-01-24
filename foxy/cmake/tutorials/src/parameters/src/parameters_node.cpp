#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class MinimalParam : public rclcpp::Node
{
public:
    MinimalParam()
    : Node("minimal_param")
    {
        // Parameter 선언
        this->declare_parameter("my_parameter", "world");
        timer_ = this->create_wall_timer(1s, std::bind(&MinimalParam::timer_callback, this));
    }

private:
    void timer_callback()
    {
        // Parameter 가져오기
        std::string my_param = this->get_parameter("my_parameter").as_string();
        RCLCPP_INFO(this->get_logger(), "Hello %s!", my_param.c_str());

        std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("my_parameter", "new world")};
        std::cout << all_new_parameters << std::endl;

        // Parameter 넣어주기
        this->set_parameters(all_new_parameters);

    }
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalParam>());
    rclcpp::shutdown();
    return 0;
}
