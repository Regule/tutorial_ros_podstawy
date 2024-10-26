#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"


//=================================================================================================
class MinimalNode: public rclcpp::Node
{
public:
    MinimalNode(const char* node_name);

private:
    rclcpp::TimerBase::SharedPtr _timer;

private:
    void _timer_callback();
};

MinimalNode::MinimalNode(const char* node_name): Node(node_name)
{
    this->declare_parameter("period", 1000);
    int period = this->get_parameter("period").as_int();
    RCLCPP_WARN(this->get_logger(), "Period is set to %d", period);
    _timer = this->create_wall_timer(std::chrono::milliseconds(period),
                                     std::bind(&MinimalNode::_timer_callback, this));
    RCLCPP_WARN(this->get_logger(), "CREATED");
}

void MinimalNode::_timer_callback()
{
    RCLCPP_INFO(this->get_logger(), "Hello world.");
}


//=================================================================================================
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalNode>("mini_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
