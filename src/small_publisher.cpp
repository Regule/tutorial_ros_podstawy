//=================================================================================================
//                                      GENERIC INCLUDES 
//=================================================================================================
#include <chrono>
#include <memory>
#include <sstream>

//=================================================================================================
//                                       ROS2 INCLUDES 
//=================================================================================================
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/color_rgba.hpp"

//=================================================================================================
//                                       PUBLISHER NODE
//=================================================================================================

class MinimalPublisher: public rclcpp::Node
{
public:
    MinimalPublisher(const char* node_name);

private:
    rclcpp::TimerBase::SharedPtr _bragging_counter_timer;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _bragging_counter_publisher;

    rclcpp::TimerBase::SharedPtr _color_timer;
    rclcpp::Publisher<std_msgs::msg::ColorRGBA>::SharedPtr _color_publisher;

    int _count;

private:
    void _bragging_counter_callback();
    void _color_callback();

};
    

MinimalPublisher::MinimalPublisher(const char* node_name): Node(node_name)
{
    this->declare_parameter<int>("count_start", 0);
    _count = this->get_parameter("count_start").as_int();
    _bragging_counter_publisher = this->create_publisher<std_msgs::msg::String>("bragging", 10);
    _color_publisher = this->create_publisher<std_msgs::msg::ColorRGBA>("color", 10);
    _bragging_counter_timer = this->create_wall_timer(std::chrono::milliseconds(1000),
                                                      std::bind(
                                                      &MinimalPublisher::_bragging_counter_callback,
                                                      this)
                                                      );
    _color_timer = this->create_wall_timer(std::chrono::milliseconds(500),
                                           std::bind(
                                           &MinimalPublisher::_color_callback,
                                           this)
                                           );

}

void MinimalPublisher::_bragging_counter_callback()
{
    auto msg = std_msgs::msg::String();
    std::stringstream str_stream;
    str_stream << "I have already counted " << _count << ".";
    _count++;
    msg.data = str_stream.str();
    RCLCPP_INFO(this->get_logger(), "PUBLISHING : %s", msg.data.c_str());
    _bragging_counter_publisher->publish(msg);
}

void MinimalPublisher::_color_callback()
{
    auto msg = std_msgs::msg::ColorRGBA();
    msg.r = 3.14;
    msg.g = 0.3;
    msg.b = 12;
    msg.a = 0.0;
    RCLCPP_INFO(this->get_logger(), "PUBLISHING : [%f,%f,%f,%f]", msg.r, msg.g, msg.b, msg.a);
    _color_publisher->publish(msg);
}

//=================================================================================================
//                                            MAIN 
//=================================================================================================
int main(int argc, char ** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<MinimalPublisher>("min_pub");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
