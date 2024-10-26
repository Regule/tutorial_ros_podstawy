//=================================================================================================
//                                      GENERIC INCLUDES 
//=================================================================================================
#include <memory>

//=================================================================================================
//                                       ROS2 INCLUDES 
//=================================================================================================
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

//=================================================================================================
//                                      USEFULL TYPEDEFS 
//=================================================================================================
typedef std_msgs::msg::String StringMsg;

//=================================================================================================
//                                       SUBSCRIBER NODE
//=================================================================================================
class MinimalSubscriber: public rclcpp::Node
{
public:
    MinimalSubscriber();

private:
    rclcpp::Subscription<StringMsg>::SharedPtr _bragging_counter_subscribtion;

private:
    void _bragging_counter_callback(StringMsg::UniquePtr msg);
};

MinimalSubscriber::MinimalSubscriber(): Node("small_sub")
{
    _bragging_counter_subscribtion = this->create_subscription<StringMsg>(
        "bragging",
        10,
        std::bind(&MinimalSubscriber::_bragging_counter_callback, this, std::placeholders::_1)
    );
}

void MinimalSubscriber::_bragging_counter_callback(StringMsg::UniquePtr msg)
{
    RCLCPP_INFO(this->get_logger(), "RECEIVED - %s", msg->data.c_str());
}

//=================================================================================================
//                                            MAIN 
//=================================================================================================
int main(int argc, char ** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<MinimalSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
