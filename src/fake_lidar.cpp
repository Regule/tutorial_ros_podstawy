#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "fake_lidar_config.hpp"


class FakeLidarNode: public rclcpp::Node
{
public:
  FakeLidarNode();
private:
  FakeLidarConfig _config;
};

FakeLidarNode::FakeLidarNode(): Node("fake_lidar")
{
  _config.declare(this);
  _config.update(this);
  _config.print(this);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FakeLidarNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
