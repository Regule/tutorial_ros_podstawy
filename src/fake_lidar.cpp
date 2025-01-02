#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "fake_lidar_config.hpp"
#include "tutorial_ros_podstawy/fake_sensor_utils.hpp"


class FakeLidarNode: public rclcpp::Node
{
public:
  FakeLidarNode();
private:
  FakeLidarConfig _config;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr _scan_publisher;
  rclcpp::TimerBase::SharedPtr _scan_timer;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> _frame_brodcaster;
  int _step;


private:
  void publish_scan();
};

FakeLidarNode::FakeLidarNode(): Node("fake_lidar")
{
  _config.declare(this);
  _config.update(this);
  _config.print(this);

  _step = 0;

  _scan_publisher = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
  _scan_timer = this->create_wall_timer(std::chrono::milliseconds(_config.get_scan_period_ms()),
                                                                  std::bind(&FakeLidarNode::publish_scan, this));

  _frame_brodcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
  geometry_msgs::msg::TransformStamped transform;
  transform.child_frame_id = "laser_frame";
  transform.header.frame_id = "robot";
  transform.header.stamp = this->now();
  _frame_brodcaster->sendTransform(transform);
}

void FakeLidarNode::publish_scan()
{
  auto msg = sensor_msgs::msg::LaserScan();

  msg.angle_min = _config.angle.first;
  msg.angle_max = _config.angle.second;
  msg.angle_increment = _config.get_scan_step();
  msg.range_min = _config.range.first;
  msg.range_max = _config.range.second;
  msg.scan_time = _config.get_scan_period_ms()/1000;

  fake_sensors::CyclicObstacle obstacle(_step, 1.0, _config.sample_count);
  std::vector<float> ranges(_config.sample_count);
  for(int i=0; i<_config.sample_count; i++)
  {
    ranges[i] = _config.get_scaled_sample(obstacle[i]);
  }
  msg.ranges = ranges;

  msg.header.frame_id = "laser_frame";
  msg.header.stamp = this->now();
  _scan_publisher->publish(msg);
  _step++;
  if(_step >= _config.sample_count)
  {
    _step = 0;
  }

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FakeLidarNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
