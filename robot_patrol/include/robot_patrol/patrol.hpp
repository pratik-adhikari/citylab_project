#ifndef PATROL_HPP_
#define PATROL_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <memory>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

class Patrol : public rclcpp::Node {
public:
  Patrol();

private:
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_subscription_;
};

#endif // PATROL_HPP_
