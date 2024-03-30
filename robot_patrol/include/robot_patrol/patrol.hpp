#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>
#include <limits>
#include <vector>

class Patrol : public rclcpp::Node {
public:
  Patrol();

private:
  // Subscriptions and publishers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr
      filtered_scan_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  float direction_ = 0.0; // Safest direction to move in radians
  std::vector<float>
      global_filtered_ranges; // Processed range data from laser scan
  size_t num_sections_ =
      15; // Number of sections to divide the scan data into for analysis

  // Callbacks and utility functions
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void filterScanAndPublish(const sensor_msgs::msg::LaserScan::SharedPtr &msg);
  std::vector<float> divideIntoSectionsAndFindMin();
  void findSafestDirection(const std::vector<float> &min_distances);
  void controlLoop();
};
