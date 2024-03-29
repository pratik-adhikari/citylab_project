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
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr
      filtered_scan_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  float direction_ = 0.0;     // Direction to move in radians, 0 means straight
  float max_distance_ = -1.0; // Maximum distance in the safest direction
  std::vector<float> global_filtered_ranges;

  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  void filterScanAndPublish(const sensor_msgs::msg::LaserScan::SharedPtr &msg);

  std::vector<float> divideIntoSectionsAndFindMin();
  void findSafestDirection(const std::vector<float> &min_distances);
  void controlLoop();
};