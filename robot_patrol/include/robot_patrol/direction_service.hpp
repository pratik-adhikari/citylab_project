#ifndef DIRECTION_SERVICE_HPP_
#define DIRECTION_SERVICE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <vector>

class DirectionService : public rclcpp::Node {
public:
  DirectionService();

private:
  rclcpp::Service<robot_patrol::srv::GetDirection>::SharedPtr service_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr
      filtered_scan_publisher_;

  std::string last_decision_ = "front";
  float decision_threshold_ = 5.0; // Threshold for decision hysteresis

  void handle_service_request(
      const std::shared_ptr<robot_patrol::srv::GetDirection::Request> request,
      std::shared_ptr<robot_patrol::srv::GetDirection::Response> response);

  std::vector<float>
  filterLaserData(const sensor_msgs::msg::LaserScan &laser_data);

  void publishFilteredScan(const sensor_msgs::msg::LaserScan &original_scan,
                           const std::vector<float> &filtered_ranges);

  std::vector<float>
  calculateSectionDistances(const std::vector<float> &filtered_ranges);

  std::string
  determineSafestDirection(const std::vector<float> &section_totals);
};

#endif // DIRECTION_SERVICE_HPP_
