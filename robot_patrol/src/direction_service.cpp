#include "rclcpp/rclcpp.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

class DirectionService : public rclcpp::Node {
public:
  DirectionService() : Node("direction_service_node") {
    service_ = this->create_service<robot_patrol::srv::GetDirection>(
        "/direction_service",
        std::bind(&DirectionService::handle_service_request, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(),
                "Direction service server has been started.");
  }

private:
  void handle_service_request(
      const std::shared_ptr<robot_patrol::srv::GetDirection::Request> request,
      std::shared_ptr<robot_patrol::srv::GetDirection::Response> response) {
    auto laser_data = request->laser_data;
    auto filtered_ranges = filterLaserData(laser_data);
    auto section_totals = calculateSectionDistances(filtered_ranges);
    auto safest_direction = determineSafestDirection(section_totals);

    response->direction = safest_direction;

    RCLCPP_INFO(this->get_logger(), "Response sent: %s",
                safest_direction.c_str());
  }

  std::vector<float>
  filterLaserData(const sensor_msgs::msg::LaserScan &laser_data) {
    std::vector<float> filtered_ranges;
    size_t quarterIdx = laser_data.ranges.size() / 4,
           threeQuartersIdx = laser_data.ranges.size() * 0.75;
    for (size_t i = threeQuartersIdx; i < laser_data.ranges.size(); ++i) {
      filtered_ranges.push_back(
          std::isinf(laser_data.ranges[i]) ? 5.0f : laser_data.ranges[i]);
    }
    for (size_t i = 0; i <= quarterIdx; ++i) {
      filtered_ranges.push_back(
          std::isinf(laser_data.ranges[i]) ? 5.0f : laser_data.ranges[i]);
    }
    return filtered_ranges;
  }

  std::vector<float>
  calculateSectionDistances(const std::vector<float> &filtered_ranges) {
    const size_t total_sections = 3;
    size_t points_per_section = filtered_ranges.size() / total_sections;
    std::vector<float> section_totals(total_sections, 0.0f);

    for (size_t section = 0; section < total_sections; ++section) {
      auto start_itr = filtered_ranges.begin() + section * points_per_section;
      auto end_itr = (section + 1 == total_sections)
                         ? filtered_ranges.end()
                         : start_itr + points_per_section;
      section_totals[section] = std::accumulate(start_itr, end_itr, 0.0f);
    }

    RCLCPP_INFO(this->get_logger(),
                "Total distances - Left: %f, Front: %f, Right: %f",
                section_totals[0], section_totals[1], section_totals[2]);

    return section_totals;
  }

  std::string
  determineSafestDirection(const std::vector<float> &section_totals) {
    auto max_iter =
        std::max_element(section_totals.begin(), section_totals.end());
    size_t safest_section = std::distance(section_totals.begin(), max_iter);

    switch (safest_section) {
    case 0:
      return "left";
    case 1:
      return "front";
    case 2:
      return "right";
    default:
      return "unknown";
    }
  }

  rclcpp::Service<robot_patrol::srv::GetDirection>::SharedPtr service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DirectionService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
