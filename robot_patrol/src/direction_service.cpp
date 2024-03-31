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
        "get_direction",
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
    std::vector<float> filtered_ranges;

    // Convert 360-degree laser data to front 180-degree data
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

    // Divide the 180-degree data into 3 sections and calculate total distance
    // in each section
    size_t total_sections = 3; // left, front, right
    size_t points_per_section = filtered_ranges.size() / total_sections;
    std::vector<float> section_totals(total_sections, 0.0f);

    for (size_t section = 0; section < total_sections; ++section) {
      auto start_itr = filtered_ranges.begin() + section * points_per_section;
      auto end_itr = (section + 1 == total_sections)
                         ? filtered_ranges.end()
                         : start_itr + points_per_section;
      section_totals[section] = std::accumulate(start_itr, end_itr, 0.0f);
    }

    // Determine the safest direction based on the section with the maximum
    // total distance
    auto max_iter =
        std::max_element(section_totals.begin(), section_totals.end());
    size_t safest_section = std::distance(section_totals.begin(), max_iter);

    // Set the response direction based on the safest section
    switch (safest_section) {
    case 0:
      response->direction = "left";
      break;
    case 1:
      response->direction = "front";
      break;
    case 2:
      response->direction = "right";
      break;
    default:
      response->direction = "unknown";
      break;
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
