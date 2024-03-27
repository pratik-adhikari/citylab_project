#include "robot_patrol/patrol.hpp"
#include <limits>
#include <vector>

using std::placeholders::_1;

Patrol::Patrol() : Node("patrol") {
  laser_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&Patrol::laserCallback, this, _1));
}

void Patrol::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  int sections = 36;
  std::vector<float> min_distances(sections, std::numeric_limits<float>::max());

  auto front_start_angle = M_PI;
  auto front_end_angle = 2 * M_PI;
  int start_index = static_cast<int>((front_start_angle - msg->angle_min) /
                                     msg->angle_increment);
  int end_index = static_cast<int>((front_end_angle - msg->angle_min) /
                                   msg->angle_increment);

  // Adjust start and end index to ensure they are within the valid range
  start_index = std::max(start_index, 0);
  end_index = std::min(end_index, static_cast<int>(msg->ranges.size()));

  // Segment and find minimum distances
  for (int i = start_index; i < end_index; ++i) {
    int section = (i - start_index) * sections / (end_index - start_index);
    min_distances[section] = std::min(min_distances[section], msg->ranges[i]);
  }

  // Print minimum distances of each section
  for (int i = 0; i < sections; ++i) {
    RCLCPP_INFO(this->get_logger(), "Section %d: Min Distance: %f", i + 1,
                min_distances[i]);
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Patrol>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
