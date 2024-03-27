#include "robot_patrol/patrol.hpp"

using std::placeholders::_1;

Patrol::Patrol() : Node("patrol") {
  laser_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&Patrol::laserCallback, this, _1));
}

void Patrol::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  int sections = 36;
  std::vector<float> min_distances(sections, std::numeric_limits<float>::max());

  auto front_start_angle = M_PI;   // Start of the front 180 degrees in radians
  auto front_end_angle = 2 * M_PI; // End of the front 180 degrees in radians
  int start_index = static_cast<int>((front_start_angle - msg->angle_min) /
                                     msg->angle_increment);
  int end_index = static_cast<int>((front_end_angle - msg->angle_min) /
                                   msg->angle_increment);

  start_index = std::max(start_index, 0);
  end_index = std::min(end_index, static_cast<int>(msg->ranges.size()));

  // Assuming a large distance for out-of-range values
  const float max_reasonable_distance = 30.0; // laser's maximum range

  for (int i = start_index; i < end_index; ++i) {
    float distance = msg->ranges[i];
    if (std::isinf(distance)) {
      distance = max_reasonable_distance;
    }
    int section = (i - start_index) * sections / (end_index - start_index);
    min_distances[section] = std::min(min_distances[section], distance);
  }

  // the section with maximum minimum distance (safest direction)
  auto max_it = std::max_element(min_distances.begin(), min_distances.end());
  int safest_section = std::distance(min_distances.begin(), max_it);

  // the angle to the center of the safest section
  float angle_increment_per_section =
      (front_end_angle - front_start_angle) / sections;
  float safest_angle =
      front_start_angle + (safest_section + 0.5) * angle_increment_per_section;
  safest_angle -= M_PI; // Adjusting to the [-pi, pi]

  // safest angle
  float safest_angle_degrees = safest_angle * 180.0 / M_PI;

  RCLCPP_INFO(
      this->get_logger(),
      "Safest Direction: Section %d at angle %f degrees with distance %f",
      safest_section + 1, safest_angle_degrees, *max_it);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Patrol>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
