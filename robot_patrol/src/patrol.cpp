#include "robot_patrol/patrol.hpp"

using std::placeholders::_1;

Patrol::Patrol() : Node("patrol") {
  laser_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&Patrol::laserCallback, this, _1));
  filtered_scan_publisher_ =
      this->create_publisher<sensor_msgs::msg::LaserScan>("filtered_scan", 10);
  velocity_publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                   std::bind(&Patrol::controlLoop, this));
}

void Patrol::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  filterScanAndPublish(msg);
  auto min_distances = divideIntoSectionsAndFindMin();

  findSafestDirection(min_distances);
}

void Patrol::filterScanAndPublish(
    const sensor_msgs::msg::LaserScan::SharedPtr &msg) {
  // Prepare for new scan data
  global_filtered_ranges.clear();

  // Indices for quarter and three quarters
  size_t quarterIdx = msg->ranges.size() / 4;
  size_t threeQuartersIdx = msg->ranges.size() * 0.75;

  // Process and combine ranges
  for (size_t i = threeQuartersIdx; i < msg->ranges.size(); ++i) {
    global_filtered_ranges.push_back(std::min(msg->ranges[i], 50.0f));
  }
  for (size_t i = 0; i <= quarterIdx; ++i) {
    global_filtered_ranges.push_back(std::min(msg->ranges[i], 50.0f));
  }

  sensor_msgs::msg::LaserScan filtered_scan = *msg;
  filtered_scan.ranges = global_filtered_ranges;
  filtered_scan_publisher_->publish(filtered_scan);
}

std::vector<float> Patrol::divideIntoSectionsAndFindMin() {
  const int sections = 36;
  std::vector<float> section_min_distances(sections,
                                           std::numeric_limits<float>::max());

  // Cap all distances at 50
  for (auto &distance : global_filtered_ranges) {
    distance = std::min(distance, 50.0f);
  }

  int points_per_section = global_filtered_ranges.size() / sections;

  for (int section = 0; section < sections; ++section) {
    // Define section boundaries
    auto start_itr =
        global_filtered_ranges.begin() + section * points_per_section;
    auto end_itr = (section + 1 == sections) ? global_filtered_ranges.end()
                                             : start_itr + points_per_section;

    // Find minimum in section
    float min_distance = *std::min_element(start_itr, end_itr);
    section_min_distances[section] = min_distance;
  }

  return section_min_distances;
}

void Patrol::findSafestDirection(const std::vector<float> &min_distances) {
  // Initialize the safest section and its maximum distance.
  size_t safest_section = 0;
  float max_distance = 0.0;

  // Iterate through each section to find the one with the maximum distance
  // and ensure neighboring sections have distances greater than 0.2.
  for (size_t i = 0; i < min_distances.size(); ++i) {
    // Check the current section's distance and its neighbors.
    bool is_safe = min_distances[i] > max_distance;
    bool left_neighbor_safe = (i == 0) || (min_distances[i - 1] > 0.6);
    bool right_neighbor_safe =
        (i == min_distances.size() - 1) || (min_distances[i + 1] > 0.6);

    // Update the safest section if conditions are met.
    if (is_safe && left_neighbor_safe && right_neighbor_safe) {
      safest_section = i;
      max_distance = min_distances[i];
    }
  }

  // Calculate the angle for the safest section. Each section represents 5
  // degrees of the field of view (180 degrees / 36 sections).
  float angle_per_section =
      M_PI / 36.0; // 180 degrees divided by 36 sections, in radians.
  float angle_to_turn = (safest_section * angle_per_section) +
                        (angle_per_section / 2.0) - (M_PI / 2.0);

  // Update the direction based on the safest section.
  direction_ = angle_to_turn;

  // Print section distances for debugging.
  std::string section_distances;
  for (const auto &distance : min_distances) {
    section_distances += std::to_string(distance) + " ";
  }

  // Log information about the safest direction.
  RCLCPP_INFO(this->get_logger(),
              "Safest direction: Section %zu, Angle: %f radians (%f degrees), "
              "Max min distance: %f. Section distances: [%s]",
              safest_section + 1, direction_, direction_ * (180.0 / M_PI),
              max_distance, section_distances.c_str());
}

void Patrol::controlLoop() {
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0.1; // Forward speed

  cmd_vel.angular.z = direction_ / 2; // Angular velocity: direction_ by 1/2

  velocity_publisher_->publish(cmd_vel); // Publish the command velocity

  //   RCLCPP_INFO(this->get_logger(),
  //               "Publishing velocity: linear.x=%f, angular.z=%f degrees",
  //               cmd_vel.linear.x, cmd_vel.angular.z * (180 / M_PI));
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Patrol>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}