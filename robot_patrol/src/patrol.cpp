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
  global_filtered_ranges.clear();
  size_t totalRanges = msg->ranges.size();
  size_t quarterIdx = totalRanges / 4, threeQuartersIdx = totalRanges * 0.75;

  // process fourth & first quadrant data
  for (size_t i = threeQuartersIdx; i < totalRanges; ++i) {
    global_filtered_ranges.push_back(
        std::isinf(msg->ranges[i]) ? 5.0f : msg->ranges[i]);
  }
  for (size_t i = 0; i <= quarterIdx; ++i) {
    global_filtered_ranges.push_back(
        std::isinf(msg->ranges[i]) ? 5.0f : msg->ranges[i]);
  }

  // Update and publish the filtered scan
  sensor_msgs::msg::LaserScan filtered_scan = *msg;
  filtered_scan.ranges = global_filtered_ranges;
  filtered_scan.angle_min = -M_PI / 2;
  filtered_scan.angle_max = M_PI / 2;
  filtered_scan_publisher_->publish(filtered_scan);
}

std::vector<float> Patrol::divideIntoSectionsAndFindMin() {
  const size_t sections = 13;
  std::vector<float> section_min_distances(sections,
                                           std::numeric_limits<float>::max());
  size_t points_per_section = global_filtered_ranges.size() / sections;

  for (size_t section = 0; section < sections; ++section) {
    auto start_itr =
        global_filtered_ranges.begin() + section * points_per_section;
    auto end_itr = (section + 1 == sections) ? global_filtered_ranges.end()
                                             : start_itr + points_per_section;
    section_min_distances[section] = *std::min_element(start_itr, end_itr);

    // if (section_min_distances[section] < 0.5) {
    //   float angle = -M_PI / 2 + (static_cast<float>(section) / sections) *
    //   M_PI; RCLCPP_INFO(this->get_logger(),
    //               "Section %zu: Min distance %.2f at angle %.2f radians",
    //               section + 1, section_min_distances[section], angle);
    // }
  }

  return section_min_distances;
}

void Patrol::findSafestDirection(const std::vector<float> &min_distances) {
  const float criticalDistance = 0.4;
  size_t safest_section = 0;
  float max_distance = 0.0; // tracking maximum individual distance

  for (size_t i = 0; i < min_distances.size(); ++i) {
    float current_distance = min_distances[i];
    // Adding adjacent distances if they exist
    if (i > 0)
      current_distance +=
          std::max(0.0f, min_distances[i - 1] - criticalDistance);
    if (i < min_distances.size() - 1)
      current_distance +=
          std::max(0.0f, min_distances[i + 1] - criticalDistance);

    if (current_distance > max_distance && min_distances[i] > 0.6) {
      safest_section = i;
      max_distance =
          current_distance; // Useing the combined metric for comparison
    }
  }

  // the turning angle calculate
  float angle_per_section = M_PI / (min_distances.size() - 1);
  direction_ =
      (safest_section - (min_distances.size() / 2.0f)) * angle_per_section;

  RCLCPP_INFO(this->get_logger(),
              "Safest direction: Section %zu/%zu, Angle: %.2f radians (%.2f "
              "degrees), Max distance: %.2f",
              safest_section + 1, min_distances.size(), direction_,
              direction_ * (180.0 / M_PI), max_distance);
}

void Patrol::controlLoop() {
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0.1; // Forward speed

  cmd_vel.angular.z = direction_; // Angular velocity: direction_ by 1/2

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