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
  }

  return section_min_distances;
}

void Patrol::findSafestDirection(const std::vector<float> &min_distances) {
  const float criticalDistance = 0.4;
  const float forwardBias = 3.5;
  size_t safest_section = min_distances.size() / 2;
  float max_distance = 0.0;

  for (size_t i = 0; i < min_distances.size(); ++i) {
    float current_distance = min_distances[i];

    if (i == safest_section) {
      current_distance *= forwardBias;
    }

    if (i > 0)
      current_distance +=
          std::max(0.0f, min_distances[i - 1] - criticalDistance);
    if (i < min_distances.size() - 1)
      current_distance +=
          std::max(0.0f, min_distances[i + 1] - criticalDistance);

    if (current_distance > max_distance) {
      safest_section = i;
      max_distance = current_distance;
    }
  }

  float angle_per_section = M_PI / (min_distances.size() - 1);
  direction_ =
      (safest_section - (min_distances.size() / 2.0f)) * angle_per_section;

  direction_ *= angleBias(min_distances[6], criticalDistance);

  float section_7_distance = min_distances[6];
  RCLCPP_INFO(this->get_logger(),
              "Safest direction: Section %zu/%zu, Angle: %.2f radians (%.2f "
              "degrees), Max distance: %.2f, Section 7 Distance: %.2f",
              safest_section + 1, min_distances.size(), direction_,
              direction_ * (180.0 / M_PI), max_distance, section_7_distance);
}

float Patrol::angleBias(float forward_distance, float criticalDistance) {
  // Reduce turning angle if the forward distance is greater than
  // criticalDistance
  if (forward_distance > criticalDistance) {
    return std::max(0.1f, 1 - (forward_distance - criticalDistance) /
                                  forward_distance);
  }

  return 1.0f;
}

void Patrol::controlLoop() {
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0.1;
  cmd_vel.angular.z = direction_;
  velocity_publisher_->publish(cmd_vel);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Patrol>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}