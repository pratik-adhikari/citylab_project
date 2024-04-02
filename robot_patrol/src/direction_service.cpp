#include "robot_patrol/direction_service.hpp" // Make sure the path matches the actual location

DirectionService::DirectionService() : Node("direction_service_node") {
  service_ = this->create_service<robot_patrol::srv::GetDirection>(
      "/direction_service",
      std::bind(&DirectionService::handle_service_request, this,
                std::placeholders::_1, std::placeholders::_2));
  filtered_scan_publisher_ =
      this->create_publisher<sensor_msgs::msg::LaserScan>("filtered_scan", 10);
  RCLCPP_INFO(this->get_logger(), "Direction service server has been started.");
}

void DirectionService::handle_service_request(
    const std::shared_ptr<robot_patrol::srv::GetDirection::Request> request,
    std::shared_ptr<robot_patrol::srv::GetDirection::Response> response) {
  auto laser_data = request->laser_data;
  auto filtered_ranges = filterLaserData(laser_data);
  auto section_totals = calculateSectionDistances(filtered_ranges);
  auto safest_direction = determineSafestDirection(section_totals);

  response->direction = safest_direction;
  last_decision_ =
      safest_direction; // remembering the last decision sent for comparison

  RCLCPP_INFO(this->get_logger(), "Response sent: %s",
              safest_direction.c_str());
}

std::vector<float> DirectionService::filterLaserData(
    const sensor_msgs::msg::LaserScan &laser_data) {
  std::vector<float> filtered_ranges;
  size_t totalRanges = laser_data.ranges.size();
  size_t quarterIdx = totalRanges / 4, threeQuartersIdx = totalRanges * 0.75;

  for (size_t i = threeQuartersIdx; i < totalRanges; ++i) {
    filtered_ranges.push_back(std::min(laser_data.ranges[i], 3.0f));
  }
  for (size_t i = 0; i <= quarterIdx; ++i) {
    filtered_ranges.push_back(std::min(laser_data.ranges[i], 3.0f));
  }

  publishFilteredScan(laser_data, filtered_ranges);

  return filtered_ranges;
}

void DirectionService::publishFilteredScan(
    const sensor_msgs::msg::LaserScan &original_scan,
    const std::vector<float> &filtered_ranges) {
  sensor_msgs::msg::LaserScan filtered_scan = original_scan;
  filtered_scan.ranges = filtered_ranges;
  filtered_scan.angle_min = -M_PI / 2;
  filtered_scan.angle_max = M_PI / 2;
  filtered_scan_publisher_->publish(filtered_scan);
}

std::vector<float> DirectionService::calculateSectionDistances(
    const std::vector<float> &filtered_ranges) {
  std::vector<float> section_totals(3, 0.0);
  size_t third = filtered_ranges.size() / 3;
  for (size_t i = 0; i < filtered_ranges.size(); ++i) {
    size_t section = i / third;

    if (section < 3) {
      size_t corrected_section = 2 - section;
      section_totals[corrected_section] += filtered_ranges[i];
    }
  }
  return section_totals;
}

std::string DirectionService::determineSafestDirection(
    const std::vector<float> &section_totals) {
  float left_distance = section_totals[0];
  float front_distance = section_totals[1];
  float right_distance = section_totals[2];

  RCLCPP_INFO(this->get_logger(),
              "Total distances - Left: %f, Front: %f, Right: %f", left_distance,
              front_distance, right_distance);

  static int left_count = 0, right_count = 0,
             front_count = 0; // Stability counters
  const int decision_stability_threshold =
      3; // Number of cycles before a decision is considered stable
  std::string decision = last_decision_; // Default to last decision

  if (left_distance + decision_threshold_ > right_distance &&
      left_distance > front_distance) {
    left_count++;
    right_count = 0;
    front_count = 0;
    if (left_count > decision_stability_threshold) {
      decision = "left";
    }
  } else if (right_distance + decision_threshold_ > left_distance &&
             right_distance > front_distance) {
    right_count++;
    left_count = 0;
    front_count = 0;
    if (right_count > decision_stability_threshold) {
      decision = "right";
    }
  } else if (front_distance >
             decision_threshold_ + std::max(left_distance, right_distance)) {
    front_count++;
    left_count = 0;
    right_count = 0;
    if (front_count > decision_stability_threshold) {
      decision = "front";
    }
  } else {
    // No clear decision, maintain current direction but reset counters
    left_count = right_count = front_count = 0;
  }

  // Update only if decision changes
  if (decision != last_decision_) {
    last_decision_ = decision;
    RCLCPP_INFO(this->get_logger(), "Safest direction: %s", decision.c_str());
  }

  return decision;
}
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DirectionService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
