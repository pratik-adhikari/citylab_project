#include "rclcpp/rclcpp.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

class TestServiceClient : public rclcpp::Node {
public:
  TestServiceClient() : Node("test_service_client") {
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&TestServiceClient::scan_callback, this,
                  std::placeholders::_1));

    client_ = this->create_client<robot_patrol::srv::GetDirection>(
        "/direction_service");

    RCLCPP_INFO(this->get_logger(),
                "TestServiceClient has been started, subscribing to /scan and "
                "waiting for /direction_service");
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (!client_->wait_for_service(1s)) {
      RCLCPP_WARN(this->get_logger(),
                  "Waiting for the /direction_service to be available...");
      return;
    }

    auto request = std::make_shared<robot_patrol::srv::GetDirection::Request>();
    request->laser_data = *msg;
    RCLCPP_INFO(this->get_logger(), "Sending request to /direction_service");

    auto future_result = client_->async_send_request(
        request, std::bind(&TestServiceClient::handle_service_response, this,
                           std::placeholders::_1));
  }

  void handle_service_response(
      rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture future) {

    try {
      auto response = future.get();
      RCLCPP_INFO(this->get_logger(),
                  "Received direction from /direction_service: %s",
                  response->direction.c_str());
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(),
                   "Service call to /direction_service failed: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr client_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestServiceClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
