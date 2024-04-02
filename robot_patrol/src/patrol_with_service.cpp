#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("patrol") {
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&Patrol::scan_callback, this, std::placeholders::_1));

    client_ = this->create_client<robot_patrol::srv::GetDirection>(
        "/direction_service");

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(),
                "patrol_with_service node has been started...");
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    last_laser_ = *msg;
    send_direction_request();
  }

  void send_direction_request() {
    if (!client_->wait_for_service(1s)) {
      RCLCPP_WARN(this->get_logger(),
                  "Waiting for the /direction_service to be available...");
      return;
    }

    auto request = std::make_shared<robot_patrol::srv::GetDirection::Request>();
    request->laser_data = last_laser_;

    auto future_result = client_->async_send_request(
        request, std::bind(&Patrol::control_loop, this, std::placeholders::_1));
  }

  void control_loop(
      rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture future) {
    auto response = future.get();
    geometry_msgs::msg::Twist cmd_vel;

    if (response->direction == "front") {
      cmd_vel.linear.x = 0.1;
      cmd_vel.angular.z = 0.0;
    } else if (response->direction == "left") {
      cmd_vel.linear.x = 0.1;
      cmd_vel.angular.z = 0.5;
    } else if (response->direction == "right") {
      cmd_vel.linear.x = 0.1;
      cmd_vel.angular.z = -0.5;
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Unknown direction received. Stopping the robot.");
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
    }

    // Publish the velocity command based on the service response
    publisher_->publish(cmd_vel);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr client_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  sensor_msgs::msg::LaserScan last_laser_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Patrol>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
