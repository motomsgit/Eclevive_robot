#include <cmath>
#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

class zed_odom_watcher : public rclcpp::Node
{
public:
  zed_odom_watcher() : Node("zed_odom_watcher")
  {
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/zed/zed_node/odom", 10, std::bind(&zed_odom_watcher::odomCallback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    client_ = this->create_client<std_srvs::srv::SetBool>("zed/zed_node/reset_odometry");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    double linear_speed = std::sqrt(
      msg->twist.twist.linear.x * msg->twist.twist.linear.x +
      msg->twist.twist.linear.y * msg->twist.twist.linear.y +
      msg->twist.twist.linear.z * msg->twist.twist.linear.z);

    if (linear_speed >= 1.0)
    {
      RCLCPP_INFO(this->get_logger(), "Linear speed exceeded 1 m/s, updating position");
      auto prev_pos = msg->pose.pose.position;
      msg->pose.pose.position.x = prev_position_.x;
      msg->pose.pose.position.y = prev_position_.y;
      msg->pose.pose.position.z = prev_position_.z;
      prev_position_ = prev_pos;

      // Call the service
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      request->data = true;

      while (!client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
          return;
        }
        RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
      }

      auto result = client_->async_send_request(request);
    }

    publisher_->publish(*msg);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
  geometry_msgs::msg::Point prev_position_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<zed_odom_watcher>());
  rclcpp::shutdown();
  return 0;
}