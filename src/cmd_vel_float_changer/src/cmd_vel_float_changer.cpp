#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"

class CmdVelSubscriberPublisher : public rclcpp::Node
{
public:
  CmdVelSubscriberPublisher()
  : Node("cmd_vel_float_changer")
  {
    joy_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "mecanum/cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Assume linear x velocity is the value to be published as Float32
        auto x_float_msg = std::make_unique<std_msgs::msg::Float32>();
        auto y_float_msg = std::make_unique<std_msgs::msg::Float32>();
        auto z_float_msg = std::make_unique<std_msgs::msg::Float32>();
        x_float_msg->data = msg->linear.x;
        y_float_msg->data = msg->linear.y;
        z_float_msg->data = msg->angular.z;
        x_float_publisher_->publish(std::move(x_float_msg));
        y_float_publisher_->publish(std::move(y_float_msg));
        z_float_publisher_->publish(std::move(z_float_msg));
    });

    map_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Assume linear x velocity is the value to be published as Float32
        auto map_x_float_msg = std::make_unique<std_msgs::msg::Float32>();
        auto map_y_float_msg = std::make_unique<std_msgs::msg::Float32>();
        auto map_z_float_msg = std::make_unique<std_msgs::msg::Float32>();
        map_x_float_msg->data = msg->linear.x;
        map_y_float_msg->data = msg->linear.y;
        map_z_float_msg->data = msg->angular.z;
        map_x_float_publisher_->publish(std::move(map_x_float_msg));
        map_y_float_publisher_->publish(std::move(map_y_float_msg));
        map_z_float_publisher_->publish(std::move(map_z_float_msg));
    });

    x_float_publisher_ = this->create_publisher<std_msgs::msg::Float32>("cmd_vel_x_float", 10);
    y_float_publisher_ = this->create_publisher<std_msgs::msg::Float32>("cmd_vel_y_float", 10);
    z_float_publisher_ = this->create_publisher<std_msgs::msg::Float32>("cmd_vel_z_float", 10);

    map_x_float_publisher_ = this->create_publisher<std_msgs::msg::Float32>("map_vel_x_float", 10);
    map_y_float_publisher_ = this->create_publisher<std_msgs::msg::Float32>("map_vel_y_float", 10);
    map_z_float_publisher_ = this->create_publisher<std_msgs::msg::Float32>("map_vel_z_float", 10);
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr joy_vel_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr map_vel_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr x_float_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr y_float_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr z_float_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr map_x_float_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr map_y_float_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr map_z_float_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CmdVelSubscriberPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}