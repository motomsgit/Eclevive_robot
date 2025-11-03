#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class ImuRepublisher : public rclcpp::Node {
public:
    ImuRepublisher() : Node("imu_republisher") {
        // Subscribe to ZED IMU data
        subscription_ = create_subscription<sensor_msgs::msg::Imu>(
            "/zed/zed_node/imu/data", 1,
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                // Create new IMU message and modify frame_id
                sensor_msgs::msg::Imu new_msg = *msg;
                //sensor_msgs::msg::Imu new_msg;
                //new_msg.orientation = msg -> orientation;
                //new_msg.orientation_covariance = msg -> orientation_covariance;
                //new_msg.angular_velocity = msg -> angular_velocity;
                //new_msg.angular_velocity_covariance = msg -> angular_velocity_covariance;

                new_msg.header.frame_id = "zed_camera_link";
                
                // Publish message
                publisher_->publish(new_msg);
            });

        // Create publisher
        publisher_ = create_publisher<sensor_msgs::msg::Imu>("/imu", 2);

        RCLCPP_INFO(this->get_logger(), "IMU Republisher node initialized");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuRepublisher>());
    rclcpp::shutdown();
    return 0;
}