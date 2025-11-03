/**
 * @file zed_zupt_filter_node.cpp
 * @brief ZED2i Odometry ZUPT (Zero-Velocity Update) Filter Node
 *
 * このノードは、ZED2iのVisual OdometryとIMUデータを統合し、
 * ZUPT理論を用いてオドメトリの暴走を防ぎます。
 *
 * 原理:
 * - IMUの加速度・角速度がほぼゼロ → ロボットは静止している
 * - この時、Visual Odometryが移動を報告 → 異常値として補正
 *
 * @author jetros
 * @date 2025-10-19
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <deque>
#include <cmath>
#include <algorithm>

class ZedZuptFilterNode : public rclcpp::Node
{
public:
    ZedZuptFilterNode() : Node("zed_zupt_filter_node")
    {
        // パラメータ宣言
        this->declare_parameter("input_odom_topic", "/zed/zed_node/odom");
        this->declare_parameter("input_imu_topic", "/zed/zed_node/imu/data");
        this->declare_parameter("output_odom_topic", "/zed/zed_node/odom_zupt");

        // ZUPT検出閾値
        this->declare_parameter("zupt_accel_threshold", 0.5);      // m/s² (静止判定)
        this->declare_parameter("zupt_gyro_threshold", 0.1);       // rad/s (静止判定)
        this->declare_parameter("zupt_window_size", 10);           // サンプル数
        this->declare_parameter("zupt_confidence_threshold", 0.7); // 静止判定の信頼度

        // オドメトリ補正パラメータ
        this->declare_parameter("max_velocity_jump", 0.5);         // m/s (急激な速度変化を制限)
        this->declare_parameter("max_angular_jump", 1.0);          // rad/s
        this->declare_parameter("velocity_decay_rate", 0.95);      // 静止時の速度減衰率
        this->declare_parameter("enable_zupt", true);              // ZUPT有効化

        // パラメータ取得
        input_odom_topic_ = this->get_parameter("input_odom_topic").as_string();
        input_imu_topic_ = this->get_parameter("input_imu_topic").as_string();
        output_odom_topic_ = this->get_parameter("output_odom_topic").as_string();

        zupt_accel_threshold_ = this->get_parameter("zupt_accel_threshold").as_double();
        zupt_gyro_threshold_ = this->get_parameter("zupt_gyro_threshold").as_double();
        zupt_window_size_ = this->get_parameter("zupt_window_size").as_int();
        zupt_confidence_threshold_ = this->get_parameter("zupt_confidence_threshold").as_double();

        max_velocity_jump_ = this->get_parameter("max_velocity_jump").as_double();
        max_angular_jump_ = this->get_parameter("max_angular_jump").as_double();
        velocity_decay_rate_ = this->get_parameter("velocity_decay_rate").as_double();
        enable_zupt_ = this->get_parameter("enable_zupt").as_bool();

        // Subscriber
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            input_odom_topic_, 10,
            std::bind(&ZedZuptFilterNode::odomCallback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            input_imu_topic_, 10,
            std::bind(&ZedZuptFilterNode::imuCallback, this, std::placeholders::_1));

        // Publisher
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, 10);

        // 統計Publisher（デバッグ用）
        zupt_status_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/zupt/status", 10);

        RCLCPP_INFO(this->get_logger(), "ZED ZUPT Filter Node initialized");
        RCLCPP_INFO(this->get_logger(), "  Input Odom: %s", input_odom_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Input IMU: %s", input_imu_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Output Odom: %s", output_odom_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  ZUPT Enabled: %s", enable_zupt_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "  Accel Threshold: %.3f m/s²", zupt_accel_threshold_);
        RCLCPP_INFO(this->get_logger(), "  Gyro Threshold: %.3f rad/s", zupt_gyro_threshold_);
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // IMUデータをバッファに保存
        imu_buffer_.push_back(*msg);

        // バッファサイズを制限
        if (imu_buffer_.size() > static_cast<size_t>(zupt_window_size_)) {
            imu_buffer_.pop_front();
        }

        // ZUPT判定を更新
        is_stationary_ = detectZeroVelocity();
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!enable_zupt_) {
            // ZUPTが無効の場合、そのまま出力
            odom_pub_->publish(*msg);
            return;
        }

        // フィルタリングされたオドメトリを作成
        auto filtered_odom = *msg;

        if (is_stationary_) {
            // 静止判定時: 速度をゼロに補正
            filtered_odom.twist.twist.linear.x = 0.0;
            filtered_odom.twist.twist.linear.y = 0.0;
            filtered_odom.twist.twist.linear.z = 0.0;
            filtered_odom.twist.twist.angular.x = 0.0;
            filtered_odom.twist.twist.angular.y = 0.0;
            filtered_odom.twist.twist.angular.z = 0.0;

            // 位置の累積誤差を抑制（前回の位置を維持）
            if (last_odom_received_) {
                filtered_odom.pose.pose.position = last_filtered_odom_.pose.pose.position;
                filtered_odom.pose.pose.orientation = last_filtered_odom_.pose.pose.orientation;
            }

            zupt_correction_count_++;
        } else {
            // 移動判定時: 急激な速度変化を制限
            if (last_odom_received_) {
                filtered_odom.twist.twist.linear.x = limitJump(
                    filtered_odom.twist.twist.linear.x,
                    last_filtered_odom_.twist.twist.linear.x,
                    max_velocity_jump_
                );
                filtered_odom.twist.twist.linear.y = limitJump(
                    filtered_odom.twist.twist.linear.y,
                    last_filtered_odom_.twist.twist.linear.y,
                    max_velocity_jump_
                );
                filtered_odom.twist.twist.angular.z = limitJump(
                    filtered_odom.twist.twist.angular.z,
                    last_filtered_odom_.twist.twist.angular.z,
                    max_angular_jump_
                );
            }
        }

        // 出力
        odom_pub_->publish(filtered_odom);

        // 前回値を保存
        last_filtered_odom_ = filtered_odom;
        last_odom_received_ = true;

        // ステータス出力（デバッグ用）
        publishZuptStatus();
    }

    bool detectZeroVelocity()
    {
        if (imu_buffer_.size() < static_cast<size_t>(zupt_window_size_)) {
            return false;  // データ不足
        }

        // ウィンドウ内のIMUデータを解析
        double accel_sum = 0.0;
        double gyro_sum = 0.0;
        int stationary_count = 0;

        for (const auto& imu : imu_buffer_) {
            // 加速度のノルム（重力を除去）
            double accel_norm = std::sqrt(
                imu.linear_acceleration.x * imu.linear_acceleration.x +
                imu.linear_acceleration.y * imu.linear_acceleration.y +
                (imu.linear_acceleration.z - 9.81) * (imu.linear_acceleration.z - 9.81)
            );

            // 角速度のノルム
            double gyro_norm = std::sqrt(
                imu.angular_velocity.x * imu.angular_velocity.x +
                imu.angular_velocity.y * imu.angular_velocity.y +
                imu.angular_velocity.z * imu.angular_velocity.z
            );

            accel_sum += accel_norm;
            gyro_sum += gyro_norm;

            // 閾値判定
            if (accel_norm < zupt_accel_threshold_ && gyro_norm < zupt_gyro_threshold_) {
                stationary_count++;
            }
        }

        // 平均値を計算
        avg_accel_ = accel_sum / imu_buffer_.size();
        avg_gyro_ = gyro_sum / imu_buffer_.size();

        // 信頼度計算
        double confidence = static_cast<double>(stationary_count) / imu_buffer_.size();

        // 静止判定
        return confidence >= zupt_confidence_threshold_;
    }

    double limitJump(double new_value, double old_value, double max_jump)
    {
        double diff = new_value - old_value;

        if (std::abs(diff) > max_jump) {
            // 急激な変化を制限
            return old_value + (diff > 0 ? max_jump : -max_jump);
        }

        return new_value;
    }

    void publishZuptStatus()
    {
        geometry_msgs::msg::Twist status;
        status.linear.x = is_stationary_ ? 1.0 : 0.0;  // 静止フラグ
        status.linear.y = avg_accel_;                   // 平均加速度
        status.linear.z = avg_gyro_;                    // 平均角速度
        status.angular.x = static_cast<double>(zupt_correction_count_);  // 補正回数

        zupt_status_pub_->publish(status);
    }

    // Subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    // Publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr zupt_status_pub_;

    // パラメータ
    std::string input_odom_topic_;
    std::string input_imu_topic_;
    std::string output_odom_topic_;

    double zupt_accel_threshold_;
    double zupt_gyro_threshold_;
    int zupt_window_size_;
    double zupt_confidence_threshold_;

    double max_velocity_jump_;
    double max_angular_jump_;
    double velocity_decay_rate_;
    bool enable_zupt_;

    // 状態変数
    std::deque<sensor_msgs::msg::Imu> imu_buffer_;
    nav_msgs::msg::Odometry last_filtered_odom_;
    bool last_odom_received_ = false;
    bool is_stationary_ = false;

    double avg_accel_ = 0.0;
    double avg_gyro_ = 0.0;
    int zupt_correction_count_ = 0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ZedZuptFilterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
