#include <chrono>
#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "tf2_ros/static_transform_broadcaster.h"
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <zed_msgs/msg/object.hpp>
#include <zed_msgs/msg/objects_stamped.hpp>

using namespace std::chrono_literals;

/**
 * @brief 手を上げた人を検出し追従するノード
 *
 * 機能:
 * 1. ZED2iの骨格検出から複数人を認識
 * 2. 肩より上に手を上げている人を検出
 * 3. 検出した人の追跡ID（label_id）を記録
 * 4. その人だけを継続的に追従（距離と角度を維持）
 * 5. 追従対象のTFフレーム（target_person）を配信
 */
class HandRaisedPersonFollower : public rclcpp::Node {
public:
    HandRaisedPersonFollower() : Node("hand_raised_person_follower") {
        // パラメータ宣言
        skeleton_topic_ = this->declare_parameter<std::string>("skeleton_topic", "zed/zed_node/body_trk/skeletons");
        cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
        target_distance_ = this->declare_parameter<double>("target_distance", 1.5);  // 目標距離 [m]
        distance_tolerance_ = this->declare_parameter<double>("distance_tolerance", 0.3);  // 距離許容誤差 [m]
        angle_tolerance_ = this->declare_parameter<double>("angle_tolerance", 0.1);  // 角度許容誤差 [rad]
        max_linear_speed_ = this->declare_parameter<double>("max_linear_speed", 0.3);  // 最大前後速度 [m/s]
        max_angular_speed_ = this->declare_parameter<double>("max_angular_speed", 0.5);  // 最大角速度 [rad/s]
        hand_raise_threshold_ = this->declare_parameter<double>("hand_raise_threshold", 0.3);  // 手上げ判定閾値 [m]
        tracking_timeout_ = this->declare_parameter<double>("tracking_timeout", 3.0);  // 追跡タイムアウト [s]

        // Subscriber: ZED骨格検出
        skeleton_subscriber_ = this->create_subscription<zed_msgs::msg::ObjectsStamped>(
            skeleton_topic_, 10, std::bind(&HandRaisedPersonFollower::skeletonCallback, this, std::placeholders::_1));

        // Publisher: 速度指令
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

        // Publisher: 追跡状態
        tracking_status_publisher_ = this->create_publisher<std_msgs::msg::String>("hand_follower/status", 10);

        // Publisher: ターゲットID
        target_id_publisher_ = this->create_publisher<std_msgs::msg::Int32>("hand_follower/target_id", 10);

        // Static TF Broadcaster
        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // タイマー: 追跡タイムアウト監視
        timeout_timer_ = this->create_wall_timer(
            500ms, std::bind(&HandRaisedPersonFollower::checkTrackingTimeout, this));

        RCLCPP_INFO(this->get_logger(), "Hand Raised Person Follower Node Started");
        RCLCPP_INFO(this->get_logger(), "  Target distance: %.2f m", target_distance_);
        RCLCPP_INFO(this->get_logger(), "  Distance tolerance: %.2f m", distance_tolerance_);
        RCLCPP_INFO(this->get_logger(), "  Hand raise threshold: %.2f m", hand_raise_threshold_);
    }

private:
    /**
     * @brief ZED骨格検出コールバック
     */
    void skeletonCallback(const zed_msgs::msg::ObjectsStamped::SharedPtr msg) {
        if (msg->objects.empty()) {
            publishStatus("No person detected");
            return;
        }

        bool hand_raised_person_found = false;
        geometry_msgs::msg::Point target_position;
        int32_t detected_person_id = -1;

        // 全ての人物をチェック
        for (const auto& obj : msg->objects) {
            // 追跡状態が有効な人物のみ処理
            if (obj.tracking_state <= 0) {
                continue;
            }

            // 骨格キーポイントが存在するか確認
            if (obj.skeleton_3d.keypoints.empty()) {
                continue;
            }

            // 骨格キーポイントのインデックス (ZED Body Tracking 38キーポイント)
            // 2: NOSE, 4: RIGHT_SHOULDER, 5: LEFT_SHOULDER
            // 7: RIGHT_ELBOW, 8: LEFT_ELBOW
            const int NOSE = 2;
            const int RIGHT_SHOULDER = 4;
            const int LEFT_SHOULDER = 5;
            const int RIGHT_ELBOW = 7;
            const int LEFT_ELBOW = 8;

            // キーポイント数のチェック
            if (obj.skeleton_3d.keypoints.size() <= std::max({NOSE, RIGHT_SHOULDER, LEFT_SHOULDER, RIGHT_ELBOW, LEFT_ELBOW})) {
                continue;
            }

            // 各キーポイントのY座標を取得（Y軸は上下）
            float nose_y = obj.skeleton_3d.keypoints[NOSE].kp[1];
            float right_shoulder_y = obj.skeleton_3d.keypoints[RIGHT_SHOULDER].kp[1];
            float left_shoulder_y = obj.skeleton_3d.keypoints[LEFT_SHOULDER].kp[1];
            float right_elbow_y = obj.skeleton_3d.keypoints[RIGHT_ELBOW].kp[1];
            float left_elbow_y = obj.skeleton_3d.keypoints[LEFT_ELBOW].kp[1];

            // 手を上げているか判定（肩より手が上にあるか）
            // ZED座標系ではY軸が下向きなので、Y値が小さいほど上
            bool right_hand_raised = right_shoulder_y < (right_elbow_y - hand_raise_threshold_);
            bool left_hand_raised = left_shoulder_y < (left_elbow_y - hand_raise_threshold_);

            // どちらかの手が上がっていれば検出
            if (right_hand_raised || left_hand_raised) {
                // 既に追跡中の場合: 同じIDのみ追従
                if (is_tracking_ && obj.label_id == tracked_person_id_) {
                    hand_raised_person_found = true;
                    target_position.x = obj.position[0];
                    target_position.y = obj.position[1];
                    target_position.z = obj.position[2];
                    detected_person_id = obj.label_id;
                    last_detection_time_ = this->now();

                    RCLCPP_DEBUG(this->get_logger(), "Tracking person ID %d (hand raised)", obj.label_id);
                    break;
                }
                // 追跡中でない場合: 新しい人を追跡開始
                else if (!is_tracking_) {
                    hand_raised_person_found = true;
                    target_position.x = obj.position[0];
                    target_position.y = obj.position[1];
                    target_position.z = obj.position[2];
                    detected_person_id = obj.label_id;
                    tracked_person_id_ = obj.label_id;
                    is_tracking_ = true;
                    last_detection_time_ = this->now();

                    RCLCPP_INFO(this->get_logger(), "🎯 Started tracking person ID %d (hand raised)", obj.label_id);

                    // ターゲットID公開
                    std_msgs::msg::Int32 id_msg;
                    id_msg.data = detected_person_id;
                    target_id_publisher_->publish(id_msg);
                    break;
                }
            }
        }

        // 手を上げた人が見つかった場合
        if (hand_raised_person_found) {
            followPerson(target_position);
            publishTargetTF(target_position);
            publishStatus("Tracking ID " + std::to_string(detected_person_id));
        }
        // 追跡中だが見失った場合
        else if (is_tracking_) {
            stopRobot();
            publishStatus("Lost target ID " + std::to_string(tracked_person_id_));
        }
        // 追跡中でなく、手を上げた人もいない場合
        else {
            publishStatus("Waiting for hand raise signal");
        }
    }

    /**
     * @brief 人物追従制御
     */
    void followPerson(const geometry_msgs::msg::Point& target_pos) {
        // 人物までの距離と角度を計算
        double distance = std::sqrt(target_pos.x * target_pos.x + target_pos.y * target_pos.y);
        double angle = std::atan2(target_pos.y, target_pos.x);

        // 距離誤差
        double distance_error = distance - target_distance_;

        // 速度指令を生成
        geometry_msgs::msg::Twist cmd_vel;

        // 角度制御（比例制御）
        if (std::abs(angle) > angle_tolerance_) {
            cmd_vel.angular.z = std::clamp(angle * 1.0, -max_angular_speed_, max_angular_speed_);
        }

        // 距離制御（比例制御）
        if (std::abs(distance_error) > distance_tolerance_) {
            cmd_vel.linear.x = std::clamp(distance_error * 0.5, -max_linear_speed_, max_linear_speed_);
        }

        // 速度指令を公開
        cmd_vel_publisher_->publish(cmd_vel);

        RCLCPP_DEBUG(this->get_logger(),
            "Following: dist=%.2f (target=%.2f), angle=%.2f, cmd_vel=(%.2f, %.2f)",
            distance, target_distance_, angle, cmd_vel.linear.x, cmd_vel.angular.z);
    }

    /**
     * @brief ロボット停止
     */
    void stopRobot() {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_vel_publisher_->publish(cmd_vel);
    }

    /**
     * @brief ターゲット人物のTFフレームを配信
     */
    void publishTargetTF(const geometry_msgs::msg::Point& target_pos) {
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = this->now();
        transform_stamped.header.frame_id = "zed_camera_link";
        transform_stamped.child_frame_id = "target_person";

        // 位置設定（少し手前にオフセット）
        transform_stamped.transform.translation.x = target_pos.x - 0.6;  // 60cm手前
        transform_stamped.transform.translation.y = target_pos.y;
        transform_stamped.transform.translation.z = target_pos.z;

        // 回転設定（人物の方向を向く）
        double yaw = std::atan2(target_pos.y, target_pos.x);
        transform_stamped.transform.rotation.x = 0.0;
        transform_stamped.transform.rotation.y = 0.0;
        transform_stamped.transform.rotation.z = std::sin(yaw / 2.0);
        transform_stamped.transform.rotation.w = std::cos(yaw / 2.0);

        static_tf_broadcaster_->sendTransform(transform_stamped);
    }

    /**
     * @brief 状態メッセージ公開
     */
    void publishStatus(const std::string& status) {
        std_msgs::msg::String status_msg;
        status_msg.data = status;
        tracking_status_publisher_->publish(status_msg);
    }

    /**
     * @brief 追跡タイムアウトチェック
     */
    void checkTrackingTimeout() {
        if (is_tracking_) {
            auto elapsed = (this->now() - last_detection_time_).seconds();
            if (elapsed > tracking_timeout_) {
                RCLCPP_WARN(this->get_logger(),
                    "⏱️  Tracking timeout (%.1f sec). Resetting target.", elapsed);
                is_tracking_ = false;
                tracked_person_id_ = -1;
                stopRobot();
                publishStatus("Tracking timeout - waiting for new target");
            }
        }
    }

    // メンバ変数
    std::string skeleton_topic_;
    std::string cmd_vel_topic_;
    double target_distance_;
    double distance_tolerance_;
    double angle_tolerance_;
    double max_linear_speed_;
    double max_angular_speed_;
    double hand_raise_threshold_;
    double tracking_timeout_;

    bool is_tracking_ = false;
    int32_t tracked_person_id_ = -1;
    rclcpp::Time last_detection_time_;

    rclcpp::Subscription<zed_msgs::msg::ObjectsStamped>::SharedPtr skeleton_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tracking_status_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr target_id_publisher_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timeout_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HandRaisedPersonFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
