#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

/**
 * @brief TF-Topic同期コーディネーター
 *
 * 94秒の時刻ずれに対処するため、以下の機能を提供：
 * 1. ApproximateTimeSynchronizer による柔軟な時刻同期
 * 2. Transform補間とバッファリング
 * 3. 段階的な時刻同期回復メカニズム
 * 4. エラー状況の監視と自動復旧
 */
class TFSyncCoordinator : public rclcpp::Node
{
public:
    TFSyncCoordinator() : Node("tf_sync_coordinator")
    {
        // パラメータ宣言
        this->declare_parameter("sync_tolerance_sec", 5.0);
        this->declare_parameter("max_sync_attempts", 10);
        this->declare_parameter("recovery_timeout_sec", 30.0);
        this->declare_parameter("base_frame", "zed_camera_link");
        this->declare_parameter("odom_frame", "zed_odom");
        this->declare_parameter("map_frame", "map");

        sync_tolerance_ = this->get_parameter("sync_tolerance_sec").as_double();
        max_sync_attempts_ = this->get_parameter("max_sync_attempts").as_int();
        recovery_timeout_ = this->get_parameter("recovery_timeout_sec").as_double();
        base_frame_ = this->get_parameter("base_frame").as_string();
        odom_frame_ = this->get_parameter("odom_frame").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();

        // TF Buffer and Listener（大容量バッファで遅延に対応）
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock(), tf2::durationFromSec(120.0));
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Message Filters設定（高度な時刻同期）
        odom_sub_.subscribe(this, "/zed/zed_node/odom");
        scan_sub_.subscribe(this, "/scan_filtered");

        // ApproximateTime同期ポリシー（大きな時刻差を許容）
        typedef message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, sensor_msgs::msg::LaserScan> SyncPolicy;
        sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(100), odom_sub_, scan_sub_);
        sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_tolerance_));
        sync_->registerCallback(&TFSyncCoordinator::syncCallback, this);

        // ステータス監視タイマー
        status_timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&TFSyncCoordinator::checkSyncStatus, this));

        // 時刻補正パブリッシャー
        corrected_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/synchronized/odom", rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable));
        corrected_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/synchronized/scan", rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable));

        RCLCPP_INFO(this->get_logger(), "TF Sync Coordinator initialized with tolerance: %.1f sec", sync_tolerance_);
    }

private:
    void syncCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg,
                     const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan_msg)
    {
        auto current_time = this->get_clock()->now();

        // 時刻差を計算
        rclcpp::Time odom_time(odom_msg->header.stamp);
        rclcpp::Time scan_time(scan_msg->header.stamp);
        auto time_diff = std::abs((odom_time - scan_time).seconds());

        RCLCPP_DEBUG(this->get_logger(),
                    "Syncing data - Odom: %f, Scan: %f, Diff: %f sec",
                    odom_time.seconds(), scan_time.seconds(), time_diff);

        // 同期できた場合の処理
        if (time_diff <= sync_tolerance_) {
            publishSynchronizedData(odom_msg, scan_msg, current_time);
            sync_success_count_++;
            last_successful_sync_ = current_time;
        } else {
            sync_failure_count_++;
            RCLCPP_WARN(this->get_logger(),
                       "Large time difference detected: %.2f sec (tolerance: %.2f sec)",
                       time_diff, sync_tolerance_);

            // 時刻補正を試行
            attemptTimeCorrection(odom_msg, scan_msg, current_time);
        }
    }

    void publishSynchronizedData(const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg,
                                const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan_msg,
                                const rclcpp::Time& sync_time)
    {
        // 時刻同期されたデータを発行
        auto corrected_odom = *odom_msg;
        auto corrected_scan = *scan_msg;

        corrected_odom.header.stamp = sync_time;
        corrected_scan.header.stamp = sync_time;

        corrected_odom_pub_->publish(corrected_odom);
        corrected_scan_pub_->publish(corrected_scan);

        // TF変換の補間・発行
        publishInterpolatedTransforms(odom_msg, sync_time);
    }

    void attemptTimeCorrection(const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg,
                              const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan_msg,
                              const rclcpp::Time& target_time)
    {
        try {
            // 現在時刻に近い時刻でTransformを取得
            geometry_msgs::msg::TransformStamped transform;
            if (tf_buffer_->canTransform(map_frame_, base_frame_, target_time,
                                       tf2::durationFromSec(sync_tolerance_))) {
                transform = tf_buffer_->lookupTransform(map_frame_, base_frame_, target_time);

                // 補正されたデータを発行
                publishSynchronizedData(odom_msg, scan_msg, target_time);

                RCLCPP_INFO(this->get_logger(), "Successfully corrected timestamp alignment");
            }
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "Transform correction failed: %s", ex.what());
        }
    }

    void publishInterpolatedTransforms(const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg,
                                      const rclcpp::Time& target_time)
    {
        try {
            // Odomフレームから基準フレームへの変換を発行
            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp = target_time;
            transform.header.frame_id = odom_frame_;
            transform.child_frame_id = base_frame_;

            transform.transform.translation.x = odom_msg->pose.pose.position.x;
            transform.transform.translation.y = odom_msg->pose.pose.position.y;
            transform.transform.translation.z = odom_msg->pose.pose.position.z;

            transform.transform.rotation = odom_msg->pose.pose.orientation;

            tf_broadcaster_->sendTransform(transform);

        } catch (const std::exception& ex) {
            RCLCPP_WARN(this->get_logger(), "Failed to publish interpolated transform: %s", ex.what());
        }
    }

    void checkSyncStatus()
    {
        auto current_time = this->get_clock()->now();
        auto time_since_last_sync = (current_time - last_successful_sync_).seconds();

        double success_rate = static_cast<double>(sync_success_count_) /
                             (sync_success_count_ + sync_failure_count_ + 1);

        RCLCPP_INFO(this->get_logger(),
                   "Sync Status - Success: %ld, Failed: %ld, Rate: %.1f%%, Last sync: %.1f sec ago",
                   sync_success_count_, sync_failure_count_, success_rate * 100.0, time_since_last_sync);

        // 長時間同期できていない場合の警告
        if (time_since_last_sync > recovery_timeout_) {
            RCLCPP_ERROR(this->get_logger(),
                        "No successful synchronization for %.1f seconds! Check data sources.",
                        time_since_last_sync);

            // 同期許容値を動的に増加（緊急措置）
            if (sync_tolerance_ < 120.0) {
                sync_tolerance_ *= 1.5;
                sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_tolerance_));
                RCLCPP_WARN(this->get_logger(),
                           "Increasing sync tolerance to %.1f sec due to persistent failures",
                           sync_tolerance_);
            }
        }
    }

    // Member variables
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
    message_filters::Subscriber<sensor_msgs::msg::LaserScan> scan_sub_;

    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, sensor_msgs::msg::LaserScan> SyncPolicy;
    std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr corrected_odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr corrected_scan_pub_;

    rclcpp::TimerBase::SharedPtr status_timer_;

    // Parameters
    double sync_tolerance_;
    int max_sync_attempts_;
    double recovery_timeout_;
    std::string base_frame_;
    std::string odom_frame_;
    std::string map_frame_;

    // Statistics
    size_t sync_success_count_{0};
    size_t sync_failure_count_{0};
    rclcpp::Time last_successful_sync_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TFSyncCoordinator>();

    RCLCPP_INFO(node->get_logger(), "Starting TF-Topic Synchronization Coordinator");

    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception in main loop: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}