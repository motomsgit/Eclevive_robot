# 主要トピック仕様

## センサートピック

### LiDAR
- `/front_scan` - 前方LiDAR生データ (`sensor_msgs/msg/LaserScan`)
- `/back_scan` - 後方LiDAR生データ (`sensor_msgs/msg/LaserScan`)
- `/merged_scan` - マージされたスキャン (`sensor_msgs/msg/LaserScan`)
  - QoS: RELIABLE, TRANSIENT_LOCAL
  - Publisher: `laser_merger2`
- `/scan_filtered` - フィルタ後スキャン (ロボット本体除去)
  - nvbloxとNav2に入力

### オドメトリ
- `/zed/zed_node/odom` - ZED Visual Odometry (生データ) (`nav_msgs/msg/Odometry`)
- `/zed/zed_node/odom_zupt` - ZUPT Filter適用後 (`nav_msgs/msg/Odometry`)
- `/zed/zed_node/pose` - ZED Pose (nvbloxに入力) (`geometry_msgs/msg/PoseStamped`)

### IMU
- `/zed/zed_node/imu/data` - フィルタ済みIMU (`sensor_msgs/msg/Imu`)
- `/zed/zed_node/imu/data_raw` - 生IMU (`sensor_msgs/msg/Imu`)
- `/zed/zed_node/imu/mag` - 磁気センサー (`sensor_msgs/msg/MagneticField`)

### カメラ（ZED2i）
- `/zed/zed_node/rgb/image_rect_color` - RGB画像 (nvbloxに入力)
- `/zed/zed_node/rgb/camera_info` - RGBカメラ情報
- `/zed/zed_node/depth/depth_registered` - 深度画像 (nvbloxに入力)
- `/zed/zed_node/depth/camera_info` - 深度カメラ情報
- `/zed/zed_node/point_cloud/cloud_registered` - ポイントクラウド

### Body Tracking（ZED2i）
- `/zed/zed_node/body_trk/skeletons` - 人体骨格検出
  - 型: `zed_interfaces/msg/ObjectsStamped`
  - ZED Goal Publisherで使用

## 制御トピック

### 速度指令
- `/cmd_vel` - 統合速度指令 (`geometry_msgs/msg/Twist`)
  - QoS: RELIABLE, VOLATILE
  - Publishers: velocity_smoother, behavior_server, zed_goal_publisher
  - Subscribers: joy_mecanum_controller, cmd_vel_float_changer

- `/cmd_vel_teleop` - テレオペ指令
- `/cmd_vel_nav` - ナビゲーション指令
- `/mecanum/cmd_vel` - メカナム変換後指令

### ゴール・ナビゲーション
- `/goal_pose` - ゴール位置 (`geometry_msgs/msg/PoseStamped`)
- `/initialpose` - 初期位置設定

## マップ・3Dマッピング（nvblox）

- `/map` - nvblox Occupancy Grid（Nav2互換）
  - 型: `nav_msgs/msg/OccupancyGrid`
  - QoS: RELIABLE, TRANSIENT_LOCAL
  - Publisher: `nvblox_node`
  - リマップ元: `/nvblox_node/static_occupancy_grid`

- `/nvblox_node/mesh` - 3Dメッシュ (`nvblox_msgs/msg/Mesh`)
- `/nvblox_node/map_slice` - マップスライス（2D可視化用、14m範囲）
- `/nvblox_node/esdf_pointcloud` - ESDF点群（コストマップソース）
  - Global Costmapで使用
- `/nvblox_node/pointcloud` - 統合3D点群

## ナビゲーション（Nav2）

- `/goal_pose` - ナビゲーションゴール
  - ZED Goal PublisherまたはRViz2から配信
- `/plan` - グローバルプラン (`nav_msgs/msg/Path`)
- `/plan_smoothed` - スムージング後プラン
- `/transformed_global_plan` - 変換後グローバルプラン
- `/local_costmap/costmap` - 局所コストマップ
- `/global_costmap/costmap` - 大域コストマップ

## デバッグ・ステータス

- `/zupt/status` - ZUPT Filter状態（デバッグ情報）

## ジョイスティック

- `/joy` - PS5コントローラー入力 (`sensor_msgs/msg/Joy`)
  - 更新レート: 30 Hz

## QoS設定

### RELIABLE + TRANSIENT_LOCAL
- `/scan_filtered` (SLAM用スキャン保持)
- `/map` (マップデータ保持)

### RELIABLE + VOLATILE
- `/cmd_vel` (速度指令)
- その他制御トピック

### BEST_EFFORT + VOLATILE
- センサー生データ (高頻度更新)
