# ロボットシステムアーキテクチャ

## システム概要

### 起動コマンド
- **現行版（推奨）**: `ros2 launch bringup zed2i_nvblox_nav2_launch.py`
  - ZED2i + nvblox（3Dマッピング） + Nav2統合システム
  - **特徴**:
    - NVIDIA Isaac ROS nvbloxによる3Dマッピング（ESDF生成）
    - ZED2i Visual SLAMで自己位置推定（AMCLなし）
    - LiDAR統合による高精度マッピング
    - ZED ZUPT Filterによるオドメトリ安定化
    - ZED Body Trackingによるジェスチャー制御

- **旧版（非推奨）**: `ros2 launch bringup bringup_ps5_all_launch.py`
  - SLAM Toolbox使用版（2D SLAM）
  - 非推奨の理由: 2D SLAM、TF同期の複雑性、nvbloxに比べて精度が低い

### ハードウェア構成
- **プラットフォーム**: Jetson (Linux 5.15.148-tegra)
- **ロボットタイプ**: メカナムホイール移動ロボット
- **主要センサー**:
  - ZED2i ステレオカメラ（Visual SLAM、Body Tracking）
  - LiDAR x2（前後、障害物検出）
  - IMU（ZED2i内蔵）
- **制御方式**: PS5コントローラー (DualSense Wireless Controller)

## 実行中のノード数（zed2i_nvblox_nav2_launch.py使用時）
- **総ノード数**: 約40-50ノード（nvblox統合版）
- **総トピック数**: 約150-200トピック

## 主要ノード一覧

### センサー・デバイス系
- **ZED2iカメラ** (zed_components/ZedCamera - Composable Node)
  - ステレオカメラ、IMU、Visual Odometry、Body Tracking
  - 設定: `common_stereo_isaac.yaml`

- **LiDAR** (ldlidar_stl_ros2)
  - 前方: `/front_scan`、後方: `/back_scan`
  - マージ: `/merged_scan` → `/scan_filtered`

- **IMU** (ZED2i内蔵)
  - `/zed/zed_node/imu/data` - フィルタ済み
  - `/zed/zed_node/imu/data_raw` - 生データ

### 制御・通信系
- **micro_ros_agent**: マイコン通信 (multiserial接続)
- **joy_linux**: PS5コントローラー (30 Hz)
- **joy_mecanum_controller**: メカナム制御変換

### マッピング・ナビゲーション系（現行版）
- **nvblox_node** (NVIDIA Isaac ROS 3Dマッピング)
  - ESDF生成、拡張範囲: 14m
  - Occupancy Grid: `/map` (Nav2互換)
  - LiDAR統合: 有効

- **Navigation2スタック**
  - controller_server、planner_server、behavior_server
  - bt_navigator、smoother_server、velocity_smoother
  - **AMCLは不使用**（ZED Visual Odometry + nvbloxで自己位置推定）

### 独自開発ノード
- **zed_zupt_filter_node**: オドメトリ暴走防止
- **laser_merger2**: 前後LiDARスキャンのマージ
- **zed_goal_publisher**: ジェスチャー制御ゴール配信
- **safety_sensor**: 安全監視

## データフロー（現行版）
1. センサー取得: ZED2i (VO + IMU + RGB + Depth), LiDAR前後
2. オドメトリ安定化: ZED ZUPT Filter → `/zed/zed_node/odom_zupt`
3. スキャンマージ: laser_merger2 → `/merged_scan`
4. フィルタリング: laser_filters → `/scan_filtered`
5. 3Dマッピング: nvblox (RGB + Depth + LiDAR → ESDF + Occupancy Grid)
6. マップ配信: nvblox → `/map` (Nav2互換)
7. ナビゲーション: Nav2 (プランニング → 制御)
8. 速度スムージング: velocity_smoother
9. メカナム変換: joy_mecanum_controller
10. 実機制御: micro_ros_agent → マイコン

## Launch構造（現行版）
```
zed2i_nvblox_nav2_launch.py (メイン)
├─ zed2i_nvblox_fixed.launch.py (センサー・nvblox統合)
│  ├─ Component Container (nvblox_container)
│  ├─ Robot State Publisher (ZED URDF)
│  ├─ ZED Camera Node (2秒後ロード)
│  ├─ LiDAR (前後)
│  ├─ laser_merger + box_filter
│  ├─ micro_ros_agent, joy_linux, joy_mecanum_controller
│  ├─ static_transform_publishers
│  ├─ ZED ZUPT Filter (3秒後)
│  ├─ nvblox Node (4秒後)
│  ├─ ZED Goal Publisher (5秒後)
│  └─ RViz2 (6秒後、オプション)
└─ my_navigation_launch.py (Nav2、8秒後)
```

### 必須パラメータファイル
- ZED2i: `common_stereo_isaac.yaml`, `zed2i.yaml`
- nvblox: `nvblox_base.yaml`, `nvblox_zed.yaml`, `nvblox_extended_range.yaml`, `nvblox_nav2_integration.yaml`
- Navigation: `my_nav2_params_mppi1.yaml`
- ZED ZUPT Filter: `zed_zupt_params.yaml`
- ZED Goal Publisher: `zed_goal_publisher.yaml`
