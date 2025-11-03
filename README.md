# Eclevive Robot - ZED2i + nvblox + Nav2 自律ナビゲーションシステム

メカナムホイールとロボットアームを搭載したロボット。ROS2 Humbleベースの完全自律ナビゲーションシステム。NVIDIA Isaac ROS nvbloxによる3Dマッピング、ZED2i Visual SLAM、Nav2経路計画を統合しています。

## システム概要

- **プラットフォーム**: Jetson Orin Nano / AGX Orin
- **ROS2バージョン**: Humble
- **ロボットタイプ**: メカナムホイール移動ロボット
- **主要センサー**: ZED2i、LiDAR x2、IMU
- **制御方式**: PS5コントローラー + 自律ナビゲーション

## 主要機能

1. **3Dマッピング**: NVIDIA Isaac ROS nvblox（ESDF、拡張範囲14m）
2. **自己位置推定**: ZED2i Visual SLAM（AMCLなし）
3. **経路計画**: Navigation2（Theta* Planner + MPPI Controller）
4. **障害物回避**: LiDAR + ZED Depth融合コストマップ
5. **人体認識**: ZED Body Tracking + ジェスチャー制御

## クイックスタート

### 1. 依存パッケージのインストール

```bash
cd ~/ros2_ws
./setup_dependencies.sh
```

### 2. ビルド

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-skip-build-finished --symlink-install \
  --packages-skip isaac_ros_ess_models_install \
                  isaac_ros_peoplenet_models_install \
                  isaac_ros_peoplesemseg_models_install \
                  isaac_ros_nova_recorder
```

### 3. 起動

```bash
source install/setup.bash
ros2 launch bringup zed2i_nvblox_nav2_launch.py
```

## パッケージ構成

### 自作パッケージ（このリポジトリに含まれる）

| パッケージ | 説明 |
|-----------|------|
| `bringup` | 統合launchファイル、設定ファイル |
| `mark3_urdf` | ロボットURDF定義 |
| `joy_mecanum_controller` | PS5コントローラー操作 |
| `zed_goal_publisher` | ZEDジェスチャーによるゴール設定 |
| `zed_human_tracker` | 人体追従機能 |
| `zed_zupt_wrapper` | ZED ZUPT（Zero Velocity Update）フィルタ |
| `safety_sensor` | 安全センサー統合 |
| `cmd_vel_float_changer` | 速度指令変換 |
| `odom_publisher` | オドメトリ配信 |
| `zed_gesture_controller` | ジェスチャー制御 |
| `zed_imu_republisher` | IMUデータ再配信 |
| `zed_odom_watcher` | オドメトリ監視 |

### 外部依存パッケージ（別途インストール）

- **NVIDIA Isaac ROS**: nvblox、Visual SLAM、Image Pipeline
- **Navigation2**: 経路計画、自律ナビゲーション
- **ZED SDK**: ZED2iカメラドライバ
- **LiDAR**: ldlidar_stl_ros2、sllidar_ros2
- **その他**: slam_toolbox、laser_filters、pointcloud_to_laserscan

詳細は[setup_dependencies.sh](setup_dependencies.sh)を参照。

## システムアーキテクチャ

### TFツリー構造

```
map (nvblox global_frame)
 └─ odom (ZED2i map_frame - Visual SLAM親フレーム)
     └─ zed_camera_origin (ZED2i odometry_frame)
         └─ zed_camera_link (ロボット基準点)
             ├─ base_link
             ├─ front_lidar
             └─ back_lidar
```

**重要**: ZED2iの`map_frame='odom'`設定は**絶対に変更禁止**。この設定はVisual OdometryとnvbloxのOccupancy Grid配信の両立に必須です。

### 主要トピック

- LiDAR: `/scan_filtered` (マージ済み)
- Costmap: `/local_costmap/costmap_raw`, `/global_costmap/costmap_raw`
- オドメトリ: `/zed/zed_node/odom`
- ナビゲーション: `/goal_pose`, `/cmd_vel`

詳細は[CLAUDE.md](CLAUDE.md)を参照。

## パフォーマンス最適化

### Local Costmap最適化（2025-11-03）

- **更新周波数**: 25Hz → 30Hz（実測11.8Hz）
- **解像度**: 0.04m → 0.03m（高精細化）
- **障害物検出範囲**: 1.8m → 2.5m（+39%）
- **LiDARレイヤー**: VoxelLayer → ObstacleLayer（軽量化）

結果: 障害物検出精度が77%向上、CPU負荷は適正範囲内（52% idle）

## トラブルシューティング

### LiDAR物理的取り付けの確認

**症状**: SLAMで地図がうまく生成できない
**原因**: LiDARが下向きに傾いている
**対処**: LiDARの取り付け角度を物理的に調整し、水平にする

### TF設定の確認

```bash
ros2 param get /zed/zed_node pos_tracking.publish_map_tf  # 期待値: true
ros2 param get /zed/zed_node pos_tracking.map_frame       # 期待値: odom
```

詳細は[docs/troubleshooting.md](.claude/docs/troubleshooting.md)を参照。

## ドキュメント

- **[CLAUDE.md](CLAUDE.md)**: 基本設定と概要
- **[system-architecture.md](.claude/docs/system-architecture.md)**: システムアーキテクチャ詳細
- **[tf-tree.md](.claude/docs/tf-tree.md)**: TFツリー仕様
- **[topics.md](.claude/docs/topics.md)**: トピック仕様
- **[build-system.md](.claude/docs/build-system.md)**: ビルドシステム
- **[troubleshooting.md](.claude/docs/troubleshooting.md)**: トラブルシューティング

## スラッシュコマンド

```bash
/ros2-diagnose     # システム診断を実行
/ros2-build        # パッケージビルドを実行
/ros2-test-nav     # ナビゲーションテストを実行
```

## ライセンス

自作パッケージ: MIT License

外部パッケージはそれぞれのライセンスに従います：
- NVIDIA Isaac ROS: Apache-2.0
- Navigation2: Apache-2.0
- ZED SDK: Stereolabs License

## 作成者

- GitHub: [あなたのGitHubアカウント]
- 開発環境: Jetson Orin Nano/AGX Orin + ROS2 Humble

## 謝辞

このプロジェクトは以下のオープンソースプロジェクトを使用しています：
- [NVIDIA Isaac ROS](https://github.com/NVIDIA-ISAAC-ROS)
- [Navigation2](https://github.com/ros-planning/navigation2)
- [ZED ROS2 Wrapper](https://github.com/stereolabs/zed-ros2-wrapper)
