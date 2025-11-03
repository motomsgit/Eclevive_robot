# zed_goal_publisher

ZED2iカメラを使用した人物認識とジェスチャー制御によるロボット操作パッケージ

## 概要

このパッケージは、ZED2iステレオカメラで人物の骨格を認識し、ジェスチャーに応じてメカナムホイールロボットを制御します。

## 機能

### ジェスチャー認識
- **両手を肩から上に上げる** → 前進
- **右手を曲げて上げる** → 右旋回
- **左手を曲げて上げる** → 左旋回
- **右手を水平に伸ばす** → 右移動
- **左手を水平に伸ばす** → 左移動

## 起動方法

### Launch ファイルを使用（推奨）

```bash
ros2 launch zed_goal_publisher zed_goal_publisher.launch.py
```

### 直接実行

```bash
ros2 run zed_goal_publisher zed_goal_publisher --ros-args --params-file $(ros2 pkg prefix zed_goal_publisher)/share/zed_goal_publisher/config/zed_goal_publisher.yaml
```

## パラメータ設定

パラメータは `config/zed_goal_publisher.yaml` で設定できます。

### トピック名設定

```yaml
skeleton_topic: "zed/zed_node/body_trk/skeletons"  # ZEDの骨格認識トピック
target_arm_pose_topic: "target_arm_pose"            # アーム目標姿勢
current_arm_pose_topic: "current_arm_pose"          # アーム現在姿勢
cmd_vel_topic: "cmd_vel"                            # 速度指令出力
goal_pose_topic: "goal_pose"                        # ゴール位置
zed_goal_status_topic: "zed_goal_status"            # ゴール状態
```

### 速度パラメータ

#### 前進（両手上げ）
```yaml
forward_vel_fast: 0.5    # 前進高速（m/s）
forward_vel_slow: 0.1    # 前進低速（m/s）
```

#### 右旋回（右手曲げ）
```yaml
turn_right_vel_fast: 1.6   # 右旋回高速（rad/s）
turn_right_vel_slow: 0.05  # 右旋回低速（rad/s）
```

#### 左旋回（左手曲げ）
```yaml
turn_left_vel_fast: -1.6   # 左旋回高速（rad/s）
turn_left_vel_slow: -0.05  # 左旋回低速（rad/s）
```

#### 右移動（右手水平）
```yaml
strafe_right_vel_fast: 0.70   # 右移動高速（m/s）
strafe_right_vel_slow: 0.10   # 右移動低速（m/s）
```

#### 左移動（左手水平）
```yaml
strafe_left_vel_fast: -0.70   # 左移動高速（m/s）
strafe_left_vel_slow: -0.10   # 左移動低速（m/s）
```

## パラメータ調整方法

1. `config/zed_goal_publisher.yaml` を編集
2. 速度値を変更（例：もっとゆっくり動かしたい場合は値を小さくする）
3. パッケージをビルド（symlink-installを使用している場合は不要）

```bash
colcon build --packages-select zed_goal_publisher --symlink-install
```

4. 再起動

```bash
ros2 launch zed_goal_publisher zed_goal_publisher.launch.py
```

## トピック

### Subscribe
- `/zed/zed_node/body_trk/skeletons` (zed_msgs/msg/ObjectsStamped) - 骨格認識データ
- `/target_arm_pose` (std_msgs/msg/Int32MultiArray) - アーム目標姿勢

### Publish
- `/cmd_vel` (geometry_msgs/msg/Twist) - 速度指令
- `/goal_pose` (geometry_msgs/msg/PoseStamped) - ナビゲーションゴール
- `/target_arm_pose` (std_msgs/msg/Int32MultiArray) - アーム目標姿勢
- `/current_arm_pose` (std_msgs/msg/Int32MultiArray) - アーム現在姿勢
- `/zed_goal_status` (std_msgs/msg/Int32) - ゴール状態

## ファイル構成

```
zed_goal_publisher/
├── src/
│   └── zed_goal_publisher.cpp      # メインプログラム
├── launch/
│   └── zed_goal_publisher.launch.py # Launchファイル
├── config/
│   └── zed_goal_publisher.yaml      # パラメータ設定
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 依存関係

- rclcpp
- rclcpp_action
- std_msgs
- zed_msgs
- geometry_msgs
- nav2_msgs
- tf2
- tf2_ros
- tf2_geometry_msgs
- sensor_msgs

## 改良履歴

### 2025-10-12
- トピック名をYAMLパラメータから設定可能に変更
- 速度値（linear.x, linear.y, angular.z）をYAMLパラメータ化
- launchファイル追加
- パラメータファイル追加
