# TFツリー仕様

## 標準TFツリー（ZED2i + nvblox + Nav2統合）【2025-11-02更新版】

**重要**: ZED2iは`odom → zed_camera_origin`のTFを配信する構成です。

```
map (nvblox global_frame)
 └─ odom (ZED2i map_frame - Visual SLAM親フレーム)
     └─ zed_camera_origin (ZED2i odometry_frame - Visual Odometryフレーム)
         └─ zed_camera_link (ZED2i base_frame、ロボット基準点)
             ├─ base_link (ロボット中心, static)
             ├─ front_lidar (前方LiDAR, static)
             ├─ back_lidar (後方LiDAR, static, -0.296m X, -0.03m Z, 180° pitch)
             ├─ target_person (人物追跡用, static)
             └─ zed_camera_center (カメラ中心, static)
                 ├─ zed_left_camera_frame (左カメラ, static)
                 │   └─ zed_left_camera_optical_frame (光学座標系, static)
                 └─ zed_right_camera_frame (右カメラ, static)
                     └─ zed_right_camera_optical_frame (光学座標系, static)
```

## 重要な設計決定（2025-11-02更新）
- `map → odom`は`static_transform_publisher`が静的TF（0, 0, 0）で固定配信
- `odom → zed_camera_origin`の接続はZED2iが動的TFで配信（`publish_map_tf: true`）
- ZED2iの`map_frame`パラメータは`'odom'`を使用（ZED2iにとっての最上位フレーム）
- ZED2iの`odometry_frame`パラメータは`'zed_camera_origin'`を使用
- nvbloxは`global_frame: 'map'`でマップを配信するが、TFは配信しない

## TF配信責任とフレーム名の定義

| TF変換 | 配信ノード | 説明 | 設定パラメータ |
|--------|-----------|------|---------------|
| `map` → `odom` | static_transform_publisher | **静的TF**：nvbloxのmapフレームとZED2iのodomフレームを固定結合 | launchファイル内で定義<br>`['0', '0', '0', '0', '0', '0', 'map', 'odom']` |
| `odom` → `zed_camera_origin` | ZED2i (zed_node) | ZED2i最上位フレームから中間フレームへの変換（Visual SLAM） | `map_frame: 'odom'`<br>`odometry_frame: 'zed_camera_origin'`<br>`publish_map_tf: true` |
| `zed_camera_origin` → `zed_camera_link` | ZED2i (zed_node) | 中間フレームからカメラリンクへの変換（Visual Odometry） | `base_frame: 'zed_camera_link'`<br>`publish_tf: true` |
| `zed_camera_link` → `base_link` | static_transform_publisher | カメラからロボット中心への変換 | `0.0 0.0 0.0 0.0 0.0 3.14159` |
| `zed_camera_link` → `front_lidar` | static_transform_publisher | カメラから前方LiDARへの変換 | `0.0 0 -0.03 0 0 0` |
| `zed_camera_link` → `back_lidar` | static_transform_publisher | カメラから後方LiDARへの変換 | `-0.296 0 -0.03 0 3.14159 0` |

## TF配信レート（実測値）
- `map` → `odom`: Static（static_transform_publisher、固定値）
- `odom` → `zed_camera_origin`: ~30 Hz（Visual SLAM、動的TF）
- `zed_camera_origin` → `zed_camera_link`: ~30 Hz（Visual Odometry）
- 静的TF（base_link, LiDAR等）: 10000 Hz (static_transform_publisher)

## フレーム名の役割
- **`map`**: nvbloxが配信するグローバル座標系（世界基準、3Dマッピング用）
- **`odom`**: ZED2iにとっての最上位フレーム（ZED2iが`odom → zed_camera_origin`のTFを動的配信）
- **`zed_camera_origin`**: ZED内部の中間フレーム（Visual SLAM基準点）
- **`zed_camera_link`**: ZEDカメラの物理的取り付け位置（ロボット基準点、Visual Odometry基準点）
- **`base_link`**: ロボットの幾何中心（制御基準点）
- **`front_lidar` / `back_lidar`**: 各LiDARセンサーの位置

## Nav2統合時のフレーム設定

**重要**: 現行版では**AMCLは使用しません**。自己位置推定はZED Visual Odometry + nvbloxで実現します。

### Local Costmap（局所コストマップ）
```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      global_frame: odom                 # オドメトリ基準で動的更新
      robot_base_frame: zed_camera_link  # ロボット基準
```

### Global Costmap（大域コストマップ）
```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      global_frame: map                  # マップ基準で静的
      robot_base_frame: zed_camera_link  # ロボット基準
```

### Behavior Server（行動制御）
```yaml
behavior_server:
  ros__parameters:
    global_frame: odom                    # オドメトリ基準
    robot_base_frame: zed_camera_link     # ロボット基準
```

## nvblox統合時の注意事項

**この設定で自律ナビゲーションが正常動作することを実機で確認済みです。**

1. **nvbloxの`global_frame`設定**: `map`を使用
   - nvbloxは`map`フレームでOccupancy Gridを配信（トピック: `/map`）
   - nvblox自体はTFを配信しない

2. **ZED2iの`publish_map_tf`設定**: `true`に設定（重要）
   - ZED2iが`odom → zed_camera_origin`のTFを動的に配信
   - Visual SLAMの位置推定を反映

3. **ZED2iの`map_frame`パラメータ**: `'odom'`を使用（最も重要）
   - ZED2iにとっての最上位フレーム
   - `publish_map_tf: true`により`odom → zed_camera_origin`のTFを配信

4. **ZED2iの`odometry_frame`パラメータ**: `'zed_camera_origin'`を使用
   - Visual SLAMの基準フレーム

5. **TFツリーの完全性確認が重要**
   - `map` → `odom` → `zed_camera_origin` → `zed_camera_link` が必須
   - どこかが欠けるとNav2が動作しない
   - `map → odom`のTFは`static_transform_publisher`が静的に固定（0, 0, 0）

6. **LiDAR統合**: 有効化（`use_lidar: true`）
   - nvbloxがLiDARスキャンを統合してマッピング精度を向上

## TFツリー確認コマンド

```bash
# TFツリー全体を可視化
ros2 run tf2_tools view_frames

# 特定の座標変換を確認
ros2 run tf2_ros tf2_echo map zed_camera_link

# ZED2iのTF設定を確認
ros2 param get /zed/zed_node pos_tracking.map_frame        # 期待値: odom
ros2 param get /zed/zed_node pos_tracking.odometry_frame   # 期待値: zed_camera_origin
ros2 param get /zed/zed_node pos_tracking.base_frame       # 期待値: zed_camera_link
ros2 param get /zed/zed_node pos_tracking.publish_map_tf   # 期待値: true

# 動的TFの確認
ros2 topic echo /tf | grep -A 10 "odom.*zed_camera_origin"

# nvbloxのマップ配信確認
ros2 topic echo /map --once
```
