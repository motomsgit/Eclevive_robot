# nvblox + Nav2 統合ガイド（/mapトピック版）

## 概要

このドキュメントは、nvbloxが生成する`/map`トピックと`map`フレームを使用してNav2自動航行を実現する設定を説明します。

## 実装内容

### 1. nvbloxから/mapトピックを出力

**目的**: Nav2が期待する`/map`トピック（`nav_msgs/OccupancyGrid`）をnvbloxから直接配信

**設定ファイル**:
- [nvblox_nav2_integration.yaml](/home/jetros/ros2_ws/src/bringup/config/nvblox/nvblox_nav2_integration.yaml)

**主要設定**:
```yaml
global_frame: "map"                      # mapフレームで出力
publish_occupancy_layer: true            # Occupancy Grid出力を有効化
mapping_type: "static_tsdf"              # TSDFベースの静的マッピング

static_mapper:
  free_region_occupancy_probability: 0.3        # 空き領域（低確率）
  occupied_region_occupancy_probability: 0.9    # 占有領域（高確率）
  unobserved_region_occupancy_probability: 0.5  # 未観測領域（中間）
```

**トピックリマッピング**:
```python
# zed2i_nvblox_fixed.launch.py
nvblox_remappings = [
    # ... ZEDトピック ...
    ('static_occupancy_layer', '/map'),  # nvbloxの出力を/mapにリマップ
]
```

### 2. ZEDからmap TFを配信

**目的**: Nav2が期待する`map -> zed_odom -> zed_camera_link`のTFツリーを構築

**設定**:
```python
# zed2i_nvblox_fixed.launch.py
{
    'pos_tracking.publish_map_tf': True,       # map TF配信を有効化
    'pos_tracking.map_frame': 'map',           # mapフレーム名
    'pos_tracking.odometry_frame': 'zed_odom', # オドメトリフレーム名
}
```

**TFツリー**:
```
map (ZED Visual SLAM)
 └─ zed_odom (Visual Odometry)
     └─ zed_camera_link (ロボット基準)
         ├─ base_link
         ├─ front_lidar
         └─ back_lidar
```

### 3. Nav2 Global CostmapでnvbloxのMapを使用

**設定**:
```yaml
# my_nav2_params_mppi1.yaml
global_costmap:
  global_costmap:
    plugins: ["voxel_layer", "obstacle_layer", "denoise_layer", "static_layer", "inflation_layer"]

    # nvblox ESDFポイントクラウド用
    voxel_layer:
      observation_sources: pointcloud
      pointcloud:
        topic: /nvblox_node/esdf_pointcloud
        data_type: "PointCloud2"

    # nvblox Occupancy Grid用
    static_layer:
      map_subscribe_transient_local: False  # nvbloxはVolatile QoS
      map_topic: /map                       # nvbloxからの/map

    # LiDARスキャン用
    obstacle_layer:
      observation_sources: scan
      scan:
        topic: /merged_scan_filtered
```

## システム構成

### データフロー

```
ZED2iカメラ
  ├─ Depth画像 ──→ nvblox ──→ /nvblox_node/esdf_pointcloud ──→ Global Costmap (voxel_layer)
  ├─ Pose ──────→ nvblox ──→ /map (OccupancyGrid) ─────────→ Global Costmap (static_layer)
  └─ Pose ──────→ ZED SDK → map TF (map → zed_odom)

LiDAR前後
  └─ Scan ──────→ Merger ──→ /merged_scan_filtered ────────→ Global/Local Costmap (obstacle_layer)
```

### コストマップレイヤー構成

#### Global Costmap (広域計画用)
1. **Voxel Layer** - nvblox ESDF点群（3D障害物情報）
2. **Static Layer** - nvblox Occupancy Grid（静的マップ）
3. **Obstacle Layer** - LiDARスキャン（動的障害物）
4. **Denoise Layer** - ノイズ除去
5. **Inflation Layer** - 安全マージン

#### Local Costmap (局所制御用)
1. **Voxel Layer (ZED)** - ZEDポイントクラウド
2. **Voxel Layer (LiDAR)** - LiDARスキャン
3. **Denoise Layer** - ノイズ除去
4. **Inflation Layer** - 安全マージン

## 使用方法

### 統合システムの起動

```bash
# ワークスペースのソース
source /home/jetros/ros2_ws/install/setup.bash

# 統合システム起動（推奨）
ros2 launch bringup zed2i_nvblox_nav2_launch.py

# または、RViz2可視化付き
ros2 launch bringup zed2i_nvblox_nav2_launch.py enable_visualization:=true
```

### 個別起動の場合

```bash
# ターミナル1: ZED + nvblox + デバイス
ros2 launch bringup zed2i_nvblox_fixed.launch.py

# ターミナル2: Nav2（10秒後）
sleep 10
ros2 launch nav2_bringup my_navigation_launch.py
```

## 動作確認

### 専用テストスクリプト

```bash
# /mapトピックとmap frame専用テスト
/home/jetros/ros2_ws/test_nvblox_nav2_map.sh
```

このスクリプトは以下をチェック：
- ✅ /mapトピックの存在と型
- ✅ map TFフレームの存在
- ✅ Global Costmapが/mapを購読しているか
- ✅ Nav2ノードの起動状態

### 手動確認コマンド

#### /mapトピック確認

```bash
# トピック存在確認
ros2 topic list | grep "/map"

# トピック詳細
ros2 topic info /map -v

# メッセージタイプ確認
ros2 topic type /map
# 期待値: nav_msgs/msg/OccupancyGrid

# Publisher確認
ros2 topic info /map -v | grep Publisher
# 期待: nvblox_nodeがpublish

# データ受信テスト
ros2 topic hz /map

# マップの内容確認（1回だけ）
ros2 topic echo /map --once
```

#### map TF確認

```bash
# TFツリー全体
ros2 run tf2_tools view_frames

# map -> zed_odom変換
ros2 run tf2_ros tf2_echo map zed_odom

# map -> zed_camera_link変換（full chain）
ros2 run tf2_ros tf2_echo map zed_camera_link

# map -> base_link変換
ros2 run tf2_ros tf2_echo map base_link
```

#### Global Costmap確認

```bash
# /mapの購読者確認
ros2 topic info /map -v | grep Subscription

# Global Costmap出力確認
ros2 topic hz /global_costmap/costmap

# Global Costmapのパラメータ確認
ros2 param get /global_costmap/global_costmap plugins
ros2 param get /global_costmap/global_costmap static_layer.map_topic
```

## トラブルシューティング

### 問題1: /mapトピックが出力されない

**症状**:
```bash
$ ros2 topic list | grep "/map"
# 何も表示されない
```

**原因**:
- nvbloxノードが起動していない
- nvbloxの設定で`publish_occupancy_layer`が無効
- トピックリマッピングが正しくない

**確認**:
```bash
# nvbloxノード確認
ros2 node list | grep nvblox

# nvbloxのトピック確認
ros2 topic list | grep nvblox

# static_occupancy_layerが出力されているか
ros2 topic list | grep occupancy

# nvbloxパラメータ確認
ros2 param get /nvblox_node global_frame
# 期待値: map

ros2 param get /nvblox_node publish_occupancy_layer
# 期待値: True
```

**対処**:
1. nvbloxノードが起動しているか確認
2. ZEDカメラが正常に動作しているか確認
3. nvblox設定ファイルを確認
4. launchファイルのリマッピング設定を確認

### 問題2: map TFフレームが存在しない

**症状**:
```bash
$ ros2 run tf2_ros tf2_echo map zed_odom
Lookup would require extrapolation into the past.
```

**原因**:
- ZEDの`pos_tracking.publish_map_tf`が無効
- ZED Visual SLAMが初期化されていない

**確認**:
```bash
# ZEDノード確認
ros2 node list | grep zed

# ZED Pose確認
ros2 topic hz /zed/zed_node/pose

# TFフレーム一覧
ros2 run tf2_ros tf2_monitor

# ZED設定確認（runtime中）
ros2 param get /zed/zed_node pos_tracking.publish_map_tf
# 期待値: True
```

**対処**:
1. ZEDカメラが正常に動作しているか確認
2. Visual SLAMが初期化されるまで待つ（10-30秒）
3. ロボットを少し動かしてSLAMを初期化
4. launch設定を確認

### 問題3: Global Costmapが/mapを購読しない

**症状**:
```bash
$ ros2 topic info /map -v
Subscription count: 0
```

**原因**:
- Nav2が起動していない
- Global Costmap設定が誤っている
- QoS設定の不一致

**確認**:
```bash
# Nav2ノード確認
ros2 node list | grep costmap

# Global Costmapパラメータ
ros2 param get /global_costmap/global_costmap static_layer.map_topic

# Global Costmapログ
ros2 node info /global_costmap/global_costmap
```

**対処**:
1. Nav2が正常に起動しているか確認
2. パラメータファイルの`static_layer.map_topic`を確認
3. QoS設定を確認（nvbloxはVolatile、Nav2はTransient Localがデフォルト）

### 問題4: マップが空または更新されない

**症状**:
- /mapトピックは存在するがデータが空
- Global Costmapが更新されない

**原因**:
- nvbloxがまだマップを生成していない
- ZEDカメラの視界に何も映っていない
- TSDF統合の設定問題

**確認**:
```bash
# マップデータ確認
ros2 topic echo /map --once | grep "data: \[" | head -1

# nvbloxの状態確認
ros2 topic hz /nvblox_node/mesh
ros2 topic hz /nvblox_node/esdf_pointcloud

# ZED深度画像確認
ros2 topic hz /zed/zed_node/depth/depth_registered
```

**対処**:
1. ロボットを移動させてマップを生成
2. ZEDカメラの視界に障害物があることを確認
3. nvbloxパラメータを調整（統合距離、解像度など）
4. 十分な時間待つ（初期マップ生成には1-2分かかる場合あり）

### 問題5: 自動航行が開始しない

**症状**:
- ゴールを設定してもロボットが動かない

**原因**:
- TFチェーン全体が繋がっていない
- Global Costmapが未初期化
- Nav2ノードが未起動

**確認**:
```bash
# TFチェーン全体確認
ros2 run tf2_ros tf2_echo map base_link

# Nav2ノード確認
ros2 node list | grep -E "controller|planner|bt_navigator"

# 速度指令確認
ros2 topic echo /cmd_vel

# Global Plan確認
ros2 topic echo /plan
```

**対処**:
1. 統合テストスクリプトを実行して問題箇所を特定
2. すべてのTF変換が可能か確認
3. Global Costmapが/mapを受信しているか確認
4. Nav2のライフサイクル状態を確認

## パラメータ調整

### nvbloxマップ生成の調整

```yaml
# nvblox_nav2_integration.yaml

# 解像度（より細かく/粗く）
voxel_size: 0.05  # 5cm（デフォルト）

# Occupancy確率の調整
static_mapper:
  free_region_occupancy_probability: 0.3  # 空き判定（下げると保守的）
  occupied_region_occupancy_probability: 0.9  # 占有判定（上げると保守的）
  unobserved_region_occupancy_probability: 0.5  # 未観測判定

# マップ更新頻度
publish_layer_rate_hz: 5.0  # 高くすると更新頻度向上（CPU負荷増）
```

### Global Costmapの調整

```yaml
# my_nav2_params_mppi1.yaml

global_costmap:
  global_costmap:
    # 更新頻度
    update_frequency: 3.0  # 高くするとリアルタイム性向上

    # 解像度
    resolution: 0.05  # nvbloxと一致させる

    # voxel_layer（nvblox ESDF）
    voxel_layer:
      pointcloud:
        obstacle_max_range: 9.5  # ESDF点群の最大使用距離

    # static_layer（nvblox map）
    static_layer:
      map_topic: /map  # nvbloxの/map

    # inflation_layer（安全マージン）
    inflation_layer:
      inflation_radius: 0.20  # 安全マージン距離
```

## 関連ファイル

### 新規作成ファイル

- [nvblox_nav2_integration.yaml](/home/jetros/ros2_ws/src/bringup/config/nvblox/nvblox_nav2_integration.yaml) - nvblox Nav2統合設定
- [test_nvblox_nav2_map.sh](/home/jetros/ros2_ws/test_nvblox_nav2_map.sh) - /mapトピック専用テストスクリプト

### 修正ファイル

- [zed2i_nvblox_fixed.launch.py](/home/jetros/ros2_ws/src/bringup/launch/zed2i_nvblox_fixed.launch.py)
  - nvblox設定ファイル追加（nvblox_nav2_integration.yaml）
  - `global_frame: 'map'`パラメータ追加
  - トピックリマッピング追加（`static_occupancy_layer -> /map`）
  - ZED `publish_map_tf: True`設定

- [my_nav2_params_mppi1.yaml](/home/jetros/ros2_ws/src/navigation2/nav2_bringup/params/my_nav2_params_mppi1.yaml)
  - Global Costmap `static_layer`設定更新
  - `map_subscribe_transient_local: False`（nvbloxはVolatile QoS）
  - `map_topic: /map`明示

## 動作原理

### 1. マップ生成フロー

```
ZED Depth → nvblox TSDF統合 → nvblox Occupancy Grid生成 → /map配信
```

### 2. TF配信フロー

```
ZED Visual SLAM → map座標系推定 → map TF配信 (map → zed_odom)
ZED Visual Odometry → zed_odom TF配信 (zed_odom → zed_camera_link)
```

### 3. Nav2統合フロー

```
/map → Global Costmap (static_layer)
nvblox ESDF → Global Costmap (voxel_layer)
LiDAR → Global/Local Costmap (obstacle_layer)
↓
経路計画 (Planner Server)
↓
経路追従 (Controller Server)
↓
/cmd_vel → ロボット制御
```

## 期待される動作

### 正常動作時

1. **起動直後（0-10秒）**
   - nvbloxノード起動
   - ZEDカメラ初期化
   - デバイス起動

2. **SLAM初期化（10-30秒）**
   - ZED Visual SLAM初期化
   - map TF配信開始
   - 初期マップ生成開始

3. **マップ生成（30秒-2分）**
   - ロボット移動でマップ拡大
   - /mapトピックにデータ配信
   - Global Costmap更新開始

4. **自動航行準備完了（2分-）**
   - map TFチェーン完全
   - Global Costmap初期化完了
   - ゴール設定で自動航行可能

### 確認方法

```bash
# 完全テスト
/home/jetros/ros2_ws/test_nvblox_nav2_map.sh

# 期待される出力:
# ✅ /map トピック存在
# ✅ 正しいタイプ: nav_msgs/msg/OccupancyGrid
# ✅ map -> zed_odom TF変換可能
# ✅ map -> zed_camera_link TF変換可能
# ✅ Global Costmapが/mapを購読
# ✅ すべての必須要件を満たしています
# ✅ 自動航行が可能な状態です
```

## 参考リンク

- [Isaac ROS nvblox Documentation](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/index.html)
- [Navigation2 Costmap Documentation](https://navigation.ros.org/configuration/packages/configuring-costmaps.html)
- [ZED ROS2 Wrapper Documentation](https://www.stereolabs.com/docs/ros2/)

## 更新履歴

- **2025-10-19**: 初版作成
  - nvbloxから/mapトピック出力設定
  - ZEDからmap TF配信設定
  - Global Costmapでnvbloxマップ使用設定
  - 専用テストスクリプト作成
