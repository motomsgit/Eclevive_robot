# TFフレーム統一ガイド（odom統一版）

## 問題の概要

### 発生した問題

`ros2 launch bringup zed2i_nvblox_nav2_launch.py`を実行したところ、TFツリーで`odom`と`zed_odom`の両方のフレームが存在し、Local CostmapとGlobal Costmapが正しく動作しない状態でした。

**症状**:
```
TFツリー:
  map
   ├─ odom ???
   └─ zed_odom ???
```

**原因**:
1. ZEDのデフォルト設定ファイル（`common_stereo_isaac.yaml`）で`odometry_frame: 'odom'`が設定
2. launchファイルで`odometry_frame: 'zed_odom'`に上書きしようとしたが、設定ファイルの読み込み順序により反映されず
3. Nav2パラメータでは`zed_odom`を指定していたが、実際には`odom`が配信されていた
4. フレーム名の不一致により、Local/Global Costmapが機能しない

## 解決策

### アプローチ: すべてを`odom`に統一

`zed_odom`を使用する代わりに、**すべてのフレーム名を`odom`に統一**しました。

**理由**:
1. ZEDのデフォルトは`odom`
2. nvbloxのデフォルトは`odom`
3. Nav2のデフォルトは`odom`
4. 設定変更が最小限で済む
5. 他のROS2パッケージとの互換性が高い

## 修正内容

### 1. ZED設定（launch）

**ファイル**: [zed2i_nvblox_fixed.launch.py](/home/jetros/ros2_ws/src/bringup/launch/zed2i_nvblox_fixed.launch.py)

```python
# 修正前
'pos_tracking.odometry_frame': 'zed_odom',  # 反映されない

# 修正後
'pos_tracking.odometry_frame': 'odom',  # Nav2デフォルトに統一
```

### 2. Nav2パラメータ

**ファイル**: [my_nav2_params_mppi1.yaml](/home/jetros/ros2_ws/src/navigation2/nav2_bringup/params/my_nav2_params_mppi1.yaml)

#### AMCL
```yaml
# 修正前
odom_frame_id: "zed_odom"

# 修正後
odom_frame_id: "odom"
```

#### Local Costmap
```yaml
# 修正前
global_frame: zed_odom

# 修正後
global_frame: odom
```

#### Behavior Server
```yaml
# 修正前
global_frame: zed_odom

# 修正後
global_frame: odom
```

## 正しいTFツリー

修正後の期待されるTFツリー:

```
map (ZED Visual SLAM)
 └─ odom (ZED Visual Odometry)
     └─ zed_camera_link (ロボット基準)
         ├─ base_link
         ├─ front_lidar
         └─ back_lidar
```

**各フレームの役割**:
- **map**: グローバル座標系（ZED Visual SLAMが配信）
- **odom**: オドメトリ座標系（ZED Visual Odometryが配信）
- **zed_camera_link**: ZEDカメラのローカル座標系
- **base_link**: ロボット中心
- **front_lidar**, **back_lidar**: LiDARセンサー

## 確認方法

### 専用テストスクリプト

```bash
# TFツリー統合確認
/home/jetros/ros2_ws/test_tf_integration.sh
```

**期待される出力**:
```
✅ map -> odom 変換可能
✅ odom -> zed_camera_link 変換可能
✅ map -> zed_camera_link 変換可能
✅ zed_odom は存在しません（正しい）
✅ odom フレームが存在します
✅ TFツリーの整合性: 正常
✅ Nav2自動航行が可能な状態です
```

### 手動確認コマンド

#### TF変換確認

```bash
# map -> odom
ros2 run tf2_ros tf2_echo map odom

# odom -> zed_camera_link
ros2 run tf2_ros tf2_echo odom zed_camera_link

# map -> zed_camera_link (full chain)
ros2 run tf2_ros tf2_echo map zed_camera_link

# map -> base_link
ros2 run tf2_ros tf2_echo map base_link
```

#### フレーム一覧確認

```bash
# 全TFフレームをモニター
ros2 run tf2_ros tf2_monitor

# zed_odomが存在しないことを確認（存在するとNG）
ros2 run tf2_ros tf2_monitor | grep zed_odom
# 何も表示されなければOK

# odomが存在することを確認
ros2 run tf2_ros tf2_monitor | grep "^odom$"
# odomが表示されればOK
```

#### TFツリー可視化

```bash
# TFツリーを可視化（frames.pdf生成）
ros2 run tf2_tools view_frames

# PDFを開く
evince frames.pdf
# または
firefox frames.pdf
```

#### Nav2パラメータ確認

```bash
# Local Costmap
ros2 param get /local_costmap/local_costmap global_frame
# 期待値: odom

# Global Costmap
ros2 param get /global_costmap/global_costmap global_frame
# 期待値: map

# Behavior Server
ros2 param get /behavior_server global_frame
# 期待値: odom

# BT Navigator
ros2 param get /bt_navigator robot_base_frame
# 期待値: zed_camera_link
```

## トラブルシューティング

### 問題1: zed_odomフレームが残っている

**症状**:
```bash
$ ros2 run tf2_ros tf2_monitor | grep zed_odom
Frame: zed_odom, published by ...
```

**原因**:
- 古いlaunchファイルの設定が残っている
- 再ビルドしていない
- キャッシュが残っている

**対処**:
```bash
# 1. すべてのROS2プロセスを終了
pkill -9 ros
pkill -9 python

# 2. 再ビルド
cd /home/jetros/ros2_ws
colcon build --packages-select bringup nav2_bringup --symlink-install

# 3. ワークスペースを再ソース
source install/setup.bash

# 4. 再起動
ros2 launch bringup zed2i_nvblox_nav2_launch.py
```

### 問題2: TF変換ができない

**症状**:
```bash
$ ros2 run tf2_ros tf2_echo map odom
Lookup would require extrapolation into the past.
```

**原因**:
- ZED Visual SLAMが初期化されていない
- ZEDカメラが動作していない
- TF配信が遅延している

**対処**:
```bash
# 1. ZEDノード確認
ros2 node list | grep zed

# 2. ZED Pose確認
ros2 topic hz /zed/zed_node/pose

# 3. ZED設定確認
ros2 param get /zed/zed_node pos_tracking.publish_map_tf
# 期待値: True

ros2 param get /zed/zed_node pos_tracking.map_frame
# 期待値: map

ros2 param get /zed/zed_node pos_tracking.odometry_frame
# 期待値: odom

# 4. ロボットを動かしてSLAMを初期化
# （10-30秒待つ）
```

### 問題3: Local Costmapが更新されない

**症状**:
- Local Costmapが空
- `/local_costmap/costmap`が更新されない

**原因**:
- `global_frame`がTFツリーと一致していない
- センサーデータが配信されていない

**対処**:
```bash
# 1. Local Costmap設定確認
ros2 param get /local_costmap/local_costmap global_frame
# 期待値: odom

ros2 param get /local_costmap/local_costmap robot_base_frame
# 期待値: zed_camera_link

# 2. TF確認
ros2 run tf2_ros tf2_echo odom zed_camera_link

# 3. センサーデータ確認
ros2 topic hz /merged_scan_filtered
ros2 topic hz /zed/zed_node/point_cloud/cloud_registered

# 4. パラメータファイル確認
cat /home/jetros/ros2_ws/src/navigation2/nav2_bringup/params/my_nav2_params_mppi1.yaml | grep -A5 "local_costmap:"
```

### 問題4: Global Costmapが更新されない

**症状**:
- Global Costmapが空
- `/global_costmap/costmap`が更新されない

**原因**:
- `/map`トピックが配信されていない
- TFチェーンが繋がっていない

**対処**:
```bash
# 1. /mapトピック確認
ros2 topic hz /map

# 2. Global Costmap設定確認
ros2 param get /global_costmap/global_costmap global_frame
# 期待値: map

# 3. TF確認（full chain）
ros2 run tf2_ros tf2_echo map zed_camera_link

# 4. nvblox確認
ros2 node list | grep nvblox
ros2 topic list | grep nvblox

# 5. /mapトピック専用テスト
/home/jetros/ros2_ws/test_nvblox_nav2_map.sh
```

## 使用方法

### システム起動

```bash
# ワークスペースのソース
source /home/jetros/ros2_ws/install/setup.bash

# 統合システム起動
ros2 launch bringup zed2i_nvblox_nav2_launch.py
```

### 確認手順

1. **TFツリー確認** (10秒後)
```bash
/home/jetros/ros2_ws/test_tf_integration.sh
```

2. **/mapトピック確認** (30秒後)
```bash
/home/jetros/ros2_ws/test_nvblox_nav2_map.sh
```

3. **ナビゲーションテスト** (1分後)
```bash
# ゴール設定
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'

# 速度指令確認
ros2 topic echo /cmd_vel
```

## フレーム名統一の原則

### なぜ`odom`に統一したか

| 選択肢 | メリット | デメリット |
|--------|---------|----------|
| **`odom`に統一** | ✅ ZED/nvblox/Nav2のデフォルト<br>✅ 設定変更が最小<br>✅ 他パッケージとの互換性高 | なし |
| `zed_odom`に統一 | ZED由来が明示的 | ❌ すべての設定を変更必要<br>❌ デフォルトと異なる<br>❌ 他パッケージとの互換性低 |

### 統一ルール

1. **オドメトリフレーム**: `odom` (固定)
2. **マップフレーム**: `map` (固定)
3. **ロボット基準フレーム**: `zed_camera_link` (ZEDベース)
4. **ロボット中心フレーム**: `base_link` (ロボット幾何中心)

### フレーム命名規則

- **グローバル**: `map` - 世界座標系
- **オドメトリ**: `odom` - ドリフトありの推定座標系
- **センサー**: `<sensor_name>_<location>` - 例: `front_lidar`, `back_lidar`
- **ロボット**: `base_link`, `zed_camera_link` など

## 関連ファイル

### 修正ファイル

- [zed2i_nvblox_fixed.launch.py](/home/jetros/ros2_ws/src/bringup/launch/zed2i_nvblox_fixed.launch.py) - ZED設定（`odom`統一）
- [my_nav2_params_mppi1.yaml](/home/jetros/ros2_ws/src/navigation2/nav2_bringup/params/my_nav2_params_mppi1.yaml) - Nav2設定（`odom`統一）

### 新規テストツール

- [test_tf_integration.sh](/home/jetros/ros2_ws/test_tf_integration.sh) - TFツリー統合確認スクリプト

### 関連ドキュメント

- [NVBLOX_NAV2_MAP_INTEGRATION.md](/home/jetros/ros2_ws/NVBLOX_NAV2_MAP_INTEGRATION.md) - nvblox + Nav2統合ガイド
- [NVBLOX_NAV2_INTEGRATION.md](/home/jetros/ros2_ws/NVBLOX_NAV2_INTEGRATION.md) - 以前の統合方法

## チェックリスト

システムが正常に動作するための必須確認項目:

- [ ] `ros2 run tf2_ros tf2_echo map odom` が成功
- [ ] `ros2 run tf2_ros tf2_echo odom zed_camera_link` が成功
- [ ] `ros2 run tf2_ros tf2_echo map zed_camera_link` が成功（full chain）
- [ ] `ros2 run tf2_ros tf2_monitor | grep zed_odom` が何も表示しない
- [ ] `ros2 topic hz /map` がデータ配信中
- [ ] `ros2 topic hz /local_costmap/costmap` がデータ配信中
- [ ] `ros2 topic hz /global_costmap/costmap` がデータ配信中
- [ ] `ros2 node list` にNav2ノードがすべて存在
- [ ] `/home/jetros/ros2_ws/test_tf_integration.sh` がすべて✅

すべて✅なら自動航行可能！

## まとめ

### 修正前の問題

```
❌ odom と zed_odom の両方が存在
❌ Local/Global Costmapが動作しない
❌ TFチェーンが不整合
```

### 修正後の状態

```
✅ odom に統一
✅ Local/Global Costmapが正常動作
✅ TFチェーンが整合
✅ Nav2自動航行が可能
```

### キーポイント

1. **フレーム名はシンプルに統一** - `odom`をグローバルスタンダードとして使用
2. **デフォルト設定を尊重** - ZED/nvblox/Nav2のデフォルトに合わせる
3. **TFツリーの可視化** - `view_frames`で常に確認
4. **テストスクリプトの活用** - 自動診断で問題を早期発見

---

## 更新履歴

- **2025-10-19**: 初版作成
  - odom vs zed_odom問題の解決
  - すべてのフレーム名をodomに統一
  - TF統合確認スクリプト作成
