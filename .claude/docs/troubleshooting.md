# トラブルシューティング

## デバッグ・確認コマンド

### ノード確認
```bash
ros2 node list
ros2 node list | grep -E "nvblox|zed|controller|planner"
ros2 node info /nvblox_node
ros2 node info /zed/zed_node
```

### トピック確認
```bash
ros2 topic list
ros2 topic hz /scan_filtered
ros2 topic hz /map
ros2 topic echo /cmd_vel
ros2 topic info /map -v
```

### TF確認（重要）
```bash
# TFツリー全体を可視化
ros2 run tf2_tools view_frames

# 特定の座標変換を確認
ros2 run tf2_ros tf2_echo map zed_camera_link
ros2 run tf2_ros tf2_echo map odom

# 動的TF確認（ZED2iがmap→odomを配信）
ros2 topic echo /tf | grep -A 10 "map.*odom"
```

### パラメータ確認
```bash
# ZED2i設定
ros2 param get /zed/zed_node pos_tracking.publish_map_tf  # 期待値: true
ros2 param get /zed/zed_node pos_tracking.map_frame       # 期待値: map
ros2 param get /zed/zed_node pos_tracking.odometry_frame  # 期待値: odom

# nvblox設定
ros2 param get /nvblox_node global_frame     # 期待値: map
ros2 param get /nvblox_node use_lidar        # 期待値: true
```

### ZUPT Filter確認
```bash
ros2 topic echo /zupt/status
ros2 topic hz /zed/zed_node/odom_zupt
```

## 既知の課題・注意点

### 1. 同名ノードの警告
```
WARNING: Be aware that are nodes in the graph that share an exact name
```
複数のtransform_listener_implノードが同名で起動している（無害）

### 2. センサー高さ統一の重要性
前後LiDARの高さ(-0.03m Z)を一致させることでマッピングの安定性を確保

### 3. TF更新レート
- Visual Odometry: ~30 Hz
- ZED map→odom TF: ~30 Hz（動的TF）
- nvblox更新: 設定による

### 4. ZED2iのTF設定の重要性
- `publish_map_tf: true` が必須
- `map_frame: 'map'` が必須
- `odometry_frame: 'odom'` が必須
- この設定を間違えるとNav2が動作しない

### 5. LiDAR物理的取り付けの確認（重要）

⚠️ **地図生成の前に必ず確認すべき項目**

**症状**: SLAMで地図がうまく生成できない、壁面を認識しない

**原因**: LiDARが下向きに傾いており、地面に向かって照射されている

**確認方法**:
```bash
# RViz2でLiDARスキャンを可視化
rviz2
# LaserScanトピック (/front_scan, /back_scan, /merged_scan) を追加
# 点群が水平に壁面を捉えているか確認
# 地面のノイズが多い場合は下向き傾斜の可能性
```

**対処法**:
- LiDARの取り付け角度を物理的に調整し、水平にする
- 点群が壁面の適切な高さ（床から20-30cm程度）を捉えるよう調整
- 調整後、再度スキャンデータを確認

**備考**:
- パラメータ調整では解決できない物理的問題
- 気づきにくく、忘れやすい課題のため要注意
- 定期的な動作確認で早期発見が重要

## よくある問題と解決策

### 問題1: Nav2が起動しない
**確認事項**:
1. TFツリーが完全か (`map → odom → zed_camera_link`)
2. `/map`トピックが配信されているか
3. ZED2iのパラメータが正しいか

### 問題2: マップが生成されない
**確認事項**:
1. nvbloxノードが起動しているか
2. LiDARスキャンが配信されているか (`/scan_filtered`)
3. ZED2iのRGB/Depthが配信されているか

### 問題3: オドメトリが暴走する
**確認事項**:
1. ZUPT Filterが動作しているか
2. `/zed/zed_node/odom_zupt`が配信されているか
3. Visual Odometryの品質（テクスチャの有無）

## 緊急時の対処

### システム全体の再起動
```bash
# 1. 全ノードを停止
Ctrl+C

# 2. ROS2デーモンを再起動
ros2 daemon stop
ros2 daemon start

# 3. システムを再起動
ros2 launch bringup zed2i_nvblox_nav2_launch.py
```

### 特定ノードの再起動
```bash
# ノードリストを確認
ros2 node list

# 問題のあるノードを特定して個別に再起動
# （launchファイルを編集して該当ノードのみ起動）
```
