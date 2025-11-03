---
description: ROS2システムの健全性をチェック
---

ROS2システムの健全性を診断してください。以下の項目を確認して報告してください：

## 確認項目

### 1. アクティブなノード一覧
```bash
ros2 node list
ros2 node list | grep -E "nvblox|zed|controller|planner"
```

### 2. 主要トピックの配信レート
以下のトピックのHz（配信レート）を確認：
- `/map` - nvbloxマップ配信
- `/scan_filtered` - フィルタ済みLiDARスキャン
- `/zed/zed_node/odom` - オドメトリ
- `/cmd_vel` - 速度指令

```bash
ros2 topic hz /map
ros2 topic hz /scan_filtered
ros2 topic hz /zed/zed_node/odom
ros2 topic hz /cmd_vel
```

### 3. TFツリーの整合性
以下のTF変換が正常に配信されているか確認：
- `map → odom` (ZED2iが動的配信)
- `odom → zed_camera_origin`
- `zed_camera_origin → zed_camera_link`

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo map zed_camera_link
```

### 4. nvbloxとNav2の動作状態
以下のパラメータが正しく設定されているか確認：

**ZED2i設定:**
```bash
ros2 param get /zed/zed_node pos_tracking.publish_map_tf  # 期待値: true
ros2 param get /zed/zed_node pos_tracking.map_frame       # 期待値: map
ros2 param get /zed/zed_node pos_tracking.odometry_frame  # 期待値: odom
```

**nvblox設定:**
```bash
ros2 param get /nvblox_node global_frame  # 期待値: map
ros2 param get /nvblox_node use_lidar     # 期待値: true
```

### 5. エラーログの有無
ROS2ログを確認してエラーやワーニングを検出：
```bash
# 最近のログを確認（実行中のシステムの場合）
# または問題のあるノードのログを確認
```

## 報告フォーマット

以下の形式で報告してください：

```
## ROS2システム診断結果

### ✅ 正常項目
- ノード起動数: XX個（期待: 40-50個）
- /map配信レート: XX Hz
- TFツリー: 正常

### ⚠️ 警告項目
- （警告がある場合に記載）

### ❌ エラー項目
- （エラーがある場合に記載）

### 推奨事項
- （問題がある場合の解決策を提案）
```

問題があれば原因と解決策を提案してください。
