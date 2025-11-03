---
description: Nav2システムの動作確認テストを実行
---

Nav2ナビゲーションシステムの動作確認テストを実施してください。以下の項目を順番に確認してください：

## テスト項目

### 1. ノードの起動確認
Nav2関連ノードが正常に起動しているか確認：
```bash
ros2 node list | grep -E "controller_server|planner_server|behavior_server|bt_navigator"
```

**期待結果:**
- `/controller_server`
- `/planner_server`
- `/behavior_server`
- `/bt_navigator`
- `/smoother_server`
- `/velocity_smoother`

### 2. /mapトピックの配信確認
nvbloxからのマップ配信を確認：
```bash
ros2 topic info /map -v
ros2 topic hz /map
ros2 topic echo /map --once
```

**期待結果:**
- トピック型: `nav_msgs/msg/OccupancyGrid`
- QoS: RELIABLE, TRANSIENT_LOCAL
- 配信レート: 設定による（通常0.5-2 Hz）

### 3. TF変換の確認
ナビゲーションに必要なTF変換が正常か確認：
```bash
# map -> zed_camera_link の変換
ros2 run tf2_ros tf2_echo map zed_camera_link

# map -> odom の変換（ZED2iが動的配信）
ros2 run tf2_ros tf2_echo map odom
```

**期待結果:**
- エラーなくTF変換が表示される
- 座標値が合理的な範囲内（例: ±100m以内）

### 4. パラメータ設定の確認
Nav2とnvbloxのパラメータが正しく設定されているか確認：
```bash
# ZED2i設定
ros2 param get /zed/zed_node pos_tracking.publish_map_tf  # true
ros2 param get /zed/zed_node pos_tracking.map_frame       # map

# nvblox設定
ros2 param get /nvblox_node global_frame                  # map

# Nav2 Global Costmap設定
ros2 param get /global_costmap/global_costmap global_frame        # map
ros2 param get /global_costmap/global_costmap robot_base_frame    # zed_camera_link

# Nav2 Local Costmap設定
ros2 param get /local_costmap/local_costmap global_frame          # odom
ros2 param get /local_costmap/local_costmap robot_base_frame      # zed_camera_link
```

### 5. テストゴールの送信（オプション）
簡単なナビゲーションゴールを送信して経路計画を確認：
```bash
# テストゴールを送信（座標は環境に応じて調整）
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 1.0, y: 0.0, z: 0.0},
    orientation: {w: 1.0}
  }
}"
```

**確認項目:**
- `/plan`トピックに経路が配信されるか
- エラーログが出ないか
- RViz2で経路が可視化されるか（RViz起動時）

### 6. Costmapの確認
コストマップが正常に生成されているか確認：
```bash
ros2 topic hz /global_costmap/costmap
ros2 topic hz /local_costmap/costmap
```

**期待結果:**
- Global Costmap: 設定による更新レート（通常1-5 Hz）
- Local Costmap: 設定による更新レート（通常5-10 Hz）

## 報告フォーマット

以下の形式で報告してください：

```
## Nav2動作確認テスト結果

### ✅ 正常項目
1. ノード起動: 全ノード起動確認
2. /map配信: XX Hz
3. TF変換: map→zed_camera_link 正常
4. パラメータ設定: 全て正常

### ⚠️ 警告項目
- （警告がある場合に記載）

### ❌ エラー項目
- （エラーがある場合に記載）

### 🧪 テスト結果（ゴール送信した場合）
- 経路計画: 成功/失敗
- エラーログ: 有/無

### 📋 推奨事項
- （問題がある場合の解決策を提案）
```

問題があれば原因と解決策を提案してください。
詳細なトラブルシューティングは `.claude/docs/troubleshooting.md` を参照してください。
