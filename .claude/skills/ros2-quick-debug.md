# ROS2 クイックデバッグ

ROS2システムの基本的な動作確認とデバッグを高速実行するスキル。

## 実行内容

以下の診断を自動的に実行し、問題を特定します：

### 1. ノード・トピック確認
```bash
ros2 node list
ros2 topic list
ros2 topic hz /merged_scan /cmd_vel /zed/zed_node/odom --window 10
```

### 2. TFツリー確認
```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo map base_link
```

### 3. 主要トピックの健全性チェック
- `/merged_scan` - LiDARデータ配信確認
- `/cmd_vel` - 速度指令確認
- `/zed/zed_node/odom` - オドメトリ確認
- `/map` - SLAM地図確認

### 4. ノードグラフ可視化
```bash
ros2 node info /slam_toolbox
ros2 node info /controller_server
ros2 node info /joy_mecanum_controller
```

### 5. エラーログ確認
最新のROS2ログから警告・エラーを抽出して表示

## 出力形式

診断結果を以下の形式で報告：

```
✅ 正常: ノード起動確認 (53ノード)
✅ 正常: /merged_scan 配信中 (15.2 Hz)
⚠️  警告: /cmd_vel 配信レート低下 (5.1 Hz, 期待値: 30 Hz)
❌ エラー: TF map -> base_link タイムアウト
```

## 使用タイミング

- システム起動後の動作確認
- 異常動作時の初期診断
- 修正後の検証
- 定期的なヘルスチェック

## トークン削減効果

**従来**: 各コマンドを個別に実行依頼 → 5-10往復 (約3000トークン)
**Skills使用後**: 1回の呼び出しで完結 (約500トークン)
**削減率**: 約83%
