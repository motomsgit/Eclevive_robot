# 手を上げた人物追従システム (Hand Raised Person Follower)

## 概要

ZED2iカメラの骨格検出機能を使用して、**手を上げた人を検出し、その人だけを追従する**ROS2ノードです。

### 主な機能

1. **手上げジェスチャー検出**
   - ZED2iの38キーポイント骨格検出を使用
   - 肩より上に手を上げている人を自動検出
   - 左手または右手どちらでも検出可能

2. **個人追跡（Person ID Tracking）**
   - 検出した人の追跡ID（label_id）を記録
   - 同じIDの人だけを継続的に追従
   - 複数人がいても、最初に手を上げた人のみを追従

3. **自律追従制御**
   - 指定距離（デフォルト1.5m）を維持しながら追従
   - 比例制御による滑らかな速度制御
   - 角度・距離の許容誤差による安定動作

4. **追跡タイムアウト**
   - 一定時間（デフォルト3秒）見失うと追跡リセット
   - 新しい手上げ信号を待機

5. **TFフレーム配信**
   - 追従対象のTFフレーム `target_person` を配信
   - Nav2などのナビゲーションシステムと統合可能

---

## システム構成

```
ZED2iカメラ (骨格検出)
    ↓ /zed/zed_node/body_trk/skeletons
hand_raised_person_follower ノード
    ↓ /cmd_vel (速度指令)
    ↓ /hand_follower/status (状態)
    ↓ /hand_follower/target_id (追跡ID)
    ↓ TF: target_person フレーム
メカナム制御システム
```

---

## 使用方法

### 1. 基本起動

```bash
# システム全体起動（ZED2i + メカナム制御含む）
ros2 launch bringup bringup_ps5_all_launch.py

# 別のターミナルで手上げ追従ノード起動
ros2 launch zed_goal_publisher hand_raised_person_follower_launch.py
```

### 2. カスタムパラメータで起動

```bash
ros2 launch zed_goal_publisher hand_raised_person_follower_launch.py \
  target_distance:=2.0 \
  max_linear_speed:=0.5 \
  hand_raise_threshold:=0.25
```

### 3. 動作確認

```bash
# ターミナル1: 追跡状態の確認
ros2 topic echo /hand_follower/status

# ターミナル2: 追跡IDの確認
ros2 topic echo /hand_follower/target_id

# ターミナル3: 速度指令の確認
ros2 topic echo /cmd_vel

# ターミナル4: TFフレームの確認
ros2 run tf2_ros tf2_echo zed_camera_link target_person
```

---

## パラメータ一覧

| パラメータ名 | デフォルト値 | 単位 | 説明 |
|------------|------------|------|------|
| `skeleton_topic` | `zed/zed_node/body_trk/skeletons` | - | ZED骨格検出トピック |
| `cmd_vel_topic` | `cmd_vel` | - | 速度指令トピック |
| `target_distance` | `1.5` | m | 追従目標距離 |
| `distance_tolerance` | `0.3` | m | 距離許容誤差 |
| `angle_tolerance` | `0.1` | rad | 角度許容誤差（約5.7度） |
| `max_linear_speed` | `0.3` | m/s | 最大前後速度 |
| `max_angular_speed` | `0.5` | rad/s | 最大角速度（約28.6度/秒） |
| `hand_raise_threshold` | `0.3` | m | 手上げ判定閾値（肩と手の高低差） |
| `tracking_timeout` | `3.0` | s | 追跡タイムアウト時間 |

---

## 骨格キーポイント仕様

ZED2iは38個の骨格キーポイントを検出します。このノードで使用するキーポイント：

| インデックス | 名前 | 説明 |
|------------|------|------|
| 2 | `NOSE` | 鼻 |
| 4 | `RIGHT_SHOULDER` | 右肩 |
| 5 | `LEFT_SHOULDER` | 左肩 |
| 7 | `RIGHT_ELBOW` | 右肘 |
| 8 | `LEFT_ELBOW` | 左肘 |

### 手上げ判定ロジック

```cpp
// ZED座標系ではY軸が下向き（Y値が小さいほど上）
bool right_hand_raised = right_shoulder_y < (right_elbow_y - hand_raise_threshold_);
bool left_hand_raised = left_shoulder_y < (left_elbow_y - hand_raise_threshold_);

// どちらかの手が上がっていればtrue
if (right_hand_raised || left_hand_raised) {
    // 追跡開始
}
```

---

## 追従制御アルゴリズム

### 1. 距離制御（比例制御）

```cpp
distance_error = current_distance - target_distance;
linear_velocity = clamp(distance_error * 0.5, -max_linear_speed, max_linear_speed);
```

- 目標距離より遠い → 前進
- 目標距離より近い → 後退
- 許容誤差内 → 停止

### 2. 角度制御（比例制御）

```cpp
angle = atan2(target_y, target_x);
angular_velocity = clamp(angle * 1.0, -max_angular_speed, max_angular_speed);
```

- ターゲットが右側 → 右旋回
- ターゲットが左側 → 左旋回
- 許容誤差内 → 停止

---

## 使用例

### 例1: ショッピングカートロボット

```bash
# 追従距離を2m、速度を遅めに設定
ros2 launch zed_goal_publisher hand_raised_person_follower_launch.py \
  target_distance:=2.0 \
  max_linear_speed:=0.2 \
  max_angular_speed:=0.3
```

**シナリオ**:
1. ユーザーが手を上げる
2. ロボットが検出してID記録
3. ユーザーの2m後方を追従
4. ユーザーが荷物を載せて手を下ろしても追従継続

### 例2: 高速追従（デモ用）

```bash
ros2 launch zed_goal_publisher hand_raised_person_follower_launch.py \
  target_distance:=1.0 \
  max_linear_speed:=0.5 \
  max_angular_speed:=0.8
```

### 例3: Nav2統合（自律ナビゲーション）

```bash
# 手上げ追従ノードを起動
ros2 launch zed_goal_publisher hand_raised_person_follower_launch.py

# 別ターミナル: target_personフレームへナビゲーション
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 0.0, y: 0.0, z: 0.0},
    orientation: {w: 1.0}
  }
}" --once
```

---

## トラブルシューティング

### ❌ 問題: 手を上げても検出されない

**原因**: 骨格検出が正しく動作していない

**確認**:
```bash
# ZED骨格検出トピックを確認
ros2 topic echo /zed/zed_node/body_trk/skeletons
```

**解決策**:
- ZED2iのBody Trackingが有効になっているか確認
- カメラから1〜5m程度の距離で試す
- 全身が映るようにカメラ視野に入る

---

### ❌ 問題: 追従が不安定

**原因**: パラメータ調整不足

**解決策**:
```bash
# 許容誤差を大きくして安定化
ros2 launch zed_goal_publisher hand_raised_person_follower_launch.py \
  distance_tolerance:=0.5 \
  angle_tolerance:=0.2
```

---

### ❌ 問題: 他の人に切り替わってしまう

**原因**: 追跡タイムアウトが短すぎる

**解決策**:
```bash
# タイムアウトを延長
ros2 launch zed_goal_publisher hand_raised_person_follower_launch.py \
  tracking_timeout:=5.0
```

---

### ❌ 問題: ロボットが動かない

**確認**:
```bash
# 速度指令が出ているか確認
ros2 topic echo /cmd_vel

# 追跡状態を確認
ros2 topic echo /hand_follower/status
```

**解決策**:
- `/cmd_vel`トピックがメカナム制御に正しく接続されているか確認
- 他のノードが`/cmd_vel`を上書きしていないか確認

---

## 出力トピック

| トピック名 | 型 | 説明 |
|-----------|-----|------|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 速度指令 |
| `/hand_follower/status` | `std_msgs/msg/String` | 追跡状態メッセージ |
| `/hand_follower/target_id` | `std_msgs/msg/Int32` | 追跡中の人物ID |
| `TF: target_person` | `TransformStamped` | 追従対象のTFフレーム |

### 状態メッセージの種類

- `"Waiting for hand raise signal"` - 手上げ待機中
- `"Tracking ID X"` - ID=Xの人を追跡中
- `"Lost target ID X"` - 追跡対象を見失った
- `"No person detected"` - 人物未検出
- `"Tracking timeout - waiting for new target"` - タイムアウト

---

## 実装の特徴

### ✅ 利点

1. **追加センサー不要** - ZED2iカメラのみで実現
2. **自然なインタラクション** - 手を上げるだけで追跡開始
3. **個人識別** - 複数人がいても特定の人だけを追従
4. **既存システム統合** - Nav2やTF2と互換性あり
5. **リアルタイム性** - 低遅延で応答性が高い

### ⚠️ 制限事項

1. ZED2iのBody Trackingライセンスが必要
2. 屋内環境推奨（明るさが重要）
3. 全身が映る必要がある（1〜5m程度の距離）
4. 処理負荷がやや高い（骨格検出）

---

## 応用例

### 1. 荷物運搬ロボット
- 倉庫や工場でスタッフに追従
- 手を上げた人の後を自動追従
- 荷物を置いたら次のスタッフへ

### 2. ガイドロボット
- 博物館や施設での案内
- ゲストが手を上げて呼び出し
- 目的地まで先導

### 3. 介護支援ロボット
- 介護施設での物品運搬
- 介護士が手を上げて呼び出し
- 必要な場所まで追従

---

## ログ出力例

```
[INFO] [hand_raised_person_follower]: Hand Raised Person Follower Node Started
[INFO] [hand_raised_person_follower]:   Target distance: 1.50 m
[INFO] [hand_raised_person_follower]:   Distance tolerance: 0.30 m
[INFO] [hand_raised_person_follower]:   Hand raise threshold: 0.30 m
[INFO] [hand_raised_person_follower]: 🎯 Started tracking person ID 42 (hand raised)
[DEBUG] [hand_raised_person_follower]: Tracking person ID 42 (hand raised)
[DEBUG] [hand_raised_person_follower]: Following: dist=2.35 (target=1.50), angle=0.12, cmd_vel=(0.42, 0.12)
[WARN] [hand_raised_person_follower]: ⏱️  Tracking timeout (3.2 sec). Resetting target.
```

---

## 今後の拡張案

1. **ジェスチャー認識**
   - 両手上げ：前進
   - 左手上げ：左旋回
   - 右手上げ：右旋回

2. **音声統合**
   - 「ついてきて」の音声コマンドで追跡開始

3. **複数人優先順位**
   - 最も近い人を優先
   - VIP IDの優先追従

4. **障害物回避**
   - nvbloxやLiDARと統合
   - 安全な経路での追従

---

## ライセンス

このノードはzed_goal_publisherパッケージの一部です。

## 作成日

2025-10-26

## バージョン

1.0.0
