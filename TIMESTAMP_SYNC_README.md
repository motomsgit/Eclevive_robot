# laser_merger2 タイムスタンプ同期機能

## 概要

front_scanとback_scanをマージする際のタイムスタンプ精度問題に対応するため、以下の機能を実装しました。

## 実装した機能

### 1. タイムスタンプ検証機能 (`validateTimestampSync`)

**目的**: 複数のLiDARスキャン間のタイムスタンプ差を検証し、同期精度を確保

**動作**:
- 受信した全スキャンのタイムスタンプを比較
- 最古と最新のタイムスタンプ差を計算
- 閾値 (`max_time_difference`) を超える場合は警告を出力
- 閾値超過時はマージをスキップ（データ整合性を優先）

**パラメータ**:
- `max_time_difference`: タイムスタンプ差の許容値（秒）
  - デフォルト: 0.05秒（50ms）
  - SLAM用途では20-50msが推奨
- `enable_timestamp_validation`: 検証機能の有効/無効

### 2. 複数のタイムスタンプ戦略 (`calculateMergedTimestamp`)

**目的**: 用途に応じて最適なタイムスタンプ計算方法を選択

**実装した戦略**:

#### a) `oldest` (最古タイムスタンプ) ⭐推奨 for SLAM
- 全スキャンの中で最も古いタイムスタンプを使用
- **利点**: データ取得開始時点を基準にするため保守的で安全
- **用途**: SLAM、ナビゲーション、ローカライゼーション
- **理由**: 「すべてのセンサーデータが揃った時点」を基準とすべき

#### b) `newest` (最新タイムスタンプ)
- 全スキャンの中で最も新しいタイムスタンプを使用
- **利点**: リアルタイム性重視
- **用途**: 障害物検出、リアルタイム制御

#### c) `average` (平均タイムスタンプ)
- 全スキャンのタイムスタンプの算術平均
- **利点**: バランス型
- **用途**: 一般的なアプリケーション

#### d) `weighted_average` (加重平均タイムスタンプ)
- 有効点数に応じて重み付けした平均
- **利点**: データ量が多いスキャンを優先
- **用途**: 非対称なセンサー配置

**パラメータ**:
- `timestamp_strategy`: "oldest" | "newest" | "average" | "weighted_average"

### 3. タイムスタンプ診断機能 (`publishTimestampDiagnostics`)

**目的**: 動作中のタイムスタンプ同期状態を監視

**出力内容**:
- スキャン間のタイムスタンプ差（ms単位）
- 受信したスキャン数
- 使用している戦略
- 5秒ごとにログ出力

**パラメータ**:
- `publish_diagnostics`: 診断出力の有効/無効

## 設定方法

### launchファイル設定例

```python
parameters=[
    # 既存パラメータ...

    # タイムスタンプ同期パラメータ
    {'max_time_difference': 0.05},  # 50ms
    {'timestamp_strategy': 'oldest'},
    {'enable_timestamp_validation': True},
    {'publish_diagnostics': True}
]
```

### 推奨設定（用途別）

#### SLAM用途
```python
{'max_time_difference': 0.05},      # 50ms閾値
{'timestamp_strategy': 'oldest'},   # 保守的なアプローチ
{'enable_timestamp_validation': True},
{'publish_diagnostics': True}
```

#### リアルタイム障害物回避
```python
{'max_time_difference': 0.1},       # 100ms閾値（緩め）
{'timestamp_strategy': 'newest'},   # 最新データ優先
{'enable_timestamp_validation': False},  # 低遅延優先
{'publish_diagnostics': False}
```

#### 一般用途
```python
{'max_time_difference': 0.1},
{'timestamp_strategy': 'average'},
{'enable_timestamp_validation': True},
{'publish_diagnostics': True}
```

## テスト方法

### 1. LiDARスキャンレート測定
```bash
cd /home/jetros/ros2_ws
./test_scan_rate.sh
```

### 2. タイムスタンプ同期テスト
```bash
cd /home/jetros/ros2_ws
./test_timestamp_sync.sh
```

### 3. 手動テスト
```bash
# ターミナル1: laser_merger2起動
ros2 launch laser_merger2 laser_merger_for_slam.launch.py

# ターミナル2: ログ監視
ros2 topic echo /rosout --field msg | grep -i timestamp

# ターミナル3: 出力レート確認
ros2 topic hz /merged_scan
```

## ログ出力例

### 正常動作時
```
[laser_merger2]: Timestamp sync config: strategy=oldest, max_diff=0.050s, validation=enabled, diagnostics=enabled
[laser_merger2]: Timestamp diagnostics: diff=12.34 ms, scans=2, strategy=oldest
```

### タイムスタンプ差が大きい場合
```
[WARN] [laser_merger2]: Timestamp差が閾値を超えています: 65.5 ms (閾値: 50.0 ms)
[DEBUG] [laser_merger2]:   /front_scan: 0.0 ms old
[DEBUG] [laser_merger2]:   /back_scan: 65.5 ms old
[DEBUG] [laser_merger2]: Timestamp validation failed, skipping merge
```

## トラブルシューティング

### 問題1: タイムスタンプ差の警告が頻発する

**原因**: LiDARの同期がずれている

**対策**:
1. `max_time_difference`を緩める（0.1秒程度）
2. LiDARのスキャンレートを確認（`ros2 topic hz /front_scan`）
3. 両LiDARのUSB接続を確認

### 問題2: merged_scanが出力されない

**原因**: タイムスタンプ検証でスキップされている

**対策**:
1. `enable_timestamp_validation: False`に設定して一時的に無効化
2. 個々のLiDARトピックを確認（`ros2 topic list`）
3. ログで詳細を確認（`--ros-args --log-level debug`）

### 問題3: SLAMの精度が悪い

**対策**:
1. `timestamp_strategy`を`oldest`に設定
2. `max_time_difference`を50ms以下に厳格化
3. LiDARのハードウェア同期を検討（外部トリガー信号など）

## 将来の拡張案

### message_filters統合（次フェーズ）

より厳密な時刻同期のために、ROS2の`message_filters::ApproximateTimeSynchronizer`を統合する案があります。

**メリット**:
- タイムスタンプが近いペアのみを自動マッチング
- ROS2標準機能で信頼性が高い
- コールバックベースで効率的

**実装イメージ**:
```cpp
#include "message_filters/sync_policies/approximate_time.h"

typedef message_filters::sync_policies::ApproximateTime<LaserScan, LaserScan> SyncPolicy;
typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;

// 10ms以内のタイムスタンプを同期
sync_ = std::make_shared<Synchronizer>(SyncPolicy(10), front_sub, back_sub);
```

## 参考情報

- ROS2 message_filters: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Time-Travel-With-Tf2-Cpp.html
- タイムスタンプ同期のベストプラクティス: https://docs.ros.org/en/humble/Concepts/About-Time.html
- TF2とタイミング: https://docs.ros.org/en/humble/Concepts/About-Tf2.html

## 変更履歴

- 2025-10-09: 初版作成
  - タイムスタンプ検証機能追加
  - 4種類のタイムスタンプ戦略実装
  - 診断機能追加
  - SLAM用推奨パラメータ設定
