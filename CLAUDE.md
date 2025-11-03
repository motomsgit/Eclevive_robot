# Claude Code 設定

## 言語設定
- 回答言語: 日本語
- すべての応答は日本語で行う

## 初期設定
このファイルはClaude Codeの初期設定を記録するためのものです。

---

## 問題修正時の必須プロトコル

### 修正後の動作確認ルール
問題を修正した際は、以下の手順を**必ず実施**すること：

1. **修正内容の実装**
   - コードやパラメータファイルを修正

2. **ビルド（必要な場合）**
   - `colcon build --packages-select <package_name> --symlink-install`
   - ビルドエラーがないことを確認

3. **動作確認テストの実施**
   - 修正したコンポーネントを実際に実行
   - エラーログがないことを確認
   - 期待される動作をしているか確認
   - 可能であれば自動テストスクリプトを作成して実行

4. **確認結果の報告**
   - テスト実行結果を提示
   - エラーがないことを証明
   - 期待される出力が得られていることを示す
   - 例：
     - ✅ ノードが正常起動
     - ✅ エラーログなし
     - ✅ トピックが正常に出力（XX Hzで確認）

5. **修正の完了報告**
   - 修正内容のまとめ
   - 動作確認結果
   - 実行方法の記載

### 禁止事項
- ❌ 修正後にテストせずに「修正しました」と報告する
- ❌ 推測で「動作するはずです」と報告する
- ❌ エラーが残っているのに完了と報告する

### 推奨事項
- ✅ 実際の実行結果を提示する
- ✅ ログやtopic hzの出力を証拠として示す
- ✅ エラーが出ていないことを明示する
- ✅ テストスクリプトを作成して再現可能にする

---

## ロボットシステム仕様

### 起動コマンド
- **現行版（推奨）**: `ros2 launch bringup zed2i_nvblox_nav2_launch.py`
  - ZED2i + nvblox（3Dマッピング） + Nav2統合システム
  - 詳細: [.claude/docs/system-architecture.md](.claude/docs/system-architecture.md)

### ハードウェア構成
- **プラットフォーム**: Jetson (Linux 5.15.148-tegra)
- **ロボットタイプ**: メカナムホイール移動ロボット
- **主要センサー**: ZED2i、LiDAR x2、IMU
- **制御方式**: PS5コントローラー

詳細: [.claude/docs/system-architecture.md](.claude/docs/system-architecture.md)

---

## TFツリー構造

### 重要な設計決定（絶対に変更禁止）

**⚠️ CRITICAL: 以下のパラメータは絶対に変更しないこと ⚠️**

ZED2iカメラのTFフレーム設定（`common_stereo_isaac.yaml`および`zed2i_nvblox_fixed.launch.py`）:
- `map_frame: 'odom'` ← **絶対に変更禁止！** Visual SLAMの親フレーム（ZED2iにとっての「マップ」）
- `odometry_frame: 'zed_camera_origin'` ← **絶対に変更禁止！** 中間オドメトリフレーム
- `publish_map_tf: true` ← odom→zed_camera_origin TFを配信（必須）
- `publish_tf: true` ← zed_camera_origin→zed_camera_link TFを配信

**理由:**
- このTF構成はZED2i Visual OdometryとnvbloxのOccupancy Grid配信の両立に必須
- ZED2iは`map_frame='odom'`を自身の最上位フレームとして扱い、`odom → zed_camera_origin`のTFを配信
- nvbloxは独立して`global_frame='map'`でOccupancy Gridを配信
- `map → odom`のTFは`static_transform_publisher`が静的に固定（0, 0, 0）
- この設計により、ZED2iとnvbloxが競合せずに動作可能

### 標準TFツリー
```
map (nvblox global_frame)
 └─ odom (ZED2i map_frame - Visual SLAM親フレーム)
     └─ zed_camera_origin (ZED2i odometry_frame)
         └─ zed_camera_link (ロボット基準点)
             ├─ base_link
             ├─ front_lidar
             └─ back_lidar
```

詳細: [.claude/docs/tf-tree.md](.claude/docs/tf-tree.md)

---

## 主要トピック

### センサー
- LiDAR: `/front_scan`, `/back_scan`, `/merged_scan`, `/scan_filtered`
- オドメトリ: `/zed/zed_node/odom`, `/zed/zed_node/odom_zupt`
- カメラ: `/zed/zed_node/rgb/image_rect_color`, `/zed/zed_node/depth/depth_registered`

### 制御
- `/cmd_vel` - 統合速度指令
- `/goal_pose` - ナビゲーションゴール

### マップ・ナビゲーション
- `/map` - nvblox Occupancy Grid（Nav2互換）
- `/nvblox_node/esdf_pointcloud` - ESDF点群
- `/plan` - グローバルプラン

詳細: [.claude/docs/topics.md](.claude/docs/topics.md)

---

## ビルドシステム

### 推奨ビルドコマンド
```bash
# 通常のビルド（未完了パッケージのみ）
colcon build --packages-skip-build-finished --symlink-install \
  --packages-skip isaac_ros_ess_models_install \
                  isaac_ros_peoplenet_models_install \
                  isaac_ros_peoplesemseg_models_install \
                  isaac_ros_nova_recorder

# 特定パッケージのみビルド
colcon build --packages-select <package_name> --symlink-install
```

詳細: [.claude/docs/build-system.md](.claude/docs/build-system.md)

---

## デバッグ・確認コマンド

### 基本確認
```bash
# ノード確認
ros2 node list | grep -E "nvblox|zed|controller|planner"

# トピック確認
ros2 topic hz /map
ros2 topic hz /scan_filtered

# TF確認（重要）
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo map zed_camera_link

# パラメータ確認
ros2 param get /zed/zed_node pos_tracking.publish_map_tf  # 期待値: true
ros2 param get /zed/zed_node pos_tracking.map_frame       # 期待値: odom
ros2 param get /zed/zed_node pos_tracking.odometry_frame  # 期待値: zed_camera_origin
ros2 param get /nvblox_node global_frame                  # 期待値: map
```

詳細: [.claude/docs/troubleshooting.md](.claude/docs/troubleshooting.md)

---

## 既知の課題・注意点

### 1. ZED2iのTF設定の重要性
- `publish_map_tf: true` が必須
- `map_frame: 'odom'` が必須（ZED2iの最上位フレーム）
- `odometry_frame: 'zed_camera_origin'` が必須
- **この設定を間違えるとNav2が動作しない**

### 2. LiDAR物理的取り付けの確認（重要）
⚠️ **地図生成の前に必ず確認すべき項目**
- **症状**: SLAMで地図がうまく生成できない
- **原因**: LiDARが下向きに傾いている
- **対処**: LiDARの取り付け角度を物理的に調整し、水平にする

詳細: [.claude/docs/troubleshooting.md](.claude/docs/troubleshooting.md)

---

## ドキュメント構成

このプロジェクトのドキュメントは以下のように分割されています：

- **CLAUDE.md** (このファイル): 基本設定と概要
- **[.claude/docs/system-architecture.md](.claude/docs/system-architecture.md)**: システムアーキテクチャ詳細
- **[.claude/docs/tf-tree.md](.claude/docs/tf-tree.md)**: TFツリー仕様
- **[.claude/docs/topics.md](.claude/docs/topics.md)**: トピック仕様
- **[.claude/docs/build-system.md](.claude/docs/build-system.md)**: ビルドシステム
- **[.claude/docs/troubleshooting.md](.claude/docs/troubleshooting.md)**: トラブルシューティング

---

## スラッシュコマンド

便利なスラッシュコマンドが用意されています：

- `/ros2-diagnose`: システム診断を実行
- `/ros2-build`: パッケージビルドを実行
- `/ros2-test-nav`: ナビゲーションテストを実行

詳細は各コマンドを実行してご確認ください。
