# ROS2 Skills クイックスタートガイド

## 🚀 5分で始める

### 1. スキルの確認

作成済みのスキル一覧：

```bash
ls /home/jetros/ros2_ws/.claude/skills/
```

以下の7つのファイルが存在することを確認：
- ✅ `ros2-quick-debug.md` - システム診断
- ✅ `ros2-build-helper.md` - ビルド支援
- ✅ `ros2-slam-tuner.md` - SLAM最適化
- ✅ `ros2-launch-wizard.md` - Launch作成
- ✅ `ros2-sensor-validator.md` - センサー検証
- ✅ `README.md` - 詳細ドキュメント
- ✅ `TOKEN_EFFICIENCY_REPORT.md` - 効率化レポート

---

## 💡 基本的な使い方

### 方法1: 自然言語で依頼（推奨）

Claude Codeは、あなたの依頼内容から自動的に適切なスキルを選択します。

#### 例1: システムの状態確認
```
「システムの状態を確認して」
```
→ 自動的に `ros2-quick-debug` スキルが起動

#### 例2: ビルドエラー対応
```
「パッケージのビルドエラーを修正して」
```
→ 自動的に `ros2-build-helper` スキルが起動

#### 例3: SLAM調整
```
「地図生成の品質を改善して」
```
→ 自動的に `ros2-slam-tuner` スキルが起動

### 方法2: スキル名を明示的に指定

特定のスキルを直接呼び出すことも可能：

```
/skill ros2-quick-debug
```

---

## 📝 よくある使用例

### 🔍 Case 1: 毎朝の動作確認

**シチュエーション**: システム起動後、正常に動いているか確認したい

**依頼例**:
```
「システム全体の状態をチェックして」
```

**実行されるスキル**: `ros2-quick-debug`

**期待される出力**:
```
✅ 正常: ノード起動確認 (53ノード)
✅ 正常: /merged_scan 配信中 (15.2 Hz)
✅ 正常: TF map -> base_link 正常
```

**所要時間**: 約30秒
**トークン**: 約500

---

### 🔧 Case 2: 新しいパッケージの追加

**シチュエーション**: センサー用の新しいパッケージを作成してビルドしたい

**依頼例**:
```
「新しいLiDARパッケージをビルドして動作確認まで行って」
```

**実行される連携**:
1. `ros2-build-helper` - ビルド実行
2. `ros2-quick-debug` - 動作確認
3. `ros2-sensor-validator` - センサー検証

**所要時間**: 約5分
**トークン**: 約800

---

### 🗺️ Case 3: SLAM地図がうまく生成できない

**シチュエーション**: 旋回時に壁が曲がる、地図がずれる

**依頼例**:
```
「SLAM地図生成を最適化して。旋回時に壁が曲がる問題を解決したい」
```

**実行される連携**:
1. `ros2-sensor-validator` - ハードウェア確認（LiDAR角度など）
2. `ros2-slam-tuner` - パラメータ診断・最適化

**所要時間**: 約10分
**トークン**: 約1,200

**重要**: 物理的な問題（LiDAR下向きなど）も自動検出されます！

---

### 🚀 Case 4: 新しいLaunchファイルの作成

**シチュエーション**: 複数のノードを起動するLaunchファイルを作りたい

**依頼例**:
```
「ZED2iとLiDARを起動するLaunchファイルを作って」
```

**実行されるスキル**: `ros2-launch-wizard`

**生成されるもの**:
- 完全なLaunchファイルコード
- 依存関係チェック
- 自動デバッグ実行

**所要時間**: 約3分
**トークン**: 約900

---

### 📡 Case 5: センサーが正常か確認したい

**シチュエーション**: 地図生成前にセンサーが正常動作しているか確認

**依頼例**:
```
「全センサーの状態を検証して」
```

**実行されるスキル**: `ros2-sensor-validator`

**確認項目**:
- ZED2iカメラ（画像、深度、VO、IMU）
- LiDAR前後（スキャン品質、点群密度）
- TF同期状態

**所要時間**: 約1分
**トークン**: 約700

---

## 🎯 スキル選択ガイド

### いつどのスキルを使うべきか？

| 状況 | 使用スキル | 依頼例 |
|------|-----------|--------|
| システム起動後の確認 | ros2-quick-debug | 「状態を確認」 |
| ビルドエラー発生 | ros2-build-helper | 「ビルドエラーを修正」 |
| 地図生成がうまくいかない | ros2-slam-tuner | 「SLAM最適化」 |
| Launchファイル作成 | ros2-launch-wizard | 「Launch作成」 |
| センサー異常の疑い | ros2-sensor-validator | 「センサー検証」 |
| 地図生成前のチェック | ros2-sensor-validator | 「センサー確認」 |

---

## ⚡ トークン効率化の実感

### 従来の方法との比較

#### 従来: システムデバッグ
```
あなた: 「ノード一覧を表示して」
Claude: [実行] → 結果表示
あなた: 「次にトピック一覧を」
Claude: [実行] → 結果表示
あなた: 「/merged_scanのレートは？」
Claude: [実行] → 結果表示
あなた: 「TFの状態は？」
...（5-7往復）

合計: 約3,000トークン、10分
```

#### Skills使用後
```
あなた: 「システムを診断して」
Claude: [ros2-quick-debug起動]
  → 全項目を自動確認
  → 統合レポート表示

合計: 約500トークン、2分
```

**削減**: 83%のトークン、80%の時間

---

## 📊 効果測定

### あなたの削減効果を確認

1ヶ月使用後、以下を確認してください：

#### 定量的指標
- **会話の往復回数**: 減っているはず
- **作業完了までの時間**: 大幅に短縮
- **トークン使用量**: 約80-85%削減

#### 定性的指標
- **ストレス**: 定型作業の自動化で軽減
- **集中力**: 本質的な設計に集中できる
- **品質**: 見落としが減る

---

## 🔧 カスタマイズ

### プロジェクトに合わせた調整

各スキルは、あなたのプロジェクトに合わせてカスタマイズ可能です。

#### 例: SLAM調整の頻度が高い場合

[ros2-slam-tuner.md](ros2-slam-tuner.md)を編集して、あなたのロボットに最適なデフォルト値を設定：

```yaml
# あなたの環境で実験済みの最適値
angle_variance_penalty: 2.3  # デフォルトを2.3に変更
coarse_angle_resolution: 0.016
minimum_travel_heading: 0.015
```

#### 例: カスタムセンサーの追加

[ros2-sensor-validator.md](ros2-sensor-validator.md)に、あなた独自のセンサーチェックを追加：

```bash
#### カスタムセンサー
# 追加のLiDARやカメラなど
ros2 topic hz /your_custom_sensor
ros2 topic echo /your_custom_sensor --once
```

---

## ❓ トラブルシューティング

### Q: スキルが起動しない

**A**: スキルファイルのパスを確認してください：
```bash
ls /home/jetros/ros2_ws/.claude/skills/*.md
```

7つのファイルが存在することを確認。

---

### Q: 期待と違う動作をする

**A**: より具体的に依頼してください：

❌ 「確認して」
✅ 「システム全体の状態を診断して」

❌ 「最適化して」
✅ 「SLAM地図生成を最適化して。旋回時の壁面湾曲を改善したい」

---

### Q: 複数のスキルを組み合わせたい

**A**: 自然言語で複合的なタスクを依頼すれば、Claude Codeが自動的に適切なスキルを連携実行します：

```
「新しいセンサーを追加して、Launchファイルを作成し、動作確認までやって」

→ 自動連携:
  1. ros2-sensor-validator (現状確認)
  2. ros2-build-helper (ビルド)
  3. ros2-launch-wizard (Launch作成)
  4. ros2-quick-debug (動作確認)
```

---

## 📚 さらに詳しく知りたい

### 詳細ドキュメント

- **[README.md](README.md)**: 全スキルの詳細説明
- **[TOKEN_EFFICIENCY_REPORT.md](TOKEN_EFFICIENCY_REPORT.md)**: 詳細な効率化分析

### 個別スキルのドキュメント

- [ros2-quick-debug.md](ros2-quick-debug.md)
- [ros2-build-helper.md](ros2-build-helper.md)
- [ros2-slam-tuner.md](ros2-slam-tuner.md)
- [ros2-launch-wizard.md](ros2-launch-wizard.md)
- [ros2-sensor-validator.md](ros2-sensor-validator.md)

---

## 🎓 ベストプラクティス

### 1. 毎日のルーチン

```
朝: 「システム全体を診断して」
  → ros2-quick-debug + ros2-sensor-validator

作業中: 必要に応じて個別スキル使用

終了前: 「最終確認して」
  → ros2-quick-debug
```

### 2. 新機能開発時

```
1. 「既存システムの状態を確認」
   → ros2-sensor-validator

2. 「新パッケージをビルド」
   → ros2-build-helper

3. 「Launchファイル作成」
   → ros2-launch-wizard

4. 「統合テスト」
   → ros2-quick-debug
```

### 3. 問題発生時

```
1. まず「システム診断」
   → ros2-quick-debug

2. 問題箇所に応じて専門スキル使用
   - ビルド問題 → ros2-build-helper
   - SLAM問題 → ros2-slam-tuner
   - センサー問題 → ros2-sensor-validator
   - Launch問題 → ros2-launch-wizard
```

---

## 🚀 今すぐ始めよう！

### ステップ1: 簡単なテスト

```
「システムの状態を確認して」
```

と依頼してみてください。`ros2-quick-debug`が起動するはずです。

### ステップ2: 実際の作業に適用

今抱えている問題や、やりたいことを自然に依頼してみてください。

### ステップ3: 効果を実感

従来の方法と比べて、どれだけ早く、少ない手間で完了するか体感してください。

---

**Happy Coding with ROS2 Skills! 🤖✨**

---

**最終更新**: 2025-10-19
**バージョン**: 1.0
