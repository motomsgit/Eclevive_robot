# ROS2 ビルドヘルパー

ROS2パッケージのビルド、エラー対応、依存関係解決を自動化するスキル。

## 実行内容

### 1. 環境変数自動設定
```bash
export ISAAC_ROS_WS=/home/jetros/ros2_ws
export CUDA_HOME=/usr/local/cuda
export PATH=$CUDA_HOME/bin:$PATH
export CUDAARCHS=87
```

### 2. ビルド前チェック
- ディスク容量確認 (最低10GB必要)
- 既存ビルドキャッシュ確認
- 依存関係パッケージ確認

### 3. スマートビルド実行

#### 個別パッケージビルド
```bash
colcon build --packages-select <package_name> --symlink-install
```

#### 依存関係込みビルド
```bash
colcon build --packages-up-to <package_name> --symlink-install
```

#### オプショナルパッケージスキップビルド
```bash
colcon build --packages-skip-build-finished --symlink-install \
  --packages-skip isaac_ros_ess_models_install \
                  isaac_ros_peoplenet_models_install \
                  isaac_ros_peoplesemseg_models_install \
                  isaac_ros_nova_recorder
```

### 4. エラー自動診断

#### snprintf警告エラー
- **症状**: `__builtin_snprintf output may be truncated`
- **対処**: `-Wno-format-truncation`フラグ追加またはバッファサイズ修正

#### シンボリックリンクエラー
- **症状**: `failed to create symbolic link`
- **対処**: ビルドキャッシュ削除 `rm -rf build/<package>`

#### サブモジュール未初期化
- **症状**: `CMakeLists.txt file not found`
- **対処**: `git submodule update --init --recursive`

#### 依存パッケージ不足
- **症状**: `fatal error: *.hpp: No such file`
- **対処**: `sudo apt install ros-humble-<package>`

### 5. ビルド後検証
```bash
# インストール確認
ls install/ | grep <package_name>

# パッケージ実行テスト
ros2 pkg list | grep <package_name>
ros2 run <package_name> <executable> --help
```

### 6. ビルドログ保存
エラー発生時のログを自動的に保存し、再現可能な形で報告

## 出力形式

```
🔧 ビルド開始: <package_name>
✅ 環境変数設定完了
✅ 依存関係確認完了
🔨 ビルド実行中...
⚠️  警告検出: format-truncation (自動修正適用)
🔨 再ビルド中...
✅ ビルド成功 (所要時間: 45秒)
✅ パッケージ動作確認完了
```

## 使用タイミング

- 新規パッケージのビルド
- ビルドエラー発生時
- 依存関係の追加・更新後
- システム全体の再ビルド

## トークン削減効果

**従来**: エラー確認 → 調査 → 修正方法検索 → 実行 → 再確認 (約5000トークン)
**Skills使用後**: エラー自動診断・修正 → 結果報告 (約800トークン)
**削減率**: 約84%

## 高度な機能

### 並列ビルド最適化
Jetson環境でのメモリ不足を防ぐため、自動的に並列数を調整：
```bash
colcon build --parallel-workers 2 --symlink-install
```

### ビルド時間予測
パッケージサイズとCUDA使用有無から、ビルド時間を事前推定

### キャッシュ管理
ccache統計表示とキャッシュクリアの提案
