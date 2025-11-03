# Isaac ROS ビルド手順書

## 問題の概要

元の `build_isaac_ros_staged.sh` スクリプトには以下の問題がありました:

1. **依存パッケージの順序エラー**: Stage 3-14で、必要な依存パッケージがビルドされる前にビルドを試みていた
2. **欠落していた依存パッケージ**:
   - `negotiated_interfaces` と `negotiated`
   - `isaac_ros_gxf`
   - `isaac_ros_nitros` と多数のnitros typeパッケージ
   - GXF基礎パッケージ群 (gems, atlas, gxf_helpers, messages, optimizer, sight, tensorops等)
   - ZEDパッケージ名の変更 (`zed_interfaces` → `zed_msgs`)

## 解決策

修正版スクリプト `build_isaac_ros_fixed.sh` を作成しました。このスクリプトは:

- **31ステージ**に分割された段階的ビルド
- **正しい依存関係の順序**でパッケージをビルド
- すべての必要な依存パッケージを含む

## ビルド手順

### オプション1: 修正版スクリプトを使用 (推奨 - クリーンビルド時)

```bash
cd /home/jetros/ros2_ws

# 既存のビルドをクリーンアップ (必要に応じて)
rm -rf build install log

# 修正版スクリプトを実行
./build_isaac_ros_fixed.sh
```

### オプション2: シンプルな一括ビルド (既に依存パッケージがビルド済みの場合)

現在実行中の元のスクリプトのStage 15が完了すれば、すべての依存パッケージがビルドされているため、以下のシンプルなコマンドですべてをビルドできます:

```bash
cd /home/jetros/ros2_ws

# 環境変数設定
export CUDA_HOME=/usr/local/cuda
export PATH=$CUDA_HOME/bin:$PATH
export CUDAARCHS=87
export GXF_SDK_ROOT=/opt/nvidia/graph-composer/extension-dev
export CMAKE_PREFIX_PATH=$GXF_SDK_ROOT:$CMAKE_PREFIX_PATH
export CPLUS_INCLUDE_PATH=$GXF_SDK_ROOT:$CPLUS_INCLUDE_PATH
export LD_LIBRARY_PATH=/opt/nvidia/graph-composer:$LD_LIBRARY_PATH
export CVCUDA_DIR=/opt/nvidia/cvcuda0
export CMAKE_PREFIX_PATH=$CVCUDA_DIR:$CMAKE_PREFIX_PATH
export CPLUS_INCLUDE_PATH=$CVCUDA_DIR/include:$CPLUS_INCLUDE_PATH
export LD_LIBRARY_PATH=$CVCUDA_DIR/lib:$LD_LIBRARY_PATH

source /opt/ros/humble/setup.bash

# 一括ビルド
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 1
```

### オプション3: 特定のパッケージのみ再ビルド

```bash
cd /home/jetros/ros2_ws
source install/setup.bash

# 例: isaac_ros_apriltag のみビルド
colcon build --packages-select isaac_ros_apriltag --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 修正版スクリプトのステージ構成

### 基礎レイヤー (Stage 1-7)
1. **Interface Packages**: メッセージ定義
2. **Common & Negotiated**: 共通ライブラリ
3. **GXF Core**: Isaac ROS GXF基盤
4. **GXF Base Extensions**: GXF基礎拡張
5. **NITROS Core**: NITROSフレームワーク本体
6. **NITROS Types**: NITROSメッセージ型
7. **Managed NITROS**: NITROS管理層

### 画像処理レイヤー (Stage 8-9)
8. **GXF Image Extensions**: GXF画像処理拡張
9. **Image Pipeline**: 画像処理パイプライン

### DNNレイヤー (Stage 10-13)
10. **GXF DNN Extensions**: GXF DNN拡張
11. **DNN Inference Base**: DNN推論基盤
12. **GXF Object Detection Extensions**: GXF物体検出拡張
13. **Object Detection**: 物体検出

### ステレオ・セグメンテーションレイヤー (Stage 14-17)
14. **GXF ESS Extension**: GXF ESS拡張
15. **Stereo Depth**: ステレオ深度推定
16. **GXF Segmentation Extensions**: GXFセグメンテーション拡張
17. **Segmentation**: セグメンテーション

### 位置推定・SLAMレイヤー (Stage 18-21)
18. **GXF Pose Extensions**: GXF姿勢推定拡張
19. **Pose Estimation**: 姿勢推定
20. **AprilTag**: AprilTag検出
21. **Visual SLAM**: Visual SLAM

### 統合レイヤー (Stage 22-31)
22-24. **Nvblox**: 3Dマッピング
25-28. **Nova**: センサースイート
29. **NITROS Bridge**: ROSブリッジ
30. **ZED Wrapper**: ZEDカメララッパー
31. **Remaining**: その他すべてのパッケージ

## ビルド時間の目安

- **クリーンビルド (修正版スクリプト)**: 約3-4時間 (Jetson環境、parallel-workers 1)
- **差分ビルド**: 数分~数十分

## トラブルシューティング

### ビルドが途中で失敗する場合

```bash
# 失敗したステージから再開
source install/setup.bash
colcon build --packages-select <失敗したパッケージ> --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### メモリ不足エラー

```bash
# スワップを増やす、または他のプロセスを停止
# parallel-workers を 1 に設定済み (推奨)
```

### 依存関係エラー

```bash
# 依存パッケージを先にビルド
colcon list --packages-up-to <パッケージ名>
colcon build --packages-up-to <パッケージ名> --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## ビルド完了後の確認

```bash
# インストールされたパッケージの確認
source install/setup.bash
ros2 pkg list | grep isaac_ros
ros2 pkg list | grep gxf_isaac
ros2 pkg list | grep nvblox
ros2 pkg list | grep zed

# 動作確認 (例: isaac_ros_apriltag)
ros2 launch isaac_ros_apriltag isaac_ros_apriltag.launch.py
```

## 注意事項

1. **CUDA環境変数**: CUDAARCHS=87 はJetson Orin用です。他のハードウェアでは調整が必要です。
2. **並列ビルド**: Jetsonのメモリ制限により `--parallel-workers 1` を使用しています。
3. **ビルド時間**: 初回ビルドは非常に時間がかかります。気長に待ちましょう。
4. **警告**: ビルド中に表示される警告(deprecated, header install destination等)は無視して問題ありません。

## 元のスクリプトとの違い

| 項目 | 元のスクリプト | 修正版スクリプト |
|------|--------------|------------------|
| ステージ数 | 15 | 31 |
| 依存関係 | Stage 15で自動解決 | 明示的に順序指定 |
| negotiated | ✗ 欠落 | ✓ Stage 2 |
| isaac_ros_gxf | ✗ 欠落 | ✓ Stage 3 |
| GXF基礎パッケージ | ✗ 欠落 | ✓ Stage 4 |
| NITROS | ✗ 欠落 | ✓ Stage 5-7 |
| ZED | zed_interfaces (誤) | zed_msgs (正) |

## まとめ

修正版スクリプト `build_isaac_ros_fixed.sh` を使用することで:
- ✅ すべての依存関係が正しい順序でビルドされる
- ✅ ビルドエラーが発生しにくい
- ✅ 再ビルド時の効率が向上
- ✅ 将来的な保守性が向上
