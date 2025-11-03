# Bringup ZED2i Isaac ROS nvblox 統合Launch

## 概要

`zed2i_isaac_nvblox.launch.py`は、bringupパッケージに追加された統合launchファイルで、ZED2iカメラとIsaac ROS nvbloxを1つのコマンドで起動できます。

## ファイル情報

**ファイル名:** `zed2i_isaac_nvblox.launch.py`
**場所:** `src/bringup/launch/zed2i_isaac_nvblox.launch.py`
**パッケージ:** bringup

## 特徴

### 統合されるコンポーネント

1. **ZED2iカメラドライバ**
   - Isaac ROS互換設定（`common_stereo_isaac.yaml`使用）
   - `zed2i_isaac_camera.launch.py`経由で起動
   - Visual Odometry、深度画像、カラー画像を配信

2. **nvblox 3Dマッピング**
   - リアルタイム3D環境マッピング
   - ZED2iの深度データを使用してTSDFボリュームを構築
   - メッシュ、ESDF（距離場）、2Dマップスライスを生成

3. **可視化ツール（オプショナル）**
   - RViz2設定
   - 3Dメッシュ、ポイントクラウド、TFフレーム表示

4. **コンポーネントコンテナ**
   - すべてのノードを同一プロセス内で実行（効率化）
   - `component_container_mt`使用（マルチスレッド対応）

## 使用方法

### 基本起動

```bash
ros2 launch bringup zed2i_isaac_nvblox.launch.py
```

すべてのコンポーネント（カメラ、nvblox、可視化）が起動します。

### 可視化なしで起動

```bash
ros2 launch bringup zed2i_isaac_nvblox.launch.py enable_visualization:=false
```

RViz2可視化を無効化し、カメラとnvbloxのみ起動します（軽量化）。

### カスタムコンテナ名で起動

```bash
ros2 launch bringup zed2i_isaac_nvblox.launch.py container_name:=my_nvblox_container
```

コンポーネントコンテナに別名を指定します。

### デバッグモードで起動

```bash
ros2 launch bringup zed2i_isaac_nvblox.launch.py log_level:=debug
```

詳細なログ出力を有効化します。

## Launch引数

| 引数名 | デフォルト値 | 説明 | 選択肢 |
|--------|-------------|------|--------|
| `camera_name` | `zed` | カメラのネームスペース | - |
| `camera_model` | `zed2i` | ZEDカメラモデル | - |
| `enable_visualization` | `true` | RViz2可視化の有効化 | `true`, `false` |
| `container_name` | `nvblox_container` | コンテナ名 | - |
| `log_level` | `info` | ログレベル | `debug`, `info`, `warn`, `error` |

## 配信されるトピック

### ZED2iカメラ

| トピック名 | 型 | 説明 |
|-----------|-----|------|
| `/zed/zed_node/rgb/image_rect_color` | `sensor_msgs/Image` | 補正済みカラー画像 |
| `/zed/zed_node/rgb/camera_info` | `sensor_msgs/CameraInfo` | カメラ情報 |
| `/zed/zed_node/depth/depth_registered` | `sensor_msgs/Image` | 登録済み深度画像 |
| `/zed/zed_node/depth/camera_info` | `sensor_msgs/CameraInfo` | 深度カメラ情報 |
| `/zed/zed_node/pose` | `geometry_msgs/PoseStamped` | カメラポーズ |
| `/zed/zed_node/odom` | `nav_msgs/Odometry` | Visual Odometry |
| `/zed/zed_node/imu/data` | `sensor_msgs/Imu` | IMUデータ |

### nvblox

| トピック名 | 型 | 説明 |
|-----------|-----|------|
| `/nvblox_node/mesh` | `nvblox_msgs/Mesh` | 3Dメッシュ（可視化用） |
| `/nvblox_node/map_slice` | `nav_msgs/OccupancyGrid` | 2Dマップスライス |
| `/nvblox_node/static_esdf_pointcloud` | `sensor_msgs/PointCloud2` | ESDF距離場 |
| `/nvblox_node/back_projected_depth` | `sensor_msgs/PointCloud2` | バックプロジェクション点群 |

## TFフレーム構造

```
map
 └─ odom (nvbloxが配信 + ZED Visual Odometry)
     └─ zed_camera_link (ZED基準フレーム)
         └─ base_link (ロボット基準フレーム、URDFで定義)
             ├─ zed_camera_center
             ├─ zed_left_camera_frame
             ├─ zed_right_camera_frame
             └─ ...
```

**重要なポイント:**
- `map → odom`: nvbloxが配信（マッピング結果）
- `odom → zed_camera_link`: ZED Visual Odometryが配信
- `zed_camera_link → base_link`: URDF static_transform_publisherが配信

## 内部構造

### 起動シーケンス

```
1. Component Container起動
   ↓
2. ZED2iカメラ起動 (zed2i_isaac_camera.launch.py)
   ├─ URDF/xacroロード
   ├─ Robot State Publisher起動
   └─ ZED Nodeをコンテナに読み込み
   ↓
3. nvblox起動 (nvblox.launch.py)
   ├─ nvblox Nodeをコンテナに読み込み
   └─ トピックリマッピング設定
   ↓
4. 可視化起動 (visualization.launch.py) ※条件付き
   └─ RViz2設定ロード
```

### データフロー

```
ZED2iカメラ
    ↓ (RGB画像、深度画像、ポーズ)
/zed/zed_node/*
    ↓ (リマッピング)
camera_0/* (nvblox内部)
    ↓ (3Dマッピング処理)
nvblox TSDF統合
    ↓
/nvblox_node/* (メッシュ、ESDF、マップスライス)
```

## 動作確認方法

### 1. トピック確認

起動後、別のターミナルで以下を実行：

```bash
# ZEDトピック確認
ros2 topic list | grep zed

# nvbloxトピック確認
ros2 topic list | grep nvblox

# カラー画像レート確認
ros2 topic hz /zed/zed_node/rgb/image_rect_color

# nvbloxメッシュレート確認
ros2 topic hz /nvblox_node/mesh
```

### 2. ノード確認

```bash
# 全ノード一覧
ros2 node list

# ZEDノード情報
ros2 node info /zed/zed_node

# nvbloxノード情報
ros2 node info /nvblox_node
```

### 3. TFフレーム確認

```bash
# TFツリー可視化（PDFファイル生成）
ros2 run tf2_tools view_frames

# TF変換確認
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom zed_camera_link
```

### 4. RViz2で可視化

```bash
# 自動でRViz2が起動している場合はそのまま使用
# 手動起動する場合
rviz2
```

追加する表示項目：
- **Image**: `/zed/zed_node/rgb/image_rect_color`
- **Image**: `/zed/zed_node/depth/depth_registered`
- **PointCloud2**: `/nvblox_node/static_esdf_pointcloud`
- **Mesh**: `/nvblox_node/mesh`
- **Map**: `/nvblox_node/map_slice`
- **TF**: すべてのフレーム表示
- **Odometry**: `/zed/zed_node/odom`

## トラブルシューティング

### 問題1: カメラが起動しない

**症状:**
```
[ERROR] [zed_node]: Failed to open camera
```

**対処法:**
```bash
# カメラ接続確認
lsusb | grep "ZED"

# カメラデバイス確認
ls /dev/video*

# ZED SDK診断
/usr/local/zed/tools/ZED_Diagnostic
```

### 問題2: nvbloxがデータを受信しない

**症状:**
```
[WARN] [nvblox_node]: No depth images received
```

**対処法:**
```bash
# トピック配信確認
ros2 topic hz /zed/zed_node/depth/depth_registered

# トピックリマッピング確認
ros2 node info /nvblox_node | grep Subscribers
```

### 問題3: TFエラーが発生

**症状:**
```
[WARN] [tf2]: Lookup would require extrapolation into the future
```

**対処法:**
- ZEDカメラのフレームレートを確認: `ros2 param get /zed/zed_node general.pub_frame_rate`
- TF配信レートを確認: `ros2 topic hz /tf`
- `use_sim_time`パラメータの確認（通常は`false`）

### 問題4: 処理が重い・フレームドロップする

**対処法:**

1. **解像度を下げる**
   `common_stereo_isaac.yaml`を編集：
   ```yaml
   general:
       pub_downscale_factor: 3.0  # 2.0から3.0に変更
   ```

2. **フレームレートを下げる**
   ```yaml
   general:
       pub_frame_rate: 15.0  # 30.0から15.0に変更
   ```

3. **可視化を無効化**
   ```bash
   ros2 launch bringup zed2i_isaac_nvblox.launch.py enable_visualization:=false
   ```

## パフォーマンスチューニング

### GPU使用の最適化

```yaml
# common_stereo_isaac.yaml
general:
    gpu_id: 0  # 特定のGPUを明示的に指定
```

### nvbloxパラメータ調整

nvbloxの設定ファイル（`config/nvblox/fuser.yaml`）で以下を調整：

```yaml
voxel_size: 0.1  # 0.05から0.1に（処理軽量化）
projective_integrator_max_integration_distance_m: 5.0  # 統合距離を制限
```

## 既存システムとの統合

### オプション1: 既存bringupに追加

`bringup_ps5_all_launch.py`に以下を追加：

```python
# nvblox統合launchをインクルード
nvblox_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(bringup_dir, 'launch', 'zed2i_isaac_nvblox.launch.py')
    ),
    launch_arguments={
        'enable_visualization': 'false',  # 既存ビューワーを使用
    }.items()
)
```

### オプション2: 個別起動

```bash
# ターミナル1: 既存システム
ros2 launch bringup bringup_ps5_all_launch.py

# ターミナル2: nvblox追加（ZEDカメラは既存のものを利用）
# ※この場合、ZEDカメラ部分を無効化した別バージョンが必要
```

## 関連ファイル

### 依存launchファイル
1. [zed2i_isaac_camera.launch.py](src/zed-ros2-wrapper/zed_wrapper/launch/zed2i_isaac_camera.launch.py)
2. [nvblox.launch.py](src/isaac_ros_nvblox/nvblox_examples/nvblox_examples_bringup/launch/perception/nvblox.launch.py)
3. [visualization.launch.py](src/isaac_ros_nvblox/nvblox_examples/nvblox_examples_bringup/launch/visualization/visualization.launch.py)

### 設定ファイル
1. [common_stereo_isaac.yaml](src/zed-ros2-wrapper/zed_wrapper/config/common_stereo_isaac.yaml) - Isaac ROS互換ZED設定
2. [fuser.yaml](src/isaac_ros_nvblox/nvblox_examples/nvblox_examples_bringup/config/nvblox/fuser.yaml) - nvbloxパラメータ

### テストスクリプト
- [test_zed2i_isaac_nvblox_bringup.sh](/home/jetros/ros2_ws/test_zed2i_isaac_nvblox_bringup.sh)

## 技術情報

### 使用パッケージ
- **zed-ros2-wrapper**: v4.x (ZEDカメラドライバ)
- **isaac_ros_nvblox**: latest (3Dマッピング)
- **ROS 2 Humble**

### コンポーネント方式の利点
- **低レイテンシ**: プロセス間通信を排除
- **高効率**: 共有メモリ経由でのデータ転送
- **リソース削減**: 単一プロセスで複数ノード実行

### nvbloxについて

**nvblox**は、NVIDIAが開発したリアルタイム3Dマッピングライブラリです：
- **TSDF（Truncated Signed Distance Function）**: ボリューム統合による高精度3D再構成
- **ESDF（Euclidean Signed Distance Field）**: ナビゲーション用距離場生成
- **GPU高速化**: CUDA最適化による高速処理

## まとめ

`zed2i_isaac_nvblox.launch.py`は、以下を実現します：

✅ **ワンコマンド起動**: ZED2i + nvbloxを統合起動
✅ **Isaac ROS互換**: 最適化された設定とトピック構造
✅ **高効率実行**: コンポーネントコンテナによる低レイテンシ
✅ **柔軟な設定**: 多数のlaunch引数でカスタマイズ可能
✅ **既存システム統合**: bringupパッケージで一元管理

このlaunchファイルにより、ZED2iカメラのVisual Odometryとnvbloxの3Dマッピングを組み合わせた、高精度なロボットナビゲーション・マッピングシステムを簡単に構築できます。
