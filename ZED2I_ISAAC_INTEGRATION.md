# ZED2i Isaac ROS nvblox 連携設定ドキュメント

## 概要

このドキュメントは、ZED2iカメラとNVIDIA Isaac ROS nvbloxパッケージを連携させるために作成したファイルと設定についてまとめたものです。

## 作成したファイル

### 1. zed2i_example.launch.py
**場所:** `src/isaac_ros_nvblox/nvblox_examples/nvblox_examples_bringup/launch/zed2i_example.launch.py`

**目的:** ZED2iカメラとnvbloxを統合したサンプルlaunchファイル

**主な機能:**
- ZED2iカメラドライバの起動（Isaac ROS互換設定）
- nvblox 3Dマッピングノードの起動
- 可視化ツールの起動
- ROSbag再生対応

**起動方法:**
```bash
ros2 launch nvblox_examples_bringup zed2i_example.launch.py
```

**引数:**
- `camera`: カメラタイプ（デフォルト: `zed2i`、固定）
- `rosbag`: ROSbagファイルのパス（省略時はライブカメラ使用）
- `rosbag_args`: ROSbag再生の追加引数
- `log_level`: ログレベル（`debug`, `info`, `warn`）

---

### 2. zed2i_isaac_camera.launch.py
**場所:** `src/zed-ros2-wrapper/zed_wrapper/launch/zed2i_isaac_camera.launch.py`

**目的:** ZED2iカメラをIsaac ROS nvblox互換設定で起動するlaunchファイル

**主な特徴:**
- `common_stereo_isaac.yaml`を使用した設定
- nvbloxが期待するトピック名とTFフレーム構造を提供
- コンポーネントコンテナでの実行をサポート

**起動方法:**
```bash
ros2 launch zed_wrapper zed2i_isaac_camera.launch.py
```

**主要引数:**
- `camera_model`: カメラモデル（デフォルト: `zed2i`）
- `camera_name`: カメラ名（デフォルト: `zed`）
- `container_name`: 使用するコンテナ名（空の場合は新規作成）
- `publish_tf`: TF配信の有効化（デフォルト: `true`）
- `publish_map_tf`: map->odom TFの配信（デフォルト: `true`）

---

### 3. common_stereo_isaac.yaml
**場所:** `src/zed-ros2-wrapper/zed_wrapper/config/common_stereo_isaac.yaml`

**目的:** Isaac ROS nvblox連携に最適化されたZEDカメラパラメータ設定

#### 重要な設定項目

##### 一般設定
```yaml
general:
    camera_name: 'zed'
    node_name: 'zed_node'
    pub_resolution: 'CUSTOM'
    pub_downscale_factor: 2.0  # 帯域削減
    pub_frame_rate: 30.0
```

##### TFフレーム設定（nvblox互換）
```yaml
pos_tracking:
    pos_tracking_enabled: true
    pos_tracking_mode: 'GEN_3'
    imu_fusion: true
    publish_tf: true
    publish_map_tf: false  # nvbloxがマッピングを担当

    # フレーム名
    map_frame: 'map'
    odometry_frame: 'odom'  # nvblox global_frameと一致
    base_frame: 'zed_camera_link'
```

##### 深度設定（高品質）
```yaml
depth:
    depth_mode: 'NEURAL_PLUS'  # 最高品質の深度推定
    depth_stabilization: 50
    openni_depth_mode: false  # 32bit float形式
    point_cloud_freq: 15.0
    depth_confidence: 90
    depth_texture_conf: 80
```

##### 無効化された機能
```yaml
mapping:
    mapping_enabled: false  # nvbloxがマッピングを担当

object_detection:
    od_enabled: false  # Isaac ROS統合のため無効化

body_tracking:
    bt_enabled: false  # Isaac ROS統合のため無効化
```

---

## トピック構造とTFフレーム

### パブリッシュされるトピック

nvbloxが期待するトピック構造：

| nvblox入力 | ZED2i出力トピック |
|-----------|------------------|
| `camera_0/depth/image` | `/zed/zed_node/depth/depth_registered` |
| `camera_0/depth/camera_info` | `/zed/zed_node/depth/camera_info` |
| `camera_0/color/image` | `/zed/zed_node/rgb/image_rect_color` |
| `camera_0/color/camera_info` | `/zed/zed_node/rgb/camera_info` |
| `pose` | `/zed/zed_node/pose` |

### TFフレーム構造

```
map
 └─ odom (ZED Visual Odometry)
     └─ zed_camera_link (カメラ基準フレーム)
         └─ base_link (URDF定義)
             ├─ zed_camera_center
             ├─ zed_left_camera_frame
             └─ zed_right_camera_frame
```

**重要なポイント:**
- nvbloxの`global_frame`は`odom`に設定
- ZED Visual Odometryが`odom -> zed_camera_link`のTFを配信
- `map -> odom`のTFはnvbloxが配信（publish_map_tf: false）

---

## 連携の仕組み

### 1. データフロー

```
ZED2iカメラ
    ↓
[zed2i_isaac_camera.launch.py]
    ↓ (トピック配信)
    • /zed/zed_node/rgb/image_rect_color
    • /zed/zed_node/depth/depth_registered
    • /zed/zed_node/pose
    ↓
[nvblox remapping]
    ↓
camera_0/* トピック
    ↓
[nvblox 3D mapping]
    ↓
    • /nvblox_node/mesh (3Dメッシュ)
    • /nvblox_node/map_slice (2Dマップスライス)
    • /nvblox_node/static_esdf_pointcloud (距離場)
```

### 2. TF変換フロー

```
ZED Visual Odometry
    ↓
TF: odom -> zed_camera_link
    ↓
URDF: zed_camera_link -> base_link
    ↓
nvblox (マッピング結果)
    ↓
TF: map -> odom
```

---

## 検証方法

### 1. 設定ファイルとlaunchファイルの確認

自動テストスクリプトを実行：
```bash
cd /home/jetros/ros2_ws
./test_zed2i_isaac_integration.sh
```

### 2. システム全体の起動

```bash
ros2 launch nvblox_examples_bringup zed2i_example.launch.py
```

### 3. トピック確認

```bash
# ZEDトピック一覧
ros2 topic list | grep zed

# カラー画像のレート確認
ros2 topic hz /zed/zed_node/rgb/image_rect_color

# 深度画像のレート確認
ros2 topic hz /zed/zed_node/depth/depth_registered

# nvbloxメッシュ確認
ros2 topic hz /nvblox_node/mesh
```

### 4. TFフレーム確認

```bash
# TFツリー可視化
ros2 run tf2_tools view_frames

# 特定のTF変換確認
ros2 run tf2_ros tf2_echo odom zed_camera_link

# TFレート確認
ros2 topic hz /tf
```

### 5. RViz2での可視化

```bash
rviz2
```

追加する表示項目：
- **Image**: `/zed/zed_node/rgb/image_rect_color`
- **Image**: `/zed/zed_node/depth/depth_registered`
- **PointCloud2**: `/nvblox_node/static_esdf_pointcloud`
- **Mesh**: `/nvblox_node/mesh`
- **TF**: すべてのフレーム
- **Map**: `/nvblox_node/map_slice`

---

## トラブルシューティング

### 問題1: トピックが配信されない

**確認事項:**
```bash
# カメラが認識されているか
ls /dev/video*

# ZEDノードが起動しているか
ros2 node list | grep zed
```

**対処法:**
- カメラのUSB接続を確認
- カメラのシリアル番号が正しいか確認
- ZED SDK がインストールされているか確認

### 問題2: TFエラーが発生する

**症状:**
```
Lookup would require extrapolation into the future
```

**対処法:**
- `use_sim_time`パラメータの確認
- カメラのフレームレートが安定しているか確認
- TF配信レートを確認: `ros2 topic hz /tf`

### 問題3: nvbloxがデータを受信しない

**確認事項:**
```bash
# nvbloxノードの状態
ros2 node info /nvblox_node

# トピックのリマッピングを確認
ros2 topic info /camera_0/depth/image -v
```

**対処法:**
- トピック名のリマッピングが正しいか確認
- QoS設定の互換性を確認
- nvbloxのログを確認: `ros2 run rqt_console rqt_console`

### 問題4: 深度品質が低い

**対処法:**
`common_stereo_isaac.yaml`を調整：
```yaml
depth:
    depth_mode: 'NEURAL_PLUS'  # または 'ULTRA'
    depth_confidence: 95  # 90から上げる
    depth_texture_conf: 90  # 80から上げる
```

---

## パフォーマンスチューニング

### 1. 帯域削減

画像解像度を下げる：
```yaml
general:
    pub_downscale_factor: 3.0  # 2.0から3.0に変更
```

### 2. フレームレート調整

処理負荷を下げる：
```yaml
general:
    pub_frame_rate: 15.0  # 30.0から15.0に変更

depth:
    point_cloud_freq: 10.0  # 15.0から10.0に変更
```

### 3. GPU最適化

特定のGPUを使用：
```yaml
general:
    gpu_id: 0  # -1（自動）から明示的に指定
```

---

## 既存システムとの統合

現在のbringupシステム（`bringup_ps5_all_launch.py`）と統合する場合：

### オプション1: 並行実行

既存のZED launchとnvbloxを別々に起動：
```bash
# ターミナル1: 既存システム
ros2 launch bringup bringup_ps5_all_launch.py

# ターミナル2: nvblox（既存ZEDトピックを利用）
ros2 launch nvblox_examples_bringup zed2i_example.launch.py
```

### オプション2: 統合launchファイル作成

`bringup_ps5_all_launch.py`にnvbloxを追加：
```python
# nvblox起動を追加
actions.append(
    lu.include(
        'nvblox_examples_bringup',
        'launch/perception/nvblox.launch.py',
        launch_arguments={
            'mode': 'static',
            'camera': 'zed2',
        }
    )
)
```

---

## 参考情報

### 関連ドキュメント
- [ZED ROS2 Wrapper Documentation](https://www.stereolabs.com/docs/ros2/)
- [Isaac ROS nvblox](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/index.html)
- [ROS 2 TF2 Documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)

### 使用しているパッケージ
- `zed-ros2-wrapper`: v4.x
- `isaac_ros_nvblox`: latest
- ROS 2 Humble

### カスタム設定ファイル一覧
1. `common_stereo_isaac.yaml` - Isaac ROS互換パラメータ
2. `zed2i_isaac_camera.launch.py` - Isaac ROS互換ZEDドライバ
3. `zed2i_example.launch.py` - 統合サンプルlaunch

---

## まとめ

この設定により、ZED2iカメラとIsaac ROS nvbloxが以下の点で連携可能になりました：

✅ **トピック互換性**: nvbloxが期待する`/zed/zed_node/rgb/*`と`/zed/zed_node/depth/*`トピックを配信

✅ **TFフレーム整合性**: nvbloxの`global_frame: odom`に対応したTF構造

✅ **高品質深度**: `NEURAL_PLUS`モードによる高精度深度推定

✅ **パフォーマンス最適化**: ダウンスケーリングとフレームレート調整による負荷削減

✅ **既存システムとの共存**: 既存のbringupシステムと並行実行可能

この設定を使用することで、ZED2iカメラのVisual Odometryとnvbloxの3Dマッピング機能を組み合わせた、高精度なロボットナビゲーションシステムを構築できます。
