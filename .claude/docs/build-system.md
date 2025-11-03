# ビルドシステム設定

## 環境変数設定（必須）

**✅ 完了済み:** `ISAAC_ROS_WS`は既に`~/.bashrc`で設定済みです。

```bash
# 確認方法
echo $ISAAC_ROS_WS  # 出力: /home/jetros/ros2_ws
```

## 推奨ビルドコマンド

### 通常のビルド（未完了パッケージのみ）
```bash
cd /home/jetros/ros2_ws
colcon build --packages-skip-build-finished --symlink-install \
  --packages-skip isaac_ros_ess_models_install \
                  isaac_ros_peoplenet_models_install \
                  isaac_ros_peoplesemseg_models_install \
                  isaac_ros_nova_recorder
```

### クリーンビルド（全パッケージ）
```bash
colcon build --symlink-install \
  --packages-skip isaac_ros_ess_models_install \
                  isaac_ros_peoplenet_models_install \
                  isaac_ros_peoplesemseg_models_install \
                  isaac_ros_nova_recorder
```

### 特定パッケージのビルド
```bash
# 単一パッケージ
colcon build --packages-select <package_name> --symlink-install

# パッケージと依存関係
colcon build --packages-up-to <package_name> --symlink-install
```

## よくあるビルドエラー

### 1. シンボリックリンクエラー
```bash
# 解決方法: ビルドキャッシュをクリア
rm -rf build/<package_name> install/<package_name>
colcon build --packages-select <package_name> --symlink-install
```

### 2. nvblox_ros - サブモジュール未初期化
```bash
# 解決方法
cd /home/jetros/ros2_ws/src/isaac_ros_nvblox
git submodule update --init --recursive
colcon build --packages-select nvblox_ros --symlink-install --cmake-args -DBUILD_TESTING=OFF
```

### 3. 依存パッケージ不足
```bash
# 例: camera_info_manager
sudo apt update
sudo apt install -y ros-humble-camera-info-manager
```

## メモリ不足対策（Jetson環境）

```bash
# 並列ビルド数を制限
colcon build --symlink-install --parallel-workers 2

# 大規模パッケージは個別にビルド
colcon build --packages-select nvblox_ros --symlink-install
colcon build --packages-skip-build-finished --symlink-install
```

## ビルド後の確認

```bash
# インストール済みパッケージ数を確認
ls install/ | wc -l

# ビルドログの確認
colcon build --packages-skip-build-finished --symlink-install 2>&1 | tail -20
```

## トラブルシューティングチェックリスト

1. **環境変数の確認**
   ```bash
   echo $ISAAC_ROS_WS
   echo $CUDA_HOME
   ```

2. **ビルドキャッシュのクリア**
   ```bash
   rm -rf build/<package_name> install/<package_name>
   ```

3. **ディスク容量の確認**
   ```bash
   df -h /home/jetros/ros2_ws
   ```

詳細なトラブルシューティングは元のCLAUDE.mdのビルドセクションを参照してください。
