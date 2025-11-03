---
description: ROS2パッケージを効率的にビルド
---

ROS2パッケージをビルドしてください。以下の手順で実行してください：

## ビルド手順

### 1. 環境変数の確認
```bash
echo $ISAAC_ROS_WS  # 出力: /home/jetros/ros2_ws
```

### 2. ビルド対象の決定
ユーザーが指定したパッケージ、または以下のデフォルト動作：
- パッケージ名が指定されている場合: そのパッケージのみビルド
- 指定がない場合: 未完了パッケージのみビルド

### 3. ビルド実行

#### パッケージが指定されている場合
```bash
cd /home/jetros/ros2_ws
colcon build --packages-select <package_name> --symlink-install
```

#### 未完了パッケージのみビルド（デフォルト）
```bash
cd /home/jetros/ros2_ws
colcon build --packages-skip-build-finished --symlink-install \
  --packages-skip isaac_ros_ess_models_install \
                  isaac_ros_peoplenet_models_install \
                  isaac_ros_peoplesemseg_models_install \
                  isaac_ros_nova_recorder
```

#### 特定パッケージとその依存関係をビルド
```bash
colcon build --packages-up-to <package_name> --symlink-install
```

### 4. ビルド結果の報告
ビルド完了後、以下を報告：
- ビルド成功したパッケージ数
- ビルドエラーの有無
- エラーがある場合は原因と解決策

## エラー処理

### よくあるエラーと対処法

#### シンボリックリンクエラー
```bash
rm -rf build/<package_name> install/<package_name>
colcon build --packages-select <package_name> --symlink-install
```

#### メモリ不足（Jetson環境）
```bash
colcon build --symlink-install --parallel-workers 2
```

#### サブモジュール未初期化
```bash
cd src/<package_directory>
git submodule update --init --recursive
```

## 報告フォーマット

以下の形式で報告してください：

```
## ビルド結果

### ✅ 成功
- ビルドパッケージ数: XX個
- ビルド時間: XX分XX秒

### ❌ エラー（ある場合）
- パッケージ名: <package_name>
- エラー内容: <error_message>
- 推奨対処法: <solution>

### 📋 次のステップ
- （必要に応じて追加のビルドコマンドを提案）
```

詳細なトラブルシューティングは `.claude/docs/build-system.md` を参照してください。
