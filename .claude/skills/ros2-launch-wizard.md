# ROS2 Launch ウィザード

複雑なlaunchファイル作成・編集・デバッグを効率化するスキル。

## 実行内容

### 1. Launch構造分析

既存のlaunchファイルを解析し、構造を可視化：

```
bringup_ps5_all_launch.py (メインLaunch)
├─ bringup_ps5_jetson_launch.py
│  ├─ bringup_ps5_devices.launch.py
│  │  ├─ back_stl27l.launch.py (後方LiDAR)
│  │  ├─ stl27l.launch.py (前方LiDAR)
│  │  ├─ laser_merger.launch.py (スキャンマージ)
│  │  ├─ box_filter_example.launch.py (フィルタリング)
│  │  ├─ micro_ros_agent (マイコン通信)
│  │  ├─ joy_linux (PS5コントローラー)
│  │  ├─ joy_mecanum_controller (メカナム制御)
│  │  └─ static_transform_publishers (TF配信)
│  └─ zed_camera.launch.py (ZED2iカメラ)
├─ online_async_launch.py (SLAM Toolbox)
├─ my_navigation_launch.py (Nav2スタック)
├─ zed_goal_publisher (ゴール配信)
└─ tf_sync_coordinator (TF同期)

起動ノード数: 53
トピック数: 198
```

### 2. Launch自動生成

テンプレートからカスタムlaunchファイルを生成：

#### 基本ノード起動Launch
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='<package_name>',
            executable='<executable_name>',
            name='<node_name>',
            parameters=[{'param_name': 'param_value'}],
            remappings=[
                ('/old_topic', '/new_topic')
            ],
            output='screen'
        )
    ])
```

#### 複数ノード + パラメータファイル
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('<package_name>')
    config_file = PathJoinSubstitution([pkg_share, 'config', 'params.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='<package_name>',
            executable='<executable_name>',
            parameters=[config_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        )
    ])
```

#### 階層的Launch（IncludeLaunchDescription）
```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('<package_name>'),
                '/launch/<launch_file>.py'
            ]),
            launch_arguments={
                'arg_name': 'arg_value'
            }.items()
        )
    ])
```

### 3. Launch依存関係チェック

起動に必要なパッケージ・ファイルの存在確認：

```bash
# パッケージ存在確認
ros2 pkg list | grep <package_name>

# 実行ファイル確認
ros2 pkg executables <package_name>

# パラメータファイル確認
find /home/jetros/ros2_ws -name "*.yaml" | grep <config_file>

# 依存launchファイル確認
find /home/jetros/ros2_ws -name "*.launch.py" | grep <launch_file>
```

### 4. Launch実行テスト

段階的な起動テストを実施：

```bash
# 1. 構文チェック
python3 <launch_file>.py

# 2. ドライラン（実行せずに解析）
ros2 launch --show-args <package> <launch_file>

# 3. 短時間起動テスト
timeout 10 ros2 launch <package> <launch_file>

# 4. ノード起動確認
ros2 node list | grep <expected_node>

# 5. トピック配信確認
ros2 topic list | grep <expected_topic>
ros2 topic hz <topic> --window 10
```

### 5. Launchデバッグ

よくある問題の自動診断：

#### ノードが起動しない
- **チェック項目**:
  - パッケージ名のタイプミス
  - 実行ファイル名の間違い
  - パラメータファイルパスの誤り
  - 依存パッケージ未ビルド

#### トピックが配信されない
- **チェック項目**:
  - リマッピング設定の誤り
  - QoS互換性の問題
  - ノード起動失敗

#### パラメータが反映されない
- **チェック項目**:
  - YAMLファイルの構文エラー
  - パラメータ名の名前空間ミス
  - use_sim_timeの不一致

### 6. Launch最適化提案

起動時間短縮とリソース効率化：

#### 起動順序の最適化
依存関係を解析し、並列起動可能なノードを特定

#### 遅延起動設定
```python
from launch.actions import TimerAction

TimerAction(
    period=5.0,
    actions=[Node(...)]  # 5秒後に起動
)
```

#### 条件付き起動
```python
from launch.conditions import IfCondition

Node(
    ...,
    condition=IfCondition(LaunchConfiguration('use_camera'))
)
```

## 出力形式

```
🚀 Launch分析開始: bringup_ps5_all_launch.py

📊 Launch構造:
  - 階層レベル: 3
  - 起動ノード数: 53
  - 依存launchファイル: 8個
  - 使用パラメータファイル: 12個

✅ 依存関係チェック完了
  - 全パッケージ存在確認済み
  - 全パラメータファイル確認済み

⚠️  最適化提案:
  1. slam_toolboxとnavigation2を並列起動可能 (起動時間-3秒)
  2. static_transform_publisherを1つに統合可能 (CPU使用率-5%)

🔧 デバッグモード起動中...
✅ 全ノード正常起動 (起動時間: 12.3秒)
```

## 使用タイミング

- 新規launchファイル作成時
- 複雑なlaunch構成の理解
- 起動エラーのデバッグ
- Launch最適化

## トークン削減効果

**従来**: Launch構文確認 → サンプル検索 → 手動作成 → デバッグ → 修正 (約6000トークン)
**Skills使用後**: テンプレート生成 → 自動チェック → デバッグ支援 (約900トークン)
**削減率**: 約85%

## 高度な機能

### Launch可視化
Graphvizを使用してlaunch構造を図示

### パフォーマンスプロファイリング
各ノードの起動時間とリソース使用量を測定

### Launchテンプレートライブラリ
よく使うパターンをライブラリ化して再利用
