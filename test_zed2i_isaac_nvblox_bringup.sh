#!/bin/bash
# ZED2i Isaac ROS nvblox Bringup Launch Test Script
# このスクリプトは、bringup/zed2i_isaac_nvblox.launch.pyの動作確認を行います

set -e

echo "========================================="
echo "ZED2i Isaac nvblox Bringup Launch Test"
echo "========================================="
echo ""

# 環境変数設定
export ISAAC_ROS_WS=/home/jetros/ros2_ws
source install/setup.bash

echo "📋 Step 1: launchファイルの存在確認"
echo "--------------------------------------"
if [ -f "install/bringup/share/bringup/launch/zed2i_isaac_nvblox.launch.py" ]; then
    echo "✅ zed2i_isaac_nvblox.launch.py found"
else
    echo "❌ zed2i_isaac_nvblox.launch.py not found"
    exit 1
fi

echo ""
echo "📋 Step 2: 依存launchファイルの確認"
echo "--------------------------------------"
dependencies=(
    "install/zed_wrapper/share/zed_wrapper/launch/zed2i_isaac_camera.launch.py"
    "install/nvblox_examples_bringup/share/nvblox_examples_bringup/launch/perception/nvblox.launch.py"
    "install/nvblox_examples_bringup/share/nvblox_examples_bringup/launch/visualization/visualization.launch.py"
)

for dep in "${dependencies[@]}"; do
    if [ -f "$dep" ]; then
        echo "✅ $(basename $dep)"
    else
        echo "❌ $(basename $dep) not found"
        exit 1
    fi
done

echo ""
echo "📋 Step 3: launch引数の確認"
echo "--------------------------------------"
echo "Available arguments:"
ros2 launch --show-args bringup zed2i_isaac_nvblox.launch.py 2>/dev/null | grep -E "^\s+'[a-z_]+'" | head -5

echo ""
echo "📋 Step 4: launchファイルの構造確認"
echo "--------------------------------------"
python3 << 'PYTHON_EOF'
import sys
sys.path.insert(0, 'src/bringup/launch')

try:
    # Import check
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
    print("✅ All required imports available")

    # Check file syntax
    with open('src/bringup/launch/zed2i_isaac_nvblox.launch.py', 'r') as f:
        content = f.read()

    # Check key components
    checks = {
        'generate_launch_description': 'generate_launch_description' in content,
        'ZED2i camera launch': 'zed2i_isaac_camera.launch.py' in content,
        'nvblox launch': 'nvblox.launch.py' in content,
        'visualization launch': 'visualization.launch.py' in content,
        'container': 'component_container_mt' in content or 'ComposableNodeContainer' in content,
    }

    print("\nFile structure checks:")
    for check, passed in checks.items():
        status = "✅" if passed else "❌"
        print(f"  {status} {check}")

    if not all(checks.values()):
        sys.exit(1)

except Exception as e:
    print(f"❌ Error: {e}")
    sys.exit(1)
PYTHON_EOF

echo ""
echo "📋 Step 5: 統合構成の確認"
echo "--------------------------------------"
echo "Launch構成:"
echo "  1. Component Container (nvblox_container)"
echo "  2. ZED2i Camera (via zed2i_isaac_camera.launch.py)"
echo "  3. nvblox Mapping (via nvblox.launch.py)"
echo "  4. Visualization (via visualization.launch.py, conditional)"
echo ""
echo "期待されるトピックフロー:"
echo "  ZED2i → /zed/zed_node/rgb/* → nvblox → /nvblox_node/*"
echo ""
echo "TFフレーム:"
echo "  map → odom → zed_camera_link → base_link"

echo ""
echo "========================================="
echo "✅ All checks passed!"
echo "========================================="
echo ""
echo "🚀 使用方法:"
echo ""
echo "1. 基本起動:"
echo "   ros2 launch bringup zed2i_isaac_nvblox.launch.py"
echo ""
echo "2. 可視化なしで起動:"
echo "   ros2 launch bringup zed2i_isaac_nvblox.launch.py enable_visualization:=false"
echo ""
echo "3. カスタムコンテナ名で起動:"
echo "   ros2 launch bringup zed2i_isaac_nvblox.launch.py container_name:=my_container"
echo ""
echo "4. デバッグモードで起動:"
echo "   ros2 launch bringup zed2i_isaac_nvblox.launch.py log_level:=debug"
echo ""
echo "📊 起動後の確認コマンド:"
echo ""
echo "  # トピック確認"
echo "  ros2 topic list | grep -E '(zed|nvblox)'"
echo ""
echo "  # ノード確認"
echo "  ros2 node list"
echo ""
echo "  # TF確認"
echo "  ros2 run tf2_tools view_frames"
echo ""
echo "  # nvbloxメッシュ確認"
echo "  ros2 topic hz /nvblox_node/mesh"
echo ""
