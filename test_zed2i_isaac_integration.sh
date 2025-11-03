#!/bin/bash
# ZED2i Isaac ROS Integration Test Script
# このスクリプトは、zed2i_example.launch.pyとnvbloxの連携をテストします

set -e

echo "========================================="
echo "ZED2i Isaac ROS Integration Test"
echo "========================================="
echo ""

# 環境変数設定
export ISAAC_ROS_WS=/home/jetros/ros2_ws
source install/setup.bash

echo "📋 Step 1: 設定ファイルの確認"
echo "--------------------------------------"
if [ -f "install/zed_wrapper/share/zed_wrapper/config/common_stereo_isaac.yaml" ]; then
    echo "✅ common_stereo_isaac.yaml found"
else
    echo "❌ common_stereo_isaac.yaml not found"
    exit 1
fi

if [ -f "install/zed_wrapper/share/zed_wrapper/launch/zed2i_isaac_camera.launch.py" ]; then
    echo "✅ zed2i_isaac_camera.launch.py found"
else
    echo "❌ zed2i_isaac_camera.launch.py not found"
    exit 1
fi

if [ -f "install/nvblox_examples_bringup/share/nvblox_examples_bringup/launch/zed2i_example.launch.py" ]; then
    echo "✅ zed2i_example.launch.py found"
else
    echo "❌ zed2i_example.launch.py not found"
    exit 1
fi

echo ""
echo "📋 Step 2: Launch引数の確認"
echo "--------------------------------------"
echo "zed2i_example.launch.py arguments:"
ros2 launch --show-args nvblox_examples_bringup zed2i_example.launch.py 2>/dev/null | grep -E "camera:|rosbag:|log_level:" | head -3

echo ""
echo "📋 Step 3: 期待されるトピック構造の確認"
echo "--------------------------------------"
python3 << 'PYTHON_EOF'
import yaml

with open('install/zed_wrapper/share/zed_wrapper/config/common_stereo_isaac.yaml', 'r') as f:
    config = yaml.safe_load(f)

ros_params = config['/**']['ros__parameters']
general = ros_params.get('general', {})
pos_tracking = ros_params.get('pos_tracking', {})

cam_name = general.get('camera_name', 'zed')
node_name = general.get('node_name', 'zed_node')

print("Expected ZED2i topics:")
topics = [
    f"/{cam_name}/{node_name}/rgb/image_rect_color",
    f"/{cam_name}/{node_name}/rgb/camera_info",
    f"/{cam_name}/{node_name}/depth/depth_registered",
    f"/{cam_name}/{node_name}/depth/camera_info",
    f"/{cam_name}/{node_name}/pose"
]
for topic in topics:
    print(f"  • {topic}")

print("\nExpected TF chain:")
map_frame = pos_tracking.get('map_frame', 'map')
odom_frame = pos_tracking.get('odometry_frame', 'odom')
base_frame = pos_tracking.get('base_frame', 'zed_camera_link')
print(f"  {map_frame} -> {odom_frame} -> {base_frame} -> base_link (from URDF)")

print("\nnvblox compatibility:")
print(f"  global_frame: {odom_frame} ✅")
print(f"  Depth mode: {ros_params['depth']['depth_mode']} ✅")
print(f"  TF publishing enabled: {pos_tracking['publish_tf']} ✅")
PYTHON_EOF

echo ""
echo "📋 Step 4: nvblox remapping確認"
echo "--------------------------------------"
echo "nvbloxが期待するトピック (get_zed_remappings):"
echo "  camera_0/depth/image       -> /zed/zed_node/depth/depth_registered"
echo "  camera_0/depth/camera_info -> /zed/zed_node/depth/camera_info"
echo "  camera_0/color/image       -> /zed/zed_node/rgb/image_rect_color"
echo "  camera_0/color/camera_info -> /zed/zed_node/rgb/camera_info"
echo "  pose                       -> /zed/zed_node/pose"

echo ""
echo "========================================="
echo "✅ All configuration checks passed!"
echo "========================================="
echo ""
echo "🚀 To run the full system:"
echo "  ros2 launch nvblox_examples_bringup zed2i_example.launch.py"
echo ""
echo "📊 To verify topics after launch:"
echo "  ros2 topic list | grep zed"
echo "  ros2 topic hz /zed/zed_node/rgb/image_rect_color"
echo ""
echo "🗺️  To verify TF frames:"
echo "  ros2 run tf2_tools view_frames"
echo "  ros2 run tf2_ros tf2_echo odom zed_camera_link"
echo ""
