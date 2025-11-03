#!/bin/bash
# nvblox + Nav2 統合テストスクリプト

echo "=========================================="
echo "nvblox + Nav2 統合テスト"
echo "=========================================="
echo ""

# ワークスペースのソース
source /home/jetros/ros2_ws/install/setup.bash

echo "1. 必須トピックの存在確認..."
sleep 2

# nvbloxトピック
echo "  [nvblox] /nvblox_node/esdf_pointcloud"
timeout 3 ros2 topic info /nvblox_node/esdf_pointcloud 2>/dev/null && echo "    ✅ 存在" || echo "    ❌ 不在"

echo "  [nvblox] /nvblox_node/mesh"
timeout 3 ros2 topic info /nvblox_node/mesh 2>/dev/null && echo "    ✅ 存在" || echo "    ❌ 不在"

echo "  [nvblox] /nvblox_node/map_slice"
timeout 3 ros2 topic info /nvblox_node/map_slice 2>/dev/null && echo "    ✅ 存在" || echo "    ❌ 不在"

# LiDARトピック
echo "  [LiDAR] /merged_scan_filtered"
timeout 3 ros2 topic info /merged_scan_filtered 2>/dev/null && echo "    ✅ 存在" || echo "    ❌ 不在"

# ZEDトピック
echo "  [ZED] /zed/zed_node/odom"
timeout 3 ros2 topic info /zed/zed_node/odom 2>/dev/null && echo "    ✅ 存在" || echo "    ❌ 不在"

# Nav2トピック
echo "  [Nav2] /global_costmap/costmap"
timeout 3 ros2 topic info /global_costmap/costmap 2>/dev/null && echo "    ✅ 存在" || echo "    ❌ 不在"

echo "  [Nav2] /local_costmap/costmap"
timeout 3 ros2 topic info /local_costmap/costmap 2>/dev/null && echo "    ✅ 存在" || echo "    ❌ 不在"

echo "  [Nav2] /cmd_vel"
timeout 3 ros2 topic info /cmd_vel 2>/dev/null && echo "    ✅ 存在" || echo "    ❌ 不在"

echo ""
echo "2. TF確認..."

# map -> zed_odomの確認
echo "  [TF] map -> zed_odom"
timeout 5 ros2 run tf2_ros tf2_echo map zed_odom 2>/dev/null | head -5 && echo "    ✅ 変換可能" || echo "    ❌ 変換不可"

# zed_odom -> zed_camera_linkの確認
echo "  [TF] zed_odom -> zed_camera_link"
timeout 5 ros2 run tf2_ros tf2_echo zed_odom zed_camera_link 2>/dev/null | head -5 && echo "    ✅ 変換可能" || echo "    ❌ 変換不可"

# map -> zed_camera_linkの確認（full chain）
echo "  [TF] map -> zed_camera_link (full chain)"
timeout 5 ros2 run tf2_ros tf2_echo map zed_camera_link 2>/dev/null | head -5 && echo "    ✅ 変換可能" || echo "    ❌ 変換不可"

echo ""
echo "3. Nav2ノード確認..."

nav2_nodes=("controller_server" "planner_server" "behavior_server" "bt_navigator" "smoother_server" "velocity_smoother" "waypoint_follower")

for node in "${nav2_nodes[@]}"; do
    echo "  [$node]"
    timeout 3 ros2 node info "/$node" >/dev/null 2>&1 && echo "    ✅ 起動中" || echo "    ❌ 未起動"
done

echo ""
echo "4. トピック更新レート確認（5秒間）..."

echo "  [nvblox ESDF]"
timeout 6 ros2 topic hz /nvblox_node/esdf_pointcloud --window 5 2>/dev/null | grep "average rate" || echo "    ⚠️ データなし"

echo "  [LiDAR]"
timeout 6 ros2 topic hz /merged_scan_filtered --window 5 2>/dev/null | grep "average rate" || echo "    ⚠️ データなし"

echo "  [Global Costmap]"
timeout 6 ros2 topic hz /global_costmap/costmap --window 5 2>/dev/null | grep "average rate" || echo "    ⚠️ データなし"

echo ""
echo "=========================================="
echo "テスト完了"
echo "=========================================="
echo ""
echo "次のステップ:"
echo "  1. すべて✅ならナビゲーションテストを実行"
echo "  2. RViz2でゴールを設定してテスト"
echo "  3. または、以下のコマンドでゴールをpublish:"
echo ""
echo "  ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \\"
echo "    '{header: {frame_id: \"map\"}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'"
echo ""
