#!/bin/bash
# nvblox + Nav2 統合テストスクリプト（/mapトピックとmap frame確認）

echo "=========================================="
echo "nvblox + Nav2 統合テスト"
echo "/map トピックと map フレーム確認"
echo "=========================================="
echo ""

# ワークスペースのソース
source /home/jetros/ros2_ws/install/setup.bash

echo "1. /map トピックの確認..."
echo ""
echo "  [nvblox → /map] トピック存在確認"
timeout 3 ros2 topic info /map 2>/dev/null && echo "    ✅ /map トピック存在" || echo "    ❌ /map トピック不在"

echo ""
echo "  [nvblox → /map] メッセージタイプ確認"
MAP_TYPE=$(timeout 3 ros2 topic type /map 2>/dev/null)
if [ "$MAP_TYPE" == "nav_msgs/msg/OccupancyGrid" ]; then
    echo "    ✅ 正しいタイプ: $MAP_TYPE"
else
    echo "    ❌ 誤ったタイプまたは不在: $MAP_TYPE"
fi

echo ""
echo "  [nvblox → /map] Publisher確認"
timeout 3 ros2 topic info /map -v 2>/dev/null | grep -A5 "Publisher count"

echo ""
echo "  [nvblox → /map] データ受信テスト（5秒）"
timeout 6 ros2 topic hz /map --window 5 2>/dev/null | grep "average rate" && echo "    ✅ データ受信成功" || echo "    ⚠️ データなし（nvbloxがまだマップ生成中の可能性）"

echo ""
echo "=========================================="
echo "2. TFフレーム確認"
echo "=========================================="
echo ""

echo "  [TF] map frame存在確認"
timeout 5 ros2 run tf2_ros tf2_echo map zed_odom 2>&1 | head -5
if [ $? -eq 0 ]; then
    echo "    ✅ map -> zed_odom TF変換可能"
else
    echo "    ❌ map -> zed_odom TF変換不可"
fi

echo ""
echo "  [TF] map -> zed_camera_link (full chain)"
timeout 5 ros2 run tf2_ros tf2_echo map zed_camera_link 2>&1 | head -5
if [ $? -eq 0 ]; then
    echo "    ✅ map -> zed_camera_link TF変換可能"
else
    echo "    ❌ map -> zed_camera_link TF変換不可"
fi

echo ""
echo "=========================================="
echo "3. Nav2コストマップ確認"
echo "=========================================="
echo ""

echo "  [Global Costmap] /mapを購読しているか"
timeout 3 ros2 topic info /map 2>/dev/null | grep -i "subscriber" | grep -i "global_costmap" && echo "    ✅ Global Costmapが/mapを購読" || echo "    ❌ Global Costmapが/mapを購読していない"

echo ""
echo "  [Global Costmap] トピック出力確認"
timeout 3 ros2 topic info /global_costmap/costmap 2>/dev/null && echo "    ✅ Global Costmap出力中" || echo "    ❌ Global Costmap未出力"

echo ""
echo "  [Global Costmap] 更新レート（5秒）"
timeout 6 ros2 topic hz /global_costmap/costmap --window 5 2>/dev/null | grep "average rate" || echo "    ⚠️ データなし"

echo ""
echo "=========================================="
echo "4. Nav2ノード起動確認"
echo "=========================================="
echo ""

nav2_nodes=("controller_server" "planner_server" "behavior_server" "bt_navigator" "smoother_server" "velocity_smoother" "waypoint_follower")

for node in "${nav2_nodes[@]}"; do
    echo "  [$node]"
    timeout 3 ros2 node info "/$node" >/dev/null 2>&1 && echo "    ✅ 起動中" || echo "    ❌ 未起動"
done

echo ""
echo "=========================================="
echo "5. 統合診断"
echo "=========================================="
echo ""

# 必須要件チェック
ERRORS=0

# /map トピック
timeout 3 ros2 topic info /map >/dev/null 2>&1 || ((ERRORS++))

# map TF
timeout 5 ros2 run tf2_ros tf2_echo map zed_camera_link >/dev/null 2>&1 || ((ERRORS++))

# Global Costmap
timeout 3 ros2 topic info /global_costmap/costmap >/dev/null 2>&1 || ((ERRORS++))

if [ $ERRORS -eq 0 ]; then
    echo "  ✅ すべての必須要件を満たしています"
    echo "  ✅ 自動航行が可能な状態です"
else
    echo "  ❌ $ERRORS 個の問題があります"
    echo "  ⚠️ 上記のエラーを確認してください"
fi

echo ""
echo "=========================================="
echo "次のステップ"
echo "=========================================="
echo ""

if [ $ERRORS -eq 0 ]; then
    echo "1. RViz2でゴールを設定してテスト:"
    echo "   rviz2"
    echo ""
    echo "2. コマンドラインからゴールを設定:"
    echo "   ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \\"
    echo "     '{header: {frame_id: \"map\"}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'"
    echo ""
    echo "3. /mapの内容を可視化:"
    echo "   ros2 run nav2_map_server map_saver_cli -f /tmp/test_map"
else
    echo "問題がある場合:"
    echo "1. nvbloxが起動しているか確認:"
    echo "   ros2 node list | grep nvblox"
    echo ""
    echo "2. ZEDカメラが動作しているか確認:"
    echo "   ros2 topic hz /zed/zed_node/pose"
    echo ""
    echo "3. ログを確認:"
    echo "   ros2 node info /nvblox_node"
fi

echo ""
echo "=========================================="
