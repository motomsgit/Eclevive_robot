#!/bin/bash
# TFツリー統合確認スクリプト（odom vs zed_odom問題対応）

echo "=========================================="
echo "TFツリー統合確認テスト"
echo "=========================================="
echo ""

# ワークスペースのソース
source /home/jetros/ros2_ws/install/setup.bash

echo "1. TFフレーム一覧"
echo "=========================================="
echo ""

# 全フレーム一覧を取得
timeout 3 ros2 run tf2_ros tf2_monitor 2>/dev/null | head -20

echo ""
echo "2. 期待されるTFツリー構造"
echo "=========================================="
echo ""
echo "正しいTFツリー:"
echo "  map"
echo "   └─ odom (ZED Visual Odometry)"
echo "       └─ zed_camera_link"
echo "           ├─ base_link"
echo "           ├─ front_lidar"
echo "           └─ back_lidar"
echo ""

echo "3. 主要TF変換の確認"
echo "=========================================="
echo ""

# map -> odom
echo "  [TF] map -> odom"
timeout 5 ros2 run tf2_ros tf2_echo map odom 2>&1 | head -10
if [ ${PIPESTATUS[0]} -eq 0 ]; then
    echo "    ✅ map -> odom 変換可能"
else
    echo "    ❌ map -> odom 変換不可"
fi

echo ""

# odom -> zed_camera_link
echo "  [TF] odom -> zed_camera_link"
timeout 5 ros2 run tf2_ros tf2_echo odom zed_camera_link 2>&1 | head -10
if [ ${PIPESTATUS[0]} -eq 0 ]; then
    echo "    ✅ odom -> zed_camera_link 変換可能"
else
    echo "    ❌ odom -> zed_camera_link 変換不可"
fi

echo ""

# map -> zed_camera_link (full chain)
echo "  [TF] map -> zed_camera_link (full chain)"
timeout 5 ros2 run tf2_ros tf2_echo map zed_camera_link 2>&1 | head -10
if [ ${PIPESTATUS[0]} -eq 0 ]; then
    echo "    ✅ map -> zed_camera_link 変換可能"
else
    echo "    ❌ map -> zed_camera_link 変換不可"
fi

echo ""

# map -> base_link
echo "  [TF] map -> base_link"
timeout 5 ros2 run tf2_ros tf2_echo map base_link 2>&1 | head -10
if [ ${PIPESTATUS[0]} -eq 0 ]; then
    echo "    ✅ map -> base_link 変換可能"
else
    echo "    ❌ map -> base_link 変換不可"
fi

echo ""
echo "4. 問題のあるフレームの検出"
echo "=========================================="
echo ""

# zed_odomフレームが存在しないことを確認
echo "  [CHECK] zed_odom フレームは存在しないはず"
timeout 3 ros2 run tf2_ros tf2_monitor 2>/dev/null | grep "zed_odom" && echo "    ⚠️ 警告: zed_odom フレームが存在します（odomに統一すべき）" || echo "    ✅ zed_odom は存在しません（正しい）"

echo ""

# odomフレームが存在することを確認
echo "  [CHECK] odom フレームが存在するはず"
timeout 3 ros2 run tf2_ros tf2_monitor 2>/dev/null | grep -E "^Frame: odom$|^odom$" && echo "    ✅ odom フレームが存在します" || echo "    ❌ odom フレームが存在しません"

echo ""
echo "5. Nav2との整合性確認"
echo "=========================================="
echo ""

# AMCLパラメータ確認（起動している場合）
echo "  [AMCL] odom_frame_id 設定確認"
AMCL_ODOM=$(timeout 3 ros2 param get /amcl odom_frame_id 2>/dev/null | grep -o "'.*'" | tr -d "'")
if [ -n "$AMCL_ODOM" ]; then
    if [ "$AMCL_ODOM" == "odom" ]; then
        echo "    ✅ AMCLのodom_frame_id = odom"
    else
        echo "    ❌ AMCLのodom_frame_id = $AMCL_ODOM (odom であるべき)"
    fi
else
    echo "    ℹ️ AMCL未起動（正常、nvblox使用時は不要）"
fi

echo ""

# Local Costmap確認
echo "  [Local Costmap] global_frame 設定確認"
LOCAL_FRAME=$(timeout 3 ros2 param get /local_costmap/local_costmap global_frame 2>/dev/null | grep -o "'.*'" | tr -d "'")
if [ -n "$LOCAL_FRAME" ]; then
    if [ "$LOCAL_FRAME" == "odom" ]; then
        echo "    ✅ Local Costmapのglobal_frame = odom"
    else
        echo "    ❌ Local Costmapのglobal_frame = $LOCAL_FRAME (odom であるべき)"
    fi
else
    echo "    ℹ️ Local Costmap未起動"
fi

echo ""

# Global Costmap確認
echo "  [Global Costmap] global_frame 設定確認"
GLOBAL_FRAME=$(timeout 3 ros2 param get /global_costmap/global_costmap global_frame 2>/dev/null | grep -o "'.*'" | tr -d "'")
if [ -n "$GLOBAL_FRAME" ]; then
    if [ "$GLOBAL_FRAME" == "map" ]; then
        echo "    ✅ Global Costmapのglobal_frame = map"
    else
        echo "    ❌ Global Costmapのglobal_frame = $GLOBAL_FRAME (map であるべき)"
    fi
else
    echo "    ℹ️ Global Costmap未起動"
fi

echo ""

# Behavior Server確認
echo "  [Behavior Server] global_frame 設定確認"
BEHAVIOR_FRAME=$(timeout 3 ros2 param get /behavior_server global_frame 2>/dev/null | grep -o "'.*'" | tr -d "'")
if [ -n "$BEHAVIOR_FRAME" ]; then
    if [ "$BEHAVIOR_FRAME" == "odom" ]; then
        echo "    ✅ Behavior Serverのglobal_frame = odom"
    else
        echo "    ❌ Behavior Serverのglobal_frame = $BEHAVIOR_FRAME (odom であるべき)"
    fi
else
    echo "    ℹ️ Behavior Server未起動"
fi

echo ""
echo "6. 統合診断"
echo "=========================================="
echo ""

ERRORS=0

# 必須TF確認
timeout 5 ros2 run tf2_ros tf2_echo map odom >/dev/null 2>&1 || ((ERRORS++))
timeout 5 ros2 run tf2_ros tf2_echo odom zed_camera_link >/dev/null 2>&1 || ((ERRORS++))
timeout 5 ros2 run tf2_ros tf2_echo map zed_camera_link >/dev/null 2>&1 || ((ERRORS++))

# zed_odomが存在しないことを確認
timeout 3 ros2 run tf2_ros tf2_monitor 2>/dev/null | grep "zed_odom" >/dev/null && ((ERRORS++))

if [ $ERRORS -eq 0 ]; then
    echo "  ✅ TFツリーの整合性: 正常"
    echo "  ✅ すべてのフレームが正しく統一されています"
    echo "  ✅ Nav2自動航行が可能な状態です"
else
    echo "  ❌ TFツリーに $ERRORS 個の問題があります"
    echo "  ⚠️ 上記のエラーを確認してください"
fi

echo ""
echo "=========================================="
echo "TFツリー可視化"
echo "=========================================="
echo ""
echo "詳細なTFツリーを確認するには以下を実行:"
echo "  ros2 run tf2_tools view_frames"
echo ""
echo "生成されたframes.pdfをブラウザで開く:"
echo "  evince frames.pdf"
echo ""

echo "=========================================="
echo "次のステップ"
echo "=========================================="
echo ""

if [ $ERRORS -eq 0 ]; then
    echo "1. /mapトピックも確認:"
    echo "   /home/jetros/ros2_ws/test_nvblox_nav2_map.sh"
    echo ""
    echo "2. ナビゲーションテスト:"
    echo "   ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \\"
    echo "     '{header: {frame_id: \"map\"}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'"
else
    echo "問題がある場合:"
    echo "1. システムを再起動:"
    echo "   ros2 launch bringup zed2i_nvblox_nav2_launch.py"
    echo ""
    echo "2. ZED設定を確認:"
    echo "   ros2 param get /zed/zed_node pos_tracking.odometry_frame"
    echo "   （期待値: odom）"
    echo ""
    echo "3. ログを確認:"
    echo "   ros2 node info /zed/zed_node"
fi

echo ""
echo "=========================================="
