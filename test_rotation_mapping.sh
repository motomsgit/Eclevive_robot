#!/bin/bash
# 旋回時の地図生成テストスクリプト

echo "🔄 旋回対応パラメータの動作確認テスト"
echo "========================================="
echo ""

# ROS2環境のセットアップ
source /opt/ros/humble/setup.bash
source /home/jetros/ros2_ws/install/setup.bash

echo "✅ パラメータファイルの確認"
PARAM_FILE="/home/jetros/ros2_ws/src/bringup/config/mapper_params_online_async copy.yaml"

if [ -f "$PARAM_FILE" ]; then
    echo "  - ファイル存在: OK"

    # 重要なパラメータの確認
    echo ""
    echo "📋 修正されたパラメータ:"
    echo "  - transform_publish_period: $(grep 'transform_publish_period:' "$PARAM_FILE" | awk '{print $2}')"
    echo "  - map_update_interval: $(grep 'map_update_interval:' "$PARAM_FILE" | awk '{print $2}')"
    echo "  - minimum_time_interval: $(grep 'minimum_time_interval:' "$PARAM_FILE" | awk '{print $2}')"
    echo "  - minimum_travel_heading: $(grep 'minimum_travel_heading:' "$PARAM_FILE" | awk '{print $2}')"
    echo "  - correlation_search_space_dimension: $(grep 'correlation_search_space_dimension:' "$PARAM_FILE" | awk '{print $2}')"
    echo "  - coarse_angle_resolution: $(grep 'coarse_angle_resolution:' "$PARAM_FILE" | awk '{print $2}')"
else
    echo "  ❌ パラメータファイルが見つかりません: $PARAM_FILE"
    exit 1
fi

echo ""
echo "========================================="
echo "✅ パラメータファイルの確認完了"
echo ""
echo "🚀 実行方法:"
echo "  1. bringup_ps5_all_launch.py を起動してください"
echo "  2. 機体を旋回させて地図生成を確認してください"
echo ""
echo "🔍 確認ポイント:"
echo "  - 旋回時に地図がスムーズに更新されるか"
echo "  - 地図のもじゃもじゃが軽減されているか"
echo "  - スキャンマッチングが追従しているか"
echo "  - TFの更新頻度が適切か"
