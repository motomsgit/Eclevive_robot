#!/bin/bash

echo "========================================="
echo "LiDAR スキャンレート測定テスト"
echo "========================================="
echo ""

# LiDARが起動しているか確認
echo "1. トピックリスト確認:"
ros2 topic list | grep scan

echo ""
echo "2. /front_scan のレート測定 (10秒間):"
timeout 10 ros2 topic hz /front_scan

echo ""
echo "3. /back_scan のレート測定 (10秒間):"
timeout 10 ros2 topic hz /back_scan

echo ""
echo "4. タイムスタンプ差の確認 (5秒間サンプリング):"
timeout 5 ros2 topic echo /front_scan --field header.stamp &
PID1=$!
timeout 5 ros2 topic echo /back_scan --field header.stamp &
PID2=$!

wait $PID1
wait $PID2

echo ""
echo "========================================="
echo "測定完了"
echo "========================================="
