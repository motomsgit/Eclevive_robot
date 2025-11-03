#!/bin/bash

# TF-Topic同期診断スクリプト
# Usage: ./tf_sync_diagnostics.sh [duration_seconds]

DURATION=${1:-30}  # デフォルト30秒間

echo "=== TF-Topic同期診断開始 ==="
echo "診断期間: ${DURATION}秒"
echo "開始時刻: $(date)"
echo

# 1. 現在のTFツリー構造を確認
echo "1. TFツリー構造:"
echo "======================================"
timeout 5s ros2 run tf2_tools view_frames || echo "Warning: TF frames could not be retrieved"
echo

# 2. アクティブなトピック一覧
echo "2. 関連トピック一覧:"
echo "======================================"
echo "Odometry topics:"
ros2 topic list | grep -E "(odom|odometry)" || echo "No odometry topics found"
echo "Scan topics:"
ros2 topic list | grep -E "(scan|laser)" || echo "No scan topics found"
echo "TF topics:"
ros2 topic list | grep -E "(tf|transform)" || echo "No tf topics found"
echo

# 3. TF変換の遅延チェック
echo "3. TF変換遅延テスト:"
echo "======================================"
echo "Testing map -> zed_camera_link transform..."
ros2 run tf2_ros tf2_echo map zed_camera_link &
TF_ECHO_PID=$!
sleep 5
kill $TF_ECHO_PID 2>/dev/null || true
echo

echo "Testing zed_odom -> zed_camera_link transform..."
ros2 run tf2_ros tf2_echo zed_odom zed_camera_link &
TF_ECHO_PID=$!
sleep 5
kill $TF_ECHO_PID 2>/dev/null || true
echo

# 4. トピックの時刻同期チェック
echo "4. トピック時刻同期分析:"
echo "======================================"
echo "Odometry timestamp check:"
timeout 10s ros2 topic echo --once /zed/zed_node/odom --field header.stamp 2>/dev/null || echo "No odometry data received"

echo "Scan timestamp check:"
timeout 10s ros2 topic echo --once /scan_filtered --field header.stamp 2>/dev/null || echo "No scan data received"

echo "Current ROS time:"
timeout 5s ros2 param get /use_sim_time use_sim_time 2>/dev/null || echo "Could not retrieve sim_time setting"
echo

# 5. 同期コーディネーターの状態確認
echo "5. TF同期コーディネーター状態:"
echo "======================================"
if ros2 node list | grep -q "tf_sync_coordinator"; then
    echo "✓ TF同期コーディネーターが実行中"
    echo "統計情報を取得中..."
    timeout 10s ros2 topic echo --once /rosout --field msg | grep -i "sync" | tail -5 || echo "同期統計が取得できませんでした"
else
    echo "✗ TF同期コーディネーターが見つかりません"
fi
echo

# 6. Navigation2ノードの状態確認
echo "6. Navigation2ノード状態:"
echo "======================================"
NAV_NODES=("planner_server" "controller_server" "bt_navigator" "behavior_server")
for node in "${NAV_NODES[@]}"; do
    if ros2 node list | grep -q $node; then
        echo "✓ $node: 実行中"
    else
        echo "✗ $node: 停止中"
    fi
done
echo

# 7. リアルタイム診断（指定期間）
echo "7. リアルタイム時刻同期監視 (${DURATION}秒間):"
echo "======================================"
echo "Ctrl+Cで中断..."

# バックグラウンドで各トピックの時刻を監視
{
    while true; do
        ODOM_TIME=$(timeout 2s ros2 topic echo --once /zed/zed_node/odom --field header.stamp.sec 2>/dev/null || echo "N/A")
        SCAN_TIME=$(timeout 2s ros2 topic echo --once /scan_filtered --field header.stamp.sec 2>/dev/null || echo "N/A")
        CURRENT_TIME=$(date +%s)

        if [[ "$ODOM_TIME" != "N/A" && "$SCAN_TIME" != "N/A" ]]; then
            TIME_DIFF=$((ODOM_TIME - SCAN_TIME))
            ODOM_DELAY=$((CURRENT_TIME - ODOM_TIME))
            SCAN_DELAY=$((CURRENT_TIME - SCAN_TIME))

            echo "$(date '+%H:%M:%S') - Odom: ${ODOM_TIME}s (delay: ${ODOM_DELAY}s), Scan: ${SCAN_TIME}s (delay: ${SCAN_DELAY}s), Diff: ${TIME_DIFF}s"
        else
            echo "$(date '+%H:%M:%S') - Data unavailable (Odom: $ODOM_TIME, Scan: $SCAN_TIME)"
        fi

        sleep 3
    done
} &
MONITOR_PID=$!

# 指定時間後に監視を終了
sleep $DURATION
kill $MONITOR_PID 2>/dev/null || true
wait $MONITOR_PID 2>/dev/null || true

echo
echo "=== 診断完了 ==="
echo "終了時刻: $(date)"

# 推奨対策の表示
echo
echo "=== 推奨対策 ==="
echo "1. 時刻ずれが大きい場合:"
echo "   - システム時刻の同期を確認"
echo "   - transform_tolerance値を調整"
echo "   - TF同期コーディネーターの許容値を増加"
echo
echo "2. データが取得できない場合:"
echo "   - センサーノードの起動状態を確認"
echo "   - トピック名の不整合をチェック"
echo "   - ネットワーク接続を確認"
echo
echo "3. Navigation2エラーが続く場合:"
echo "   - QoS設定の互換性を確認"
echo "   - ノードの再起動を試行"
echo "   - ログファイルで詳細エラーを確認"