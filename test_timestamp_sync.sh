#!/bin/bash

echo "========================================="
echo "laser_merger2 タイムスタンプ同期テスト"
echo "========================================="
echo ""

# ビルド確認
echo "1. ビルド確認"
if [ -f "/home/jetros/ros2_ws/install/laser_merger2/lib/laser_merger2/laser_merger2" ]; then
    echo "✅ ビルド成功: laser_merger2実行ファイルが存在"
else
    echo "❌ ビルドエラー: 実行ファイルが見つかりません"
    exit 1
fi

echo ""
echo "2. テストlaunchファイル起動 (バックグラウンド)"
source /home/jetros/ros2_ws/install/setup.bash

# LiDARが起動しているか確認
if ! ros2 topic list | grep -q "/front_scan"; then
    echo "⚠️  警告: /front_scan トピックが見つかりません。LiDARを起動してください。"
fi

if ! ros2 topic list | grep -q "/back_scan"; then
    echo "⚠️  警告: /back_scan トピックが見つかりません。LiDARを起動してください。"
fi

# laser_merger2起動
echo ""
echo "3. laser_merger2起動中..."
ros2 launch laser_merger2 laser_merger_for_slam.launch.py &
LAUNCH_PID=$!

echo "起動PID: $LAUNCH_PID"
sleep 5

echo ""
echo "4. ノード起動確認"
if ros2 node list | grep -q "laser_merger2"; then
    echo "✅ laser_merger2ノードが起動"
else
    echo "❌ laser_merger2ノードが起動していません"
    kill $LAUNCH_PID 2>/dev/null
    exit 1
fi

echo ""
echo "5. トピック出力確認"
if ros2 topic list | grep -q "/merged_scan"; then
    echo "✅ /merged_scan トピックが存在"
else
    echo "❌ /merged_scan トピックが見つかりません"
    kill $LAUNCH_PID 2>/dev/null
    exit 1
fi

echo ""
echo "6. /merged_scan レート確認 (5秒間)"
timeout 5 ros2 topic hz /merged_scan || echo "⚠️  データが出力されていない可能性があります"

echo ""
echo "7. タイムスタンプ診断ログ確認 (10秒間)"
echo "以下のログを確認してください:"
echo "  - 'Timestamp sync config' でパラメータ設定を確認"
echo "  - 'Timestamp diagnostics' でタイムスタンプ差を確認"
echo "  - 'Timestamp差が閾値を超えています' という警告がないか確認"
echo ""
timeout 10 ros2 topic echo /rosout --field msg | grep -i "timestamp" || echo "タイムスタンプ関連のログが見つかりませんでした"

echo ""
echo "8. クリーンアップ"
kill $LAUNCH_PID 2>/dev/null
sleep 2

echo ""
echo "========================================="
echo "テスト完了"
echo "========================================="
echo ""
echo "【確認事項】"
echo "1. ノードが正常に起動したか"
echo "2. /merged_scan が出力されているか（20Hz前後を期待）"
echo "3. タイムスタンプ診断で異常がないか"
echo "4. エラーログがないか"
echo ""
echo "【次のステップ】"
echo "問題がなければ、以下のコマンドでSLAMテストを実施:"
echo "  ros2 launch laser_merger2 laser_merger_for_slam.launch.py"
echo ""
