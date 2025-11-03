-- "map_builder.lua"と"trajectory_builder.lua"の設定ファイルをインクルードする
include "map_builder.lua"
include "trajectory_builder.lua"

-- オプション設定の開始
options = {
  map_builder = MAP_BUILDER,  -- 地図生成の設定オブジェクト
  trajectory_builder = TRAJECTORY_BUILDER,  -- 軌跡生成の設定オブジェクト
  map_frame = "map",  -- 地図の基準フレームの名前
  --tracking_frame = "base_link",  -- ロボットの位置を追跡するためのフレーム
  --published_frame = "base_link",  -- 発行するフレームの名前
  tracking_frame  = "zed_camera_link",  -- ロボットの位置を追跡するためのフレーム
  published_frame = "zed_odom",  -- 発行するフレームの名前
  odom_frame = "zed_odom",  -- オドメトリの基準フレームの名前
  --provide_odom_frame = false,  -- 自動的にオドメトリフレー0
  provide_odom_frame = false, --true,  -- 自動的にオドメトリフレー0
  --publish_frame_projected_to_2d = false,  -- 発行フレームを2次元に投影するか
  publish_frame_projected_to_2d = true,  -- 発行フレームを2次元に投影するか
  --use_odometry = false,  -- オドメトリデータを使用するか
  use_odometry = true,  -- オドメトリデータを使用するか
  use_nav_sat = false,  -- GPSデータを使用するか
  use_landmarks = false,  -- ランドマークデータを使用するか
  num_laser_scans = 1,  -- 使用するレーザースキャンの数
  num_multi_echo_laser_scans = 0,  -- マルチエコーレーザースキャンの数
  num_subdivisions_per_laser_scan = 1,  -- レーザースキャンごとに分割する数
  num_point_clouds = 0,  -- 使用するポイントクラウドの数
  lookup_transform_timeout_sec = 10.0,  -- トランスフォームのタイムアウト時間（秒）- 時刻同期問題に対応
  submap_publish_period_sec = 0.05,  -- サブマップの発行周期（秒）- 停止時も頻繁な更新
  pose_publish_period_sec = 5e-3,  -- ポーズの発行周期（秒）
  trajectory_publish_period_sec = 30e-3,  -- 軌跡の発行周期（秒）
  rangefinder_sampling_ratio = 1.0,  -- 距離センサーのサンプリング割合（自由空間更新のため全データ使用）
  odometry_sampling_ratio = 1.,  -- オドメトリのサンプリング割合（ZEDオドメトリを全て使用）
  fixed_frame_pose_sampling_ratio = 1.,  -- 固定フレームのポーズのサンプリング割合
  imu_sampling_ratio = 1.,  -- IMUのサンプリング割合
  landmarks_sampling_ratio = 1.,  -- ランドマークのサンプリング割合
}

-- 2次元の軌跡生成を使用する設定
MAP_BUILDER.use_trajectory_builder_2d = true

-- 2D軌跡生成の設定
TRAJECTORY_BUILDER_2D.min_range = 0.4  -- 距離測定の最小範囲（メートル）
TRAJECTORY_BUILDER_2D.max_range = 12.0  -- 距離測定の最大範囲（メートル）- ノイズ点群での地図膨張を防止
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 12.0  -- 欠損データの代わりに使用する距離（max_range以上に設定）
TRAJECTORY_BUILDER_2D.use_imu_data = false  -- IMUデータを使用するか
--TRAJECTORY_BUILDER_2D.use_imu_data = true  -- IMUデータを使用するか
--TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false  -- オンラインスキャンマッチングを使用するか
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true  -- オンラインスキャンマッチングを使用する by motoms
-- リアルタイムスキャンマッチャーの設定（旋回時の安定性向上）
-- translation_delta_cost_weight: 平行移動の変化に対するコスト重み（高い値=平行移動を抑制、旋回時のブレ軽減）
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.0
-- rotation_delta_cost_weight: 回転の変化に対するコスト重み（高い値=回転推定の安定性向上）
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e3
-- 累積範囲データ数（スキャンデータの蓄積数）
-- 値を増やすとスキャンマッチングの精度が向上するが、計算負荷とレスポンス遅延が増加
-- 1: 高速だがノイズに敏感、2-3: バランス良好、4以上: 高精度だが遅延大
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1                 -- 1 (停止時の安定性向上) by motoms

-- POSE_GRAPHの制約ビルダーのスコア設定（ループクロージャの信頼性制御）
-- min_score: ローカルマッチングの最小信頼度（0.0-1.0、高い値=厳格な制約、誤マッチング減少）
POSE_GRAPH.constraint_builder.min_score = 0.65  -- スキャンマッチングの最小スコア
-- global_localization_min_score: グローバル位置推定の最小信頼度（高い値=より確実な位置推定）
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7  -- グローバル位置推定の最小スコア

-- POSE_GRAPHの最適化問題の重み設定（グローバルマップ最適化）
-- local_slam_pose_translation_weight: ローカルSLAMの平行移動信頼度（高い値=ローカルマップ重視）
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5  -- ローカルSLAMの平行移動に対する重み
-- local_slam_pose_rotation_weight: ローカルSLAMの回転信頼度（高い値=ローカル回転推定重視）
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5  -- ローカルSLAMの回転に対する重み
-- odometry_translation_weight: オドメトリの平行移動信頼度（高い値=車輪エンコーダ重視）
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5  -- オドメトリの平行移動に対する重み
-- odometry_rotation_weight: オドメトリの回転信頼度（高い値=車輪による回転推定重視）
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5  -- オドメトリの回転に対する重み
-- huber_scale: 外れ値に対するロバスト性制御（高い値=外れ値の影響軽減）
POSE_GRAPH.optimization_problem.huber_scale = 1e3  -- Huber損失関数のスケール

-- スキャンマッチングの重み設定
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10  -- 占有空間に対する重み
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40  -- 回転に対する重み
-- Ceresスキャンマッチャーの重み設定（非線形最適化による高精度マッチング）
-- occupied_space_weight: 占有空間への適合度重み（高い値=障害物検出精度向上、地図の鮮明度向上）
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1  -- 占有空間に対する重み
-- rotation_weight: 回転推定の重み（高い値=旋回時の角度推定精度向上、地図歪み軽減）
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40  -- 回転に対する重み

-- サブマップ設定（局所的な地図セグメントの管理）
-- num_range_data: サブマップあたりのスキャンデータ数（多い値=詳細な地図、少ない値=軽量処理）
-- 一般的な値: 60-120（室内）、200-400（屋外）
-- 静止時も地図更新を継続するため、データ数を減らして更新頻度を向上
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 60  -- サブマップあたりの距離データの数（停止時の高頻度更新）
-- モーションフィルター設定（不要な小さな動きをフィルタリング）
-- max_distance_meters: この距離以下の移動は無視（小さい値=細かい動きも記録、大きい値=大きな動きのみ記録）
-- 静止時の地図更新を有効にするため、フィルタ値を緩和
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.01  -- 最大距離のフィルタ（メートル）- 停止時も地図更新
-- max_angle_radians: この角度以下の回転は無視（小さい値=細かい回転も記録、旋回時の精度向上）
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.01)  -- 最大角度のフィルタ（ラジアン）- 停止時も地図更新

--SPARSE_POSE_GRAPH.optimize_every_n_scans = 0 --by motoms submapの中で滑りが起きているため、local SLAMの問題です。それでは、global SLAMをoffにして、チューニングの邪魔をしないようにしましょう。
POSE_GRAPH.optimize_every_n_nodes = 0

-- 静止時の地図更新を強制するための追加設定
-- 地図の確率値更新を有効化（静止時でもスキャンデータによる地図更新を継続）
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type = "PROBABILITY_GRID"
-- 占有確率の更新頻度を向上（静止時の障害物検出精度向上）
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.insert_free_space = true
-- 自由空間の挿入を有効化（白いエリアの更新を促進）
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.7
-- ヒット確率の調整（障害物検出感度）
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.3
-- 最終的なオプション設定を返す
return options

