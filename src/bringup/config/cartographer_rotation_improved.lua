-- "map_builder.lua"と"trajectory_builder.lua"の設定ファイルをインクルードする
include "map_builder.lua"
include "trajectory_builder.lua"

-- オプション設定の開始
options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame  = "zed_camera_link",
  published_frame = "zed_odom",
  odom_frame = "zed_odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 10.0,
  submap_publish_period_sec = 0.05,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,  -- オドメトリを使用（但し重みは下げる）
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- ===== 旋回時の円弧形状を防ぐための重要設定 =====

-- スキャン範囲設定
TRAJECTORY_BUILDER_2D.min_range = 0.4
TRAJECTORY_BUILDER_2D.max_range = 12.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 12.0
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- ===== オンライン相関スキャンマッチング（旋回補正を最優先）=====
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- リアルタイム相関スキャンマッチャー
-- スキャンマッチングを最優先にするため、コストを大幅に緩和
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(45.0)  -- ±45度
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1.0  -- オドメトリ依存を下げる
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1.0  -- オドメトリ依存を下げる

-- 累積範囲データ（旋回時の安定性向上）
-- 複数スキャンを蓄積してマッチング精度を向上
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 3  -- 1→3: 旋回時の安定性大幅向上

-- ===== Ceresスキャンマッチャー（非線形最適化でスキャンマッチング最優先）=====
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 20.0  -- 10→20: スキャンマッチング重視
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 200.0  -- 100→200: 回転補正を最優先

-- Ceres最適化設定（高精度・高速化）
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 50  -- 30→50: 旋回時の収束改善
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 4

-- ===== モーションフィルター =====
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5.0  -- 0.2→5.0: タイムアウトを緩和
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.01  -- 0.02→0.01: より細かく更新
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.2)  -- 0.5→0.2度: より細かく更新

-- ===== サブマップ設定 =====
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 60  -- 80→60: 更新頻度を上げる
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type = "PROBABILITY_GRID"
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

-- 占有確率の更新設定
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.insert_free_space = true
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.70
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.30

-- ===== ポーズグラフ最適化（オドメトリ依存を下げる）=====
POSE_GRAPH.optimize_every_n_nodes = 20  -- 30→20: より高頻度に最適化

-- 制約ビルダー設定
POSE_GRAPH.constraint_builder.min_score = 0.55  -- 0.60→0.55: 制約をやや緩和
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.60  -- 0.65→0.60
POSE_GRAPH.constraint_builder.sampling_ratio = 0.5
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0

-- 高速相関スキャンマッチャー（ループクロージャ用）
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(45.0)  -- 30→45度
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 8

-- Ceresスキャンマッチャー（ループクロージャ用）
POSE_GRAPH.constraint_builder.ceres_scan_matcher.occupied_space_weight = 20.0
POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 100.0
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 50
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.num_threads = 4

-- ===== 最適化問題の重み設定（スキャンマッチング最優先）=====
POSE_GRAPH.optimization_problem.huber_scale = 1e3
POSE_GRAPH.optimization_problem.acceleration_weight = 1e3

-- オドメトリ重みを大幅に下げる（円弧形状の根本原因対策）
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e3  -- 1e5→1e3: オドメトリ依存を大幅に低減
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e3  -- 1e5→1e3: 回転もオドメトリ依存を低減

-- ローカルSLAM（スキャンマッチング）重みを最優先
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e6  -- 1e5→1e6: スキャンマッチング最優先
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e6  -- 1e5→1e6: 回転もスキャンマッチング最優先

-- グローバル最適化の収束設定
POSE_GRAPH.optimization_problem.ceres_solver_options.use_nonmonotonic_steps = true
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 100
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 4

-- 最適化頻度とタイミング
POSE_GRAPH.optimization_problem.log_solver_summary = false
POSE_GRAPH.max_num_final_iterations = 400

return options
