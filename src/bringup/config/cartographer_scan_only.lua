-- "map_builder.lua"と"trajectory_builder.lua"の設定ファイルをインクルードする
include "map_builder.lua"
include "trajectory_builder.lua"

-- ===== 純粋スキャンマッチングモード（旋回時の円弧形状を完全回避） =====
-- ZED Wrapperのオドメトリのブレを回避し、
-- Cartographerのスキャンマッチングのみで位置推定を行う
-- 旋回時も円弧を描かず、直線の壁を維持

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame  = "zed_camera_link",
  published_frame = "zed_odom",
  odom_frame = "zed_odom",
  provide_odom_frame = true,  -- Cartographerがodomフレームを提供
  publish_frame_projected_to_2d = true,
  use_odometry = false,  -- 外部オドメトリを使用しない
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
  odometry_sampling_ratio = 0.0,  -- オドメトリを完全に無視
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- ===== 旋回時の円弧形状を防ぐ重要設定 =====

-- スキャン範囲設定
TRAJECTORY_BUILDER_2D.min_range = 0.4
TRAJECTORY_BUILDER_2D.max_range = 12.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 12.0
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- ===== オンライン相関スキャンマッチング（最重要）=====
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- リアルタイム相関スキャンマッチャー（広範囲・高速探索）
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15  -- 0.2→0.15: 適正化
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(45.0)  -- 60→45度: 適正化
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 0.1  -- 探索自由度最大
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 0.1  -- 探索自由度最大

-- ===== 累積範囲データ（円弧形状回避の鍵）=====
-- 重要: オドメトリがない状態で複数スキャンを蓄積すると、
-- 蓄積中のスキャン間の相対位置が不正確になり円弧を描く
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1  -- 2→1: 即座にマッチング（円弧防止）

-- ===== Ceresスキャンマッチャー（高精度・高速バランス）=====
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 20.0  -- 30→20: バランス調整
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 250.0  -- 300→250: 高精度維持

-- Ceres最適化設定（精度と速度のバランス）
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20  -- 100→20: 高速化
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 4

-- ===== モーションフィルター（細かい更新）=====
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5.0
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.01  -- 1cm
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)  -- 0.2→0.1度: より細かく

-- ===== サブマップ設定 =====
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90  -- 60→90: 安定性向上
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type = "PROBABILITY_GRID"
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

-- 占有確率の更新設定
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.insert_free_space = true
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.70
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.30

-- ===== ポーズグラフ最適化（スキャンマッチング100%依存）=====
POSE_GRAPH.optimize_every_n_nodes = 90  -- 15→90: 最適化を抑制（円弧防止）

-- 制約ビルダー設定
POSE_GRAPH.constraint_builder.min_score = 0.65  -- 0.60→0.65: 厳格化
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.70  -- 0.65→0.70
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3  -- 0.5→0.3: サンプリング抑制
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0

-- 高速相関スキャンマッチャー（ループクロージャ用）
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(45.0)  -- 60→45度
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7  -- 8→7: 高速化

-- Ceresスキャンマッチャー（ループクロージャ用）
POSE_GRAPH.constraint_builder.ceres_scan_matcher.occupied_space_weight = 20.0  -- 30→20
POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 200.0
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20  -- 100→20: 高速化
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.num_threads = 4

-- ===== 最適化問題の重み設定（スキャンマッチング100%）=====
POSE_GRAPH.optimization_problem.huber_scale = 1e3  -- 5e2→1e3: 安定化

-- オドメトリ重みをゼロに（完全無視）
POSE_GRAPH.optimization_problem.odometry_translation_weight = 0
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 0

-- ローカルSLAM（スキャンマッチング）重みを最大化
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e7
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e7

-- 加速度制約（急激な変化を抑制）
POSE_GRAPH.optimization_problem.acceleration_weight = 1e4  -- 5e3→1e4: より滑らかに

-- グローバル最適化の収束設定
POSE_GRAPH.optimization_problem.ceres_solver_options.use_nonmonotonic_steps = true
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 50  -- 200→50: 高速化
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 4

-- 最適化頻度とタイミング
POSE_GRAPH.optimization_problem.log_solver_summary = false
POSE_GRAPH.max_num_final_iterations = 200  -- 500→200: 高速化

return options
