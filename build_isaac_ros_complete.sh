#!/bin/bash

###############################################################################
# Isaac ROS Complete Build Script with NITROS Support
# パッケージを依存関係順に段階的にビルドします
###############################################################################

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

WORKSPACE_DIR=/home/jetros/ros2_ws
cd $WORKSPACE_DIR

# 環境変数の設定
export CUDA_HOME=/usr/local/cuda
export PATH=$CUDA_HOME/bin:$PATH
export LD_LIBRARY_PATH=$CUDA_HOME/lib64:/usr/local/zed/lib:$LD_LIBRARY_PATH
export CUDAARCHS=87

# GXF SDK パス設定
export GXF_SDK_ROOT=/opt/nvidia/graph-composer/extension-dev
export CMAKE_PREFIX_PATH=$GXF_SDK_ROOT:$CMAKE_PREFIX_PATH
export CPLUS_INCLUDE_PATH=$GXF_SDK_ROOT:$CPLUS_INCLUDE_PATH
export LD_LIBRARY_PATH=/opt/nvidia/graph-composer:$LD_LIBRARY_PATH

source /opt/ros/humble/setup.bash

###############################################################################
# Stage 1: Isaac ROS Common（基本インターフェース）
###############################################################################
log_info "Stage 1: Building Isaac ROS Common interfaces..."
colcon build \
    --packages-select \
        isaac_ros_apriltag_interfaces \
        isaac_ros_bi3d_interfaces \
        isaac_ros_tensor_list_interfaces \
        isaac_ros_pointcloud_interfaces \
        isaac_ros_nova_interfaces \
        isaac_ros_nitros_bridge_interfaces \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

###############################################################################
# Stage 2: Negotiated + Isaac ROS Common（コアパッケージ）
###############################################################################
log_info "Stage 2a: Building negotiated packages..."
colcon build \
    --packages-select \
        negotiated_interfaces \
        negotiated \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

log_info "Stage 2b: Building Isaac ROS Common core packages..."
colcon build \
    --packages-select \
        isaac_common \
        isaac_common_py \
        isaac_ros_common \
        isaac_ros_launch_utils \
        isaac_ros_r2b_galileo \
        isaac_ros_rosbag_utils \
        isaac_ros_test_cmake \
        isaac_ros_test \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

###############################################################################
# Stage 3: Isaac ROS GXF Core
###############################################################################
log_info "Stage 3: Building Isaac ROS GXF core..."
colcon build \
    --packages-select \
        isaac_ros_gxf \
    --metas colcon_isaac.meta \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

###############################################################################
# Stage 4: Isaac ROS GXF Extensions（基本メッセージ）
###############################################################################
log_info "Stage 4a: Building Isaac ROS GXF Extensions (gems)..."
colcon build \
    --packages-select \
        gxf_isaac_gems \
        gxf_isaac_gxf_helpers \
    --metas colcon_isaac.meta \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

log_info "Stage 4b: Building Isaac ROS GXF Extensions (messages)..."
colcon build \
    --packages-select \
        gxf_isaac_messages \
        gxf_isaac_ros_messages \
    --metas colcon_isaac.meta \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

log_info "Stage 4c: Building Isaac ROS GXF Extensions (additional utilities)..."
colcon build \
    --packages-select \
        gxf_isaac_optimizer \
        gxf_isaac_sight \
        gxf_isaac_atlas \
        gxf_isaac_message_compositor \
    --metas colcon_isaac.meta \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1 \
    --continue-on-error

source install/setup.bash

###############################################################################
# Stage 5: Isaac ROS NITROS Core
###############################################################################
log_info "Stage 5: Building Isaac ROS NITROS core..."
colcon build \
    --packages-select \
        isaac_ros_nitros \
        isaac_ros_managed_nitros \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

###############################################################################
# Stage 6: Isaac ROS NITROS Types（基本型）
###############################################################################
log_info "Stage 6: Building Isaac ROS NITROS types..."
colcon build \
    --packages-select \
        isaac_ros_nitros_std_msg_type \
        isaac_ros_nitros_image_type \
        isaac_ros_nitros_camera_info_type \
        isaac_ros_nitros_tensor_list_type \
        isaac_ros_nitros_compressed_image_type \
        isaac_ros_nitros_disparity_image_type \
        isaac_ros_nitros_point_cloud_type \
        isaac_ros_nitros_pose_cov_stamped_type \
        isaac_ros_nitros_imu_type \
        isaac_ros_nitros_twist_type \
        isaac_ros_nitros_odometry_type \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

###############################################################################
# Stage 7: Isaac ROS GXF Extensions（画像処理）
###############################################################################
log_info "Stage 7: Building Isaac ROS GXF Extensions (image pipeline)..."
colcon build \
    --packages-select \
        gxf_isaac_image_flip \
        gxf_isaac_tensorops \
        gxf_isaac_rectify \
        gxf_isaac_sgm \
        gxf_isaac_depth_image_proc \
    --metas colcon_isaac.meta \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

###############################################################################
# Stage 8: Isaac ROS Image Pipeline
###############################################################################
log_info "Stage 8: Building Isaac ROS Image Pipeline..."
colcon build \
    --packages-select \
        isaac_ros_image_proc \
        isaac_ros_stereo_image_proc \
        isaac_ros_depth_image_proc \
        isaac_ros_image_pipeline \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

###############################################################################
# Stage 9: Isaac ROS GXF Extensions（DNN推論）
###############################################################################
log_info "Stage 9: Building Isaac ROS GXF Extensions (DNN inference)..."
colcon build \
    --packages-select \
        gxf_isaac_tensor_rt \
        gxf_isaac_triton \
    --metas colcon_isaac.meta \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

###############################################################################
# Stage 10: Isaac ROS DNN Inference
###############################################################################
log_info "Stage 10: Building Isaac ROS DNN Inference..."
colcon build \
    --packages-select \
        isaac_ros_dnn_image_encoder \
        isaac_ros_tensor_proc \
        isaac_ros_tensor_rt \
        isaac_ros_triton \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

###############################################################################
# Stage 11: Isaac ROS NITROS Types（検出・追加型）
###############################################################################
log_info "Stage 11: Building Isaac ROS NITROS additional types..."
colcon build \
    --packages-select \
        isaac_ros_nitros_detection2_d_array_type \
        isaac_ros_nitros_detection3_d_array_type \
        isaac_ros_nitros_pose_array_type \
        isaac_ros_nitros_occupancy_grid_type \
        isaac_ros_nitros_flat_scan_type \
        isaac_ros_nitros_encoder_ticks_type \
        isaac_ros_nitros_battery_state_type \
        isaac_ros_nitros_correlated_timestamp_type \
        isaac_ros_nitros_compressed_video_type \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

###############################################################################
# Stage 12: Isaac ROS GXF Extensions（物体検出）
###############################################################################
log_info "Stage 12: Building Isaac ROS GXF Extensions (object detection)..."
colcon build \
    --packages-select \
        gxf_isaac_detectnet \
        gxf_isaac_yolov8 \
        gxf_isaac_rtdetr \
    --metas colcon_isaac.meta \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1 \
    --continue-on-error

source install/setup.bash

###############################################################################
# Stage 13: Isaac ROS Object Detection
###############################################################################
log_info "Stage 13: Building Isaac ROS Object Detection..."
colcon build \
    --packages-select \
        isaac_ros_detectnet \
        isaac_ros_yolov8 \
        isaac_ros_rtdetr \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1 \
    --continue-on-error

source install/setup.bash

###############################################################################
# Stage 14: Isaac ROS DNN Stereo Depth
###############################################################################
log_info "Stage 14: Building Isaac ROS DNN Stereo Depth..."
colcon build \
    --packages-select \
        gxf_isaac_ess \
        gxf_isaac_video_buffer_utils \
        isaac_ros_ess \
    --metas colcon_isaac.meta \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1 \
    --continue-on-error

source install/setup.bash

###############################################################################
# Stage 15: Isaac ROS GXF Extensions（画像セグメンテーション）
###############################################################################
log_info "Stage 15: Building Isaac ROS GXF Extensions (segmentation)..."
colcon build \
    --packages-select \
        gxf_isaac_ros_unet \
        gxf_isaac_ros_segment_anything \
        gxf_isaac_segformer \
    --metas colcon_isaac.meta \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1 \
    --continue-on-error

source install/setup.bash

###############################################################################
# Stage 16: Isaac ROS Image Segmentation
###############################################################################
log_info "Stage 16: Building Isaac ROS Image Segmentation..."
colcon build \
    --packages-select \
        isaac_ros_segformer \
        isaac_ros_unet \
        isaac_ros_segment_anything \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1 \
    --continue-on-error

source install/setup.bash

###############################################################################
# Stage 17: Isaac ROS GXF Extensions（姿勢推定）
###############################################################################
log_info "Stage 17: Building Isaac ROS GXF Extensions (pose estimation)..."
colcon build \
    --packages-select \
        gxf_isaac_dope \
        gxf_isaac_centerpose \
        gxf_isaac_foundationpose \
    --metas colcon_isaac.meta \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1 \
    --continue-on-error

source install/setup.bash

###############################################################################
# Stage 18: Isaac ROS Pose Estimation
###############################################################################
log_info "Stage 18: Building Isaac ROS Pose Estimation..."
colcon build \
    --packages-select \
        isaac_ros_pose_proc \
        isaac_ros_dope \
        isaac_ros_centerpose \
        isaac_ros_foundationpose \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1 \
    --continue-on-error

source install/setup.bash

###############################################################################
# Stage 19: Isaac ROS AprilTag
###############################################################################
log_info "Stage 19: Building Isaac ROS AprilTag..."
colcon build \
    --packages-select \
        isaac_ros_apriltag \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

###############################################################################
# Stage 20: Isaac ROS Visual SLAM
###############################################################################
log_info "Stage 20: Building Isaac ROS Visual SLAM..."
colcon build \
    --packages-select \
        isaac_ros_visual_slam_interfaces \
        isaac_ros_visual_slam \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1 \
    --continue-on-error

source install/setup.bash

###############################################################################
# Stage 21: Isaac ROS Nvblox
###############################################################################
log_info "Stage 21: Building Isaac ROS Nvblox..."
colcon build \
    --packages-select \
        nvblox_msgs \
        nvblox \
        nvblox_ros \
        nvblox_examples_bringup \
        nvblox_nav2 \
        nvblox_rviz_plugin \
        isaac_ros_nvblox \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1 \
    --continue-on-error

source install/setup.bash

###############################################################################
# Stage 22: Isaac ROS GXF Extensions（Nova）
###############################################################################
log_info "Stage 22: Building Isaac ROS GXF Extensions (Nova)..."
colcon build \
    --packages-select \
        gxf_isaac_timestamp_correlator \
        gxf_isaac_hesai \
        gxf_isaac_message_compositor \
        gxf_isaac_bmi088_imu \
        gxf_isaac_imu_utils \
    --metas colcon_isaac.meta \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1 \
    --continue-on-error

source install/setup.bash

###############################################################################
# Stage 23: Isaac ROS Nova
###############################################################################
log_info "Stage 23: Building Isaac ROS Nova..."
colcon build \
    --packages-select \
        isaac_ros_correlated_timestamp_driver \
        isaac_ros_hawk \
        isaac_ros_owl \
        isaac_ros_hesai \
        isaac_ros_imu_bmi088 \
        isaac_ros_data_recorder \
        isaac_ros_data_replayer \
        isaac_ros_data_validation \
        isaac_ros_ground_calibration \
        isaac_ros_nova_recorder \
        isaac_ros_nova \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1 \
    --continue-on-error

source install/setup.bash

###############################################################################
# Stage 24: Isaac ROS NITROS Additional Tools
###############################################################################
log_info "Stage 24: Building Isaac ROS NITROS tools..."
colcon build \
    --packages-select \
        isaac_ros_nitros_topic_tools \
        isaac_ros_nitros_bridge_ros2 \
        isaac_ros_pynitros \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1 \
    --continue-on-error

source install/setup.bash

###############################################################################
# Stage 25: ZED ROS 2 Wrapper
###############################################################################
log_info "Stage 25: Building ZED ROS 2 Wrapper..."
colcon build \
    --packages-select \
        zed_interfaces \
        zed_components \
        zed_wrapper \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

###############################################################################
# Stage 26: 残りのパッケージ
###############################################################################
log_info "Stage 26: Building remaining packages..."
colcon build \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1 \
    --continue-on-error

log_info "Complete Isaac ROS build finished!"
log_info "Check build log for any failures."
