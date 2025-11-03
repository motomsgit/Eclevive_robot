#!/bin/bash

###############################################################################
# Isaac ROS Fixed Staged Build Script
# 依存関係を正しく解決したビルドスクリプト
###############################################################################

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
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

# CV-CUDA パス設定
export CVCUDA_DIR=/opt/nvidia/cvcuda0
export CMAKE_PREFIX_PATH=$CVCUDA_DIR:$CMAKE_PREFIX_PATH
export CPLUS_INCLUDE_PATH=$CVCUDA_DIR/include:$CPLUS_INCLUDE_PATH
export LD_LIBRARY_PATH=$CVCUDA_DIR/lib:$LD_LIBRARY_PATH

source /opt/ros/humble/setup.bash

###############################################################################
# Stage 1: Interface Packages (メッセージ定義)
###############################################################################
log_info "Stage 1: Building Interface packages..."
colcon build \
    --packages-select \
        isaac_ros_apriltag_interfaces \
        isaac_ros_bi3d_interfaces \
        isaac_ros_tensor_list_interfaces \
        isaac_ros_pointcloud_interfaces \
        isaac_ros_nova_interfaces \
        isaac_ros_nitros_bridge_interfaces \
        isaac_ros_visual_slam_interfaces \
        negotiated_interfaces \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

###############################################################################
# Stage 2: Isaac ROS Common & Negotiated
###############################################################################
log_info "Stage 2: Building Isaac ROS Common and Negotiated..."
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
        negotiated \
        negotiated_examples \
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
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

###############################################################################
# Stage 4: GXF Isaac Base Extensions
###############################################################################
log_info "Stage 4: Building GXF Isaac base extensions..."
colcon build \
    --packages-select \
        gxf_isaac_gems \
        gxf_isaac_gxf_helpers \
        gxf_isaac_messages \
        gxf_isaac_atlas \
        gxf_isaac_message_compositor \
        gxf_isaac_optimizer \
        gxf_isaac_sight \
        gxf_isaac_cuda \
        gxf_isaac_ros_cuda \
        gxf_isaac_utils \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

###############################################################################
# Stage 5: Isaac ROS NITROS Core
###############################################################################
log_info "Stage 5: Building Isaac ROS NITROS core..."
colcon build \
    --packages-select \
        isaac_ros_nitros \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

###############################################################################
# Stage 6: Isaac ROS NITROS Types
###############################################################################
log_info "Stage 6: Building Isaac ROS NITROS types..."
colcon build \
    --packages-select \
        isaac_ros_nitros_tensor_list_type \
        isaac_ros_nitros_image_type \
        isaac_ros_nitros_camera_info_type \
        isaac_ros_nitros_compressed_image_type \
        isaac_ros_nitros_disparity_image_type \
        isaac_ros_nitros_point_cloud_type \
        isaac_ros_nitros_detection2_d_array_type \
        isaac_ros_nitros_detection3_d_array_type \
        isaac_ros_nitros_imu_type \
        isaac_ros_nitros_odometry_type \
        isaac_ros_nitros_pose_array_type \
        isaac_ros_nitros_pose_cov_stamped_type \
        isaac_ros_nitros_twist_type \
        isaac_ros_nitros_flat_scan_type \
        isaac_ros_nitros_encoder_ticks_type \
        isaac_ros_nitros_battery_state_type \
        isaac_ros_nitros_correlated_timestamp_type \
        isaac_ros_nitros_compressed_video_type \
        isaac_ros_nitros_occupancy_grid_type \
        isaac_ros_nitros_std_msg_type \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

###############################################################################
# Stage 7: Isaac ROS Managed NITROS
###############################################################################
log_info "Stage 7: Building Isaac ROS Managed NITROS..."
colcon build \
    --packages-select \
        isaac_ros_managed_nitros \
        isaac_ros_nitros_topic_tools \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

###############################################################################
# Stage 8: GXF Isaac Image Processing Extensions
###############################################################################
log_info "Stage 8: Building GXF Isaac image processing extensions..."
colcon build \
    --packages-select \
        gxf_isaac_tensorops \
        gxf_isaac_image_flip \
        gxf_isaac_rectify \
        gxf_isaac_sgm \
        gxf_isaac_depth_image_proc \
        gxf_isaac_point_cloud \
        gxf_isaac_ros_messages \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

###############################################################################
# Stage 9: Isaac ROS Image Pipeline
###############################################################################
log_info "Stage 9: Building Isaac ROS Image Pipeline..."
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
# Stage 10: GXF Isaac DNN Extensions
###############################################################################
log_info "Stage 10: Building GXF Isaac DNN extensions..."
colcon build \
    --packages-select \
        gxf_isaac_tensor_rt \
        gxf_isaac_triton \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

###############################################################################
# Stage 11: Isaac ROS DNN Inference Base
###############################################################################
log_info "Stage 11: Building Isaac ROS DNN Inference base..."
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
# Stage 12: GXF Isaac Object Detection Extensions
###############################################################################
log_info "Stage 12: Building GXF Isaac object detection extensions..."
colcon build \
    --packages-select \
        gxf_isaac_detectnet \
        gxf_isaac_yolov8 \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

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
    --parallel-workers 1

source install/setup.bash

###############################################################################
# Stage 14: GXF Isaac ESS Extension
###############################################################################
log_info "Stage 14: Building GXF Isaac ESS extension..."
colcon build \
    --packages-select \
        gxf_isaac_ess \
        gxf_isaac_messages_throttler \
        gxf_isaac_video_buffer_utils \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

###############################################################################
# Stage 15: Isaac ROS DNN Stereo Depth
###############################################################################
log_info "Stage 15: Building Isaac ROS DNN Stereo Depth..."
colcon build \
    --packages-select \
        isaac_ros_ess_models_install \
        isaac_ros_ess \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

###############################################################################
# Stage 16: GXF Isaac Segmentation Extensions
###############################################################################
log_info "Stage 16: Building GXF Isaac segmentation extensions..."
colcon build \
    --packages-select \
        gxf_isaac_ros_unet \
        gxf_isaac_ros_segment_anything \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

###############################################################################
# Stage 17: Isaac ROS Image Segmentation
###############################################################################
log_info "Stage 17: Building Isaac ROS Image Segmentation..."
colcon build \
    --packages-select \
        isaac_ros_segformer \
        isaac_ros_unet \
        isaac_ros_segment_anything \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

###############################################################################
# Stage 18: GXF Isaac Pose Estimation Extensions
###############################################################################
log_info "Stage 18: Building GXF Isaac pose estimation extensions..."
colcon build \
    --packages-select \
        gxf_isaac_centerpose \
        gxf_isaac_dope \
        gxf_isaac_foundationpose \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

###############################################################################
# Stage 19: Isaac ROS Pose Estimation
###############################################################################
log_info "Stage 19: Building Isaac ROS Pose Estimation..."
colcon build \
    --packages-select \
        isaac_ros_pose_proc \
        isaac_ros_dope \
        isaac_ros_centerpose \
        isaac_ros_foundationpose \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

###############################################################################
# Stage 20: Isaac ROS AprilTag
###############################################################################
log_info "Stage 20: Building Isaac ROS AprilTag..."
colcon build \
    --packages-select \
        isaac_ros_apriltag \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

###############################################################################
# Stage 21: Isaac ROS Visual SLAM
###############################################################################
log_info "Stage 21: Building Isaac ROS Visual SLAM..."
colcon build \
    --packages-select \
        isaac_ros_visual_slam \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

###############################################################################
# Stage 22: Nvblox Core Packages
###############################################################################
log_info "Stage 22: Building Nvblox core packages..."
colcon build \
    --packages-select \
        nvblox_msgs \
        nvblox_ros_common \
        nvblox_ros_python_utils \
        nvblox_image_padding \
        nvblox_rviz_plugin \
        semantic_label_conversion \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

###############################################################################
# Stage 23: Nvblox Main Package
###############################################################################
log_info "Stage 23: Building Nvblox main packages..."
colcon build \
    --packages-select \
        nvblox_ros \
        nvblox_nav2 \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

###############################################################################
# Stage 24: Isaac ROS Nvblox
###############################################################################
log_info "Stage 24: Building Isaac ROS Nvblox..."
colcon build \
    --packages-select \
        isaac_ros_peoplenet_models_install \
        isaac_ros_peoplesemseg_models_install \
        isaac_ros_nvblox \
        nvblox_examples_bringup \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

###############################################################################
# Stage 25: Isaac ROS Nova - Core
###############################################################################
log_info "Stage 25: Building Isaac ROS Nova core..."
colcon build \
    --packages-select \
        gxf_isaac_timestamp_correlator \
        isaac_ros_correlated_timestamp_driver \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

###############################################################################
# Stage 26: Isaac ROS Nova - Sensors
###############################################################################
log_info "Stage 26: Building Isaac ROS Nova sensors..."
colcon build \
    --packages-select \
        gxf_isaac_hesai \
        gxf_isaac_argus \
        gxf_isaac_bmi088_imu \
        isaac_ros_hawk \
        isaac_ros_owl \
        isaac_ros_hesai \
        isaac_ros_imu_bmi088 \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

###############################################################################
# Stage 27: Isaac ROS Nova - Data Pipeline
###############################################################################
log_info "Stage 27: Building Isaac ROS Nova data pipeline..."
colcon build \
    --packages-select \
        isaac_ros_data_recorder \
        isaac_ros_data_replayer \
        isaac_ros_data_validation \
        isaac_ros_ground_calibration \
        isaac_ros_nova_recorder \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

###############################################################################
# Stage 28: Isaac ROS Nova - Main
###############################################################################
log_info "Stage 28: Building Isaac ROS Nova main..."
colcon build \
    --packages-select \
        isaac_ros_nova \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

###############################################################################
# Stage 29: Isaac ROS NITROS Bridge
###############################################################################
log_info "Stage 29: Building Isaac ROS NITROS Bridge..."
colcon build \
    --packages-select \
        isaac_ros_nitros_bridge_ros2 \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

###############################################################################
# Stage 30: ZED ROS 2 Wrapper
###############################################################################
log_info "Stage 30: Building ZED ROS 2 Wrapper..."
colcon build \
    --packages-select \
        zed_msgs \
        zed_components \
        zed_wrapper \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

###############################################################################
# Stage 31: 残りのパッケージ
###############################################################################
log_info "Stage 31: Building remaining packages..."
colcon build \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

log_info "Fixed staged build completed successfully!"
log_info "All Isaac ROS packages have been built."
