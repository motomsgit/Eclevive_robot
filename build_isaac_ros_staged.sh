
#!/bin/bash

###############################################################################
# Isaac ROS Staged Build Script
# パッケージを段階的にビルドします（メモリ不足対策）
###############################################################################

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
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
# Stage 2: Isaac ROS Common（コアパッケージ）
###############################################################################
log_info "Stage 2: Building Isaac ROS Common core packages..."
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
# Stage 3: Isaac ROS GXF Extensions（基本）
###############################################################################
log_info "Stage 3: Building Isaac ROS GXF Extensions (image pipeline)..."
colcon build \
    --packages-select \
        gxf_isaac_image_flip \
        gxf_isaac_tensorops \
        gxf_isaac_rectify \
        gxf_isaac_sgm \
        gxf_isaac_depth_image_proc \
    --metas colcon_isaac.meta \
    --symlink-install \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

###############################################################################
# Stage 4: Isaac ROS Image Pipeline
###############################################################################
log_info "Stage 4: Building Isaac ROS Image Pipeline..."
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
# Stage 5: Isaac ROS DNN Inference（GXF + 基本）
###############################################################################
log_info "Stage 5: Building Isaac ROS DNN Inference GXF extensions..."
colcon build \
    --packages-select \
        gxf_isaac_tensor_rt \
        gxf_isaac_triton \
    --metas colcon_isaac.meta \
    --symlink-install \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

log_info "Stage 5b: Building Isaac ROS DNN Inference base packages..."
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
# Stage 6: Isaac ROS Object Detection
###############################################################################
log_info "Stage 6: Building Isaac ROS Object Detection..."
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
# Stage 7: Isaac ROS Stereo Depth
###############################################################################
log_info "Stage 7: Building Isaac ROS DNN Stereo Depth..."
colcon build \
    --packages-select \
        isaac_ros_ess \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

###############################################################################
# Stage 8: Isaac ROS Image Segmentation
###############################################################################
log_info "Stage 8: Building Isaac ROS Image Segmentation..."
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
# Stage 9: Isaac ROS Pose Estimation
###############################################################################
log_info "Stage 9: Building Isaac ROS Pose Estimation..."
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
# Stage 10: Isaac ROS AprilTag
###############################################################################
log_info "Stage 10: Building Isaac ROS AprilTag..."
colcon build \
    --packages-select \
        isaac_ros_apriltag \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

###############################################################################
# Stage 11: Isaac ROS Visual SLAM
###############################################################################
log_info "Stage 11: Building Isaac ROS Visual SLAM..."
colcon build \
    --packages-select \
        isaac_ros_visual_slam_interfaces \
        isaac_ros_visual_slam \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

###############################################################################
# Stage 12: Isaac ROS Nvblox
###############################################################################
log_info "Stage 12: Building Isaac ROS Nvblox..."
colcon build \
    --packages-select \
        isaac_ros_nvblox \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

source install/setup.bash

###############################################################################
# Stage 13: Isaac ROS Nova
###############################################################################
log_info "Stage 13: Building Isaac ROS Nova..."
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
    --parallel-workers 1

source install/setup.bash

###############################################################################
# Stage 14: ZED ROS 2 Wrapper
###############################################################################
log_info "Stage 14: Building ZED ROS 2 Wrapper..."
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
# Stage 15: 残りのパッケージ
###############################################################################
log_info "Stage 15: Building remaining packages..."
colcon build \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

log_info "Staged build completed successfully!"
