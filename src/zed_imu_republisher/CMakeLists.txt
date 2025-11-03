cmake_minimum_required(VERSION 3.8)
project(zed_imu_republisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(imu_republisher src/imu_republisher.cpp)
ament_target_dependencies(imu_republisher
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  imu_republisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()