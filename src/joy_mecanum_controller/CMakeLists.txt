cmake_minimum_required(VERSION 3.8)
project(joy_mecanum_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
#find_package(voicevox_ros2_interface REQUIRED)

add_executable(joy_mecanum_controller_node src/joy_mecanum_controller.cpp)
ament_target_dependencies(joy_mecanum_controller_node rclcpp sensor_msgs geometry_msgs)

add_executable(joy_slam_maker_node src/joy_slam_maker.cpp)
ament_target_dependencies(joy_slam_maker_node rclcpp sensor_msgs geometry_msgs)

install(TARGETS
  joy_mecanum_controller_node
  joy_slam_maker_node
  DESTINATION lib/${PROJECT_NAME})

#-------motoms made ------------------------------  
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
#-------motoms made ------------------------------  

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
