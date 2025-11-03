cmake_minimum_required(VERSION 3.5)
project(zed_goal_publisher)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
#find_package(zed_interfaces REQUIRED)
find_package(zed_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


set(TARGET action_client)

add_executable(zed_goal_publisher src/zed_goal_publisher.cpp)

ament_target_dependencies(zed_goal_publisher rclcpp
                                            rclcpp_action 
                                            std_msgs 
                                            #zed_interfaces 
                                            zed_msgs
                                            geometry_msgs 
                                            nav2_msgs 
                                            tf2 
                                            tf2_ros 
                                            tf2_geometry_msgs 
                                            sensor_msgs 
                                            )

# 手を上げた人物追従ノード
add_executable(hand_raised_person_follower src/hand_raised_person_follower.cpp)

ament_target_dependencies(hand_raised_person_follower
                          rclcpp
                          std_msgs
                          zed_msgs
                          geometry_msgs
                          tf2
                          tf2_ros
                          )

install(TARGETS
zed_goal_publisher
hand_raised_person_follower
  DESTINATION lib/${PROJECT_NAME})

# launchファイルのインストール
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# configファイルのインストール
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
