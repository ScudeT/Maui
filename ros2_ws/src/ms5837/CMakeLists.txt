cmake_minimum_required(VERSION 3.8)
project(ms5837)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

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

include_directories(include)

add_executable(ms5837_node src/ms5837_node.cpp src/MS5837.cpp)
ament_target_dependencies(ms5837_node rclcpp sensor_msgs std_msgs  geometry_msgs)

add_executable(pose_node src/pose_node.cpp src/MS5837.cpp)
ament_target_dependencies(pose_node rclcpp geometry_msgs)

add_executable(depth_to_pose src/depth_to_pose.cpp)
ament_target_dependencies(depth_to_pose rclcpp sensor_msgs std_msgs geometry_msgs)

install(TARGETS
  ms5837_node
  depth_to_pose
  pose_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
