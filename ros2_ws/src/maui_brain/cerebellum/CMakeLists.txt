cmake_minimum_required(VERSION 3.8)
project(cerebellum)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME})

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


add_executable(ctrl_node 
  src/ctrl_node.cpp
)

ament_target_dependencies(ctrl_node
  rclcpp
  std_msgs
  std_srvs
)

# PID controller
add_executable(pid_node src/pid_node.cpp)
ament_target_dependencies(pid_node
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  tf2
)

# Attitude controller
add_executable(attitude_ctrl_node src/attitude_ctrl_node.cpp)
ament_target_dependencies(attitude_ctrl_node
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  tf2
)

# Attitude controller devided
add_executable(w_ctrl_node src/w_ctrl_node.cpp)
ament_target_dependencies(w_ctrl_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  std_srvs
)
add_executable(q_ctrl_node src/q_ctrl_node.cpp)
ament_target_dependencies(q_ctrl_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
)

# rpy setpoint to quaternion
add_executable(ypr2q_node src/ypr2q_node.cpp)
ament_target_dependencies(ypr2q_node
  rclcpp
  std_msgs
  geometry_msgs
  tf2
)

add_executable(w_ctrl2_node src/w_ctrl2_node.cpp)
ament_target_dependencies(w_ctrl2_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  std_srvs
)
add_executable(to_bmf_node src/to_bmf_node.cpp)
ament_target_dependencies(to_bmf_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  std_srvs
)

install(TARGETS
  ctrl_node
  pid_node
  attitude_ctrl_node
  w_ctrl_node
  q_ctrl_node
  ypr2q_node
  w_ctrl2_node
  to_bmf_node
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY config/ 
  DESTINATION share/${PROJECT_NAME}/config)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)



ament_package()
