cmake_minimum_required(VERSION 3.8)
project(parietal_lobe)

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
find_package(tf2_geometry_msgs REQUIRED)

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

add_executable(imu_bar_odom src/imu_bar_odom.cpp)
ament_target_dependencies(imu_bar_odom 
  rclcpp 
  sensor_msgs 
  geometry_msgs
  nav_msgs 
  tf2 
  tf2_ros
  tf2_geometry_msgs
)

add_executable(joint_pose_node src/joint_pose_node.cpp)
ament_target_dependencies(joint_pose_node 
  rclcpp 
  std_msgs
  sensor_msgs 
)

add_executable(imu_change_ref src/imu_change_ref_node.cpp)
ament_target_dependencies(imu_change_ref 
  rclcpp 
  sensor_msgs 
  tf2 
  tf2_ros
  tf2_geometry_msgs
)

add_executable(gps_to_local 
  src/gps_to_local.cpp
)
ament_target_dependencies(gps_to_local
   rclcpp
   sensor_msgs
   nav_msgs
   geometry_msgs
)

add_executable(local_to_gps
  src/local_to_gps.cpp
)

ament_target_dependencies(local_to_gps
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  )

install(TARGETS
  imu_bar_odom
  joint_pose_node
  imu_change_ref
  gps_to_local
  local_to_gps
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config/ 
  DESTINATION share/${PROJECT_NAME}/config)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

  install(DIRECTORY description/
  DESTINATION share/${PROJECT_NAME}/description)

ament_package()
