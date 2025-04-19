cmake_minimum_required(VERSION 3.8)
project(pwm_pca9685)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

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

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Create executable from source files in src/ and stc/
add_executable(pca9685_node 
  src/pca9685_activity.cpp
  src/pca9685_node.cpp
)

ament_target_dependencies(pca9685_node
  rclcpp
  std_msgs
  sensor_msgs
)

# Link against the i2c library
target_link_libraries(pca9685_node -li2c)

add_executable(pwm_wave_node 
  src/pwm_wave_node.cpp
)

ament_target_dependencies(pwm_wave_node
  rclcpp
  std_msgs
)

add_executable(ctrl_node 
  src/ctrl_node.cpp
)

ament_target_dependencies(ctrl_node
  rclcpp
  std_msgs
  std_srvs
)

install(TARGETS 
  pca9685_node
  pwm_wave_node
  ctrl_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/ 
  DESTINATION share/${PROJECT_NAME}/config)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME})

ament_package()
