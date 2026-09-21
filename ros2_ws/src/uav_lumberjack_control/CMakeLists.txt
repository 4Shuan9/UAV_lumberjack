cmake_minimum_required(VERSION 3.8)
project(uav_lumberjack_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

add_executable(
  arm_j1_test
  src/arm_j1_test.cpp
)

add_executable(
  arm_controller
  src/arm_controller.cpp
)

add_executable(
  target_contact_monitor
  src/target_contact_monitor.cpp
)

add_executable(
  auto_cut_controller
  src/auto_cut_controller.cpp
)

ament_target_dependencies(
  arm_j1_test
  rclcpp
  std_msgs
)

ament_target_dependencies(
  arm_controller
  rclcpp
  std_msgs
  sensor_msgs
)

ament_target_dependencies(
  target_contact_monitor
  rclcpp
  std_msgs
  ros_gz_interfaces
)

ament_target_dependencies(
  auto_cut_controller
  rclcpp
  std_msgs
  sensor_msgs
)

install(
  TARGETS
    arm_j1_test
    arm_controller
    target_contact_monitor
    auto_cut_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
