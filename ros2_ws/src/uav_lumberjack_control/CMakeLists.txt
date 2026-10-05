cmake_minimum_required(VERSION 3.8)
project(uav_lumberjack_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(uav_lumberjack_interfaces REQUIRED)

add_library(
  arm_kinematics
  src/arm_kinematics.cpp
)

target_include_directories(
  arm_kinematics
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(
  arm_kinematics_cli
  src/arm_kinematics_cli.cpp
)
target_link_libraries(arm_kinematics_cli arm_kinematics)

add_executable(
  arm_controller
  src/arm_controller.cpp
)
target_link_libraries(arm_controller arm_kinematics)
ament_target_dependencies(
  arm_controller
  rclcpp
  rclcpp_action
  std_msgs
  sensor_msgs
  geometry_msgs
  uav_lumberjack_interfaces
)

add_executable(
  target_contact_monitor
  src/target_contact_monitor.cpp
)
ament_target_dependencies(
  target_contact_monitor
  rclcpp
  std_msgs
  ros_gz_interfaces
)

add_executable(
  auto_cut_controller
  src/auto_cut_controller.cpp
)
ament_target_dependencies(
  auto_cut_controller
  rclcpp
  std_msgs
  sensor_msgs
)

install(
  TARGETS
    arm_kinematics
    arm_kinematics_cli
    arm_controller
    target_contact_monitor
    auto_cut_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

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
