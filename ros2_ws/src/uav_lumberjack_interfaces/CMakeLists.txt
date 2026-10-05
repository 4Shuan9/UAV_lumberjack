cmake_minimum_required(VERSION 3.8)
project(uav_lumberjack_interfaces)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BranchModel.msg"
  "msg/ArmStatus.msg"
  "action/ArmMotion.action"
  "srv/SetSaw.srv"
  DEPENDENCIES
  std_msgs
  geometry_msgs
  action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
