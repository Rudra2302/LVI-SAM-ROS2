cmake_minimum_required(VERSION 3.8)
project(lvi_sam_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CloudInfo.msg"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
