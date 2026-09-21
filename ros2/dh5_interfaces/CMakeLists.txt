cmake_minimum_required(VERSION 3.5)
project(dh5_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetAxisValue.srv"
  "srv/Initialize.srv"
  "srv/SetValues.srv"
  "srv/MoveAxesPercent.srv"
  "srv/TwoFingerPinch.srv"
  "msg/AxisInfos.msg"
   DEPENDENCIES std_msgs builtin_interfaces
)

ament_package()
