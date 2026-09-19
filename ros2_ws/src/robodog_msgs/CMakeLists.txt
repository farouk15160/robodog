cmake_minimum_required(VERSION 3.8)
project(robodog_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/JointCommand.msg
  msg/JointCommandArray.msg
  msg/JointTelemetry.msg
  msg/FootState.msg
  msg/SafetyStatus.msg
  msg/SimulationState.msg
  msg/ControllerState.msg
  msg/GaitCommand.msg
  msg/RobotState.msg
  srv/SetControlMode.srv
  srv/SetNamedPose.srv
  srv/SetGait.srv
  srv/EmergencyStop.srv
  srv/EnableJoints.srv
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()
