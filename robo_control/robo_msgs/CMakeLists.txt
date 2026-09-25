cmake_minimum_required(VERSION 3.10)
project(robo_msgs)

# Suppress CMP0148 warning from ROS 2 Humble's rosidl_generator_py
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate message interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointCommand.msg"
  "msg/JointState.msg"
  "msg/IMUData.msg"
  "msg/Joystick.msg"
  "msg/RobotState.msg"
  "msg/PowerData.msg"
  "srv/ManageService.srv"
  "srv/CalibrateActuators.srv"
  "srv/GetActuatorOffsets.srv"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

