#pragma once

#include "publisher.hpp"
#include "subscriber.hpp"
#include "types.hpp"

// ROS2 generated messages
#include <robo_msgs/msg/imu_data.hpp>
#include <robo_msgs/msg/joint_command.hpp>
#include <robo_msgs/msg/joint_state.hpp>
#include <robo_msgs/msg/joystick.hpp>
#include <robo_msgs/msg/power_data.hpp>
#include <robo_msgs/msg/robot_state.hpp>

namespace transport {

// Typedefs for cleaner usage
using JointCommand = robo_msgs::msg::JointCommand;
using JointState = robo_msgs::msg::JointState;
using IMUData = robo_msgs::msg::IMUData;
using Joystick = robo_msgs::msg::Joystick;
using RobotState = robo_msgs::msg::RobotState;
using PowerData = robo_msgs::msg::PowerData;

} // namespace transport
