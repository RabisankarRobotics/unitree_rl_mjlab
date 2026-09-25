#pragma once

#include <chrono>

namespace transport {

constexpr uint32_t DEFAULT_DOMAIN_ID = 0;

// Topic names (ROS2 convention)
constexpr const char *JOINT_COMMAND_TOPIC = "robo/joint_command";
constexpr const char *JOINT_STATE_TOPIC = "robo/joint_state";
constexpr const char *IMU_DATA_TOPIC = "robo/imu_data";
constexpr const char *JOYSTICK_TOPIC = "robo/joystick";
constexpr const char *ROBOT_STATE_TOPIC = "robo/robot_state";
constexpr const char *POWER_TOPIC = "robo/power";

// Helper function to get current timestamp in nanoseconds
inline int64_t get_timestamp_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

} // namespace transport
