#pragma once

#include <chrono>

namespace can_sdk {

struct MotionCommand {
  double position = 0.0;
  double velocity = 0.0;
  double torque = 0.0;
  double kp = 0.0;
  double kd = 0.0;
};

struct MotionFeedback {
  double position = 0.0;
  double velocity = 0.0;
  double torque = 0.0;
  std::chrono::steady_clock::time_point timestamp{};
};

struct MotorStatus {
  double position = 0.0;    // rad
  double velocity = 0.0;    // rad/s
  double torque = 0.0;      // Nm
  double temperature = 0.0; // Celsius
  std::chrono::steady_clock::time_point timestamp{};
};

} // namespace can_sdk
