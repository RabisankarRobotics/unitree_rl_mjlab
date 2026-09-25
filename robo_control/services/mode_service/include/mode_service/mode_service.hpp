#pragma once

#include <common/thread_loop.hpp>
#include <transport/transport.hpp>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace mode_service {

struct ModeServiceConfig {
  int loop_hz;
  int num_joints;

  std::vector<float> ready_joint_pos;
  std::vector<float> ready_kp;
  std::vector<float> ready_kd;
  std::vector<float> damping_kp;
  std::vector<float> damping_kd;
};

enum class Mode : int8_t {
  ZERO_TORQUE = 0,
  DAMPING = 1,
  READY = 2,
  POLICY = 3,
  DEBUG = 4,
};

class ModeService {
public:
  ModeService();
  ~ModeService();

  ModeService(const ModeService &) = delete;
  ModeService &operator=(const ModeService &) = delete;

  bool init();
  void start();
  void stop();

  Mode current_mode() const { return current_mode_.load(); }

private:
  void loadParameters();
  void loop();
  void joystickCallback(const transport::Joystick &msg);
  void jointStateCallback(const transport::JointState &msg);
  void handleModeChange(Mode new_mode);

  // Mode handlers
  void onZeroTorqueMode();
  void onDampingMode();
  void onReadyMode();
  void onPolicyMode();
  void onDebugMode();

  // Helpers
  void publishCommand(const std::vector<float> &position,
                      const std::vector<float> &kp,
                      const std::vector<float> &kd);
  std::vector<float> getLockedPosition();

  // Configuration
  ModeServiceConfig config_;

  // Threading
  std::unique_ptr<common::ThreadLoop> thread_loop_;

  // Transport
  std::unique_ptr<transport::Publisher<transport::JointCommand>> command_pub_;
  std::unique_ptr<transport::Publisher<transport::RobotState>> state_pub_;
  std::unique_ptr<transport::Subscriber<transport::Joystick>> joystick_sub_;
  std::unique_ptr<transport::Subscriber<transport::JointState>>
      joint_state_sub_;

  // State
  std::atomic<Mode> current_mode_{Mode::ZERO_TORQUE};
  mutable std::mutex position_mutex_;
  std::vector<float> latest_position_;
  std::vector<float> ready_start_position_;
  double ready_progress_{0.0};
  bool ready_target_reached_{false};

  // Joystick button bitmasks (Xbox-style layout)
  static constexpr uint32_t BUTTON_A = 1u << 0;
  static constexpr uint32_t BUTTON_B = 1u << 1;
  static constexpr uint32_t BUTTON_X = 1u << 3;
  static constexpr uint32_t BUTTON_Y = 1u << 2;
  static constexpr uint32_t BUTTON_LB = 1u << 4;
  static constexpr uint32_t BUTTON_RB = 1u << 5;
};

} // namespace mode_service
