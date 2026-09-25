#pragma once

#include "mnn_policy.hpp"
#include "observations.hpp"

#include <common/thread_loop.hpp>
#include <transport/transport.hpp>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace policy_service {

struct VelocityRange {
  float min;
  float max;
};

struct VelocityCommandRanges {
  VelocityRange lin_vel_x{-1.0f, 1.0f};
  VelocityRange lin_vel_y{-1.0f, 1.0f};
  VelocityRange ang_vel_z{-1.0f, 1.0f};
};

struct PolicyConfig {
  std::string policy_path;
  int num_joints;
  int loop_hz;
  std::vector<float> action_scale;
  std::vector<float> kp;
  std::vector<float> kd;
  std::vector<float> default_joint_pos;
  std::vector<int> policy_to_actuator_map;

  // Observation configuration
  int obs_history_length;
  std::vector<std::string> obs_terms;

  // Gait phase parameters (used when "gait_phase" is in obs_terms).
  float phase_period_s;

  // Velocity command ranges (applied to normalized joystick input).
  VelocityCommandRanges vel_cmd_ranges;
};

class PolicyService {
public:
  PolicyService();
  ~PolicyService();

  // Non-copyable, non-movable
  PolicyService(const PolicyService &) = delete;
  PolicyService &operator=(const PolicyService &) = delete;

  bool init();
  void start();
  void stop();

private:
  void loadParameters();
  void controlLoop();

  // Message handlers
  void handleJointState(const transport::JointState &msg);
  void handleImuData(const transport::IMUData &msg);
  void handleJoystick(const transport::Joystick &msg);
  void handleRobotState(const transport::RobotState &msg);

  // Build per-term observation vectors in configured order
  std::vector<std::vector<float>> buildObservationTerms();

  // Publish scaled actions as joint commands
  void publishActions(const std::vector<float> &actions);

  PolicyConfig policy_config_;
  std::unique_ptr<MnnPolicy> policy_;
  std::unique_ptr<ObservationHistory> obs_history_;

  // Transport
  std::unique_ptr<transport::Publisher<transport::JointCommand>> command_pub_;
  std::unique_ptr<transport::Subscriber<transport::JointState>>
      joint_state_sub_;
  std::unique_ptr<transport::Subscriber<transport::IMUData>> imu_data_sub_;
  std::unique_ptr<transport::Subscriber<transport::Joystick>> joystick_sub_;
  std::unique_ptr<transport::Subscriber<transport::RobotState>>
      robot_state_sub_;

  // Thread loop
  std::unique_ptr<common::ThreadLoop> thread_loop_;

  // State (protected by mutex, pre-allocated in init())
  std::mutex obs_mutex_;
  Observations observations_;
  std::vector<float> prev_actions_;

  // Gait phase state (advanced each policy tick when gait_phase is enabled).
  float phase_{0.0f};
  float phase_increment_{0.0f};
  bool has_gait_phase_{false};

  // Atomic flags
  std::atomic<bool> policy_mode_{false};
  std::atomic<bool> has_joint_state_{false};
  std::atomic<bool> has_imu_data_{false};
};

} // namespace policy_service
