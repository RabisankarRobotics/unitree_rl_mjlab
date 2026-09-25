#include "mode_service/mode_service.hpp"

#include <spdlog/spdlog.h>
#include <transport/node_context.hpp>
#include <yaml-cpp/yaml.h>

#include "common/config_loader.hpp"

namespace mode_service {

ModeService::ModeService() {}

ModeService::~ModeService() { stop(); }

void ModeService::loadParameters() {
  auto node = transport::NodeContext::instance().node();

  auto config_path = node->declare_parameter<std::string>("config_path");
  YAML::Node config = YAML::LoadFile(config_path);

  config_.loop_hz = common::get_config_value<int>(config, "loop_hz");
  config_.num_joints = common::get_config_value<int>(config, "num_joints");

  config_.ready_joint_pos =
      common::get_config_value<std::vector<float>>(config, "ready_joint_pos");
  config_.ready_kp =
      common::get_config_value<std::vector<float>>(config, "ready_kp");
  config_.ready_kd =
      common::get_config_value<std::vector<float>>(config, "ready_kd");
  config_.damping_kp =
      common::get_config_value<std::vector<float>>(config, "damping_kp");
  config_.damping_kd =
      common::get_config_value<std::vector<float>>(config, "damping_kd");
}

bool ModeService::init() {
  loadParameters();

  // Initialize position vectors with config size
  latest_position_.resize(config_.num_joints, 0.0f);
  ready_start_position_.resize(config_.num_joints, 0.0f);

  // Create publishers
  command_pub_ =
      std::make_unique<transport::Publisher<transport::JointCommand>>(
          transport::JOINT_COMMAND_TOPIC);
  if (!command_pub_->is_valid()) {
    spdlog::error("Failed to create joint command publisher");
    return false;
  }

  state_pub_ = std::make_unique<transport::Publisher<transport::RobotState>>(
      transport::ROBOT_STATE_TOPIC);
  if (!state_pub_->is_valid()) {
    spdlog::error("Failed to create robot state publisher");
    return false;
  }

  // Create subscribers
  joystick_sub_ = std::make_unique<transport::Subscriber<transport::Joystick>>(
      transport::JOYSTICK_TOPIC);
  if (!joystick_sub_->is_valid()) {
    spdlog::error("Failed to create joystick subscriber");
    return false;
  }
  joystick_sub_->set_callback(
      [this](const transport::Joystick &msg) { joystickCallback(msg); });

  joint_state_sub_ =
      std::make_unique<transport::Subscriber<transport::JointState>>(
          transport::JOINT_STATE_TOPIC);
  if (!joint_state_sub_->is_valid()) {
    spdlog::error("Failed to create joint state subscriber");
    return false;
  }
  joint_state_sub_->set_callback(
      [this](const transport::JointState &msg) { jointStateCallback(msg); });

  // Configure real-time thread
  common::RealtimeConfig rt_config;
  rt_config.frequency_hz = config_.loop_hz;
  rt_config.scheduler_policy = SCHED_FIFO;
  rt_config.scheduler_priority = 60;
  rt_config.cpu_affinity = -1;

  thread_loop_ = std::make_unique<common::ThreadLoop>(
      "ModeServiceLoop", rt_config, [this]() { loop(); });

  spdlog::info("ModeService initialized: loop_hz={}, num_joints={}",
               config_.loop_hz, config_.num_joints);
  return true;
}

void ModeService::start() {
  if (thread_loop_) {
    thread_loop_->start();
    spdlog::info("ModeService started");
  }
}

void ModeService::stop() {
  if (thread_loop_) {
    thread_loop_->stop();
    spdlog::info("ModeService stopped");
  }
  command_pub_.reset();
  state_pub_.reset();
  joystick_sub_.reset();
  joint_state_sub_.reset();
}

void ModeService::joystickCallback(const transport::Joystick &msg) {
  uint32_t buttons = msg.buttons;

  // Check for mode change button combos (LB + face button)
  if ((buttons & BUTTON_LB) == BUTTON_LB) {
    if ((buttons & BUTTON_X) == BUTTON_X) {
      handleModeChange(Mode::ZERO_TORQUE);
    } else if ((buttons & BUTTON_A) == BUTTON_A) {
      handleModeChange(Mode::DAMPING);
    } else if ((buttons & BUTTON_B) == BUTTON_B) {
      handleModeChange(Mode::READY);
    } else if ((buttons & BUTTON_Y) == BUTTON_Y) {
      handleModeChange(Mode::POLICY);
    } else if ((buttons & BUTTON_RB) == BUTTON_RB) {
      handleModeChange(Mode::DEBUG);
    }
  }
}

void ModeService::jointStateCallback(const transport::JointState &msg) {
  const auto &positions = msg.position;
  if (positions.size() != static_cast<size_t>(config_.num_joints)) {
    return;
  }

  std::lock_guard<std::mutex> lock(position_mutex_);
  latest_position_.assign(positions.begin(), positions.end());
}

void ModeService::handleModeChange(Mode new_mode) {
  Mode current = current_mode_.load();
  if (current == new_mode) {
    return;
  }

  // Special handling when entering READY mode
  if (new_mode == Mode::READY) {
    std::lock_guard<std::mutex> lock(position_mutex_);
    ready_start_position_ = latest_position_;
    ready_progress_ = 0.0;
    ready_target_reached_ = false;
  }

  current_mode_.store(new_mode);
  spdlog::info("Mode changed: {} -> {}", static_cast<int>(current),
               static_cast<int>(new_mode));
}

void ModeService::loop() {
  // Execute mode-specific logic
  switch (current_mode_.load()) {
  case Mode::ZERO_TORQUE:
    onZeroTorqueMode();
    break;
  case Mode::DAMPING:
    onDampingMode();
    break;
  case Mode::READY:
    onReadyMode();
    break;
  case Mode::POLICY:
    onPolicyMode();
    break;
  case Mode::DEBUG:
    onDebugMode();
    break;
  }

  // Publish current robot state
  transport::RobotState state_msg;
  state_msg.mode = static_cast<int8_t>(current_mode_.load());
  state_pub_->publish(state_msg);
}

void ModeService::publishCommand(const std::vector<float> &position,
                                 const std::vector<float> &kp,
                                 const std::vector<float> &kd) {
  transport::JointCommand cmd;
  cmd.position = position;
  cmd.velocity = std::vector<float>(config_.num_joints, 0.0f);
  cmd.torque = std::vector<float>(config_.num_joints, 0.0f);
  cmd.kp = kp;
  cmd.kd = kd;
  command_pub_->publish(cmd);
}

std::vector<float> ModeService::getLockedPosition() {
  std::lock_guard<std::mutex> lock(position_mutex_);
  return latest_position_;
}

void ModeService::onZeroTorqueMode() {
  std::vector<float> zero_kp(config_.num_joints, 0.0f);
  std::vector<float> zero_kd(config_.num_joints, 0.0f);
  publishCommand(getLockedPosition(), zero_kp, zero_kd);
}

void ModeService::onDampingMode() {
  publishCommand(getLockedPosition(), config_.damping_kp, config_.damping_kd);
}

void ModeService::onReadyMode() {
  // Ready mode: interpolate to ready position
  if (ready_target_reached_) {
    // Hold at ready position
    publishCommand(config_.ready_joint_pos, config_.ready_kp, config_.ready_kd);
    return;
  }

  // Increment progress (0.01 per cycle = ~1 second to complete at 100Hz)
  ready_progress_ += 0.01;
  if (ready_progress_ >= 1.0) {
    ready_progress_ = 1.0;
    ready_target_reached_ = true;
    spdlog::info("Ready position reached");
  }

  // Linear interpolation between start and target
  std::vector<float> interpolated_pos(config_.num_joints);
  for (int i = 0; i < config_.num_joints; ++i) {
    float start = ready_start_position_[i];
    float target = config_.ready_joint_pos[i];
    interpolated_pos[i] =
        start + static_cast<float>(ready_progress_) * (target - start);
  }

  publishCommand(interpolated_pos, config_.ready_kp, config_.ready_kd);
}

void ModeService::onPolicyMode() {}

void ModeService::onDebugMode() {}

} // namespace mode_service
