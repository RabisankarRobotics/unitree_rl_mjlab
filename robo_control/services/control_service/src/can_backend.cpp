#include "control_service/can_backend.hpp"
#include <spdlog/spdlog.h>

// Backend I/O is raw motor radians: no direction flip, no zero_offset applied.
// RobotController owns all joint-space<->motor-space math for both passthrough
// and transmission slots.

namespace control_service {

CANBackend::CANBackend(const BusConfig &config) : config_(config) {
  slot_info_.reserve(config_.actuators.size());
  for (const auto &act : config_.actuators) {
    ActuatorInfo info;
    info.bus_id = act.bus_id;
    slot_info_.push_back(info);
  }
  states_.resize(config_.actuators.size());
}

CANBackend::~CANBackend() { stop(); }

bool CANBackend::init() {
  master_ = std::make_unique<can_sdk::CanMaster>();
  if (!master_->init(config_.interface_name)) {
    spdlog::error("Failed to initialize CAN master on interface {}",
                  config_.interface_name);
    return false;
  }

  for (const auto &info : slot_info_) {
    master_->addJoint(static_cast<uint16_t>(info.bus_id));
  }

  return true;
}

void CANBackend::start() {}

void CANBackend::stop() {
  mit_mode_active_ = false;
  if (master_) {
    master_->close();
  }
}

void CANBackend::read() {
  if (!master_) return;

  // When not in MIT mode, send motor status requests to get current state
  if (!mit_mode_active_) {
    for (const auto &info : slot_info_) {
      master_->requestMotorStatus(static_cast<uint16_t>(info.bus_id));
    }
  }

  // Receive all pending frames
  master_->recvAll(100);

  for (std::size_t slot = 0; slot < slot_info_.size(); ++slot) {
    const auto &info = slot_info_[slot];
    auto &state = states_[slot];
    state.enabled = true;
    state.fault = false;

    if (mit_mode_active_) {
      auto feedback = master_->getMotionFeedback(info.bus_id);
      state.position = feedback.position;
      state.velocity = feedback.velocity;
      state.torque = feedback.torque;
      state.temperature = 0.0;
      state.lost = master_->isMotionFeedbackStale(info.bus_id, 10000);
    } else {
      auto status = master_->getMotorStatus(info.bus_id);
      state.position = status.position;
      state.velocity = status.velocity;
      state.torque = status.torque;
      state.temperature = status.temperature;
      state.lost = master_->isMotorStatusStale(info.bus_id, 10000);
    }
  }
}

void CANBackend::write(const std::vector<BackendCommand> &commands) {
  if (!master_ || commands.empty()) return;

  mit_mode_active_ = true;

  for (std::size_t slot = 0; slot < slot_info_.size() && slot < commands.size();
       ++slot) {
    const auto &info = slot_info_[slot];
    const auto &cmd = commands[slot];

    can_sdk::MotionCommand motion_cmd;
    motion_cmd.position = cmd.position;
    motion_cmd.velocity = cmd.velocity;
    motion_cmd.torque = cmd.torque;
    motion_cmd.kp = cmd.kp;
    motion_cmd.kd = cmd.kd;

    master_->sendMotionCommand(static_cast<uint16_t>(info.bus_id), motion_cmd);
  }
}

} // namespace control_service
