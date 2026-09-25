#include "control_service/ethercat_backend.hpp"
#include <map>
#include <spdlog/spdlog.h>

// Backend I/O is raw motor radians: no direction flip, no zero_offset applied.
// RobotController owns all joint-space<->motor-space math for both passthrough
// and transmission slots.

namespace control_service {

EtherCATBackend::EtherCATBackend(const BusConfig &config) : config_(config) {
  slot_info_.reserve(config_.actuators.size());
  for (const auto &act : config_.actuators) {
    ActuatorInfo info;
    info.bus_id = act.bus_id;
    slot_info_.push_back(info);
  }
  states_.resize(config_.actuators.size());
}

EtherCATBackend::~EtherCATBackend() { stop(); }

bool EtherCATBackend::init() {
  try {
    master_ = std::make_unique<ethercat_sdk::EtherCATMaster>(
        config_.interface_name, config_.cycle_time_us);
    master_->setRealtimeCpu(config_.ethercat_cpu_affinity);

    // Configure actuator type mapping (bus_id -> type name)
    std::map<int, std::string> type_map;
    for (const auto &act : config_.actuators) {
      type_map[act.bus_id] = act.type;
    }
    master_->configActuatorTypes(type_map);

    if (!master_->init()) {
      spdlog::error("Failed to initialize EtherCAT master on interface {}",
                    config_.interface_name);
      return false;
    }

    master_->setOperationMode(ethercat_sdk::OperationMode::PVT);

    return true;
  } catch (const std::exception &e) {
    spdlog::error("EtherCATBackend init exception: {}", e.what());
    return false;
  }
}

void EtherCATBackend::start() {
  if (master_) {
    master_->start();
    master_->enableAll();
  }
}

void EtherCATBackend::stop() {
  if (master_) {
    master_->disableAll();
    master_->stop();
  }
}

void EtherCATBackend::read() {
  if (!master_) return;

  for (std::size_t slot = 0; slot < slot_info_.size(); ++slot) {
    const auto &info = slot_info_[slot];
    auto ec_state = master_->getActuatorState(info.bus_id);

    auto &state = states_[slot];
    state.position = ec_state.position;
    state.velocity = ec_state.velocity;
    state.torque = ec_state.torque;
    state.temperature = ec_state.motor_temp;
    state.enabled = ec_state.enabled;
    state.fault = ec_state.fault;
    state.lost = ec_state.lost;
  }
}

void EtherCATBackend::write(const std::vector<BackendCommand> &commands) {
  if (!master_) return;

  for (std::size_t slot = 0; slot < slot_info_.size() && slot < commands.size();
       ++slot) {
    const auto &info = slot_info_[slot];
    const auto &cmd = commands[slot];
    master_->setCommand(info.bus_id, cmd.position, cmd.velocity, cmd.torque,
                        cmd.kp, cmd.kd);
  }
}

} // namespace control_service
