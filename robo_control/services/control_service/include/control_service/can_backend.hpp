#pragma once

#include "can_sdk/master.hpp"
#include "control_service/actuator_backend.hpp"
#include "control_service/config_types.hpp"
#include <memory>
#include <vector>

namespace control_service {

class CANBackend : public ActuatorBackend {
public:
  explicit CANBackend(const BusConfig &config);
  ~CANBackend() override;

  bool init() override;
  void start() override;
  void stop() override;

  void read() override;
  const std::vector<BackendState> &states() const override { return states_; }
  void write(const std::vector<BackendCommand> &commands) override;

private:
  BusConfig config_;
  std::unique_ptr<can_sdk::CanMaster> master_;
  std::vector<ActuatorInfo> slot_info_; // aligned with config_.actuators
  std::vector<BackendState> states_;    // aligned with config_.actuators
  bool mit_mode_active_ = false;
};

} // namespace control_service
