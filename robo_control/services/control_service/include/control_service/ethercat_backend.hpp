#pragma once

#include "control_service/actuator_backend.hpp"
#include "control_service/config_types.hpp"
#include "ethercat_sdk/master.hpp"
#include <memory>
#include <vector>

namespace control_service {

class EtherCATBackend : public ActuatorBackend {
public:
  explicit EtherCATBackend(const BusConfig &config);
  ~EtherCATBackend() override;

  bool init() override;
  void start() override;
  void stop() override;

  void read() override;
  const std::vector<BackendState> &states() const override { return states_; }
  void write(const std::vector<BackendCommand> &commands) override;

private:
  BusConfig config_;
  std::unique_ptr<ethercat_sdk::EtherCATMaster> master_;
  std::vector<ActuatorInfo> slot_info_; // aligned with config_.actuators
  std::vector<BackendState> states_;    // aligned with config_.actuators
};

} // namespace control_service
