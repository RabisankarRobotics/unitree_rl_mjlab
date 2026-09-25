#pragma once

#include "ethercat_sdk/config_types.hpp"
#include "ethercat_sdk/cycle_stats.hpp"
#include "ethercat_sdk/types.hpp"
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <vector>

namespace ethercat_sdk {

class InvalidActuatorTypeError : public std::runtime_error {
public:
  explicit InvalidActuatorTypeError(const std::string &type_name)
      : std::runtime_error("Unknown actuator type: '" + type_name + "'") {}
};

class EtherCATMaster {
public:
  explicit EtherCATMaster(const std::string &interface_name,
                          int cycle_time_us = 1000);
  ~EtherCATMaster();

  // Non-copyable
  EtherCATMaster(const EtherCATMaster &) = delete;
  EtherCATMaster &operator=(const EtherCATMaster &) = delete;

  void configActuatorTypes(const std::map<int, std::string> &actuator_type_map);

  std::vector<std::string> getAvailableActuatorTypes() const;
  bool hasActuatorType(const std::string &type_name) const;
  const ActuatorType *getActuatorType(const std::string &type_name) const;

  void setOperationMode(OperationMode mode);
  OperationMode getOperationMode() const;

  // Real-time thread placement. Call before start(); -1 disables affinity.
  void setRealtimeCpu(int cpu_affinity);

  // Lifecycle
  bool init();
  void start();
  void stop();

  // Status
  int getSlaveCount() const;
  bool isRunning() const;
  bool isOperational() const;
  int getWorkingCounter() const;
  int getExpectedWKC() const;

  // Topology - valid after init()
  std::vector<SlaveTopologyInfo> getTopology() const;

  // SII EEPROM station alias access - valid after init(), before start().
  // The parameter is a raw chain position (1..slave_count), NOT a configured
  // bus_id: these run before any id->slave mapping exists and are intended
  // for provisioning. Reads live from EEPROM (word 4). Writes update word 4
  // and recompute the SII CRC at word 7. The new alias takes effect only
  // after the slave is power-cycled.
  std::optional<uint16_t> readAlias(int chain_position);
  bool writeAlias(int chain_position, uint16_t alias);

  // Actuator control - thread-safe
  void enable(int actuator_id);
  void disable(int actuator_id);
  void clearFault(int actuator_id);
  void enableAll();
  void disableAll();

  // Command setters - thread-safe
  void setCommand(int actuator_id, double position, double velocity,
                  double torque, double kp = 0.0, double kd = 0.0);
  void setPosition(int actuator_id, double position);
  void setVelocity(int actuator_id, double velocity);
  void setTorque(int actuator_id, double torque);
  void setMaxTorque(int actuator_id, double max_torque);
  void setPVT(int actuator_id, double position, double velocity, double torque,
              double kp, double kd);

  // State getter - thread-safe
  ActuatorState getActuatorState(int actuator_id) const;

  // Cycle timing - thread-safe, lock-free read
  bool getCycleStats(CycleTimingSnapshot &stats) const;

  const BusConfig &getConfig() const { return bus_config_; }

private:
  struct MasterImpl;

  BusConfig bus_config_;
  std::map<int, std::string> actuator_type_map_; // bus_id -> type_name

  std::unique_ptr<MasterImpl> impl_;

  std::map<int, int> id_to_slave_index_;
};

} // namespace ethercat_sdk
