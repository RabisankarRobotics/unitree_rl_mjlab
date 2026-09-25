#pragma once

#include <string>
#include <vector>

namespace ethercat_sdk {

// Actuator Type Definition (hardcoded table lives in master.cpp).
// The type name is the lookup key in the table; only per-type parameters
// that the SDK actually consumes belong here.
struct ActuatorType {
  double rated_torque_nm;
};

// Instance Configuration (from hardware.yaml)
struct ActuatorConfig {
  int id;
  std::string name;
  std::string type;
  int bus_id;
  int direction;
  double zero_offset_rad;
};

// Bus Configuration
struct BusConfig {
  std::string interface_name;
  int loop_hz;
  int cycle_time_us;
  std::vector<ActuatorConfig> actuators;
};

} // namespace ethercat_sdk
