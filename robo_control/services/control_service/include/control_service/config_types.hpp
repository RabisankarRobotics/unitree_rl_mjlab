#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace control_service {

enum class CalibrationReference { Zero, Lower, Upper };

struct ActuatorConfig {
  int id;
  std::string name;
  std::string type;
  int bus_id;
  int direction;
  double zero_offset_rad = 0.0;
  double lower_limit_rad = 0.0;
  double upper_limit_rad = 0.0;
  CalibrationReference calibration_reference = CalibrationReference::Zero;
};

enum class BusType { ETHERCAT, CAN };

struct BusConfig {
  BusType type;
  std::string interface_name;
  int loop_hz;
  int cycle_time_us;
  int control_cpu_affinity = -1;
  int ethercat_cpu_affinity = -1;
  std::vector<ActuatorConfig> actuators;
};

enum class JointSourceKind { Passthrough, Transmission };

struct JointSourceConfig {
  JointSourceKind kind = JointSourceKind::Passthrough;
  int actuator_id = 0;
  std::string transmission_name;
  std::size_t slot = 0;
};

struct JointConfig {
  std::string name;
  double lower_limit_rad;
  double upper_limit_rad;
  JointSourceConfig source;
};

struct TransmissionGroupConfig {
  std::string name;
  std::string type;
  std::vector<int> actuator_ids;
};

struct HardwareConfig {
  std::string robot_type;
  std::vector<BusConfig> buses;
  std::vector<JointConfig> joints;
  std::vector<TransmissionGroupConfig> transmissions;
  double clip_warn_tolerance_rad = 0.0;
};

} // namespace control_service
