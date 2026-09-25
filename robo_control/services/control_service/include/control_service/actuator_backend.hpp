#pragma once

#include <vector>

namespace control_service {

struct BackendCommand {
  double position = 0.0;
  double velocity = 0.0;
  double torque = 0.0;
  double kp = 0.0;
  double kd = 0.0;
};

struct BackendState {
  double position = 0.0;
  double velocity = 0.0;
  double torque = 0.0;
  double temperature = 0.0;
  bool enabled = false;
  bool fault = false;
  bool lost = false;
};

// Shared actuator info used by all backend implementations. Backends work in
// raw motor radians; direction and zero_offset live on ActuatorConfig and are
// applied by RobotController, not here.
struct ActuatorInfo {
  int bus_id = 0;
};

// Slot-indexed backend interface. For a backend whose BusConfig lists N
// actuators, both `states()` and the `commands` vector passed to `write()` are
// of length N, indexed in the same order as BusConfig::actuators.
class ActuatorBackend {
public:
  virtual ~ActuatorBackend() = default;

  virtual bool init() = 0;
  virtual void start() = 0;
  virtual void stop() = 0;

  // Refresh internal state buffer from the bus.
  virtual void read() = 0;
  // Most recent state snapshot. Size equals BusConfig::actuators.size().
  virtual const std::vector<BackendState> &states() const = 0;
  // Push commands to the bus. Size must equal BusConfig::actuators.size().
  virtual void write(const std::vector<BackendCommand> &commands) = 0;
};

} // namespace control_service
