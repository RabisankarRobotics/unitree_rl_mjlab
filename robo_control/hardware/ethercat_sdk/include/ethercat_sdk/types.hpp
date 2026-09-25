#pragma once

#include <cmath>
#include <cstdint>
#include <string>

namespace ethercat_sdk {

// Operation modes
enum class OperationMode : int8_t {
  CSP = 8,  // Cyclic Synchronous Position (RxPDO 0x1600)
  CSV = 9,  // Cyclic Synchronous Velocity (RxPDO 0x1600)
  CST = 10, // Cyclic Synchronous Torque (RxPDO 0x1600)
  PVT = 5   // Position-Velocity-Torque with KP/KD (RxPDO 0x1601)
};

// CiA 402 drive state machine states
enum class DriveState {
  NotReadyToSwitchOn,
  SwitchOnDisabled,
  ReadyToSwitchOn,
  SwitchedOn,
  OperationEnabled,
  QuickStopActive,
  FaultReactionActive,
  Fault
};

// RxPDO 0x1600 - Standard Mode (16 bytes)
struct RxPDO_Standard {
  uint16_t control_word;    // 0x6040
  int32_t target_position;  // 0x607A
  int32_t target_velocity;  // 0x60FF
  int16_t target_torque;    // 0x6071
  uint16_t max_torque;      // 0x6072
  int8_t mode_of_operation; // 0x6060
  uint8_t _pad;             // 0x5FFE
} __attribute__((packed));

// RxPDO 0x1601 - PVT Mode (24 bytes)
struct RxPDO_PVT {
  uint16_t control_word;    // 0x6040
  int32_t target_position;  // 0x607A
  int32_t target_velocity;  // 0x60FF
  int16_t target_torque;    // 0x6071
  int32_t kp;               // 0x2000
  int32_t kd;               // 0x2001
  int8_t mode_of_operation; // 0x6060
  uint8_t _pad;
} __attribute__((packed));

// TxPDO 0x1A00 - Standard Mode Feedback (16 bytes)
struct TxPDO_Standard {
  uint16_t status_word;    // 0x6041
  int32_t actual_position; // 0x6064
  int32_t actual_velocity; // 0x606C
  int16_t actual_torque;   // 0x6077
  uint16_t error_code;     // 0x603F
  int8_t mode_display;     // 0x6061
  uint8_t _pad;            // 0x5FFE
} __attribute__((packed));

// TxPDO 0x1A01 - PVT Mode Feedback (16 bytes, same layout as 0x1A00)
struct TxPDO_PVT {
  uint16_t status_word;    // 0x6041
  int32_t actual_position; // 0x6064
  int32_t actual_velocity; // 0x606C
  int16_t actual_torque;   // 0x6077
  uint16_t error_code;     // 0x603F
  int8_t mode_display;     // 0x6061
  uint8_t _pad;            // 0x5FFE
} __attribute__((packed));

// Per-slave topology info as discovered by SOEM on the wire.
// Indices here are 1-based bus positions (auto-increment address order).
struct SlaveTopologyInfo {
  int bus_id;              // 1-based chain position
  uint16_t config_address; // fixed station address (0x1000 + i)
  uint16_t alias_address;  // configured station alias (from SII)
  uint32_t vendor_id;      // SII 0x0008 (eep_man)
  uint32_t product_code;   // SII 0x000A (eep_id)
  uint32_t revision;       // SII 0x000C (eep_rev)
  uint32_t serial;         // SII 0x000E (eep_ser)
  std::string name;        // name from SII / config
  uint8_t topology;        // 1..3 active links
  uint8_t active_ports;    // bitmask of active ESC ports (bit n = port n)
  uint8_t consumed_ports;  // bitmask of ports used by the frame path
  uint16_t parent;         // parent slave bus_id (0 = master)
  uint8_t parent_port;     // port on parent we came from
  uint8_t entry_port;      // port on this slave the parent connects to
};

// Actuator state - returned to user
struct ActuatorState {
  double position;       // rad
  double velocity;       // rad/s
  double torque;         // Nm
  double motor_temp;     // °C - not in cyclic PDO, reads 0 (SDO 0x2009)
  double voltage;        // V - not in cyclic PDO, reads 0 (SDO 0x200A)
  int32_t first_encoder; // not in cyclic PDO, reads 0
  uint16_t error_code;
  DriveState drive_state;
  OperationMode mode;
  bool enabled;
  bool fault;
  bool lost; // communication lost
};

// CiA 402 control word bits
namespace ControlWord {
constexpr uint16_t SwitchOn = 0x0001;
constexpr uint16_t EnableVoltage = 0x0002;
constexpr uint16_t QuickStop = 0x0004;
constexpr uint16_t EnableOperation = 0x0008;
constexpr uint16_t FaultReset = 0x0080;

// Common control word values
constexpr uint16_t Shutdown = 0x0006;       // Ready to switch on
constexpr uint16_t SwitchOnEnable = 0x0007; // Switched on
constexpr uint16_t EnableOp = 0x000F;       // Operation enabled
constexpr uint16_t DisableVoltage = 0x0000;
constexpr uint16_t QuickStopCmd = 0x0002;
} // namespace ControlWord

// CiA 402 status word bits
namespace StatusWord {
constexpr uint16_t ReadyToSwitchOn = 0x0001;
constexpr uint16_t SwitchedOn = 0x0002;
constexpr uint16_t OperationEnabled = 0x0004;
constexpr uint16_t Fault = 0x0008;
constexpr uint16_t VoltageEnabled = 0x0010;
constexpr uint16_t QuickStop = 0x0020;
constexpr uint16_t SwitchOnDisabled = 0x0040;
constexpr uint16_t Warning = 0x0080;
constexpr uint16_t TargetReached = 0x0400;
constexpr uint16_t InternalLimitAct = 0x0800;

// State mask for decoding
constexpr uint16_t StateMask = 0x006F;
} // namespace StatusWord

// Unit conversion constants - MyActuator specific
namespace Units {
// Position: 65535 counts = 180° = π rad
constexpr double COUNTS_PER_REV = 65535.0 * 2.0; // Full revolution
constexpr double COUNTS_PER_RAD = 65535.0 / M_PI;
constexpr double RAD_PER_COUNT = M_PI / 65535.0;

// Velocity: 131072 pulses/s = 60 RPM = 2π rad/s
constexpr double PULSES_PER_RPM = 131072.0 / 60.0;
constexpr double PULSES_PER_RAD_S = 131072.0 / (2.0 * M_PI);
constexpr double RAD_S_PER_PULSE = (2.0 * M_PI) / 131072.0;

// Torque: in 0.1% of rated current
constexpr double TORQUE_SCALE = 1000.0; // 1000 = 100% rated

// PVT gains: base of 1000 (raw 1000 = actual 1.0)
constexpr double KP_KD_SCALE = 1000.0;

// Voltage: raw value * 0.1 = volts
constexpr double VOLTAGE_SCALE = 0.1;

// Temperature: direct in °C
constexpr double TEMP_SCALE = 1.0;
} // namespace Units

// Parse status word to drive state
inline DriveState parseDriveState(uint16_t status_word) {
  uint16_t state = status_word & StatusWord::StateMask;

  if (status_word & StatusWord::Fault) {
    return DriveState::Fault;
  }

  // State decoding per CiA 402
  switch (state) {
  case 0x0000:
    return DriveState::NotReadyToSwitchOn;
  case 0x0040:
    return DriveState::SwitchOnDisabled;
  case 0x0021:
    return DriveState::ReadyToSwitchOn;
  case 0x0023:
    return DriveState::SwitchedOn;
  case 0x0027:
    return DriveState::OperationEnabled;
  case 0x0007:
    return DriveState::QuickStopActive;
  case 0x000F:
    return DriveState::FaultReactionActive;
  case 0x0008:
    return DriveState::Fault;
  default:
    return DriveState::NotReadyToSwitchOn;
  }
}

// Check if mode uses PVT PDO
inline bool isPVTMode(OperationMode mode) { return mode == OperationMode::PVT; }

} // namespace ethercat_sdk
