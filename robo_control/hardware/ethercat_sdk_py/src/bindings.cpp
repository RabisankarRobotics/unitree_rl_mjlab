#include <nanobind/nanobind.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "ethercat_sdk/config_types.hpp"
#include "ethercat_sdk/cycle_stats.hpp"
#include "ethercat_sdk/master.hpp"
#include "ethercat_sdk/types.hpp"

namespace nb = nanobind;
using namespace nb::literals;
using namespace ethercat_sdk;

NB_MODULE(_core, m) {
  m.doc() = "Python bindings for ethercat_sdk EtherCAT motor control library";

  // ---- Enums ----

  nb::enum_<OperationMode>(m, "OperationMode")
      .value("CSP", OperationMode::CSP, "Cyclic Synchronous Position")
      .value("CSV", OperationMode::CSV, "Cyclic Synchronous Velocity")
      .value("CST", OperationMode::CST, "Cyclic Synchronous Torque")
      .value("PVT", OperationMode::PVT, "Position-Velocity-Torque with KP/KD");

  nb::enum_<DriveState>(m, "DriveState")
      .value("NotReadyToSwitchOn", DriveState::NotReadyToSwitchOn)
      .value("SwitchOnDisabled", DriveState::SwitchOnDisabled)
      .value("ReadyToSwitchOn", DriveState::ReadyToSwitchOn)
      .value("SwitchedOn", DriveState::SwitchedOn)
      .value("OperationEnabled", DriveState::OperationEnabled)
      .value("QuickStopActive", DriveState::QuickStopActive)
      .value("FaultReactionActive", DriveState::FaultReactionActive)
      .value("Fault", DriveState::Fault);

  // ---- Structs ----

  nb::class_<ActuatorState>(m, "ActuatorState")
      .def(nb::init<>())
      .def_ro("position", &ActuatorState::position)
      .def_ro("velocity", &ActuatorState::velocity)
      .def_ro("torque", &ActuatorState::torque)
      .def_ro("motor_temp", &ActuatorState::motor_temp)
      .def_ro("voltage", &ActuatorState::voltage)
      .def_ro("first_encoder", &ActuatorState::first_encoder)
      .def_ro("error_code", &ActuatorState::error_code)
      .def_ro("drive_state", &ActuatorState::drive_state)
      .def_ro("mode", &ActuatorState::mode)
      .def_ro("enabled", &ActuatorState::enabled)
      .def_ro("fault", &ActuatorState::fault)
      .def_ro("lost", &ActuatorState::lost)
      .def("__repr__", [](const ActuatorState &s) {
        return "<ActuatorState pos=" + std::to_string(s.position) +
               " vel=" + std::to_string(s.velocity) +
               " tau=" + std::to_string(s.torque) +
               " enabled=" + (s.enabled ? "True" : "False") +
               " fault=" + (s.fault ? "True" : "False") + ">";
      });

  nb::class_<ActuatorType>(m, "ActuatorType")
      .def_ro("rated_torque_nm", &ActuatorType::rated_torque_nm)
      .def("__repr__", [](const ActuatorType &t) {
        return "<ActuatorType rated=" + std::to_string(t.rated_torque_nm) +
               "Nm>";
      });

  nb::class_<ActuatorConfig>(m, "ActuatorConfig")
      .def_ro("id", &ActuatorConfig::id)
      .def_ro("name", &ActuatorConfig::name)
      .def_ro("type", &ActuatorConfig::type)
      .def_ro("bus_id", &ActuatorConfig::bus_id)
      .def_ro("direction", &ActuatorConfig::direction)
      .def_ro("zero_offset_rad", &ActuatorConfig::zero_offset_rad);

  nb::class_<BusConfig>(m, "BusConfig")
      .def_ro("interface_name", &BusConfig::interface_name)
      .def_ro("loop_hz", &BusConfig::loop_hz)
      .def_ro("cycle_time_us", &BusConfig::cycle_time_us)
      .def_ro("actuators", &BusConfig::actuators);

  nb::class_<CycleTimingSnapshot>(m, "CycleTimingSnapshot")
      .def(nb::init<>())
      .def_ro("intended_wakeup_ns", &CycleTimingSnapshot::intended_wakeup_ns)
      .def_ro("wakeup_ns", &CycleTimingSnapshot::wakeup_ns)
      .def_ro("after_receive_ns", &CycleTimingSnapshot::after_receive_ns)
      .def_ro("after_txpdo_read_ns", &CycleTimingSnapshot::after_txpdo_read_ns)
      .def_ro("after_rxpdo_write_ns",
              &CycleTimingSnapshot::after_rxpdo_write_ns)
      .def_ro("after_send_ns", &CycleTimingSnapshot::after_send_ns)
      .def_ro("after_dc_sync_ns", &CycleTimingSnapshot::after_dc_sync_ns)
      .def_ro("cycle_end_ns", &CycleTimingSnapshot::cycle_end_ns)
      .def_ro("dc_offset_ns", &CycleTimingSnapshot::dc_offset_ns)
      .def_ro("wkc", &CycleTimingSnapshot::wkc)
      .def_ro("expected_wkc", &CycleTimingSnapshot::expected_wkc)
      .def_ro("cycle_count", &CycleTimingSnapshot::cycle_count);

  // ---- Exception ----

  nb::exception<InvalidActuatorTypeError>(m, "InvalidActuatorTypeError",
                                          PyExc_RuntimeError);

  // ---- EtherCATMaster ----

  nb::class_<EtherCATMaster>(m, "EtherCATMaster")
      .def(nb::init<const std::string &, int>(), "interface_name"_a,
           "cycle_time_us"_a = 1000)

      // Actuator type configuration
      .def("config_actuator_types", &EtherCATMaster::configActuatorTypes,
           "actuator_type_map"_a)
      .def("get_available_actuator_types",
           &EtherCATMaster::getAvailableActuatorTypes)
      .def("has_actuator_type", &EtherCATMaster::hasActuatorType,
           "type_name"_a)
      .def(
          "get_actuator_type",
          [](const EtherCATMaster &self, const std::string &name) {
            return self.getActuatorType(name);
          },
          "type_name"_a, nb::rv_policy::reference_internal,
          "Returns the ActuatorType for the given name, or None if not found.")

      // Operation mode
      .def("set_operation_mode", &EtherCATMaster::setOperationMode, "mode"_a)
      .def("get_operation_mode", &EtherCATMaster::getOperationMode)

      // Lifecycle (release GIL for blocking calls)
      .def("init", &EtherCATMaster::init,
           nb::call_guard<nb::gil_scoped_release>())
      .def("start", &EtherCATMaster::start,
           nb::call_guard<nb::gil_scoped_release>())
      .def("stop", &EtherCATMaster::stop,
           nb::call_guard<nb::gil_scoped_release>())

      // Status
      .def("get_slave_count", &EtherCATMaster::getSlaveCount)
      .def("is_running", &EtherCATMaster::isRunning)
      .def("is_operational", &EtherCATMaster::isOperational)
      .def("get_working_counter", &EtherCATMaster::getWorkingCounter)
      .def("get_expected_wkc", &EtherCATMaster::getExpectedWKC)

      // Actuator control
      .def("enable", &EtherCATMaster::enable, "actuator_id"_a)
      .def("disable", &EtherCATMaster::disable, "actuator_id"_a)
      .def("clear_fault", &EtherCATMaster::clearFault, "actuator_id"_a)
      .def("enable_all", &EtherCATMaster::enableAll)
      .def("disable_all", &EtherCATMaster::disableAll)

      // Command setters
      .def("set_command", &EtherCATMaster::setCommand, "actuator_id"_a,
           "position"_a, "velocity"_a, "torque"_a, "kp"_a = 0.0,
           "kd"_a = 0.0)
      .def("set_position", &EtherCATMaster::setPosition, "actuator_id"_a,
           "position"_a)
      .def("set_velocity", &EtherCATMaster::setVelocity, "actuator_id"_a,
           "velocity"_a)
      .def("set_torque", &EtherCATMaster::setTorque, "actuator_id"_a,
           "torque"_a)
      .def("set_max_torque", &EtherCATMaster::setMaxTorque, "actuator_id"_a,
           "max_torque"_a)
      .def(
          "set_pvt",
          [](EtherCATMaster &self, int actuator_id, double position, double kp,
             double kd, double velocity, double torque) {
            self.setPVT(actuator_id, position, velocity, torque, kp, kd);
          },
          "actuator_id"_a, "position"_a, "kp"_a, "kd"_a,
          "velocity"_a = 0.0, "torque"_a = 0.0)

      // State getter
      .def("get_actuator_state", &EtherCATMaster::getActuatorState,
           "actuator_id"_a)

      // Cycle timing
      .def(
          "get_cycle_stats",
          [](const EtherCATMaster &self)
              -> std::optional<CycleTimingSnapshot> {
            CycleTimingSnapshot stats;
            if (self.getCycleStats(stats))
              return stats;
            return std::nullopt;
          },
          "Returns the latest cycle timing snapshot, or None if unavailable.")

      // Config
      .def("get_config", &EtherCATMaster::getConfig,
           nb::rv_policy::reference_internal)

      // Context manager
      .def(
          "__enter__",
          [](EtherCATMaster &self) -> EtherCATMaster & {
            self.start();
            return self;
          },
          nb::rv_policy::reference)
      .def("__exit__",
           [](EtherCATMaster &self, nb::handle, nb::handle, nb::handle) {
             try {
               self.disableAll();
             } catch (...) {
             }
             self.stop();
           });
}
