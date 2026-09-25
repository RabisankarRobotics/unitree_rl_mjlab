#include "ethercat_sdk/master.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <memory>
#include <pthread.h>
#include <sched.h>
#include <thread>
#include <vector>

extern "C" {
#include <soem/soem.h>
}

#include <spdlog/spdlog.h>

#include "common/buffer.hpp"
#include "common/log_throttle.hpp"

namespace ethercat_sdk {

namespace {
constexpr int EC_TIMEOUTMON = 500;
constexpr int MIN_CYCLIC_RX_TIMEOUT_US = 50;
constexpr int64_t NSEC_PER_SEC = 1000000000;
constexpr int ETHERCAT_RT_PRIORITY = 90;


const std::map<std::string, ActuatorType> &getActuatorTypeTable() {
  static const std::map<std::string, ActuatorType> table = {
      {"X4-36", ActuatorType{.rated_torque_nm = 10.5}},
      {"X6-60", ActuatorType{.rated_torque_nm = 20.0}},
      {"X8-120", ActuatorType{.rated_torque_nm = 43.0}},
      {"X12-320", ActuatorType{.rated_torque_nm = 85.0}},
  };
  return table;
}

void addTimeNs(ec_timet *ts, int64_t add_ns) {
  ec_timet add_ts;
  add_ts.tv_nsec = add_ns % NSEC_PER_SEC;
  add_ts.tv_sec = (add_ns - add_ts.tv_nsec) / NSEC_PER_SEC;
  osal_timespecadd(ts, &add_ts, ts);
}

int cyclicReceiveTimeoutUs(int64_t cycle_time_ns) {
  const int cycle_us = static_cast<int>(cycle_time_ns / 1000);
  return std::max(MIN_CYCLIC_RX_TIMEOUT_US, cycle_us / 4);
}

void setCurrentThreadRealtime(const char *name, int priority,
                              int cpu_affinity) {
  pthread_setname_np(pthread_self(), name);

  sched_param param{};
  param.sched_priority = priority;
  int ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
  if (ret != 0) {
    spdlog::warn("Failed to set RT scheduling for thread '{}': {} ({})", name,
                 std::strerror(ret), ret);
  } else {
    spdlog::info("Set RT scheduling for thread '{}': policy={}, priority={}",
                 name, SCHED_FIFO, priority);
  }

  if (cpu_affinity >= 0) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_affinity, &cpuset);
    ret = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
    if (ret != 0) {
      spdlog::warn("Failed to set CPU affinity for thread '{}': {} ({})",
                   name, std::strerror(ret), ret);
    } else {
      spdlog::info("Set CPU affinity for thread '{}' to CPU {}", name,
                   cpu_affinity);
    }
  }
}

} // namespace

// Per-slave actuator data with thread-safe state
struct SlaveData {
  ActuatorConfig config;
  ActuatorType type_params;
  double rated_torque_nm{1.0};

  // Command state (atomic for thread safety)
  std::atomic<bool> enable_requested{false};
  std::atomic<bool> disable_requested{false};
  std::atomic<bool> fault_reset_requested{false};

  // Standard mode commands
  std::atomic<double> cmd_position{0.0};
  std::atomic<double> cmd_velocity{0.0};
  std::atomic<double> cmd_torque{0.0};
  std::atomic<double> cmd_max_torque{10.0};

  // PVT mode commands
  std::atomic<double> cmd_kp{0.0};
  std::atomic<double> cmd_kd{0.0};

  // Current state (atomic for thread safety)
  std::atomic<double> state_position{0.0};
  std::atomic<double> state_velocity{0.0};
  std::atomic<double> state_torque{0.0};
  std::atomic<double> state_temp{0.0};
  std::atomic<double> state_voltage{0.0};
  std::atomic<int32_t> state_first_encoder{0};
  std::atomic<uint16_t> state_error_code{0};
  std::atomic<uint16_t> state_status_word{0};
  std::atomic<int8_t> state_mode_display{0};
  std::atomic<bool> state_lost{false};

  // State machine
  DriveState current_drive_state{DriveState::SwitchOnDisabled};

  // Unit conversions
  int32_t radToPosition(double rad) const {
    return static_cast<int32_t>(rad * Units::COUNTS_PER_RAD);
  }

  double positionToRad(int32_t counts) const {
    return static_cast<double>(counts) * Units::RAD_PER_COUNT;
  }

  int32_t radSToVelocity(double rad_s) const {
    return static_cast<int32_t>(rad_s * Units::PULSES_PER_RAD_S);
  }

  double velocityToRadS(int32_t pulses) const {
    return static_cast<double>(pulses) * Units::RAD_S_PER_PULSE;
  }

  int16_t nmToTorque(double nm) const {
    return static_cast<int16_t>((nm / rated_torque_nm) * Units::TORQUE_SCALE);
  }

  double torqueToNm(int16_t raw) const {
    return (static_cast<double>(raw) / Units::TORQUE_SCALE) * rated_torque_nm;
  }

  // Compute control word for state transitions
  uint16_t computeControlWord() {
    // Handle fault reset request
    if (fault_reset_requested.load()) {
      if (current_drive_state == DriveState::Fault) {
        return ControlWord::FaultReset;
      }
      fault_reset_requested.store(false);
    }

    // Handle disable request
    if (disable_requested.load()) {
      return ControlWord::DisableVoltage;
    }

    // Handle enable request - step through state machine
    if (enable_requested.load()) {
      switch (current_drive_state) {
      case DriveState::SwitchOnDisabled:
        return ControlWord::Shutdown;

      case DriveState::ReadyToSwitchOn:
        return ControlWord::SwitchOnEnable;

      case DriveState::SwitchedOn:
        return ControlWord::EnableOp;

      case DriveState::OperationEnabled:
        // Already enabled, maintain state
        return ControlWord::EnableOp;

      case DriveState::QuickStopActive:
        return ControlWord::DisableVoltage;

      case DriveState::Fault:
        // Need fault reset first
        return ControlWord::FaultReset;

      default:
        return ControlWord::DisableVoltage;
      }
    }

    // Default: maintain current state
    if (current_drive_state == DriveState::OperationEnabled) {
      return ControlWord::EnableOp;
    }

    return ControlWord::DisableVoltage;
  }

  // Update state from Standard mode PDO
  void updateStateFromPDO_Standard(const TxPDO_Standard *pdo, int direction,
                                   double zero_offset) {
    state_status_word.store(pdo->status_word);
    double pos = positionToRad(pdo->actual_position);
    state_position.store((pos * direction) - zero_offset);
    state_velocity.store(velocityToRadS(pdo->actual_velocity) * direction);
    state_torque.store(torqueToNm(pdo->actual_torque) * direction);
    state_error_code.store(pdo->error_code);
    state_mode_display.store(pdo->mode_display);

    current_drive_state = parseDriveState(pdo->status_word);
  }

  // Update state from PVT mode PDO
  void updateStateFromPDO_PVT(const TxPDO_PVT *pdo, int direction,
                              double zero_offset) {
    state_status_word.store(pdo->status_word);
    double pos = positionToRad(pdo->actual_position);
    state_position.store((pos * direction) - zero_offset);
    state_velocity.store(velocityToRadS(pdo->actual_velocity) * direction);
    state_torque.store(torqueToNm(pdo->actual_torque) * direction);
    state_error_code.store(pdo->error_code);
    state_mode_display.store(pdo->mode_display);

    current_drive_state = parseDriveState(pdo->status_word);
  }

  // Apply commands to Standard mode PDO
  void applyCommandToPDO_Standard(RxPDO_Standard *pdo, int direction,
                                  double zero_offset, OperationMode op_mode) {
    double cmd_pos = (cmd_position.load() + zero_offset) * direction;
    double cmd_vel = cmd_velocity.load() * direction;
    double cmd_tau = cmd_torque.load() * direction;

    pdo->control_word = computeControlWord();
    pdo->target_position = radToPosition(cmd_pos);
    pdo->target_velocity = radSToVelocity(cmd_vel);
    pdo->target_torque = nmToTorque(cmd_tau);
    pdo->max_torque = static_cast<uint16_t>(
        (cmd_max_torque.load() / rated_torque_nm) * Units::TORQUE_SCALE);
    pdo->mode_of_operation = static_cast<int8_t>(op_mode);
    pdo->_pad = 0;
  }

  // Apply commands to PVT mode PDO
  void applyCommandToPDO_PVT(RxPDO_PVT *pdo, int direction, double zero_offset,
                             OperationMode op_mode) {
    double cmd_pos = (cmd_position.load() + zero_offset) * direction;
    double cmd_vel = cmd_velocity.load() * direction;
    double cmd_tau = cmd_torque.load() * direction;

    pdo->control_word = computeControlWord();
    pdo->target_position = radToPosition(cmd_pos);
    pdo->target_velocity = radSToVelocity(cmd_vel);
    pdo->target_torque = nmToTorque(cmd_tau);
    pdo->kp = static_cast<int32_t>(cmd_kp.load() * Units::KP_KD_SCALE);
    pdo->kd = static_cast<int32_t>(cmd_kd.load() * Units::KP_KD_SCALE);
    pdo->mode_of_operation = static_cast<int8_t>(op_mode);
    pdo->_pad = 0;
  }

  ActuatorState getState() const {
    ActuatorState state;
    state.position = state_position.load();
    state.velocity = state_velocity.load();
    state.torque = state_torque.load();
    state.motor_temp = state_temp.load();
    state.voltage = state_voltage.load();
    state.first_encoder = state_first_encoder.load();
    state.error_code = state_error_code.load();
    state.mode = static_cast<OperationMode>(state_mode_display.load());
    state.lost = state_lost.load();

    uint16_t sw = state_status_word.load();
    state.drive_state = parseDriveState(sw);
    state.enabled = (state.drive_state == DriveState::OperationEnabled);
    state.fault = (state.drive_state == DriveState::Fault);

    return state;
  }

  void markLost() { state_lost.store(true); }
  void markRecovered() { state_lost.store(false); }
};

// Implementation structure - hides SOEM details
struct EtherCATMaster::MasterImpl {
  std::string ifname;
  ecx_contextt ctx{};
  uint8_t io_map[4096]{};

  int slave_count{0};
  int expected_wkc{0};
  int64_t cycle_time_ns{1000000}; // 1ms default
  int realtime_cpu{-1};
  OperationMode op_mode{OperationMode::CSP};

  std::vector<std::unique_ptr<SlaveData>> slaves;

  std::atomic<bool> running{false};
  std::atomic<bool> in_op{false};
  std::atomic<int> wkc{0};
  std::atomic<int> wkc_errors{0};

  std::thread rt_thread;
  std::thread error_thread;

  // DC sync state
  int64_t dc_integral{0};
  static constexpr float DC_PGAIN = 0.01f;
  static constexpr float DC_IGAIN = 0.00002f;
  static constexpr int64_t DC_SYNC_OFFSET = 500000; // 500us

  // Cycle timing instrumentation
  common::TripleBuffer<CycleTimingSnapshot> cycle_stats_buf;
  uint64_t cycle_counter{0};

  void rtLoop();
  void errorLoop();
  void configureSlaves();
  void transitionToOp();
  void dcSync(int64_t ref_time, int64_t cycle_time, int64_t *offset);
};

// SDO write helpers
static bool sdoWrite8(ecx_contextt *ctx, uint16_t slave, uint16_t index,
                      uint8_t subindex, uint8_t value) {
  int wkc = ecx_SDOwrite(ctx, slave, index, subindex, FALSE, sizeof(value),
                         &value, EC_TIMEOUTRXM);
  return wkc > 0;
}

static bool sdoWrite16(ecx_contextt *ctx, uint16_t slave, uint16_t index,
                       uint8_t subindex, uint16_t value) {
  int wkc = ecx_SDOwrite(ctx, slave, index, subindex, FALSE, sizeof(value),
                         &value, EC_TIMEOUTRXM);
  return wkc > 0;
}

EtherCATMaster::EtherCATMaster(const std::string &interface_name,
                               int cycle_time_us)
    : impl_(std::make_unique<MasterImpl>()) {

  bus_config_.interface_name = interface_name;
  bus_config_.cycle_time_us = cycle_time_us;

  impl_->ifname = interface_name;
  impl_->cycle_time_ns = cycle_time_us * 1000;
}

void EtherCATMaster::configActuatorTypes(
    const std::map<int, std::string> &actuator_type_map) {
  if (impl_->running.load()) {
    throw std::runtime_error(
        "Cannot configure actuator types while master is running");
  }

  const auto &table = getActuatorTypeTable();
  for (const auto &[bus_id, type_name] : actuator_type_map) {
    if (table.find(type_name) == table.end()) {
      throw InvalidActuatorTypeError(type_name);
    }
  }

  actuator_type_map_ = actuator_type_map;
  spdlog::info("Configured {} actuator type mappings",
               actuator_type_map_.size());
}

std::vector<std::string> EtherCATMaster::getAvailableActuatorTypes() const {
  const auto &table = getActuatorTypeTable();
  std::vector<std::string> types;
  types.reserve(table.size());
  for (const auto &[name, _] : table) {
    types.push_back(name);
  }
  return types;
}

bool EtherCATMaster::hasActuatorType(const std::string &type_name) const {
  const auto &table = getActuatorTypeTable();
  return table.find(type_name) != table.end();
}

const ActuatorType *
EtherCATMaster::getActuatorType(const std::string &type_name) const {
  const auto &table = getActuatorTypeTable();
  auto it = table.find(type_name);
  return (it != table.end()) ? &it->second : nullptr;
}

EtherCATMaster::~EtherCATMaster() { stop(); }

void EtherCATMaster::setOperationMode(OperationMode mode) {
  if (impl_->running.load()) {
    spdlog::warn("Cannot change operation mode while running");
    return;
  }
  impl_->op_mode = mode;
}

OperationMode EtherCATMaster::getOperationMode() const {
  return impl_->op_mode;
}

void EtherCATMaster::setRealtimeCpu(int cpu_affinity) {
  if (impl_->running.load()) {
    spdlog::warn("Cannot change EtherCAT RT CPU affinity while running");
    return;
  }
  impl_->realtime_cpu = cpu_affinity;
}

bool EtherCATMaster::init() {
  spdlog::info("Initializing EtherCAT on {}", impl_->ifname);

  // Initialize SOEM
  if (ecx_init(&impl_->ctx, impl_->ifname.c_str()) <= 0) {
    spdlog::error("Failed to initialize interface {}", impl_->ifname);
    return false;
  }

  // Scan for slaves
  if (ecx_config_init(&impl_->ctx) <= 0) {
    spdlog::error("No slaves found on {}", impl_->ifname);
    return false;
  }

  impl_->slave_count = impl_->ctx.slavecount;
  spdlog::info("Found {} slaves", impl_->slave_count);

  // Create slave data objects
  impl_->slaves.clear();
  for (int i = 0; i < impl_->slave_count; ++i) {
    impl_->slaves.push_back(std::make_unique<SlaveData>());
  }

  // Build alias_to_chain: aliasadr -> 0-based chain index. Skip slaves with
  // aliasadr == 0 (unaliased). Duplicate non-zero aliases are a hard error.
  std::map<uint16_t, int> alias_to_chain;
  for (int i = 0; i < impl_->slave_count; ++i) {
    uint16_t alias = impl_->ctx.slavelist[i + 1].aliasadr;
    if (alias == 0) continue;
    auto [it, inserted] = alias_to_chain.emplace(alias, i);
    if (!inserted) {
      spdlog::error("Alias 0x{:04X} appears on two slaves (chain {} and {})",
                    alias, it->second + 1, i + 1);
      return false;
    }
  }

  // Resolve each configured bus_id:
  //   1. alias match  -> bind bus_id to the slave carrying alias == bus_id
  //   2. chain fallback -> bind bus_id to chain position bus_id (bus_id in 1..N)
  //   3. error -> neither works
  // Reject chain-fallback collisions against already-bound (by alias) indices.
  std::map<int, int> chain_to_bus_id; // chain_index -> bus_id for collision checks

  // Resolve alias matches first so fallbacks can detect collisions.
  struct Resolved {
    int bus_id;
    int chain_index;
    std::string source; // "alias" or "chain-fallback"
  };
  std::vector<Resolved> resolved;
  resolved.reserve(actuator_type_map_.size());

  // Pass 1: alias matches.
  for (const auto &[bus_id, _] : actuator_type_map_) {
    auto it = alias_to_chain.find(static_cast<uint16_t>(bus_id));
    if (it == alias_to_chain.end()) continue;
    resolved.push_back({bus_id, it->second, "alias"});
    chain_to_bus_id[it->second] = bus_id;
  }

  // Pass 2: chain fallback for any bus_id not matched by alias.
  for (const auto &[bus_id, _] : actuator_type_map_) {
    bool already_resolved = false;
    for (const auto &r : resolved) {
      if (r.bus_id == bus_id) { already_resolved = true; break; }
    }
    if (already_resolved) continue;

    if (bus_id < 1 || bus_id > impl_->slave_count) {
      spdlog::error(
          "bus_id {} configured but no slave has alias={} and chain "
          "position {} does not exist (slave_count={})",
          bus_id, bus_id, bus_id, impl_->slave_count);
      return false;
    }
    int chain_index = bus_id - 1;
    auto collide = chain_to_bus_id.find(chain_index);
    if (collide != chain_to_bus_id.end()) {
      spdlog::error(
          "bus_id {} chain-fallback collides with bus_id {} already bound "
          "via alias at chain position {}",
          bus_id, collide->second, chain_index + 1);
      return false;
    }
    resolved.push_back({bus_id, chain_index, "chain-fallback"});
    chain_to_bus_id[chain_index] = bus_id;
  }

  // Apply bindings and populate per-slave config.
  const auto &table = getActuatorTypeTable();
  for (const auto &r : resolved) {
    const std::string &type_name = actuator_type_map_.at(r.bus_id);
    auto actuator_it = table.find(type_name);
    if (actuator_it == table.end()) {
      spdlog::error("Unknown actuator type '{}' for bus_id {}", type_name,
                    r.bus_id);
      return false;
    }

    ActuatorConfig cfg;
    cfg.id = r.bus_id;
    cfg.bus_id = r.bus_id;
    cfg.name = "actuator_" + std::to_string(r.bus_id);
    cfg.type = type_name;
    cfg.direction = 1;
    cfg.zero_offset_rad = 0.0;

    impl_->slaves[r.chain_index]->config = cfg;
    impl_->slaves[r.chain_index]->type_params = actuator_it->second;
    impl_->slaves[r.chain_index]->rated_torque_nm =
        actuator_it->second.rated_torque_nm;

    id_to_slave_index_[r.bus_id] = r.chain_index;
  }

  // Resolution table - mandatory diagnostic.
  spdlog::info("resolved EtherCAT slaves:");
  spdlog::info("  bus_id  alias   chain  configaddr  source");
  for (const auto &r : resolved) {
    const auto &s = impl_->ctx.slavelist[r.chain_index + 1];
    spdlog::info("  {:>6}  0x{:04X}  {:>5}      0x{:04X}  {}", r.bus_id,
                 s.aliasadr, r.chain_index + 1, s.configadr, r.source);
  }

  // Warn about configured bus_ids that were not resolved? Already errored above.
  // Warn about slaves on the bus that nobody asked for.
  for (int i = 0; i < impl_->slave_count; ++i) {
    if (chain_to_bus_id.find(i) == chain_to_bus_id.end()) {
      const auto &s = impl_->ctx.slavelist[i + 1];
      spdlog::warn("chain position {} (alias=0x{:04X}, configaddr=0x{:04X}) "
                   "is on the bus but not bound to any configured bus_id",
                   i + 1, s.aliasadr, s.configadr);
    }
  }

  return true;
}

void EtherCATMaster::MasterImpl::configureSlaves() {
  // Transition to PRE_OP for SDO configuration
  for (int i = 1; i <= slave_count; i++) {
    ctx.slavelist[i].state = EC_STATE_PRE_OP;
    ecx_writestate(&ctx, i);
    ecx_statecheck(&ctx, i, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
  }

  // Configure PDO mapping for all slaves based on operation mode
  bool use_pvt = isPVTMode(op_mode);

  for (int i = 1; i <= slave_count; i++) {
    // Configure SM2 (RxPDO) assignment via 0x1C12
    sdoWrite8(&ctx, i, 0x1C12, 0x00, 0);
    if (use_pvt) {
      sdoWrite16(&ctx, i, 0x1C12, 0x01, 0x1601);
    } else {
      sdoWrite16(&ctx, i, 0x1C12, 0x01, 0x1600);
    }
    sdoWrite8(&ctx, i, 0x1C12, 0x00, 1);

    // Configure SM3 (TxPDO) assignment via 0x1C13
    sdoWrite8(&ctx, i, 0x1C13, 0x00, 0);
    if (use_pvt) {
      sdoWrite16(&ctx, i, 0x1C13, 0x01, 0x1A01);
    } else {
      sdoWrite16(&ctx, i, 0x1C13, 0x01, 0x1A00);
    }
    sdoWrite8(&ctx, i, 0x1C13, 0x00, 1);
  }

  // Configure distributed clocks
  ecx_configdc(&ctx);

  // Configure DC SYNC0 for all slaves that support DC
  for (int i = 1; i <= slave_count; i++) {
    ec_slavet *slave = &ctx.slavelist[i];
    if (slave->hasdc) {
      ecx_dcsync0(&ctx, i, TRUE, static_cast<uint32_t>(cycle_time_ns), 0);
    }
  }

  // Map all slaves to IOmap
  int io_size = ecx_config_map_group(&ctx, io_map, 0);
  if (io_size > static_cast<int>(sizeof(io_map))) {
    spdlog::error("IOmap buffer overflow: {} > {}", io_size, sizeof(io_map));
  }

  // Calculate expected working counter
  ec_groupt *group = &ctx.grouplist[0];
  expected_wkc = (group->outputsWKC * 2) + group->inputsWKC;
  spdlog::info("Expected WKC: {}", expected_wkc);

  // Add CoE slaves to cyclic mailbox handler
  for (int i = 1; i <= slave_count; i++) {
    ec_slavet *slave = &ctx.slavelist[i];
    if (slave->CoEdetails > 0) {
      ecx_slavembxcyclic(&ctx, i);
    }
  }
}

void EtherCATMaster::MasterImpl::transitionToOp() {
  for (int i = 1; i <= slave_count; i++) {
    ctx.slavelist[i].state = EC_STATE_OPERATIONAL;
    ecx_writestate(&ctx, i);
    int ret = ecx_statecheck(&ctx, i, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
    if (ret != EC_STATE_OPERATIONAL) {
      spdlog::error("Slave {} failed to reach OPERATIONAL state (state={})", i,
                    ctx.slavelist[i].state);
    }
  }

  in_op.store(true);
  spdlog::info("All slaves transitioned to OPERATIONAL");
}

void EtherCATMaster::start() {
  if (impl_->running.load()) {
    return;
  }

  impl_->configureSlaves();

  impl_->running.store(true);

  // Start RT thread
  impl_->rt_thread = std::thread(&MasterImpl::rtLoop, impl_.get());

  // Wait for mapping to complete
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Transition to operational
  impl_->transitionToOp();

  // Start error handling thread
  impl_->error_thread = std::thread(&MasterImpl::errorLoop, impl_.get());

  spdlog::info("EtherCAT master started");
}

void EtherCATMaster::stop() {
  if (!impl_->running.load()) {
    return;
  }

  spdlog::info("Stopping EtherCAT master...");

  impl_->running.store(false);
  impl_->in_op.store(false);

  // Wait for threads to finish
  if (impl_->rt_thread.joinable()) {
    impl_->rt_thread.join();
  }
  if (impl_->error_thread.joinable()) {
    impl_->error_thread.join();
  }

  // Graceful shutdown: transition through SAFE_OP -> INIT
  impl_->ctx.slavelist[0].state = EC_STATE_SAFE_OP;
  ecx_writestate(&impl_->ctx, 0);
  ecx_statecheck(&impl_->ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

  impl_->ctx.slavelist[0].state = EC_STATE_INIT;
  ecx_writestate(&impl_->ctx, 0);
  ecx_statecheck(&impl_->ctx, 0, EC_STATE_INIT, EC_TIMEOUTSTATE);

  ecx_close(&impl_->ctx);

  spdlog::info("EtherCAT master stopped");
}

void EtherCATMaster::MasterImpl::rtLoop() {
  setCurrentThreadRealtime("EtherCATRT", ETHERCAT_RT_PRIORITY, realtime_cpu);

  ec_timet ts;
  int64_t toff = 0;
  const int rx_timeout_us = cyclicReceiveTimeoutUs(cycle_time_ns);
  const int64_t late_warn_threshold_us =
      std::max<int64_t>(250, cycle_time_ns / 2000);
  common::LogThrottle late_warn_throttle(std::chrono::seconds(1));
  spdlog::info("EtherCAT cyclic receive timeout: {} us", rx_timeout_us);

  // Wait for initialization
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  // Get initial time, round to nearest ms
  osal_get_monotonic_time(&ts);
  int ht = (ts.tv_nsec / 1000000) + 1;
  ts.tv_nsec = ht * 1000000;

  // Initial send
  ecx_send_processdata(&ctx);

  bool use_pvt = isPVTMode(op_mode);

  auto to_ns = [](std::chrono::steady_clock::time_point tp) -> int64_t {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               tp.time_since_epoch())
        .count();
  };

  while (running.load()) {
    // Calculate next cycle start
    addTimeNs(&ts, cycle_time_ns + toff);
    int64_t intended_ns = ts.tv_sec * NSEC_PER_SEC + ts.tv_nsec;

    // Wait for cycle start
    osal_monotonic_sleep(&ts);
    auto t_wakeup = std::chrono::steady_clock::now();
    const int64_t wakeup_late_us = (to_ns(t_wakeup) - intended_ns) / 1000;
    if (wakeup_late_us > late_warn_threshold_us) {
      late_warn_throttle.warn("EtherCATRT wakeup late by: {} us",
                              wakeup_late_us);
    }

    // Receive process data from previous cycle
    int current_wkc = ecx_receive_processdata(&ctx, rx_timeout_us);
    wkc.store(current_wkc);
    auto t_after_recv = std::chrono::steady_clock::now();

    // Check working counter
    if (current_wkc != expected_wkc) {
      wkc_errors.fetch_add(1);
    } else {
      wkc_errors.store(0);
    }

    // Update actuator states from TxPDO
    for (int i = 1; i <= slave_count; i++) {
      ec_slavet *slave = &ctx.slavelist[i];
      auto &actuator = *slaves[i - 1];
      int direction = actuator.config.direction;
      double zero_offset = actuator.config.zero_offset_rad;

      if (slave->inputs != nullptr) {
        if (use_pvt) {
          auto *pdo = reinterpret_cast<const TxPDO_PVT *>(slave->inputs);
          actuator.updateStateFromPDO_PVT(pdo, direction, zero_offset);
        } else {
          auto *pdo = reinterpret_cast<const TxPDO_Standard *>(slave->inputs);
          actuator.updateStateFromPDO_Standard(pdo, direction, zero_offset);
        }
      }
    }
    auto t_after_txpdo = std::chrono::steady_clock::now();

    // Apply actuator commands to RxPDO
    for (int i = 1; i <= slave_count; i++) {
      ec_slavet *slave = &ctx.slavelist[i];
      auto &actuator = *slaves[i - 1];
      int direction = actuator.config.direction;
      double zero_offset = actuator.config.zero_offset_rad;

      if (slave->outputs != nullptr) {
        if (use_pvt) {
          auto *pdo = reinterpret_cast<RxPDO_PVT *>(slave->outputs);
          actuator.applyCommandToPDO_PVT(pdo, direction, zero_offset, op_mode);
        } else {
          auto *pdo = reinterpret_cast<RxPDO_Standard *>(slave->outputs);
          actuator.applyCommandToPDO_Standard(pdo, direction, zero_offset,
                                              op_mode);
        }
      }
    }
    auto t_after_rxpdo = std::chrono::steady_clock::now();

    // Send process data
    ecx_send_processdata(&ctx);
    auto t_after_send = std::chrono::steady_clock::now();

    // DC synchronization
    if (ctx.slavelist[0].hasdc && current_wkc > 0) {
      dcSync(ctx.DCtime, cycle_time_ns, &toff);
    }
    auto t_after_dc = std::chrono::steady_clock::now();

    // Handle mailbox (for SDO access from other threads)
    ecx_mbxhandler(&ctx, 0, 4);
    auto t_end = std::chrono::steady_clock::now();

    // Publish cycle timing snapshot
    auto *snap = cycle_stats_buf.get_write_buffer();
    snap->intended_wakeup_ns = intended_ns;
    snap->wakeup_ns = to_ns(t_wakeup);
    snap->after_receive_ns = to_ns(t_after_recv);
    snap->after_txpdo_read_ns = to_ns(t_after_txpdo);
    snap->after_rxpdo_write_ns = to_ns(t_after_rxpdo);
    snap->after_send_ns = to_ns(t_after_send);
    snap->after_dc_sync_ns = to_ns(t_after_dc);
    snap->cycle_end_ns = to_ns(t_end);
    snap->dc_offset_ns = toff;
    snap->wkc = current_wkc;
    snap->expected_wkc = expected_wkc;
    snap->cycle_count = cycle_counter++;
    cycle_stats_buf.push();
  }
}

void EtherCATMaster::MasterImpl::dcSync(int64_t ref_time, int64_t cycle_time,
                                        int64_t *offset) {
  int64_t delta = (ref_time - DC_SYNC_OFFSET) % cycle_time;
  if (delta > (cycle_time / 2)) {
    delta = delta - cycle_time;
  }
  int64_t time_error = -delta;
  dc_integral += time_error;
  *offset =
      static_cast<int64_t>((time_error * DC_PGAIN) + (dc_integral * DC_IGAIN));
}

void EtherCATMaster::MasterImpl::errorLoop() {
  while (running.load()) {
    if (in_op.load() &&
        (wkc_errors.load() > 2 || ctx.grouplist[0].docheckstate)) {
      ctx.grouplist[0].docheckstate = FALSE;
      ecx_readstate(&ctx);

      for (int i = 1; i <= slave_count; i++) {
        ec_slavet *slave = &ctx.slavelist[i];

        if (slave->state != EC_STATE_OPERATIONAL) {
          ctx.grouplist[0].docheckstate = TRUE;

          if (slave->state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
            // Acknowledge error
            spdlog::warn("Slave {} in SAFE_OP+ERROR (AL status 0x{:04x}: {}), "
                         "acknowledging",
                         i, slave->ALstatuscode,
                         ec_ALstatuscode2string(slave->ALstatuscode));
            slave->state = EC_STATE_SAFE_OP + EC_STATE_ACK;
            ecx_writestate(&ctx, i);
          } else if (slave->state == EC_STATE_SAFE_OP) {
            // Try to go back to OP
            spdlog::info("Slave {} in SAFE_OP, transitioning to OP", i);
            slave->state = EC_STATE_OPERATIONAL;
            if (slave->mbxhandlerstate == ECT_MBXH_LOST) {
              slave->mbxhandlerstate = ECT_MBXH_CYCLIC;
            }
            ecx_writestate(&ctx, i);
          } else if (slave->state > EC_STATE_NONE) {
            // Reconfigure slave
            spdlog::warn("Slave {} reconfiguring...", i);
            if (ecx_reconfig_slave(&ctx, i, EC_TIMEOUTMON) >= EC_STATE_PRE_OP) {
              slave->islost = FALSE;
              slaves[i - 1]->markRecovered();
              spdlog::info("Slave {} recovered", i);
            }
          } else if (!slave->islost) {
            // Re-check state
            ecx_statecheck(&ctx, i, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
            if (slave->state == EC_STATE_NONE) {
              slave->islost = TRUE;
              slave->mbxhandlerstate = ECT_MBXH_LOST;
              // Zero input data
              if (slave->Ibytes) {
                std::memset(slave->inputs, 0, slave->Ibytes);
              }
              slaves[i - 1]->markLost();
              spdlog::error("Slave {} lost!", i);
            }
          }
        }

        if (slave->islost) {
          if (slave->state <= EC_STATE_INIT) {
            if (ecx_recover_slave(&ctx, i, EC_TIMEOUTMON)) {
              slave->islost = FALSE;
              slaves[i - 1]->markRecovered();
              spdlog::info("Slave {} recovered from lost", i);
            }
          } else {
            slave->islost = FALSE;
            slaves[i - 1]->markRecovered();
          }
        }
      }

      wkc_errors.store(0);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

int EtherCATMaster::getSlaveCount() const { return impl_->slave_count; }

bool EtherCATMaster::isRunning() const { return impl_->running.load(); }

bool EtherCATMaster::isOperational() const { return impl_->in_op.load(); }

int EtherCATMaster::getWorkingCounter() const { return impl_->wkc.load(); }

int EtherCATMaster::getExpectedWKC() const { return impl_->expected_wkc; }

std::vector<SlaveTopologyInfo> EtherCATMaster::getTopology() const {
  std::vector<SlaveTopologyInfo> out;
  const int n = impl_->slave_count;
  if (n <= 0)
    return out;
  out.reserve(n);
  for (int i = 1; i <= n; ++i) {
    const ec_slavet &s = impl_->ctx.slavelist[i];
    SlaveTopologyInfo info;
    info.bus_id = i;
    info.config_address = s.configadr;
    info.alias_address = s.aliasadr;
    info.vendor_id = s.eep_man;
    info.product_code = s.eep_id;
    info.revision = s.eep_rev;
    info.serial = s.eep_ser;
    info.name = s.name;
    info.topology = s.topology;
    info.active_ports = s.activeports;
    info.consumed_ports = s.consumedports;
    info.parent = s.parent;
    info.parent_port = s.parentport;
    info.entry_port = s.entryport;
    out.push_back(std::move(info));
  }
  return out;
}

void EtherCATMaster::enable(int actuator_id) {
  auto it = id_to_slave_index_.find(actuator_id);
  if (it == id_to_slave_index_.end())
    return;

  auto &slave = impl_->slaves[it->second];
  slave->fault_reset_requested.store(false);
  slave->disable_requested.store(false);
  slave->enable_requested.store(true);
}

void EtherCATMaster::disable(int actuator_id) {
  auto it = id_to_slave_index_.find(actuator_id);
  if (it == id_to_slave_index_.end())
    return;

  auto &slave = impl_->slaves[it->second];
  slave->enable_requested.store(false);
  slave->disable_requested.store(true);
}

void EtherCATMaster::clearFault(int actuator_id) {
  auto it = id_to_slave_index_.find(actuator_id);
  if (it == id_to_slave_index_.end())
    return;

  impl_->slaves[it->second]->fault_reset_requested.store(true);
}

void EtherCATMaster::enableAll() {
  for (auto &slave : impl_->slaves) {
    slave->fault_reset_requested.store(false);
    slave->disable_requested.store(false);
    slave->enable_requested.store(true);
  }
}

void EtherCATMaster::disableAll() {
  for (auto &slave : impl_->slaves) {
    slave->enable_requested.store(false);
    slave->disable_requested.store(true);
  }
}

void EtherCATMaster::setCommand(int actuator_id, double position,
                                double velocity, double torque, double kp,
                                double kd) {
  auto it = id_to_slave_index_.find(actuator_id);
  if (it == id_to_slave_index_.end())
    return;

  auto &slave = impl_->slaves[it->second];
  slave->cmd_position.store(position);
  slave->cmd_velocity.store(velocity);
  slave->cmd_torque.store(torque);
  slave->cmd_kp.store(kp);
  slave->cmd_kd.store(kd);
}

void EtherCATMaster::setPosition(int actuator_id, double position) {
  auto it = id_to_slave_index_.find(actuator_id);
  if (it == id_to_slave_index_.end())
    return;
  impl_->slaves[it->second]->cmd_position.store(position);
}

void EtherCATMaster::setVelocity(int actuator_id, double velocity) {
  auto it = id_to_slave_index_.find(actuator_id);
  if (it == id_to_slave_index_.end())
    return;
  impl_->slaves[it->second]->cmd_velocity.store(velocity);
}

void EtherCATMaster::setTorque(int actuator_id, double torque) {
  auto it = id_to_slave_index_.find(actuator_id);
  if (it == id_to_slave_index_.end())
    return;
  impl_->slaves[it->second]->cmd_torque.store(torque);
}

void EtherCATMaster::setMaxTorque(int actuator_id, double max_torque) {
  auto it = id_to_slave_index_.find(actuator_id);
  if (it == id_to_slave_index_.end())
    return;
  impl_->slaves[it->second]->cmd_max_torque.store(max_torque);
}

void EtherCATMaster::setPVT(int actuator_id, double position, double velocity,
                            double torque, double kp, double kd) {
  setCommand(actuator_id, position, velocity, torque, kp, kd);
}

ActuatorState EtherCATMaster::getActuatorState(int actuator_id) const {
  auto it = id_to_slave_index_.find(actuator_id);
  if (it == id_to_slave_index_.end())
    return ActuatorState{};
  return impl_->slaves[it->second]->getState();
}

bool EtherCATMaster::getCycleStats(CycleTimingSnapshot &stats) const {
  return impl_->cycle_stats_buf.read(stats);
}

namespace {

// SII CRC over first 14 bytes (words 0..6), polynomial 0x07, init 0xFF.
// Result is stored in the low byte of word 7; high byte is unused (0x00).
uint8_t siiCrc8(const uint8_t *data, int len) {
  uint8_t crc = 0xFF;
  for (int i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b) {
      if (crc & 0x80) {
        crc = static_cast<uint8_t>((crc << 1) ^ 0x07);
      } else {
        crc = static_cast<uint8_t>(crc << 1);
      }
    }
  }
  return crc;
}

} // namespace

std::optional<uint16_t> EtherCATMaster::readAlias(int chain_position) {
  if (chain_position < 1 || chain_position > impl_->slave_count) {
    return std::nullopt;
  }
  if (impl_->running.load()) {
    spdlog::error("readAlias: cannot access EEPROM while master is running");
    return std::nullopt;
  }

  auto *ctx = &impl_->ctx;
  const uint16_t slave = static_cast<uint16_t>(chain_position);

  if (ecx_eeprom2master(ctx, slave) <= 0) {
    spdlog::error("readAlias: eeprom2master failed at chain position {}",
                  chain_position);
    return std::nullopt;
  }

  // Word 4 holds the configured station alias (16 bits). ecx_readeeprom
  // returns 32 bits (words 4 and 5); the alias is the low 16 bits.
  uint32_t raw = ecx_readeeprom(ctx, slave, 0x0004, EC_TIMEOUTEEP);
  return static_cast<uint16_t>(raw & 0xFFFF);
}

bool EtherCATMaster::writeAlias(int chain_position, uint16_t alias) {
  if (chain_position < 1 || chain_position > impl_->slave_count) {
    spdlog::error("writeAlias: chain position {} out of range (1..{})",
                  chain_position, impl_->slave_count);
    return false;
  }
  if (impl_->running.load()) {
    spdlog::error("writeAlias: cannot access EEPROM while master is running");
    return false;
  }

  auto *ctx = &impl_->ctx;
  const uint16_t slave = static_cast<uint16_t>(chain_position);

  // Force PRE_OP (SOEM leaves slaves here after config_init, but be explicit).
  ctx->slavelist[slave].state = EC_STATE_PRE_OP;
  ecx_writestate(ctx, slave);
  ecx_statecheck(ctx, slave, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);

  if (ecx_eeprom2master(ctx, slave) <= 0) {
    spdlog::error("writeAlias: eeprom2master failed at chain position {}",
                  chain_position);
    return false;
  }

  // Read words 0..6 (14 bytes) so we can recompute the SII CRC at word 7.
  uint8_t sii[14]{};
  for (int w = 0; w < 7; ++w) {
    uint32_t v = ecx_readeeprom(ctx, slave, static_cast<uint16_t>(w),
                                EC_TIMEOUTEEP);
    sii[w * 2 + 0] = static_cast<uint8_t>(v & 0xFF);
    sii[w * 2 + 1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  }

  // Patch the alias into the buffer before CRC'ing (word 4 = bytes 8..9).
  sii[8] = static_cast<uint8_t>(alias & 0xFF);
  sii[9] = static_cast<uint8_t>((alias >> 8) & 0xFF);

  uint8_t crc = siiCrc8(sii, 14);
  uint16_t crc_word = static_cast<uint16_t>(crc); // low byte = CRC, high = 0

  // Write alias at word 4, then CRC at word 7.
  if (ecx_writeeeprom(ctx, slave, 0x0004, alias, EC_TIMEOUTEEP) <= 0) {
    spdlog::error("writeAlias: failed to write alias word at chain position {}",
                  chain_position);
    return false;
  }
  if (ecx_writeeeprom(ctx, slave, 0x0007, crc_word, EC_TIMEOUTEEP) <= 0) {
    spdlog::error("writeAlias: failed to write CRC word at chain position {}",
                  chain_position);
    return false;
  }

  // Verify.
  uint32_t rb = ecx_readeeprom(ctx, slave, 0x0004, EC_TIMEOUTEEP);
  uint16_t rb_alias = static_cast<uint16_t>(rb & 0xFFFF);
  if (rb_alias != alias) {
    spdlog::error("writeAlias: verification failed at chain position {} "
                  "(wrote 0x{:04X}, read back 0x{:04X})",
                  chain_position, alias, rb_alias);
    return false;
  }

  spdlog::info("writeAlias: chain position {} alias set to 0x{:04X} "
               "(power-cycle required to take effect)",
               chain_position, alias);
  return true;
}

} // namespace ethercat_sdk
