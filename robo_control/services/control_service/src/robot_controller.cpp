#include "control_service/robot_controller.hpp"
#include "control_service/can_backend.hpp"
#include "control_service/ethercat_backend.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <spdlog/spdlog.h>
#include <transmission_sdk/registry.hpp>
#include <transport/node_context.hpp>
#include <unordered_map>
#include <yaml-cpp/yaml.h>

#include "common/config_loader.hpp"

namespace control_service {

RobotController::RobotController() {}
RobotController::~RobotController() { stop(); }

// ---------------------------------------------------------------------------
// Lifecycle: init / start / stop
// ---------------------------------------------------------------------------

void RobotController::loadParameters() {
  auto node = transport::NodeContext::instance().node();
  config_path_ = node->declare_parameter<std::string>("config_path");
}

bool RobotController::init() {
  loadParameters();
  if (!loadHardwareConfig()) return false;
  if (!createBackends()) return false;
  if (!buildTransmissions()) return false;
  if (!buildJointTopology()) return false;

  allocateRtBuffers();
  if (!initTransport()) return false;
  initThreadLoop();
  return true;
}

void RobotController::start() {
  for (auto &backend : backends_) backend->start();

  // One-shot read to prime state for seeding.
  for (auto &backend : backends_) backend->read();
  seedTransmissionsFromBackends();
  resetInterpolatorsFromBackends();

  if (thread_loop_) thread_loop_->start();
}

void RobotController::stop() {
  if (thread_loop_) thread_loop_->stop();
  for (auto &backend : backends_) backend->stop();
}

void RobotController::enterSafeMode(const std::string &reason) {
  if (!in_safe_mode_) {
    in_safe_mode_ = true;
    safe_mode_enter_throttle_.warn("Entering safe mode: {}", reason);
  }
}

// ---------------------------------------------------------------------------
// Config parsing
// ---------------------------------------------------------------------------

bool RobotController::loadHardwareConfig() {
  try {
    YAML::Node root = YAML::LoadFile(config_path_);

    hw_config_.robot_type =
        common::get_config_value<std::string>(root, "robot_type", "");

    std::string hardware_type =
        common::get_config_value<std::string>(root, "hardware_type");

    double clip_warn_tolerance_deg = common::get_config_value<double>(
        root, "clip_warn_tolerance_deg", 3.0);
    hw_config_.clip_warn_tolerance_rad =
        clip_warn_tolerance_deg * M_PI / 180.0;

    std::unordered_map<int, ActuatorConfig> actuator_table;
    if (!parseActuatorTable(root, actuator_table)) return false;
    if (!parseBuses(root, hardware_type, actuator_table)) return false;
    if (!parseTransmissions(root)) return false;
    if (!parseJoints(root)) return false;

    spdlog::info(
        "Loaded hardware config: robot_type={}, type={}, {} interfaces, "
        "{} joints, {} transmissions",
        hw_config_.robot_type.empty() ? "(unset)" : hw_config_.robot_type,
        hardware_type, hw_config_.buses.size(), hw_config_.joints.size(),
        hw_config_.transmissions.size());
    return true;
  } catch (const std::exception &e) {
    spdlog::error("Failed to load hardware config: {}", e.what());
    return false;
  }
}

bool RobotController::parseActuatorTable(
    const YAML::Node &root,
    std::unordered_map<int, ActuatorConfig> &out) {
  YAML::Node actuators = root["actuators"];
  if (!actuators || !actuators.IsSequence() || actuators.size() == 0) {
    spdlog::error("Missing or empty top-level 'actuators' list");
    return false;
  }

  for (const auto &act : actuators) {
    ActuatorConfig a;
    a.id = act["id"].as<int>();
    a.name = act["name"] ? act["name"].as<std::string>() : std::string{};
    a.type = act["type"].as<std::string>();
    a.direction = act["direction"].as<int>();
    a.bus_id = 0;
    a.zero_offset_rad = act["zero_offset"] ? act["zero_offset"].as<double>() : 0.0;
    a.lower_limit_rad = act["lower_limit"] ? act["lower_limit"].as<double>() : 0.0;
    a.upper_limit_rad = act["upper_limit"] ? act["upper_limit"].as<double>() : 0.0;
    if (act["calibration_reference"]) {
      std::string ref = act["calibration_reference"].as<std::string>();
      if (ref == "zero") {
        a.calibration_reference = CalibrationReference::Zero;
      } else if (ref == "lower") {
        a.calibration_reference = CalibrationReference::Lower;
      } else if (ref == "upper") {
        a.calibration_reference = CalibrationReference::Upper;
      } else {
        spdlog::error(
            "Actuator id {}: calibration_reference must be 'zero', 'lower', or 'upper' (got '{}')",
            a.id, ref);
        return false;
      }
    }
    if (!out.emplace(a.id, a).second) {
      spdlog::error("Duplicate actuator id {} in top-level 'actuators'", a.id);
      return false;
    }
  }
  return true;
}

bool RobotController::parseBuses(
    const YAML::Node &root, const std::string &hardware_type,
    const std::unordered_map<int, ActuatorConfig> &actuator_table) {
  YAML::Node hw_section = root[hardware_type];
  if (!hw_section) {
    spdlog::error("Missing {} section in hardware config", hardware_type);
    return false;
  }

  int loop_hz = common::get_config_value<int>(hw_section, "loop_hz");
  int cycle_time_us =
      common::get_config_value<int>(hw_section, "cycle_time_us", 0);
  int control_cpu_affinity =
      common::get_config_value<int>(hw_section, "control_cpu_affinity", -1);
  int ethercat_cpu_affinity =
      common::get_config_value<int>(hw_section, "ethercat_cpu_affinity", -1);

  BusType bus_type =
      (hardware_type == "ethercat") ? BusType::ETHERCAT : BusType::CAN;

  for (const auto &iface : hw_section["interfaces"]) {
    BusConfig bus;
    bus.type = bus_type;
    bus.interface_name = iface["name"].as<std::string>();
    bus.loop_hz = loop_hz;
    bus.cycle_time_us = cycle_time_us;
    bus.control_cpu_affinity = control_cpu_affinity;
    bus.ethercat_cpu_affinity = ethercat_cpu_affinity;

    for (const auto &entry : iface["actuators"]) {
      int id = entry["id"].as<int>();
      int bus_id = entry["bus_id"].as<int>();
      auto it = actuator_table.find(id);
      if (it == actuator_table.end()) {
        spdlog::error("Interface '{}' references unknown actuator id {}",
                      bus.interface_name, id);
        return false;
      }
      ActuatorConfig actuator = it->second;
      actuator.bus_id = bus_id;
      bus.actuators.push_back(actuator);
    }
    hw_config_.buses.push_back(bus);
  }
  return true;
}

bool RobotController::parseTransmissions(const YAML::Node &root) {
  if (!root["transmissions"]) return true;

  for (const auto &t : root["transmissions"]) {
    TransmissionGroupConfig g;
    g.name = t["name"].as<std::string>();
    g.type = t["type"].as<std::string>();
    for (const auto &a : t["actuator_ids"]) {
      g.actuator_ids.push_back(a.as<int>());
    }
    if (g.actuator_ids.empty()) {
      spdlog::error("Transmission '{}': actuator_ids is empty", g.name);
      return false;
    }
    hw_config_.transmissions.push_back(g);
  }
  return true;
}

bool RobotController::parseJoints(const YAML::Node &root) {
  YAML::Node joints = root["joints"];
  if (!joints || !joints.IsSequence() || joints.size() == 0) {
    spdlog::error("Missing or empty top-level 'joints' list");
    return false;
  }

  for (const auto &j : joints) {
    JointConfig jc;
    jc.name = j["name"].as<std::string>();
    jc.lower_limit_rad = j["lower_limit"].as<double>();
    jc.upper_limit_rad = j["upper_limit"].as<double>();

    YAML::Node src = j["source"];
    if (!src) {
      spdlog::error("Joint '{}': missing 'source'", jc.name);
      return false;
    }
    if (src["passthrough"]) {
      jc.source.kind = JointSourceKind::Passthrough;
      jc.source.actuator_id = src["passthrough"].as<int>();
    } else if (src["transmission"]) {
      jc.source.kind = JointSourceKind::Transmission;
      jc.source.transmission_name = src["transmission"].as<std::string>();
      jc.source.slot = src["slot"].as<std::size_t>();
    } else {
      spdlog::error(
          "Joint '{}': source must specify either 'passthrough' or 'transmission'",
          jc.name);
      return false;
    }
    hw_config_.joints.push_back(std::move(jc));
  }
  return true;
}

// ---------------------------------------------------------------------------
// init() helpers
// ---------------------------------------------------------------------------

bool RobotController::createBackends() {
  std::size_t backend_idx = 0;
  for (const auto &bus : hw_config_.buses) {
    std::unique_ptr<ActuatorBackend> backend;
    if (bus.type == BusType::ETHERCAT) {
      backend = std::make_unique<EtherCATBackend>(bus);
    } else if (bus.type == BusType::CAN) {
      backend = std::make_unique<CANBackend>(bus);
    }

    if (!backend || !backend->init()) {
      spdlog::error("Failed to initialize backend for interface {}",
                    bus.interface_name);
      return false;
    }
    backends_.push_back(std::move(backend));
    for (std::size_t slot = 0; slot < bus.actuators.size(); ++slot) {
      actuator_location_[bus.actuators[slot].id] = {backend_idx, slot};
    }
    backend_idx++;
  }
  return true;
}

bool RobotController::buildTransmissions() {
  std::unordered_map<int, const ActuatorConfig *> by_id;
  for (const auto &bus : hw_config_.buses)
    for (const auto &act : bus.actuators) by_id[act.id] = &act;

  std::set<int> claimed_actuators;
  for (const auto &cfg : hw_config_.transmissions) {
    TransmissionGroup g;
    g.name = cfg.name;
    g.actuator_ids = cfg.actuator_ids;
    try {
      g.model = transmission::createTransmission(hw_config_.robot_type, cfg.type);
    } catch (const std::exception &e) {
      spdlog::error("Failed to create transmission '{}': {}", cfg.name, e.what());
      return false;
    }
    if (g.model->dof() != cfg.actuator_ids.size()) {
      spdlog::error(
          "Transmission '{}' dof mismatch: model={} actuator_ids={}",
          cfg.name, g.model->dof(), cfg.actuator_ids.size());
      return false;
    }
    std::size_t n = g.model->dof();
    g.q_joint_seed = Eigen::VectorXd::Zero(n);

    g.backend_indices.reserve(n);
    g.backend_slots.reserve(n);
    g.motor_directions.reserve(n);
    g.motor_zero_offsets.reserve(n);
    g.joint_ids.assign(n, 0);
    g.joint_indices.assign(n, 0);
    for (int aid : cfg.actuator_ids) {
      if (!claimed_actuators.insert(aid).second) {
        spdlog::error("Actuator id {} appears in multiple transmission groups",
                      aid);
        return false;
      }
      auto it = actuator_location_.find(aid);
      if (it == actuator_location_.end()) {
        spdlog::error("Transmission '{}' references unknown actuator id {}",
                      cfg.name, aid);
        return false;
      }
      g.backend_indices.push_back(it->second.backend_idx);
      g.backend_slots.push_back(it->second.slot_idx);
      g.motor_directions.push_back(by_id.at(aid)->direction);
      g.motor_zero_offsets.push_back(by_id.at(aid)->zero_offset_rad);
    }
    transmissions_.push_back(std::move(g));
  }
  return true;
}

bool RobotController::buildJointTopology() {
  // id -> ActuatorConfig lookup for passthrough direction.
  std::unordered_map<int, const ActuatorConfig *> by_id;
  for (const auto &bus : hw_config_.buses)
    for (const auto &act : bus.actuators) by_id[act.id] = &act;

  // transmission name -> index in transmissions_.
  std::unordered_map<std::string, std::size_t> group_by_name;
  for (std::size_t gi = 0; gi < transmissions_.size(); ++gi) {
    group_by_name[transmissions_[gi].name] = gi;
  }

  // Track (group, slot) occupancy so we can detect a missing or duplicate
  // joint entry for any transmission slot.
  std::vector<std::vector<bool>> slot_filled(transmissions_.size());
  for (std::size_t gi = 0; gi < transmissions_.size(); ++gi)
    slot_filled[gi].assign(transmissions_[gi].model->dof(), false);

  joint_ids_.reserve(hw_config_.joints.size());
  joint_sources_.reserve(hw_config_.joints.size());
  joint_lower_limits_.reserve(hw_config_.joints.size());
  joint_upper_limits_.reserve(hw_config_.joints.size());
  clip_warn_throttle_.clear();
  clip_warn_throttle_.reserve(hw_config_.joints.size());
  for (std::size_t i = 0; i < hw_config_.joints.size(); ++i) {
    clip_warn_throttle_.emplace_back(std::chrono::seconds(1));
  }

  for (std::size_t i = 0; i < hw_config_.joints.size(); ++i) {
    const auto &j = hw_config_.joints[i];
    JointSource src;

    if (j.source.kind == JointSourceKind::Passthrough) {
      auto loc_it = actuator_location_.find(j.source.actuator_id);
      if (loc_it == actuator_location_.end()) {
        spdlog::error("Joint '{}' references unknown actuator id {}", j.name,
                      j.source.actuator_id);
        return false;
      }
      auto act_it = by_id.find(j.source.actuator_id);
      if (act_it == by_id.end()) {
        spdlog::error("Joint '{}': actuator id {} not in actuator table",
                      j.name, j.source.actuator_id);
        return false;
      }
      src.kind = JointKind::Passthrough;
      src.actuator_id = j.source.actuator_id;
      src.backend_idx = loc_it->second.backend_idx;
      src.backend_slot_idx = loc_it->second.slot_idx;
      src.direction = act_it->second->direction;
      src.zero_offset = act_it->second->zero_offset_rad;
    } else {
      auto git = group_by_name.find(j.source.transmission_name);
      if (git == group_by_name.end()) {
        spdlog::error("Joint '{}' references unknown transmission '{}'",
                      j.name, j.source.transmission_name);
        return false;
      }
      std::size_t gi = git->second;
      auto &g = transmissions_[gi];
      if (j.source.slot >= g.model->dof()) {
        spdlog::error(
            "Joint '{}': transmission '{}' slot {} out of range (dof={})",
            j.name, g.name, j.source.slot, g.model->dof());
        return false;
      }
      if (slot_filled[gi][j.source.slot]) {
        spdlog::error(
            "Joint '{}': transmission '{}' slot {} already claimed by another joint",
            j.name, g.name, j.source.slot);
        return false;
      }
      slot_filled[gi][j.source.slot] = true;

      src.kind = JointKind::Transmission;
      src.group_idx = gi;
      src.slot_idx = j.source.slot;
      src.backend_idx = g.backend_indices[j.source.slot];
      src.backend_slot_idx = g.backend_slots[j.source.slot];

      g.joint_indices[j.source.slot] = joint_ids_.size();
      g.joint_ids[j.source.slot] = static_cast<int>(i + 1);
    }

    // Synthetic per-joint id matches the joint's declaration order (1-based)
    // so logs read as "Joint N" = Nth entry in the joints: list.
    joint_ids_.push_back(static_cast<int>(i + 1));
    joint_sources_.push_back(src);
    joint_lower_limits_.push_back(j.lower_limit_rad);
    joint_upper_limits_.push_back(j.upper_limit_rad);
  }

  // Verify every transmission slot was claimed exactly once.
  for (std::size_t gi = 0; gi < transmissions_.size(); ++gi) {
    for (std::size_t k = 0; k < slot_filled[gi].size(); ++k) {
      if (!slot_filled[gi][k]) {
        spdlog::error(
            "Transmission '{}' slot {} has no corresponding entry in joints: list",
            transmissions_[gi].name, k);
        return false;
      }
    }
  }

  interpolators_.assign(joint_ids_.size(), JointInterpolator());
  return true;
}

void RobotController::allocateRtBuffers() {
  std::size_t num_joints = joint_ids_.size();
  state_positions_.resize(num_joints);
  state_velocities_.resize(num_joints);
  state_torques_.resize(num_joints);
  state_temperatures_.resize(num_joints);

  commands_per_backend_.resize(backends_.size());
  for (std::size_t bi = 0; bi < backends_.size(); ++bi) {
    commands_per_backend_[bi].assign(hw_config_.buses[bi].actuators.size(),
                                     BackendCommand{});
  }

  for (auto *c : {&cached_command_, &active_command_}) {
    c->position.assign(num_joints, 0.0);
    c->velocity.assign(num_joints, 0.0);
    c->torque.assign(num_joints, 0.0);
    c->kp.assign(num_joints, 0.0);
    c->kd.assign(num_joints, 0.0);
  }

  cmd_pos_.assign(num_joints, 0.0);

  std::size_t ng = transmissions_.size();
  group_q_motor_.resize(ng);
  group_dq_motor_.resize(ng);
  group_tau_motor_.resize(ng);
  group_q_joint_.resize(ng);
  group_dq_joint_.resize(ng);
  group_tau_joint_.resize(ng);
  group_cmd_q_joint_.resize(ng);
  group_cmd_dq_joint_.resize(ng);
  group_cmd_tau_joint_.resize(ng);
  for (std::size_t gi = 0; gi < ng; ++gi) {
    std::size_t n = transmissions_[gi].model->dof();
    for (auto *v : {&group_q_motor_[gi], &group_dq_motor_[gi],
                    &group_tau_motor_[gi], &group_q_joint_[gi],
                    &group_dq_joint_[gi], &group_tau_joint_[gi],
                    &group_cmd_q_joint_[gi], &group_cmd_dq_joint_[gi],
                    &group_cmd_tau_joint_[gi]}) {
      v->setZero(n);
    }
  }
}

bool RobotController::initTransport() {
  state_pub_ = std::make_unique<transport::Publisher<transport::JointState>>(
      transport::JOINT_STATE_TOPIC);
  command_sub_ =
      std::make_unique<transport::Subscriber<transport::JointCommand>>(
          transport::JOINT_COMMAND_TOPIC);
  command_sub_->set_callback([this](const transport::JointCommand &msg) {
    this->commandCallback(msg);
  });

  if (!state_pub_->is_valid() || !command_sub_->is_valid()) {
    spdlog::error("Failed to initialize transport");
    return false;
  }
  return true;
}

void RobotController::initThreadLoop() {
  if (!hw_config_.buses.empty()) {
    control_frequency_ = static_cast<double>(hw_config_.buses[0].loop_hz);
    spdlog::info("Control loop frequency set to {} Hz", control_frequency_);
  }

  common::RealtimeConfig rt_config;
  rt_config.frequency_hz = static_cast<int>(control_frequency_);
  rt_config.scheduler_policy = SCHED_FIFO;
  rt_config.scheduler_priority = 80;
  if (!hw_config_.buses.empty()) {
    rt_config.cpu_affinity = hw_config_.buses[0].control_cpu_affinity;
  }

  thread_loop_ = std::make_unique<common::ThreadLoop>(
      "ControlLoop", rt_config, [this]() { this->controlLoop(); });
}

// ---------------------------------------------------------------------------
// start() helpers
// ---------------------------------------------------------------------------

void RobotController::seedTransmissionsFromBackends() {
  for (auto &g : transmissions_) {
    std::size_t n = g.model->dof();
    Eigen::VectorXd q_motor(n);
    for (std::size_t k = 0; k < n; ++k) {
      const auto &states =
          backends_[g.backend_indices[k]]->states();
      q_motor(k) = states[g.backend_slots[k]].position *
                       static_cast<double>(g.motor_directions[k]) -
                   g.motor_zero_offsets[k];
    }
    try {
      g.model->position_inverse_into(q_motor, Eigen::VectorXd::Zero(n),
                                     g.q_joint_seed);
      g.seed_valid = true;
    } catch (const std::exception &e) {
      spdlog::error("Initial seed for transmission '{}' failed: {}",
                    g.name, e.what());
    }
  }
}

void RobotController::resetInterpolatorsFromBackends() {
  for (std::size_t i = 0; i < joint_ids_.size(); ++i) {
    const auto &src = joint_sources_[i];
    double pos = 0.0;
    if (src.kind == JointKind::Passthrough) {
      double raw =
          backends_[src.backend_idx]->states()[src.backend_slot_idx].position;
      pos = raw * static_cast<double>(src.direction) - src.zero_offset;
    } else {
      const auto &g = transmissions_[src.group_idx];
      if (g.seed_valid) pos = g.q_joint_seed(src.slot_idx);
    }
    interpolators_[i].reset(pos);
  }
}

// ---------------------------------------------------------------------------
// Command callback
// ---------------------------------------------------------------------------

double RobotController::clampJointPosition(std::size_t joint_idx,
                                           double pos) const {
  double lo = joint_lower_limits_[joint_idx];
  double hi = joint_upper_limits_[joint_idx];
  if (pos < lo || pos > hi) {
    double clipped = std::clamp(pos, lo, hi);
    double tol = hw_config_.clip_warn_tolerance_rad;
    if (pos < lo - tol || pos > hi + tol) {
      clip_warn_throttle_[joint_idx].warn(
          "Joint {} command {:.4f} out of range [{:.4f}, {:.4f}], clipped to "
          "{:.4f}",
          joint_ids_[joint_idx], pos, lo, hi, clipped);
    }
    return clipped;
  }
  return pos;
}

void RobotController::commandCallback(const transport::JointCommand &msg) {
  std::lock_guard<std::mutex> lock(command_mutex_);

  if (in_safe_mode_ && !fault_detected_) {
    in_safe_mode_ = false;
    safe_mode_exit_throttle_.info("Exiting safe mode - command received");
  }

  auto now = std::chrono::steady_clock::now();
  int steps = default_interpolation_steps_;
  if (first_command_received_) {
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
        now - last_command_time_);
    double dt_sec = duration.count() / 1e6;
    if (dt_sec > 0.0001) {
      steps = static_cast<int>(control_frequency_ * dt_sec);
      steps = std::clamp(steps, 1, 1000);
    }
  }
  last_command_time_ = now;
  first_command_received_ = true;

  auto copyField = [](const std::vector<float> &src, std::size_t idx) -> double {
    return idx < src.size() ? src[idx] : 0.0;
  };

  std::size_t count = std::min(msg.position.size(), joint_ids_.size());
  for (std::size_t i = 0; i < count; ++i) {
    cached_command_.position[i] = clampJointPosition(i, msg.position[i]);
    cached_command_.velocity[i] = copyField(msg.velocity, i);
    cached_command_.torque[i] = copyField(msg.torque, i);
    cached_command_.kp[i] = copyField(msg.kp, i);
    cached_command_.kd[i] = copyField(msg.kd, i);
  }

  pending_steps_ = steps;
  pending_command_ready_ = true;
}

// ---------------------------------------------------------------------------
// Control loop
// ---------------------------------------------------------------------------

void RobotController::controlLoop() {
#ifdef CONTROL_LOOP_BENCHMARK
  using Clock = std::chrono::steady_clock;
  auto accumulate = [this](BenchStage s, Clock::time_point a,
                           Clock::time_point b) {
    auto ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(b - a).count());
    auto &st = bench_stats_[s];
    st.sum_ns += ns;
    if (ns < st.min_ns) st.min_ns = ns;
    if (ns > st.max_ns) st.max_ns = ns;
  };
  auto t0 = Clock::now();
#endif

  readAllBackends();
#ifdef CONTROL_LOOP_BENCHMARK
  auto t1 = Clock::now();
  accumulate(BENCH_READ, t0, t1);
#endif

  faulted_joints_.clear();
  if (solveTransmissionInverses() && !in_safe_mode_) {
    enterSafeMode("Transmission inverse failure / missing actuator state");
  }
#ifdef CONTROL_LOOP_BENCHMARK
  auto t2 = Clock::now();
#endif

  scatterJointStates();
#ifdef CONTROL_LOOP_BENCHMARK
  auto t3 = Clock::now();
  accumulate(BENCH_SCATTER, t2, t3);
#endif

  if (!faulted_joints_.empty() && !fault_detected_) {
    fault_detected_ = true;
    enterSafeMode("Actuator fault/lost detected on joints");
    for (int id : faulted_joints_) spdlog::error("Fault/lost on joint {}", id);
  }

  publishJointState();
#ifdef CONTROL_LOOP_BENCHMARK
  auto t4 = Clock::now();
  accumulate(BENCH_PUBLISH, t3, t4);
#endif

  checkCommandWatchdog();
  if (!in_safe_mode_) applyPendingCommand();
#ifdef CONTROL_LOOP_BENCHMARK
  auto t5 = Clock::now();
  accumulate(BENCH_CMD, t4, t5);
#endif

  interpolateJointTargets();
#ifdef CONTROL_LOOP_BENCHMARK
  auto t6 = Clock::now();
  accumulate(BENCH_INTERP, t5, t6);
#endif

  // Reset every slot to a neutral command so fault / workspace-violation
  // branches that skip a group emit zeros rather than stale commands.
  for (auto &cmds : commands_per_backend_)
    std::fill(cmds.begin(), cmds.end(), BackendCommand{});
  writePassthroughCommands();
  writeTransmissionCommands();
#ifdef CONTROL_LOOP_BENCHMARK
  auto t7 = Clock::now();
#endif

  writeAllBackends();
#ifdef CONTROL_LOOP_BENCHMARK
  auto t8 = Clock::now();
  accumulate(BENCH_WRITE_BACKENDS, t7, t8);
  accumulate(BENCH_TOTAL, t0, t8);

  if (++bench_cycles_ >= bench_report_interval_) {
    const char *names[BENCH_COUNT] = {
        "read",       "pos_inv",      "vel_inv",         "trq_inv",
        "scatter",    "publish",      "cmd",             "interp",
        "pos_fwd",    "vel_fwd",      "trq_fwd",         "write_other",
        "write_backends",             "TOTAL",
    };
    const double n = static_cast<double>(bench_cycles_);
    spdlog::info("ControlLoop bench over {} cycles (us: avg/min/max):",
                 bench_cycles_);
    for (int i = 0; i < BENCH_COUNT; ++i) {
      const auto &st = bench_stats_[i];
      double avg_us = (static_cast<double>(st.sum_ns) / n) / 1000.0;
      double min_us = static_cast<double>(st.min_ns) / 1000.0;
      double max_us = static_cast<double>(st.max_ns) / 1000.0;
      spdlog::info("  {:<16} {:8.2f} / {:8.2f} / {:8.2f}", names[i], avg_us,
                   min_us, max_us);
      bench_stats_[i] = StageStat{};
    }
    bench_cycles_ = 0;
  }
#endif
}

void RobotController::readAllBackends() {
  for (auto &backend : backends_) backend->read();
}

bool RobotController::solveTransmissionInverses() {
  bool tx_failure = false;
  for (std::size_t gi = 0; gi < transmissions_.size(); ++gi) {
    auto &g = transmissions_[gi];
    std::size_t n = g.model->dof();
    auto &q_m = group_q_motor_[gi];
    auto &dq_m = group_dq_motor_[gi];
    auto &tau_m = group_tau_motor_[gi];
    auto &q_j = group_q_joint_[gi];
    auto &dq_j = group_dq_joint_[gi];
    auto &tau_j = group_tau_joint_[gi];

    auto fallback = [&]() {
      if (g.seed_valid) q_j = g.q_joint_seed; else q_j.setZero();
      dq_j.setZero();
      tau_j.setZero();
    };

    double max_temp = 0.0;
    for (std::size_t k = 0; k < n; ++k) {
      const auto &s =
          backends_[g.backend_indices[k]]->states()[g.backend_slots[k]];
      double dir = static_cast<double>(g.motor_directions[k]);
      q_m(k) = s.position * dir - g.motor_zero_offsets[k];
      dq_m(k) = s.velocity * dir;
      tau_m(k) = s.torque * dir;
      max_temp = std::max(max_temp, s.temperature);
      if (s.fault || s.lost) {
        faulted_joints_.push_back(g.actuator_ids[k]);
      }
    }
    g.max_temperature = max_temp;

    try {
      // Warm-start seed: q_joint_seed is Zero on the first call (set in
      // buildTransmissions) and tracks the last solution thereafter.
#ifdef CONTROL_LOOP_BENCHMARK
      using Clock = std::chrono::steady_clock;
      auto add = [this](BenchStage s, Clock::time_point a,
                        Clock::time_point b) {
        auto ns = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(b - a).count());
        auto &st = bench_stats_[s];
        st.sum_ns += ns;
        if (ns < st.min_ns) st.min_ns = ns;
        if (ns > st.max_ns) st.max_ns = ns;
      };
      auto ta = Clock::now();
      g.model->position_inverse_into(q_m, g.q_joint_seed, q_j);
      auto tb = Clock::now();
      g.model->velocity_inverse_into(q_j, dq_m, dq_j);
      auto tc = Clock::now();
      g.model->torque_inverse_into(q_j, tau_m, tau_j);
      auto td = Clock::now();
      add(BENCH_POS_INV, ta, tb);
      add(BENCH_VEL_INV, tb, tc);
      add(BENCH_TRQ_INV, tc, td);
#else
      g.model->position_inverse_into(q_m, g.q_joint_seed, q_j);
      g.model->velocity_inverse_into(q_j, dq_m, dq_j);
      g.model->torque_inverse_into(q_j, tau_m, tau_j);
#endif
      g.q_joint_seed = q_j;
      g.seed_valid = true;
    } catch (const std::exception &e) {
      g.inverse_err_throttle.error("Transmission '{}' inverse failed: {}",
                                   g.name, e.what());
      tx_failure = true;
      fallback();
    }
  }
  return tx_failure;
}

void RobotController::scatterJointStates() {
  for (std::size_t i = 0; i < joint_ids_.size(); ++i) {
    const auto &src = joint_sources_[i];
    if (src.kind == JointKind::Passthrough) {
      const auto &s =
          backends_[src.backend_idx]->states()[src.backend_slot_idx];
      double dir = static_cast<double>(src.direction);
      state_positions_[i] =
          static_cast<float>(s.position * dir - src.zero_offset);
      state_velocities_[i] = static_cast<float>(s.velocity * dir);
      state_torques_[i] = static_cast<float>(s.torque * dir);
      state_temperatures_[i] = static_cast<float>(s.temperature);
      if (s.fault || s.lost) faulted_joints_.push_back(src.actuator_id);
    } else {
      const auto &g = transmissions_[src.group_idx];
      state_positions_[i] =
          static_cast<float>(group_q_joint_[src.group_idx](src.slot_idx));
      state_velocities_[i] =
          static_cast<float>(group_dq_joint_[src.group_idx](src.slot_idx));
      state_torques_[i] =
          static_cast<float>(group_tau_joint_[src.group_idx](src.slot_idx));
      state_temperatures_[i] = static_cast<float>(g.max_temperature);
    }
  }
}

void RobotController::publishJointState() {
  if (!state_pub_) return;
  transport::JointState msg;
  msg.position = state_positions_;
  msg.velocity = state_velocities_;
  msg.torque = state_torques_;
  msg.temperature = state_temperatures_;
  state_pub_->publish(msg);
}

void RobotController::checkCommandWatchdog() {
  if (!first_command_received_ || in_safe_mode_) return;
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                     now - last_command_time_)
                     .count();
  if (elapsed > command_timeout_ms_) enterSafeMode("Command timeout");
}

void RobotController::applyPendingCommand() {
  std::unique_lock<std::mutex> lock(command_mutex_, std::try_to_lock);
  if (!lock.owns_lock() || !pending_command_ready_) return;

  active_command_ = cached_command_;
  std::size_t count =
      std::min(active_command_.position.size(), interpolators_.size());
  for (std::size_t i = 0; i < count; ++i) {
    interpolators_[i].setTarget(active_command_.position[i], pending_steps_);
  }
  pending_command_ready_ = false;
}

void RobotController::interpolateJointTargets() {
  for (std::size_t i = 0; i < interpolators_.size(); ++i) {
    cmd_pos_[i] = interpolators_[i].process();
  }
}

void RobotController::writePassthroughCommands() {
  for (std::size_t i = 0; i < joint_ids_.size(); ++i) {
    const auto &src = joint_sources_[i];
    if (src.kind != JointKind::Passthrough) continue;

    double dir = static_cast<double>(src.direction);
    BackendCommand cmd;
    // Inverse of the read-side math: joint-space position + offset gives the
    // motor-space target; multiply by direction to reach raw motor radians.
    cmd.position = (cmd_pos_[i] + src.zero_offset) * dir;
    if (i < active_command_.velocity.size())
      cmd.velocity = active_command_.velocity[i] * dir;
    if (i < active_command_.torque.size())
      cmd.torque = active_command_.torque[i] * dir;
    if (i < active_command_.kp.size())       cmd.kp       = active_command_.kp[i];
    if (i < active_command_.kd.size())       cmd.kd       = active_command_.kd[i];

    commands_per_backend_[src.backend_idx][src.backend_slot_idx] = cmd;
  }
}

void RobotController::writeTransmissionCommands() {
  if (transmissions_.empty()) return;

#ifdef CONTROL_LOOP_BENCHMARK
  using Clock = std::chrono::steady_clock;
  auto t_other_start = Clock::now();
  auto add = [this](BenchStage s, Clock::time_point a,
                    Clock::time_point b) {
    auto ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(b - a).count());
    auto &st = bench_stats_[s];
    st.sum_ns += ns;
    if (ns < st.min_ns) st.min_ns = ns;
    if (ns > st.max_ns) st.max_ns = ns;
  };
#endif

  for (auto &v : group_cmd_q_joint_) v.setZero();
  for (auto &v : group_cmd_dq_joint_) v.setZero();
  for (auto &v : group_cmd_tau_joint_) v.setZero();
  for (std::size_t i = 0; i < joint_ids_.size(); ++i) {
    const auto &src = joint_sources_[i];
    if (src.kind != JointKind::Transmission) continue;
    group_cmd_q_joint_[src.group_idx](src.slot_idx) = cmd_pos_[i];
    if (i < active_command_.velocity.size())
      group_cmd_dq_joint_[src.group_idx](src.slot_idx) = active_command_.velocity[i];
    if (i < active_command_.torque.size())
      group_cmd_tau_joint_[src.group_idx](src.slot_idx) = active_command_.torque[i];
  }

  for (std::size_t gi = 0; gi < transmissions_.size(); ++gi) {
    auto &g = transmissions_[gi];
    // Reuse the motor-space scratch buffers (already sized to dof) for the
    // forward transform outputs. Writes in-place — no heap alloc per cycle.
    auto &q_m = group_q_motor_[gi];
    auto &dq_m = group_dq_motor_[gi];
    auto &tau_m = group_tau_motor_[gi];
    try {
#ifdef CONTROL_LOOP_BENCHMARK
      auto ta = Clock::now();
      add(BENCH_WRITE_OTHER, t_other_start, ta);
      g.model->position_forward_into(group_cmd_q_joint_[gi], q_m);
      auto tb = Clock::now();
      g.model->velocity_forward_into(group_cmd_q_joint_[gi],
                                     group_cmd_dq_joint_[gi], dq_m);
      auto tc = Clock::now();
      g.model->torque_forward_into(group_cmd_q_joint_[gi],
                                   group_cmd_tau_joint_[gi], tau_m);
      auto td = Clock::now();
      add(BENCH_POS_FWD, ta, tb);
      add(BENCH_VEL_FWD, tb, tc);
      add(BENCH_TRQ_FWD, tc, td);
      t_other_start = td;
#else
      g.model->position_forward_into(group_cmd_q_joint_[gi], q_m);
      g.model->velocity_forward_into(group_cmd_q_joint_[gi],
                                     group_cmd_dq_joint_[gi], dq_m);
      g.model->torque_forward_into(group_cmd_q_joint_[gi],
                                   group_cmd_tau_joint_[gi], tau_m);
#endif

      // kp/kd are passed through joint->motor unchanged. Physically
      // incoherent (gain is calibrated for joint-space error but applied
      // in motor space), but acceptable until controller-side PD lands.
      for (std::size_t k = 0; k < g.actuator_ids.size(); ++k) {
        std::size_t joint_idx = g.joint_indices[k];
        double dir = static_cast<double>(g.motor_directions[k]);
        BackendCommand cmd;
        cmd.position = (q_m(k) + g.motor_zero_offsets[k]) * dir;
        cmd.velocity = dq_m(k) * dir;
        cmd.torque = tau_m(k) * dir;
        if (joint_idx < active_command_.kp.size()) cmd.kp = active_command_.kp[joint_idx];
        if (joint_idx < active_command_.kd.size()) cmd.kd = active_command_.kd[joint_idx];

        commands_per_backend_[g.backend_indices[k]][g.backend_slots[k]] = cmd;
      }
    } catch (const std::exception &e) {
      g.forward_err_throttle.error("Transmission '{}' forward failed: {}",
                                   g.name, e.what());
      enterSafeMode("Transmission forward workspace violation");
    }
  }
}

void RobotController::writeAllBackends() {
  for (std::size_t i = 0; i < backends_.size(); ++i) {
    backends_[i]->write(commands_per_backend_[i]);
  }
}

} // namespace control_service
