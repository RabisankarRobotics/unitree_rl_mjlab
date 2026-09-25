#include "policy_service/policy_service.hpp"

#include <algorithm>
#include <cmath>
#include <spdlog/spdlog.h>
#include <transport/node_context.hpp>
#include <yaml-cpp/yaml.h>

#include "common/config_loader.hpp"

namespace {

constexpr int8_t POLICY_MODE_VALUE = 3;

policy_service::VelocityRange parseVelocityRange(const YAML::Node &node,
                                                 const std::string &key) {
  auto range_node = node[key];
  if (!range_node || !range_node.IsSequence() || range_node.size() != 2) {
    throw std::runtime_error(
        "Velocity range '" + key +
        "' must be a sequence of [min, max] (length 2)");
  }
  policy_service::VelocityRange r{range_node[0].as<float>(),
                                  range_node[1].as<float>()};
  if (r.min > r.max) {
    throw std::runtime_error("Velocity range '" + key + "' has min > max");
  }
  return r;
}

// Linear map from normalized [-1, 1] to [range.min, range.max], clamped.
float scaleToRange(float v, const policy_service::VelocityRange &range) {
  if (v < -1.0f) v = -1.0f;
  if (v > 1.0f) v = 1.0f;
  return 0.5f * (v + 1.0f) * (range.max - range.min) + range.min;
}

std::array<float, 3> computeProjectedGravity(const std::vector<float> &quat) {
  // quat is [w, x, y, z]
  if (quat.size() < 4) {
    return {0.0f, 0.0f, -1.0f};
  }

  float w = quat[0];
  float x = quat[1];
  float y = quat[2];
  float z = quat[3];

  float norm = std::sqrt(w * w + x * x + y * y + z * z);
  if (norm > 1e-6f) {
    w /= norm;
    x /= norm;
    y /= norm;
    z /= norm;
  }

  // Rotate world gravity (0, 0, -1) into body frame
  float gx = 2.0f * (w * y - x * z);
  float gy = -2.0f * (w * x + y * z);
  float gz = -w * w + x * x + y * y - z * z;

  return {gx, gy, gz};
}

} // namespace

namespace policy_service {

PolicyService::PolicyService() {}

PolicyService::~PolicyService() { stop(); }

void PolicyService::loadParameters() {
  auto node = transport::NodeContext::instance().node();

  auto config_path = node->declare_parameter<std::string>("config_path");
  YAML::Node config = YAML::LoadFile(config_path);

  policy_config_.policy_path =
      common::get_config_value<std::string>(config, "policy_path");
  policy_config_.num_joints =
      common::get_config_value<int>(config, "num_joints");
  policy_config_.loop_hz = common::get_config_value<int>(config, "loop_hz");
  policy_config_.action_scale =
      common::get_config_value<std::vector<float>>(config, "action_scale");

  policy_config_.kp =
      common::get_config_value<std::vector<float>>(config, "kp");
  policy_config_.kd =
      common::get_config_value<std::vector<float>>(config, "kd");
  policy_config_.default_joint_pos =
      common::get_config_value<std::vector<float>>(config, "default_joint_pos");
  policy_config_.policy_to_actuator_map =
      common::get_config_value<std::vector<int>>(config,
                                                 "policy_to_actuator_map");

  // Parse required observations section
  YAML::Node obs_config = config["observations"];
  if (!obs_config || !obs_config.IsMap()) {
    throw std::runtime_error(
        "Missing required 'observations' section in config");
  }

  policy_config_.obs_history_length =
      common::get_config_value<int>(obs_config, "history_length");

  auto terms_node = obs_config["terms"];
  if (!terms_node || !terms_node.IsSequence()) {
    throw std::runtime_error(
        "Missing required 'observations.terms' list in config");
  }
  for (const auto &term : terms_node) {
    policy_config_.obs_terms.push_back(term.as<std::string>());
  }

  const bool needs_phase =
      std::find(policy_config_.obs_terms.begin(),
                policy_config_.obs_terms.end(),
                std::string("gait_phase")) != policy_config_.obs_terms.end();
  if (needs_phase) {
    policy_config_.phase_period_s =
        common::get_config_value<float>(obs_config, "phase_period_s");
    if (policy_config_.phase_period_s <= 0.0f) {
      throw std::runtime_error(
          "observations.phase_period_s must be > 0 when gait_phase is enabled");
    }
  } else {
    policy_config_.phase_period_s = 0.0f;
  }

  // Parse required velocity command ranges.
  YAML::Node commands_node = config["commands"];
  if (!commands_node || !commands_node.IsMap()) {
    throw std::runtime_error("Missing required 'commands' section in config");
  }
  YAML::Node base_velocity_node = commands_node["base_velocity"];
  if (!base_velocity_node || !base_velocity_node.IsMap()) {
    throw std::runtime_error(
        "Missing required 'commands.base_velocity' section in config");
  }
  YAML::Node ranges_node = base_velocity_node["ranges"];
  if (!ranges_node || !ranges_node.IsMap()) {
    throw std::runtime_error(
        "Missing required 'commands.base_velocity.ranges' section in config");
  }
  policy_config_.vel_cmd_ranges.lin_vel_x =
      parseVelocityRange(ranges_node, "lin_vel_x");
  policy_config_.vel_cmd_ranges.lin_vel_y =
      parseVelocityRange(ranges_node, "lin_vel_y");
  policy_config_.vel_cmd_ranges.ang_vel_z =
      parseVelocityRange(ranges_node, "ang_vel_z");
}

bool PolicyService::init() {
  loadParameters();

  // Pre-allocate vectors based on num_joints
  const int num_joints = policy_config_.num_joints;
  observations_.joint_pos.resize(num_joints, 0.0f);
  observations_.joint_vel.resize(num_joints, 0.0f);
  prev_actions_.resize(num_joints, 0.0f);

  // Initialize observation history
  const int num_terms =
      static_cast<int>(policy_config_.obs_terms.size());
  obs_history_ = std::make_unique<ObservationHistory>(
      policy_config_.obs_history_length, num_terms);

  // Compute input size: sum of all term sizes * history_length
  int single_step_size = 0;
  for (const auto &term : policy_config_.obs_terms) {
    if (term == "vel_cmd" || term == "ang_vel" || term == "proj_gravity") {
      single_step_size += 3;
    } else if (term == "joint_pos" || term == "joint_vel" ||
               term == "prev_actions") {
      single_step_size += num_joints;
    } else if (term == "gait_phase") {
      single_step_size += 2;
    } else {
      throw std::runtime_error("Unknown observation term: " + term);
    }
  }
  const int input_size = single_step_size * policy_config_.obs_history_length;

  // Precompute per-tick phase increment: (1 / loop_hz) / phase_period_s.
  has_gait_phase_ =
      std::find(policy_config_.obs_terms.begin(),
                policy_config_.obs_terms.end(),
                std::string("gait_phase")) != policy_config_.obs_terms.end();
  phase_ = 0.0f;
  phase_increment_ =
      has_gait_phase_
          ? (1.0f / static_cast<float>(policy_config_.loop_hz)) /
                policy_config_.phase_period_s
          : 0.0f;

  // Load policy model
  try {
    policy_ = std::make_unique<MnnPolicy>(policy_config_.policy_path,
                                          input_size, num_joints);
  } catch (const std::exception &ex) {
    spdlog::error("Failed to load policy: {}", ex.what());
    return false;
  }

  // Setup transport
  command_pub_ =
      std::make_unique<transport::Publisher<transport::JointCommand>>(
          transport::JOINT_COMMAND_TOPIC);
  if (!command_pub_->is_valid()) {
    spdlog::error("Failed to create joint_command publisher");
    return false;
  }

  joint_state_sub_ =
      std::make_unique<transport::Subscriber<transport::JointState>>(
          transport::JOINT_STATE_TOPIC);
  if (!joint_state_sub_->is_valid()) {
    spdlog::error("Failed to create joint_state subscriber");
    return false;
  }

  imu_data_sub_ = std::make_unique<transport::Subscriber<transport::IMUData>>(
      transport::IMU_DATA_TOPIC);
  if (!imu_data_sub_->is_valid()) {
    spdlog::error("Failed to create imu_data subscriber");
    return false;
  }

  joystick_sub_ = std::make_unique<transport::Subscriber<transport::Joystick>>(
      transport::JOYSTICK_TOPIC);
  if (!joystick_sub_->is_valid()) {
    spdlog::error("Failed to create joystick subscriber");
    return false;
  }

  robot_state_sub_ =
      std::make_unique<transport::Subscriber<transport::RobotState>>(
          transport::ROBOT_STATE_TOPIC);
  if (!robot_state_sub_->is_valid()) {
    spdlog::error("Failed to create robot_state subscriber");
    return false;
  }

  // Set callbacks
  joint_state_sub_->set_callback(
      [this](const transport::JointState &msg) { handleJointState(msg); });
  imu_data_sub_->set_callback(
      [this](const transport::IMUData &msg) { handleImuData(msg); });
  joystick_sub_->set_callback(
      [this](const transport::Joystick &msg) { handleJoystick(msg); });
  robot_state_sub_->set_callback(
      [this](const transport::RobotState &msg) { handleRobotState(msg); });

  // Setup thread loop
  common::RealtimeConfig rt_config;
  rt_config.frequency_hz = policy_config_.loop_hz;
  rt_config.scheduler_policy = SCHED_FIFO;
  rt_config.scheduler_priority = 50;
  rt_config.cpu_affinity = -1;

  thread_loop_ = std::make_unique<common::ThreadLoop>(
      "PolicyLoop", rt_config, [this]() { controlLoop(); });

  spdlog::info("PolicyService initialized: num_joints={}, loop_hz={}, "
               "obs_history_length={}, obs_terms={}",
               num_joints, policy_config_.loop_hz,
               policy_config_.obs_history_length,
               policy_config_.obs_terms.size());
  return true;
}

void PolicyService::start() {
  if (thread_loop_) {
    thread_loop_->start();
    spdlog::info("PolicyService started");
  }
}

void PolicyService::stop() {
  if (thread_loop_) {
    thread_loop_->stop();
    spdlog::info("PolicyService stopped");
  }

  command_pub_.reset();
  joint_state_sub_.reset();
  imu_data_sub_.reset();
  joystick_sub_.reset();
  robot_state_sub_.reset();
}

void PolicyService::controlLoop() {
  // Only run policy when in policy mode and have received sensor data
  if (!policy_mode_.load(std::memory_order_relaxed) ||
      !has_joint_state_.load(std::memory_order_relaxed) ||
      !has_imu_data_.load(std::memory_order_relaxed)) {
    return;
  }

  try {
    auto obs_terms = buildObservationTerms();
    obs_history_->push(obs_terms);
    auto obs_vector = obs_history_->flatten();

    auto raw_actions = policy_->infer(obs_vector);

    // Store raw actions for next iteration
    prev_actions_ = raw_actions;

    // action_scale and default_joint_pos are in actuator order; index via map to BFS.
    const int num_joints = policy_config_.num_joints;
    const auto &map = policy_config_.policy_to_actuator_map;
    std::vector<float> scaled_actions(num_joints);
    for (int i = 0; i < num_joints; ++i) {
      scaled_actions[i] = raw_actions[i] * policy_config_.action_scale[map[i]] +
                          policy_config_.default_joint_pos[map[i]];
    }

    publishActions(scaled_actions);
  } catch (const std::exception &ex) {
    spdlog::error("Inference error: {}", ex.what());
  }
}

std::vector<std::vector<float>> PolicyService::buildObservationTerms() {
  Observations obs_copy;
  {
    std::lock_guard<std::mutex> lock(obs_mutex_);
    obs_copy = observations_;
  }

  std::vector<std::vector<float>> terms;
  terms.reserve(policy_config_.obs_terms.size());

  for (const auto &name : policy_config_.obs_terms) {
    if (name == "vel_cmd") {
      terms.push_back({obs_copy.vel_cmd.begin(), obs_copy.vel_cmd.end()});
    } else if (name == "ang_vel") {
      terms.push_back({obs_copy.ang_vel.begin(), obs_copy.ang_vel.end()});
    } else if (name == "proj_gravity") {
      terms.push_back(
          {obs_copy.proj_gravity.begin(), obs_copy.proj_gravity.end()});
    } else if (name == "joint_pos") {
      terms.push_back(obs_copy.joint_pos);
    } else if (name == "joint_vel") {
      terms.push_back(obs_copy.joint_vel);
    } else if (name == "prev_actions") {
      terms.push_back(prev_actions_);
    } else if (name == "gait_phase") {
      phase_ = std::fmod(phase_ + phase_increment_, 1.0f);
      const float angle = 2.0f * static_cast<float>(M_PI) * phase_;
      terms.push_back({std::sin(angle), std::cos(angle)});
    }
  }

  return terms;
}

void PolicyService::publishActions(const std::vector<float> &actions) {
  if (!command_pub_) {
    return;
  }

  const int num_joints = policy_config_.num_joints;
  const auto &map = policy_config_.policy_to_actuator_map;

  // actions are in BFS order; map back to actuator order for the wire.
  std::vector<float> actuator_positions(num_joints);
  for (int i = 0; i < num_joints; ++i) {
    actuator_positions[map[i]] = actions[i];
  }

  std::vector<float> zeros(num_joints, 0.0f);

  transport::JointCommand cmd;
  cmd.position = actuator_positions;
  cmd.velocity = zeros;
  cmd.torque = zeros;
  cmd.kp = policy_config_.kp;
  cmd.kd = policy_config_.kd;

  command_pub_->publish(cmd);
}

void PolicyService::handleJointState(const transport::JointState &msg) {
  const int num_joints = policy_config_.num_joints;
  const auto &map = policy_config_.policy_to_actuator_map;

  std::lock_guard<std::mutex> lock(obs_mutex_);

  const auto &pos_vec = msg.position;
  const auto &vel_vec = msg.velocity;

  // Reorder actuator-order state into BFS for the policy.
  for (int i = 0; i < num_joints; ++i) {
    observations_.joint_pos[i] =
        pos_vec[map[i]] - policy_config_.default_joint_pos[map[i]];
    observations_.joint_vel[i] = vel_vec[map[i]];
  }

  has_joint_state_.store(true, std::memory_order_relaxed);
}

void PolicyService::handleImuData(const transport::IMUData &msg) {
  {
    std::lock_guard<std::mutex> lock(obs_mutex_);

    const auto &gyro = msg.gyroscope;
    for (std::size_t i = 0; i < observations_.ang_vel.size(); ++i) {
      observations_.ang_vel[i] = i < gyro.size() ? gyro[i] : 0.0f;
    }

    observations_.proj_gravity = computeProjectedGravity(msg.quaternion);
  }
  has_imu_data_.store(true, std::memory_order_relaxed);
}

void PolicyService::handleJoystick(const transport::Joystick &msg) {
  std::lock_guard<std::mutex> lock(obs_mutex_);
  // vel_cmd: [forward (left_y), lateral (right_x), yaw (left_x)]
  // Negated to match training convention, then scaled into configured ranges.
  const auto &ranges = policy_config_.vel_cmd_ranges;
  observations_.vel_cmd = {scaleToRange(-msg.left_stick_y, ranges.lin_vel_x),
                           scaleToRange(-msg.right_stick_x, ranges.lin_vel_y),
                           scaleToRange(-msg.left_stick_x, ranges.ang_vel_z)};
}

void PolicyService::handleRobotState(const transport::RobotState &msg) {
  const bool now_policy = msg.mode == POLICY_MODE_VALUE;
  const bool was_policy =
      policy_mode_.exchange(now_policy, std::memory_order_relaxed);
  if (now_policy && !was_policy) {
    std::lock_guard<std::mutex> lock(obs_mutex_);
    phase_ = 0.0f;
  }
}

} // namespace policy_service
