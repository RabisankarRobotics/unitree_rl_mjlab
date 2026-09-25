#pragma once

#include "common/log_throttle.hpp"
#include "common/thread_loop.hpp"
#include "control_service/actuator_backend.hpp"
#include "control_service/config_types.hpp"
#include "control_service/interpolator.hpp"
#include "transmission_sdk/transmission_base.hpp"
#include "transport/transport.hpp"

#include <Eigen/Dense>
#include <array>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace YAML {
class Node;
}

namespace control_service {

class RobotController {
public:
  RobotController();
  ~RobotController();

  bool init();
  void start();
  void stop();

private:
  // --- High-level lifecycle ---
  void loadParameters();
  bool loadHardwareConfig();
  void controlLoop();
  void commandCallback(const transport::JointCommand &msg);
  void enterSafeMode(const std::string &reason);

  // --- init() helpers ---
  bool createBackends();
  bool buildTransmissions();
  bool buildJointTopology();
  void allocateRtBuffers();
  bool initTransport();
  void initThreadLoop();

  // --- start() helpers ---
  void seedTransmissionsFromBackends();
  void resetInterpolatorsFromBackends();

  // --- loadHardwareConfig() helpers ---
  bool parseActuatorTable(const YAML::Node &root,
                          std::unordered_map<int, ActuatorConfig> &out);
  bool parseBuses(const YAML::Node &root, const std::string &hardware_type,
                  const std::unordered_map<int, ActuatorConfig> &actuator_table);
  bool parseTransmissions(const YAML::Node &root);
  bool parseJoints(const YAML::Node &root);

  // --- commandCallback() helpers ---
  double clampJointPosition(std::size_t joint_idx, double pos) const;

  // --- controlLoop() helpers ---
  void readAllBackends();
  bool solveTransmissionInverses();
  void scatterJointStates();
  void publishJointState();
  void checkCommandWatchdog();
  void applyPendingCommand();
  void interpolateJointTargets();
  void writePassthroughCommands();
  void writeTransmissionCommands();
  void writeAllBackends();

  // --- Backends (one per bus) ---
  std::vector<std::unique_ptr<ActuatorBackend>> backends_;

  // --- Real-time loop ---
  std::unique_ptr<common::ThreadLoop> thread_loop_;

  // --- Transport ---
  std::unique_ptr<transport::Subscriber<transport::JointCommand>> command_sub_;
  std::unique_ptr<transport::Publisher<transport::JointState>> state_pub_;

  // --- Joint / actuator topology ---
  // joint_ids_ / joint_sources_ / interpolators_ are parallel arrays sized to
  // the topic's joint count. A JointSource is either a direct passthrough to
  // one actuator, or one slot of a transmission group.
  enum class JointKind { Passthrough, Transmission };

  struct JointSource {
    JointKind kind;
    int actuator_id = -1;
    std::size_t backend_idx = 0;
    std::size_t backend_slot_idx = 0;
    std::size_t group_idx = 0;
    std::size_t slot_idx = 0;
    int direction = 1;
    double zero_offset = 0.0;
  };

  struct TransmissionGroup {
    std::string name;
    std::vector<int> actuator_ids;
    std::vector<int> joint_ids;
    std::vector<std::size_t> joint_indices;
    std::vector<std::size_t> backend_indices;
    std::vector<std::size_t> backend_slots;
    std::vector<int> motor_directions;
    // Per-motor zero offset in motor space (after direction). Subtracted from
    // raw*direction before the inverse, added back after the forward.
    std::vector<double> motor_zero_offsets;
    std::unique_ptr<transmission::TransmissionBase> model;
    Eigen::VectorXd q_joint_seed;
    bool seed_valid = false;
    // Per-cycle cached max temperature across this group's actuators.
    // Written by solveTransmissionInverses, read by scatterJointStates.
    double max_temperature = 0.0;
    // Throttle state for per-cycle inverse/forward failure logs. A failing
    // workspace condition typically persists across many control cycles.
    common::LogThrottle inverse_err_throttle{std::chrono::seconds(1)};
    common::LogThrottle forward_err_throttle{std::chrono::seconds(1)};
  };

  std::vector<int> joint_ids_;
  std::vector<JointSource> joint_sources_;
  std::vector<TransmissionGroup> transmissions_;
  std::vector<JointInterpolator> interpolators_;

  // Joint-space position limits, parallel to joint_ids_.
  std::vector<double> joint_lower_limits_;
  std::vector<double> joint_upper_limits_;
  // Throttle state for out-of-range clip warnings, parallel to joint_ids_.
  mutable std::vector<common::LogThrottle> clip_warn_throttle_;
  // Throttle state for safe-mode enter/exit transition logs, which can flap
  // at command-callback rate when a transmission keeps failing.
  common::LogThrottle safe_mode_enter_throttle_{std::chrono::seconds(1)};
  common::LogThrottle safe_mode_exit_throttle_{std::chrono::seconds(1)};

  // --- Command handling ---
  std::mutex command_mutex_;
  std::chrono::steady_clock::time_point last_command_time_;
  bool first_command_received_ = false;

  double control_frequency_ = 1000.0;

  struct Command {
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> torque;
    std::vector<double> kp;
    std::vector<double> kd;
  };
  Command cached_command_;
  Command active_command_;

  int pending_steps_ = 1;
  bool pending_command_ready_ = false;

  // --- Safety ---
  static constexpr int64_t command_timeout_ms_ = 500;
  static constexpr int default_interpolation_steps_ = 100; // 100ms at 1kHz
  bool in_safe_mode_ = false;
  bool fault_detected_ = false;
  std::vector<int> faulted_joints_;

  // Init-only: actuator id -> (backend index, slot index within that backend).
  // Not consulted on the hot path.
  struct ActuatorLocation {
    std::size_t backend_idx;
    std::size_t slot_idx;
  };
  std::map<int, ActuatorLocation> actuator_location_;

  // --- Pre-allocated RT buffers (no heap alloc per cycle) ---
  std::vector<float> state_positions_;
  std::vector<float> state_velocities_;
  std::vector<float> state_torques_;
  std::vector<float> state_temperatures_;
  // Per-backend command buffers, sized to each bus's actuator count.
  std::vector<std::vector<BackendCommand>> commands_per_backend_;

  // Per-group motor-space scratch (sized to each group's dof).
  std::vector<Eigen::VectorXd> group_q_motor_;
  std::vector<Eigen::VectorXd> group_dq_motor_;
  std::vector<Eigen::VectorXd> group_tau_motor_;
  // Per-group joint-space outputs from inverse/forward.
  std::vector<Eigen::VectorXd> group_q_joint_;
  std::vector<Eigen::VectorXd> group_dq_joint_;
  std::vector<Eigen::VectorXd> group_tau_joint_;
  std::vector<Eigen::VectorXd> group_cmd_q_joint_;
  std::vector<Eigen::VectorXd> group_cmd_dq_joint_;
  std::vector<Eigen::VectorXd> group_cmd_tau_joint_;
  // Interpolated joint-space command positions, indexed by joint_ids_ order.
  std::vector<double> cmd_pos_;

  // --- Hardware configuration ---
  HardwareConfig hw_config_;
  std::string config_path_;

#ifdef CONTROL_LOOP_BENCHMARK
  // --- Control-loop benchmark ---
  // Per-stage timing accumulated over one reporting window, dumped once per
  // `bench_report_interval_` cycles. Resident in the object so the hot path
  // never allocates. Compiled in only when CONTROL_LOOP_BENCHMARK is defined.
  enum BenchStage {
    BENCH_READ = 0,
    BENCH_POS_INV,
    BENCH_VEL_INV,
    BENCH_TRQ_INV,
    BENCH_SCATTER,
    BENCH_PUBLISH,
    BENCH_CMD,
    BENCH_INTERP,
    BENCH_POS_FWD,
    BENCH_VEL_FWD,
    BENCH_TRQ_FWD,
    BENCH_WRITE_OTHER,
    BENCH_WRITE_BACKENDS,
    BENCH_TOTAL,
    BENCH_COUNT
  };
  struct StageStat {
    uint64_t sum_ns = 0;
    uint64_t min_ns = UINT64_MAX;
    uint64_t max_ns = 0;
  };
  std::array<StageStat, BENCH_COUNT> bench_stats_{};
  uint32_t bench_cycles_ = 0;
  uint32_t bench_report_interval_ = 1000;
#endif
};

} // namespace control_service
