// Minimal ankle control TUI for validating the Tahiti C1 parallel ankle
// transmission on real hardware. Python reference: ../linkage-transmission/ankle_control.py
// (CAN + MIT motion mode). Here we drive two MyActuator motors over EtherCAT
// in PVT mode, which exposes the same on-board kp/kd impedance as MIT mode.

#include "ethercat_sdk/master.hpp"
#include "transmission_sdk/ankle_transmission_tahiti_c1.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <sys/select.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

namespace {

// --- Hardware defaults (override via CLI) ---
constexpr int kDefaultMotorABus = 1;
constexpr int kDefaultMotorBBus = 2;
constexpr double kMotorADir = -1.0; // +1/-1 flip if motor spins opposite to transmission convention
constexpr double kMotorBDir = -1.0;
constexpr double kKp = 200.0;
constexpr double kKd = 12.0;

// --- Control ---
constexpr double kStepRad = 1.0 * M_PI / 180.0; // 1 deg per keypress
constexpr auto kLoopPeriod = std::chrono::milliseconds(20); // 50 Hz

// --- ANSI escape codes ---
constexpr const char *kClear = "\033[2J";
constexpr const char *kHome = "\033[H";
constexpr const char *kBold = "\033[1m";
constexpr const char *kReset = "\033[0m";
constexpr const char *kHideCursor = "\033[?25l";
constexpr const char *kShowCursor = "\033[?25h";

volatile std::sig_atomic_t g_running = 1;
void sigintHandler(int) { g_running = 0; }

// ---------------------------------------------------------------------------
// Raw terminal RAII. Enters cbreak (no echo, non-blocking reads) on construct,
// restores original settings on destruct.
// ---------------------------------------------------------------------------
class RawTerminal {
public:
  RawTerminal() {
    if (tcgetattr(STDIN_FILENO, &original_) != 0)
      return;
    termios raw = original_;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    active_ = true;
  }
  ~RawTerminal() {
    if (active_)
      tcsetattr(STDIN_FILENO, TCSADRAIN, &original_);
  }
  RawTerminal(const RawTerminal &) = delete;
  RawTerminal &operator=(const RawTerminal &) = delete;

private:
  termios original_{};
  bool active_{false};
};

int readKey() {
  unsigned char c;
  ssize_t n = ::read(STDIN_FILENO, &c, 1);
  return n == 1 ? static_cast<int>(c) : -1;
}

// ---------------------------------------------------------------------------
// Display helpers
// ---------------------------------------------------------------------------
struct MotorDisplay {
  double pos_deg = 0.0;
  double vel_deg = 0.0;
  double torque_nm = 0.0;
};

struct JointDisplay {
  double pitch_pos_deg = 0.0;
  double pitch_vel_deg = 0.0;
  double pitch_torque = 0.0;
  double roll_pos_deg = 0.0;
  double roll_vel_deg = 0.0;
  double roll_torque = 0.0;
};

void draw(const MotorDisplay &ma, const MotorDisplay &mb, const JointDisplay &js,
          double pitch_target_deg, double roll_target_deg,
          const std::string &status) {
  std::printf("%s", kHome);
  auto line = [](const std::string &s) {
    std::printf("\r%-80s\n", s.c_str());
  };
  char buf[128];

  std::snprintf(buf, sizeof(buf),
                " %sANKLE TRANSMISSION HARDWARE VALIDATOR%s", kBold, kReset);
  line(buf);
  line(" ────────────"
       "────────────"
       "────");
  line("");
  std::snprintf(buf, sizeof(buf),
                " %sMOTOR SPACE%s            Position    Velocity      Torque",
                kBold, kReset);
  line(buf);
  std::snprintf(buf, sizeof(buf),
                "  Motor A            %9.2f°  %9.2f °/s  %8.3f Nm",
                ma.pos_deg, ma.vel_deg, ma.torque_nm);
  line(buf);
  std::snprintf(buf, sizeof(buf),
                "  Motor B            %9.2f°  %9.2f °/s  %8.3f Nm",
                mb.pos_deg, mb.vel_deg, mb.torque_nm);
  line(buf);
  line("");
  std::snprintf(buf, sizeof(buf),
                " %sJOINT SPACE%s            Position    Velocity      Torque",
                kBold, kReset);
  line(buf);
  std::snprintf(buf, sizeof(buf),
                "  Pitch              %9.2f°  %9.2f °/s  %8.3f Nm",
                js.pitch_pos_deg, js.pitch_vel_deg, js.pitch_torque);
  line(buf);
  std::snprintf(buf, sizeof(buf),
                "  Roll               %9.2f°  %9.2f °/s  %8.3f Nm",
                js.roll_pos_deg, js.roll_vel_deg, js.roll_torque);
  line(buf);
  line("");
  std::snprintf(buf, sizeof(buf), " %sCOMMAND TARGETS%s", kBold, kReset);
  line(buf);
  std::snprintf(buf, sizeof(buf), "  Pitch target:  %8.2f°",
                pitch_target_deg);
  line(buf);
  std::snprintf(buf, sizeof(buf), "  Roll  target:  %8.2f°",
                roll_target_deg);
  line(buf);
  line("");
  if (!status.empty()) {
    std::snprintf(buf, sizeof(buf), "  %s%s%s", kBold, status.c_str(), kReset);
    line(buf);
  } else {
    line("");
  }
  line("");
  line(" [W/S] Pitch +/-1°  [A/D] Roll +/-1°  [Z] Zero  [R] Read-Only "
       " [SPACE] E-Stop  [Q] Quit");
  std::fflush(stdout);
}

void printUsage(const char *prog) {
  std::cerr
      << "Usage: sudo " << prog
      << " <iface> [--motor-a ID] [--motor-b ID] [--type NAME]\n"
      << "\n"
      << "  --motor-a ID   EtherCAT bus id of motor A (default: 1)\n"
      << "  --motor-b ID   EtherCAT bus id of motor B (default: 2)\n"
      << "  --type NAME    Actuator type (default: X8-120)\n";
}

} // namespace

int main(int argc, char *argv[]) {
  std::string interface;
  int motor_a_bus = kDefaultMotorABus;
  int motor_b_bus = kDefaultMotorBBus;
  std::string actuator_type = "X8-120";

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--motor-a" && i + 1 < argc) {
      motor_a_bus = std::atoi(argv[++i]);
    } else if (a == "--motor-b" && i + 1 < argc) {
      motor_b_bus = std::atoi(argv[++i]);
    } else if (a == "--type" && i + 1 < argc) {
      actuator_type = argv[++i];
    } else if (a == "-h" || a == "--help") {
      printUsage(argv[0]);
      return 0;
    } else if (!a.empty() && a[0] != '-') {
      interface = a;
    } else {
      std::cerr << "Unknown argument: " << a << "\n";
      printUsage(argv[0]);
      return 1;
    }
  }

  if (interface.empty()) {
    printUsage(argv[0]);
    return 1;
  }
  if (motor_a_bus == motor_b_bus) {
    std::cerr << "Motor A and B must have different bus ids.\n";
    return 1;
  }

  std::signal(SIGINT, sigintHandler);

  // --- Bring up EtherCAT ---
  ethercat_sdk::EtherCATMaster master(interface);
  master.configActuatorTypes({
      {motor_a_bus, actuator_type},
      {motor_b_bus, actuator_type},
  });
  master.setOperationMode(ethercat_sdk::OperationMode::PVT);

  if (!master.init()) {
    std::cerr << "Failed to initialize EtherCAT on " << interface << "\n";
    return 1;
  }

  master.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  if (!master.isOperational()) {
    std::cerr << "EtherCAT master not operational.\n";
    master.stop();
    return 1;
  }

  // Command zero gains before enabling so motors don't jump on enable.
  master.setPVT(motor_a_bus, 0.0, 0.0, 0.0, 0.0, 0.0);
  master.setPVT(motor_b_bus, 0.0, 0.0, 0.0, 0.0, 0.0);
  master.enableAll();
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  transmission::AnkleTransmissionTahitiC1 tx;

  // --- Initial state: read motor positions at zero gain, back-solve q_s ---
  auto state_a0 = master.getActuatorState(motor_a_bus);
  auto state_b0 = master.getActuatorState(motor_b_bus);

  Eigen::VectorXd q_s_current = Eigen::VectorXd::Zero(2);
  {
    Eigen::VectorXd q_m(2);
    q_m << kMotorADir * state_a0.position, kMotorBDir * state_b0.position;
    Eigen::VectorXd q_init = Eigen::VectorXd::Zero(2);
    try {
      q_s_current = tx.position_inverse(q_m, q_init);
    } catch (const std::exception &e) {
      std::cerr << "Initial inverse kinematics failed: " << e.what()
                << " — starting targets at zero.\n";
    }
  }

  double pitch_target_rad = q_s_current(0);
  double roll_target_rad = q_s_current(1);
  Eigen::VectorXd last_q_s = q_s_current;

  MotorDisplay ma_disp{}, mb_disp{};
  JointDisplay joint_disp{};
  std::string status;
  bool e_stopped = false;
  bool readonly = false;

  // --- Enter raw terminal mode ---
  RawTerminal term;
  std::printf("%s%s%s", kClear, kHome, kHideCursor);
  std::fflush(stdout);

  // Reusable output buffers for no-alloc transmission calls.
  Eigen::VectorXd q_m_target(2);
  Eigen::VectorXd q_s_meas(2);
  Eigen::VectorXd dq_s_meas(2);
  Eigen::VectorXd tau_s_meas(2);

  while (g_running) {
    auto t0 = std::chrono::steady_clock::now();

    // --- Keyboard input ---
    int ch = readKey();
    if (ch != -1) {
      if (ch == 'q' || ch == 'Q' || ch == 0x1B) {
        break;
      } else if (ch == ' ') {
        master.disableAll();
        e_stopped = true;
        status = "E-STOP ACTIVE  (press Q to quit)";
      } else if (!e_stopped) {
        if (ch == 'r' || ch == 'R') {
          readonly = !readonly;
        } else if ((ch == 'w' || ch == 'W') && !readonly) {
          pitch_target_rad += kStepRad;
        } else if ((ch == 's' || ch == 'S') && !readonly) {
          pitch_target_rad -= kStepRad;
        } else if ((ch == 'd' || ch == 'D') && !readonly) {
          roll_target_rad += kStepRad;
        } else if ((ch == 'a' || ch == 'A') && !readonly) {
          roll_target_rad -= kStepRad;
        } else if ((ch == 'z' || ch == 'Z') && !readonly) {
          pitch_target_rad = 0.0;
          roll_target_rad = 0.0;
        }
      }
    }

    if (e_stopped) {
      draw(ma_disp, mb_disp, joint_disp, pitch_target_rad * 180.0 / M_PI,
           roll_target_rad * 180.0 / M_PI, status);
      std::this_thread::sleep_until(t0 + kLoopPeriod);
      continue;
    }

    // --- Compute motor targets via transmission ---
    status.clear();
    double cmd_kp = 0.0, cmd_kd = 0.0;
    double cmd_a = 0.0, cmd_b = 0.0;

    if (readonly) {
      status = "READ-ONLY (zero torque)";
    } else {
      Eigen::VectorXd q_s_target(2);
      q_s_target << pitch_target_rad, roll_target_rad;
      try {
        tx.position_forward_into(q_s_target, q_m_target);
        cmd_a = kMotorADir * q_m_target(0);
        cmd_b = kMotorBDir * q_m_target(1);
        cmd_kp = kKp;
        cmd_kd = kKd;
      } catch (const std::exception &e) {
        status = std::string("WORKSPACE LIMIT: ") + e.what();
      }
    }

    master.setPVT(motor_a_bus, cmd_a, 0.0, 0.0, cmd_kp, cmd_kd);
    master.setPVT(motor_b_bus, cmd_b, 0.0, 0.0, cmd_kp, cmd_kd);

    // --- Read motor state ---
    auto state_a = master.getActuatorState(motor_a_bus);
    auto state_b = master.getActuatorState(motor_b_bus);

    double qA = kMotorADir * state_a.position;
    double qB = kMotorBDir * state_b.position;
    double dqA = kMotorADir * state_a.velocity;
    double dqB = kMotorBDir * state_b.velocity;
    double tauA = kMotorADir * state_a.torque;
    double tauB = kMotorBDir * state_b.torque;

    ma_disp = {qA * 180.0 / M_PI, dqA * 180.0 / M_PI, tauA};
    mb_disp = {qB * 180.0 / M_PI, dqB * 180.0 / M_PI, tauB};

    if (state_a.lost || state_b.lost) {
      if (!status.empty())
        status += " | ";
      status += (state_a.lost ? "Motor A LOST " : "");
      status += (state_b.lost ? "Motor B LOST" : "");
    }

    // --- Joint-space feedback via transmission ---
    Eigen::VectorXd q_m_actual(2);
    q_m_actual << qA, qB;
    Eigen::VectorXd dq_m_actual(2);
    dq_m_actual << dqA, dqB;
    Eigen::VectorXd tau_m_actual(2);
    tau_m_actual << tauA, tauB;

    try {
      tx.position_inverse_into(q_m_actual, last_q_s, q_s_meas);
      last_q_s = q_s_meas;
      tx.velocity_inverse_into(q_s_meas, dq_m_actual, dq_s_meas);
      tx.torque_inverse_into(q_s_meas, tau_m_actual, tau_s_meas);

      joint_disp = {
          q_s_meas(0) * 180.0 / M_PI,  dq_s_meas(0) * 180.0 / M_PI,
          tau_s_meas(0),               q_s_meas(1) * 180.0 / M_PI,
          dq_s_meas(1) * 180.0 / M_PI, tau_s_meas(1),
      };
    } catch (const std::exception &) {
      if (status.empty())
        status = "INVERSE KINEMATICS FAILED";
    }

    draw(ma_disp, mb_disp, joint_disp, pitch_target_rad * 180.0 / M_PI,
         roll_target_rad * 180.0 / M_PI, status);

    std::this_thread::sleep_until(t0 + kLoopPeriod);
  }

  // --- Shutdown ---
  master.disableAll();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  master.stop();

  std::printf("%s\n", kShowCursor);
  std::fflush(stdout);
  return 0;
}
