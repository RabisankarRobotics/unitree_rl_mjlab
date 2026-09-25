#include "can_sdk/master.hpp"

#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>

static volatile bool running = true;
void sighandler(int) { running = false; }

int main(int argc, char *argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: sudo " << argv[0] << " <can_interface>" << std::endl;
    std::cerr << "Example: sudo " << argv[0] << " can0" << std::endl;
    return 1;
  }

  std::signal(SIGINT, sighandler);

  const std::string interface = argv[1];
  const uint16_t joint_id = 1;

  // Initialize CAN master
  can_sdk::CanMaster master;

  std::cout << "Initializing CAN interface: " << interface << std::endl;
  if (!master.init(interface)) {
    std::cerr << "Failed to initialize CAN interface" << std::endl;
    return 1;
  }

  // Add joint to track
  master.addJoint(joint_id);

  // Get initial position from motor status
  master.requestMotorStatus(joint_id);
  master.recvAll(2000); // 2ms timeout

  double init_pos = 0.0;
  auto status = master.getMotorStatus(joint_id);
  auto age = std::chrono::steady_clock::now() - status.timestamp;
  if (age < std::chrono::milliseconds(10)) {
    init_pos = status.position;
    std::cout << "Initial position: " << init_pos << " rad" << std::endl;
  } else {
    std::cout << "Warning: No status response, using 0.0 rad" << std::endl;
  }

  // Control parameters
  const double amplitude = 1.0; // rad
  const double frequency = 0.5; // Hz
  const double kp = 50.0;
  const double kd = 2.0;

  std::cout << "\nStarting sinusoidal position control..." << std::endl;
  std::cout << "Press Ctrl+C to stop\n" << std::endl;

  // Control loop
  auto t0 = std::chrono::steady_clock::now();
  auto last_loop = t0;

  while (running) {
    auto loop_start = std::chrono::steady_clock::now();

    // Calculate time
    double t = std::chrono::duration<double>(loop_start - t0).count();

    // Generate sinusoidal target position
    double target_pos =
        init_pos + amplitude * std::sin(2.0 * M_PI * frequency * t);

    // Send motion command
    can_sdk::MotionCommand cmd;
    cmd.position = target_pos;
    cmd.velocity = 0.0;
    cmd.kp = kp;
    cmd.kd = kd;
    cmd.torque = 0.0;

    if (!master.sendMotionCommand(joint_id, cmd)) {
      std::cerr << "\nFailed to send command" << std::endl;
      break;
    }

    // Receive all pending responses
    master.recvAll(500); // 500us timeout per frame

    // Get feedback and check staleness
    auto feedback = master.getMotionFeedback(joint_id);
    auto feedback_age = std::chrono::steady_clock::now() - feedback.timestamp;

    if (feedback_age < std::chrono::milliseconds(4)) {
      std::cout << "\rPos: " << feedback.position
                << " | Vel: " << feedback.velocity
                << " | Torque: " << feedback.torque << "   " << std::flush;
    } else {
      std::cout << "\rWarning: Stale data by "
                << std::chrono::duration_cast<std::chrono::microseconds>(
                       feedback_age)
                           .count() /
                       1000.0
                << "ms" << std::flush;
    }

    // Sleep to maintain control frequency (100Hz)
    auto loop_end = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
        loop_end - loop_start);
    auto sleep_time = std::chrono::microseconds(10000) - elapsed;

    if (sleep_time.count() > 0) {
      // Simple busy-wait for more accurate timing
      while (std::chrono::steady_clock::now() - loop_start <
             std::chrono::microseconds(10000)) {
      }
    }
  }

  std::cout << "\n\nShutting down..." << std::endl;

  // Send zero command before shutdown
  can_sdk::MotionCommand zero_cmd{};
  master.sendMotionCommand(joint_id, zero_cmd);

  master.close();
  std::cout << "Done." << std::endl;

  return 0;
}
