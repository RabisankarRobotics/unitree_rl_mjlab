#include "transport/transport.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>

std::atomic<bool> running(true);

void signal_handler(int signal) {
  std::cout << "\nShutting down gracefully..." << std::endl;
  running = false;
}

void joint_command_callback(const transport::JointCommand &cmd) {
  std::cout << "\n=== Joint Command Update ===" << std::endl;

  const auto &positions = cmd.position();
  const auto &velocities = cmd.velocity();
  const auto &torques = cmd.torque();
  const auto &kp_vals = cmd.kp();
  const auto &kd_vals = cmd.kd();

  // Find the number of joints (minimum size of all vectors)
  size_t num_joints =
      std::min({positions.size(), velocities.size(), torques.size(),
                kp_vals.size(), kd_vals.size()});

  if (num_joints == 0) {
    std::cout << "Warning: No joint data available" << std::endl;
    return;
  }

  std::cout << "Number of joints: " << num_joints << std::endl;

  size_t display_count = std::min(static_cast<size_t>(3), num_joints);

  std::cout << std::fixed << std::setprecision(3);
  for (size_t i = 0; i < display_count; ++i) {
    std::cout << "Joint " << i << ": "
              << "pos=" << positions[i] << " rad, "
              << "vel=" << velocities[i] << " rad/s, "
              << "torque=" << torques[i] << " Nm, "
              << "kp=" << kp_vals[i] << ", "
              << "kd=" << kd_vals[i] << std::endl;
  }
  if (num_joints > display_count) {
    std::cout << "... (" << (num_joints - display_count) << " more joints)"
              << std::endl;
  }
}

int main() {
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  std::cout << "Subscribing to Joint Commands" << std::endl;

  transport::Subscriber<transport::JointCommand> command_subscriber(
      transport::JOINT_COMMAND_TOPIC);

  if (!command_subscriber.is_valid()) {
    std::cerr << "Failed to create subscriber!" << std::endl;
    return 1;
  }

  std::cout << "Subscriber created successfully" << std::endl;

  command_subscriber.set_callback(joint_command_callback);

  std::cout << "Listening for joint commands (Press Ctrl+C to exit)..."
            << std::endl;

  while (running) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::cout << "Subscriber stopped" << std::endl;
  return 0;
}
