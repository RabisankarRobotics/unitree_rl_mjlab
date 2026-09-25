
#include "transport/transport.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <thread>
#include <vector>

std::atomic<bool> running(true);

void signal_handler(int signal) {
  std::cout << "\nShutting down gracefully..." << std::endl;
  running = false;
}

int main() {
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  std::cout << "Publishing Joint Commands (100 Hz)" << std::endl;

  transport::Publisher<transport::JointCommand> cmd_publisher(
      transport::JOINT_COMMAND_TOPIC);

  if (!cmd_publisher.is_valid()) {
    std::cerr << "Failed to create publisher!" << std::endl;
    return 1;
  }

  std::cout << "Publisher created successfully" << std::endl;
  std::cout << "Press Ctrl+C to exit" << std::endl;

  const uint32_t num_joints = 12;

  auto start_time = std::chrono::steady_clock::now();

  while (running) {
    transport::JointCommand cmd;

    auto now = std::chrono::steady_clock::now();
    auto duration = now - start_time;
    double t = std::chrono::duration<double>(duration).count();

    std::vector<float> positions(num_joints);
    std::vector<float> velocities(num_joints);
    std::vector<float> torques(num_joints, 0.0f);
    std::vector<float> kp(num_joints, 100.0f);
    std::vector<float> kd(num_joints, 10.0f);

    for (uint32_t j = 0; j < num_joints; ++j) {
      positions[j] = 0.1f * std::sin(2.0 * M_PI * 0.5 * t + j * M_PI / 6);
      velocities[j] = 0.1f * 2.0 * M_PI * 0.5 *
                      std::cos(2.0 * M_PI * 0.5 * t + j * M_PI / 6);
    }

    cmd.position(positions);
    cmd.velocity(velocities);
    cmd.torque(torques);
    cmd.kp(kp);
    cmd.kd(kd);

    if (cmd_publisher.publish(cmd)) {
      std::cout << "Published command at t=" << t << "s" << std::endl;
    } else {
      std::cerr << "Failed to publish command!" << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  std::cout << "Publisher stopped" << std::endl;
  return 0;
}
