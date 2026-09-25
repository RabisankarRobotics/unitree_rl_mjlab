#include "ethercat_sdk/master.hpp"

#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <thread>

static volatile bool running = true;
void sighandler(int) { running = false; }

int main(int argc, char *argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: sudo " << argv[0] << " <interface>" << std::endl;
    return 1;
  }

  std::signal(SIGINT, sighandler);

  ethercat_sdk::EtherCATMaster master(argv[1]);

  // Configure actuator types for discovered slaves
  master.configActuatorTypes({{1, "X8-120"}});

  // CST mode - the drive tracks the commanded torque directly
  master.setOperationMode(ethercat_sdk::OperationMode::CST);

  if (!master.init())
    return 1;

  master.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Clamp drive output as a safety limit before enabling
  master.setMaxTorque(1, 8.0);
  master.enableAll();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  auto s0 = master.getActuatorState(1);
  std::cout << "mode=" << int(s0.mode) << " enabled=" << s0.enabled
            << " error=0x" << std::hex << s0.error_code << std::dec << "\n";
  if (!s0.enabled)
    std::cerr << "Warning: actuator not enabled (check faults/power)\n";

  double center_pos = s0.position;

  // Software impedance in torque space, tracking a slow sine sweep
  double kp = 15.0, kd = 0.5, tau_limit = 6.0;
  double amplitude = 0.3, freq = 0.2;

  auto t0 = std::chrono::steady_clock::now();
  while (running) {
    double t =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
            .count();
    double target = center_pos + amplitude * std::sin(2.0 * M_PI * freq * t);

    auto state = master.getActuatorState(1);
    double tau = kp * (target - state.position) - kd * state.velocity;
    tau = std::max(-tau_limit, std::min(tau_limit, tau));

    master.setTorque(1, tau);

    std::cout << "\rtarget=" << target << " pos=" << state.position
              << " tau_cmd=" << tau << " tau_act=" << state.torque
              << std::flush;

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  std::cout << std::endl;
  master.setTorque(1, 0.0);
  master.disableAll();
  master.stop();
  return 0;
}
