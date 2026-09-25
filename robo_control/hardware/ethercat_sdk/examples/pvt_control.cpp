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

  // Set PVT operation mode
  master.setOperationMode(ethercat_sdk::OperationMode::PVT);

  if (!master.init())
    return 1;

  master.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  master.enableAll();

  double init_pos = master.getActuatorState(1).position;

  // Impedance gains
  double kp = 50.0, kd = 2.0;

  // Control loop
  auto t0 = std::chrono::steady_clock::now();
  while (running) {
    double t =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
            .count();

    double pos = init_pos + 0.5 * std::sin(2.0 * M_PI * 0.5 * t);
    master.setPVT(1, pos, 0.0, 0.0, kp, kd);

    auto state = master.getActuatorState(1);
    std::cout << "\rpos=" << state.position << " tau=" << state.torque
              << " temp=" << state.motor_temp << "C" << std::flush;

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  std::cout << std::endl;
  master.disableAll();
  master.stop();
  return 0;
}
