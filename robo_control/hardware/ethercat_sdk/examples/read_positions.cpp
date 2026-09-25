#include "ethercat_sdk/master.hpp"

#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

static volatile bool running = true;
static void sighandler(int) { running = false; }

static void printTopology(const ethercat_sdk::EtherCATMaster &master) {
  auto topo = master.getTopology();
  std::cout << "EtherCAT chain topology (order = physical auto-increment):\n";
  std::cout << "idx  cfgadr  alias  vendor:product   revision  serial     "
               "topo  active consumed  parent(pPort,eP)  name\n";
  std::cout << "---  ------  -----  ----------------  --------  ---------- "
               "---- ------ --------  ----------------  ----\n";
  for (const auto &s : topo) {
    std::cout << std::dec << std::setfill(' ') << std::setw(3) << s.bus_id
              << "  0x" << std::hex << std::setfill('0') << std::setw(4)
              << s.config_address
              << "  " << std::dec << std::setfill(' ') << std::setw(5)
              << s.alias_address
              << "  " << std::hex << std::setfill('0') << std::setw(6)
              << s.vendor_id << ":"
              << std::setw(8) << s.product_code
              << "  0x" << std::setw(6) << s.revision
              << "  0x" << std::setw(8) << s.serial
              << "   " << std::dec << std::setfill(' ') << std::setw(2)
              << int(s.topology)
              << "   0x" << std::hex << std::setfill('0') << std::setw(2)
              << int(s.active_ports)
              << "    0x" << std::setw(2) << int(s.consumed_ports)
              << "      " << std::dec << std::setfill(' ') << std::setw(3)
              << s.parent << "(" << int(s.parent_port) << "," << int(s.entry_port)
              << ")          " << s.name << "\n";
  }
  std::cout << std::dec << std::setfill(' ') << std::flush;
}

int main(int argc, char *argv[]) {
  std::signal(SIGINT, sighandler);

  std::string interface = "enP3p49s0";
  int count = 7;
  if (argc > 1) {
    interface = argv[1];
  }
  if (argc > 2) {
    count = std::atoi(argv[2]);
  }

  ethercat_sdk::EtherCATMaster master(interface);

  std::map<int, std::string> type_map;
  for (int bus_id = 1; bus_id <= count; ++bus_id) {
    type_map[bus_id] = "X8-120";
  }
  master.configActuatorTypes(type_map);

  if (!master.init()) {
    std::cerr << "Failed to initialize EtherCAT on " << interface << std::endl;
    return 1;
  }

  const int slave_count = master.getSlaveCount();
  if (slave_count == 0) {
    std::cerr << "No slaves found" << std::endl;
    return 1;
  }

  std::cout << "Found " << slave_count << " slave(s) on " << interface << "\n";
  printTopology(master);

  // PVT with default kp=kd=tau=0 -> drive outputs zero torque, joints stay free.
  master.setOperationMode(ethercat_sdk::OperationMode::PVT);
  master.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  std::cout << "\nStreaming positions. Press Ctrl+C to stop.\n";
  std::cout << std::fixed << std::setprecision(6);

  bool first = true;
  while (running) {
    if (!first) {
      std::cout << "\033[" << slave_count << "A";
    }
    first = false;

    for (int bus_id = 1; bus_id <= slave_count; ++bus_id) {
      auto state = master.getActuatorState(bus_id);
      double deg = state.position * 180.0 / M_PI;
      std::cout << "slave " << std::setw(2) << bus_id
                << ": position = " << std::setw(10) << deg << " deg ("
                << std::setw(10) << state.position << " rad)\033[K\n";
    }
    std::cout << std::flush;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  master.stop();
  return 0;
}
