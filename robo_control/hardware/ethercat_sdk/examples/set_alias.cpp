#include "ethercat_sdk/master.hpp"

#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>

namespace {

constexpr int kBusId = 1;

void printUsage(const char *prog) {
  std::cerr
      << "Usage:\n"
      << "  " << prog << " <iface>                          # interactive\n"
      << "  " << prog << " <iface> --read                   # read-only\n"
      << "  " << prog << " <iface> --alias <value> [--yes]  # scripted write\n"
      << "\n"
      << "Only a single slave is supported (bus_id = 1). Connect exactly one\n"
      << "actuator. Alias values are 0..65535 (decimal or 0x hex). The slave\n"
      << "must be power-cycled for the new alias to take effect.\n";
}

std::optional<uint32_t> parseUint(const std::string &s) {
  if (s.empty())
    return std::nullopt;
  try {
    size_t pos = 0;
    unsigned long v = std::stoul(s, &pos, 0); // auto-detect base
    if (pos != s.size())
      return std::nullopt;
    return static_cast<uint32_t>(v);
  } catch (...) {
    return std::nullopt;
  }
}

void printSlaveInfo(ethercat_sdk::EtherCATMaster &master,
                    const ethercat_sdk::SlaveTopologyInfo &s,
                    uint16_t live_alias) {
  std::cout << "Slave on bus:\n"
            << "  config address  : 0x" << std::hex << std::setw(4)
            << std::setfill('0') << s.config_address << "\n"
            << "  alias (scan)    : 0x" << std::setw(4) << s.alias_address
            << "\n"
            << "  alias (live)    : 0x" << std::setw(4) << live_alias << "\n"
            << "  vendor:product  : " << std::dec << std::setfill(' ')
            << s.vendor_id << ":" << s.product_code << "\n"
            << "  serial          : 0x" << std::hex << std::setw(8)
            << std::setfill('0') << s.serial << "\n"
            << "  name            : " << std::dec << std::setfill(' ')
            << s.name << "\n";
  (void)master;
}

bool enforceSingleSlave(ethercat_sdk::EtherCATMaster &master) {
  int n = master.getSlaveCount();
  if (n == 0) {
    std::cerr << "No slaves found on the bus.\n";
    return false;
  }
  if (n != 1) {
    std::cerr << "Expected exactly 1 slave, found " << n
              << ". Disconnect all but the actuator you want to configure.\n";
    return false;
  }
  return true;
}

int runRead(ethercat_sdk::EtherCATMaster &master) {
  auto topo = master.getTopology();
  auto live = master.readAlias(kBusId);
  if (!live) {
    std::cerr << "Failed to read alias from slave.\n";
    return 1;
  }
  printSlaveInfo(master, topo.front(), *live);
  return 0;
}

int writeAndReport(ethercat_sdk::EtherCATMaster &master, uint16_t new_alias,
                   uint16_t current) {
  std::cout << "Writing alias: 0x" << std::hex << std::setw(4)
            << std::setfill('0') << current << " -> 0x" << std::setw(4)
            << new_alias << std::dec << std::setfill(' ') << "\n";
  if (!master.writeAlias(kBusId, new_alias)) {
    std::cerr << "Write failed.\n";
    return 1;
  }
  std::cout << "OK. Power-cycle the slave for the new alias to take effect.\n";
  return 0;
}

int runInteractive(ethercat_sdk::EtherCATMaster &master) {
  auto topo = master.getTopology();
  auto current = master.readAlias(kBusId);
  if (!current) {
    std::cerr << "Failed to read current alias.\n";
    return 1;
  }
  printSlaveInfo(master, topo.front(), *current);

  std::cout << "\nNew alias (0..65535, decimal or 0x hex, or 'q' to quit): "
            << std::flush;
  std::string line;
  if (!std::getline(std::cin, line))
    return 0;
  if (line == "q" || line == "Q" || line.empty())
    return 0;

  auto new_opt = parseUint(line);
  if (!new_opt || *new_opt > 0xFFFF) {
    std::cerr << "Invalid alias value (must fit in uint16).\n";
    return 1;
  }
  uint16_t new_alias = static_cast<uint16_t>(*new_opt);

  std::cout << "Confirm write 0x" << std::hex << std::setw(4)
            << std::setfill('0') << *current << " -> 0x" << std::setw(4)
            << new_alias << std::dec << std::setfill(' ')
            << " (type 'yes' to proceed): " << std::flush;
  if (!std::getline(std::cin, line) || line != "yes") {
    std::cout << "Aborted.\n";
    return 0;
  }

  return writeAndReport(master, new_alias, *current);
}

} // namespace

int main(int argc, char *argv[]) {
  if (argc < 2) {
    printUsage(argv[0]);
    return 1;
  }

  std::string interface;
  bool read_only = false;
  bool assume_yes = false;
  std::optional<uint32_t> flag_alias;

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--read" || a == "--list") {
      read_only = true;
    } else if (a == "--yes" || a == "-y") {
      assume_yes = true;
    } else if (a == "--alias" && i + 1 < argc) {
      auto v = parseUint(argv[++i]);
      if (!v || *v > 0xFFFF) {
        std::cerr << "Invalid --alias value (must fit in uint16).\n";
        return 1;
      }
      flag_alias = *v;
    } else if (a == "-h" || a == "--help") {
      printUsage(argv[0]);
      return 0;
    } else if (!a.empty() && a[0] == '-') {
      std::cerr << "Unknown option: " << a << "\n";
      printUsage(argv[0]);
      return 1;
    } else {
      interface = a;
    }
  }

  if (interface.empty()) {
    std::cerr << "Missing interface name.\n";
    printUsage(argv[0]);
    return 1;
  }

  ethercat_sdk::EtherCATMaster master(interface);
  if (!master.init()) {
    std::cerr << "Failed to initialize EtherCAT on " << interface << "\n";
    return 1;
  }
  if (!enforceSingleSlave(master)) {
    return 1;
  }

  if (read_only) {
    return runRead(master);
  }

  if (flag_alias) {
    auto current = master.readAlias(kBusId);
    if (!current) {
      std::cerr << "Failed to read current alias.\n";
      return 1;
    }
    uint16_t new_alias = static_cast<uint16_t>(*flag_alias);
    if (!assume_yes) {
      std::cerr << "Refusing to write without --yes.\n";
      return 1;
    }
    return writeAndReport(master, new_alias, *current);
  }

  return runInteractive(master);
}
