#include "ethercat_sdk/master.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>
#include <vector>

#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

static volatile bool g_running = true;
void sighandler(int) { g_running = false; }

// ---------------------------------------------------------------------------
// Statistics
// ---------------------------------------------------------------------------

struct PhaseStats {
  double min_us = 1e9;
  double max_us = 0.0;
  double sum_us = 0.0;
  double sum_sq_us = 0.0;
  int count = 0;
  std::vector<double> samples;

  void reserve(int n) { samples.reserve(n); }

  void addSample(double us) {
    min_us = std::min(min_us, us);
    max_us = std::max(max_us, us);
    sum_us += us;
    sum_sq_us += us * us;
    count++;
    samples.push_back(us);
  }

  double mean() const { return count > 0 ? sum_us / count : 0.0; }

  double stddev() const {
    if (count <= 1)
      return 0.0;
    double m = mean();
    double variance = (sum_sq_us / count) - (m * m);
    return std::sqrt(std::max(0.0, variance));
  }

  double percentile(double p) const {
    if (samples.empty())
      return 0.0;
    auto sorted = samples;
    std::sort(sorted.begin(), sorted.end());
    int idx = static_cast<int>(p / 100.0 * (sorted.size() - 1));
    return sorted[idx];
  }

  double p99() const { return percentile(99.0); }
};

struct FrequencyResult {
  int cycle_time_us;
  int total_cycles;
  int wkc_errors;

  PhaseStats total_cycle;
  PhaseStats jitter;
  PhaseStats wakeup_jitter;
  PhaseStats receive;
  PhaseStats txpdo_read;
  PhaseStats rxpdo_write;
  PhaseStats send;
  PhaseStats dc_sync;
  PhaseStats mailbox;
  PhaseStats dc_offset;
  PhaseStats backend_read;
  PhaseStats backend_write;

  void reserveAll(int n) {
    total_cycle.reserve(n);
    jitter.reserve(n);
    wakeup_jitter.reserve(n);
    receive.reserve(n);
    txpdo_read.reserve(n);
    rxpdo_write.reserve(n);
    send.reserve(n);
    dc_sync.reserve(n);
    mailbox.reserve(n);
    dc_offset.reserve(n);
    backend_read.reserve(n);
    backend_write.reserve(n);
  }
};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static double ns_to_us(int64_t ns) { return static_cast<double>(ns) / 1000.0; }

static void printPhase(const std::string &label, const PhaseStats &s,
                       const std::string &unit = "us") {
  if (s.count == 0)
    return;
  std::cout << std::left << std::setw(22) << label << std::right << std::fixed
            << std::setprecision(1) << "min=" << std::setw(8) << s.min_us
            << " max=" << std::setw(8) << s.max_us
            << " mean=" << std::setw(8) << s.mean()
            << " stddev=" << std::setw(7) << s.stddev()
            << " p99=" << std::setw(8) << s.p99() << " " << unit << std::endl;
}

struct HardwareInfo {
  std::string interface_name;
  int cycle_time_us;
  std::map<int, std::string> actuator_type_map; // bus_id -> type_name
};

static bool loadHardwareConfig(const std::string &config_path,
                               HardwareInfo &info) {
  try {
    YAML::Node root = YAML::LoadFile(config_path);

    std::string hardware_type = root["hardware_type"].as<std::string>();
    if (hardware_type != "ethercat") {
      std::cerr << "hardware_type is '" << hardware_type
                << "', expected 'ethercat'" << std::endl;
      return false;
    }

    auto actuators = root["actuators"];
    if (!actuators || !actuators.IsSequence() || actuators.size() == 0) {
      std::cerr << "Missing or empty top-level 'actuators' list" << std::endl;
      return false;
    }
    std::map<int, std::string> id_to_type;
    for (const auto &act : actuators) {
      id_to_type[act["id"].as<int>()] = act["type"].as<std::string>();
    }

    YAML::Node hw = root[hardware_type];
    if (!hw) {
      std::cerr << "Missing '" << hardware_type << "' section" << std::endl;
      return false;
    }
    info.cycle_time_us = hw["cycle_time_us"].as<int>(1000);

    auto ifaces = hw["interfaces"];
    if (!ifaces || ifaces.size() == 0) {
      std::cerr << "No interfaces found in config" << std::endl;
      return false;
    }

    auto iface = ifaces[0];
    info.interface_name = iface["name"].as<std::string>();

    for (const auto &entry : iface["actuators"]) {
      int id = entry["id"].as<int>();
      int bus_id = entry["bus_id"].as<int>();
      auto it = id_to_type.find(id);
      if (it == id_to_type.end()) {
        std::cerr << "Interface '" << info.interface_name
                  << "' references unknown actuator id " << id << std::endl;
        return false;
      }
      info.actuator_type_map[bus_id] = it->second;
    }

    return true;
  } catch (const std::exception &e) {
    std::cerr << "Failed to load config: " << e.what() << std::endl;
    return false;
  }
}

static std::vector<int> parseFrequencies(const std::string &str) {
  std::vector<int> result;
  std::stringstream ss(str);
  std::string token;
  while (std::getline(ss, token, ',')) {
    result.push_back(std::stoi(token));
  }
  return result;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(int argc, char *argv[]) {
  // Defaults
  std::string interface_override;
  std::string config_path;
  int num_cycles = 10000;
  int warmup_cycles = 1000;
  std::string freq_str = "500,750,1000,1250,2000";

  // Parse arguments
  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "--config" && i + 1 < argc) {
      config_path = argv[++i];
    } else if (arg == "--cycles" && i + 1 < argc) {
      num_cycles = std::stoi(argv[++i]);
    } else if (arg == "--warmup" && i + 1 < argc) {
      warmup_cycles = std::stoi(argv[++i]);
    } else if (arg == "--frequencies" && i + 1 < argc) {
      freq_str = argv[++i];
    } else if (arg == "--help" || arg == "-h") {
      std::cout
          << "Usage: sudo " << argv[0] << " [options]\n"
          << "  --config PATH       Path to hardware.yaml (required)\n"
          << "  --cycles N          Cycles per frequency test (default: 10000)\n"
          << "  --warmup N          Warmup cycles to discard (default: 1000)\n"
          << "  --frequencies F     Comma-separated cycle times in us "
             "(default: 500,750,1000,1250,2000)\n"
          << std::endl;
      return 0;
    } else if (arg[0] != '-' && interface_override.empty()) {
      interface_override = arg;
    } else {
      std::cerr << "Unknown argument: " << arg << std::endl;
      return 1;
    }
  }

  if (config_path.empty()) {
    std::cerr << "Error: --config is required" << std::endl;
    return 1;
  }

  std::signal(SIGINT, sighandler);

  // Load hardware config
  HardwareInfo hw_info;
  if (!loadHardwareConfig(config_path, hw_info)) {
    return 1;
  }

  std::string iface =
      interface_override.empty() ? hw_info.interface_name : interface_override;

  auto frequencies = parseFrequencies(freq_str);

  std::cout << "=== EtherCAT Cycle Benchmark ===" << std::endl;
  std::cout << "Interface:    " << iface << std::endl;
  std::cout << "Slaves:       " << hw_info.actuator_type_map.size();
  if (!hw_info.actuator_type_map.empty()) {
    std::cout << " (" << hw_info.actuator_type_map.begin()->second << ")";
  }
  std::cout << std::endl;
  std::cout << "Frequencies:  ";
  for (size_t i = 0; i < frequencies.size(); i++) {
    if (i > 0)
      std::cout << ", ";
    std::cout << frequencies[i];
  }
  std::cout << " us" << std::endl;
  std::cout << "Cycles/test:  " << num_cycles << std::endl;
  std::cout << "Warmup:       " << warmup_cycles << std::endl;
  std::cout << std::endl;

  // ---------------------------------------------------------------------------
  // Run benchmark for each frequency
  // ---------------------------------------------------------------------------

  std::vector<FrequencyResult> all_results;

  for (int cycle_us : frequencies) {
    if (!g_running)
      break;

    double freq_khz = 1000.0 / cycle_us;
    std::cout << "Testing cycle_time = " << cycle_us << " us (" << std::fixed
              << std::setprecision(1) << freq_khz << " kHz)..." << std::endl;

    // Create fresh master for this frequency
    ethercat_sdk::EtherCATMaster master(iface, cycle_us);
    master.configActuatorTypes(hw_info.actuator_type_map);
    master.setOperationMode(ethercat_sdk::OperationMode::PVT);

    if (!master.init()) {
      std::cerr << "  Failed to initialize master at " << cycle_us
                << " us. Skipping." << std::endl;
      continue;
    }

    master.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    if (!master.isOperational()) {
      std::cerr << "  Master not operational. Skipping." << std::endl;
      master.stop();
      continue;
    }

    master.enableAll();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Warmup: discard initial cycles
    std::cout << "  Warming up (" << warmup_cycles << " cycles)..."
              << std::flush;
    {
      ethercat_sdk::CycleTimingSnapshot snap{};
      int collected = 0;
      while (collected < warmup_cycles && g_running) {
        if (master.getCycleStats(snap)) {
          collected++;
        } else {
          std::this_thread::sleep_for(std::chrono::microseconds(50));
        }
      }
    }
    std::cout << " Done." << std::endl;

    // Collect measurement samples
    std::cout << "  Collecting " << num_cycles << " samples..." << std::flush;
    std::vector<ethercat_sdk::CycleTimingSnapshot> snapshots;
    snapshots.reserve(num_cycles);
    {
      ethercat_sdk::CycleTimingSnapshot snap{};
      while (static_cast<int>(snapshots.size()) < num_cycles && g_running) {
        if (master.getCycleStats(snap)) {
          snapshots.push_back(snap);
        } else {
          std::this_thread::sleep_for(std::chrono::microseconds(50));
        }
      }
    }
    std::cout << " Done. (" << snapshots.size() << " samples)" << std::endl;

    // Benchmark backend read/write overhead
    std::cout << "  Measuring backend read/write overhead..." << std::flush;
    FrequencyResult result;
    result.cycle_time_us = cycle_us;
    result.total_cycles = static_cast<int>(snapshots.size());
    result.wkc_errors = 0;
    result.reserveAll(num_cycles);

    int num_slaves = master.getSlaveCount();
    const int backend_iters = 1000;
    result.backend_read.reserve(backend_iters);
    result.backend_write.reserve(backend_iters);

    for (int i = 0; i < backend_iters && g_running; i++) {
      // Benchmark getActuatorState for all slaves
      auto t0 = std::chrono::steady_clock::now();
      for (int s = 1; s <= num_slaves; s++) {
        auto state = master.getActuatorState(s);
        (void)state;
      }
      auto t1 = std::chrono::steady_clock::now();

      // Benchmark setCommand for all slaves
      for (int s = 1; s <= num_slaves; s++) {
        master.setCommand(s, 0.0, 0.0, 0.0, 0.0, 0.0);
      }
      auto t2 = std::chrono::steady_clock::now();

      result.backend_read.addSample(ns_to_us(
          std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0)
              .count()));
      result.backend_write.addSample(ns_to_us(
          std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1)
              .count()));
    }
    std::cout << " Done." << std::endl;

    // Stop master before computing stats
    master.disableAll();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    master.stop();

    // -----------------------------------------------------------------------
    // Compute statistics from snapshots
    // -----------------------------------------------------------------------

    int64_t target_ns = static_cast<int64_t>(cycle_us) * 1000;

    for (size_t i = 0; i < snapshots.size(); i++) {
      const auto &s = snapshots[i];

      // WKC errors
      if (s.wkc != s.expected_wkc) {
        result.wkc_errors++;
      }

      // Per-cycle phase durations
      result.receive.addSample(ns_to_us(s.after_receive_ns - s.wakeup_ns));
      result.txpdo_read.addSample(
          ns_to_us(s.after_txpdo_read_ns - s.after_receive_ns));
      result.rxpdo_write.addSample(
          ns_to_us(s.after_rxpdo_write_ns - s.after_txpdo_read_ns));
      result.send.addSample(
          ns_to_us(s.after_send_ns - s.after_rxpdo_write_ns));
      result.dc_sync.addSample(
          ns_to_us(s.after_dc_sync_ns - s.after_send_ns));
      result.mailbox.addSample(
          ns_to_us(s.cycle_end_ns - s.after_dc_sync_ns));
      result.dc_offset.addSample(ns_to_us(s.dc_offset_ns));
      result.wakeup_jitter.addSample(
          ns_to_us(s.wakeup_ns - s.intended_wakeup_ns));

      // Total cycle time and jitter (needs previous snapshot)
      if (i > 0) {
        int64_t cycle_ns = s.wakeup_ns - snapshots[i - 1].wakeup_ns;
        double cycle_us_val = ns_to_us(cycle_ns);
        result.total_cycle.addSample(cycle_us_val);
        result.jitter.addSample(std::abs(cycle_us_val - ns_to_us(target_ns)));
      }
    }

    // -----------------------------------------------------------------------
    // Print results for this frequency
    // -----------------------------------------------------------------------

    double processing_mean_us =
        result.receive.mean() + result.txpdo_read.mean() +
        result.rxpdo_write.mean() + result.send.mean() +
        result.dc_sync.mean() + result.mailbox.mean();
    double slack_us = ns_to_us(target_ns) - processing_mean_us;
    double slack_pct = (slack_us / ns_to_us(target_ns)) * 100.0;

    std::cout << std::endl;
    std::cout << "--- Results: " << cycle_us << " us (" << std::fixed
              << std::setprecision(1) << (1000.0 / cycle_us) << " kHz) ---"
              << std::endl;
    std::cout << "WKC errors:         " << result.wkc_errors << " / "
              << result.total_cycles << " ("
              << std::setprecision(2)
              << (100.0 * result.wkc_errors / std::max(1, result.total_cycles))
              << "%)" << std::endl;

    printPhase("Total cycle (us):", result.total_cycle);
    printPhase("Cycle jitter (us):", result.jitter);
    printPhase("Wakeup jitter (us):", result.wakeup_jitter);
    printPhase("Receive (us):", result.receive);
    printPhase("TxPDO read (us):", result.txpdo_read);
    printPhase("RxPDO write (us):", result.rxpdo_write);
    printPhase("Send (us):", result.send);
    printPhase("DC sync (us):", result.dc_sync);
    printPhase("Mailbox (us):", result.mailbox);
    printPhase("DC offset (ns):", result.dc_offset, "ns");
    printPhase("Backend read (us):", result.backend_read);
    printPhase("Backend write (us):", result.backend_write);

    std::cout << std::fixed << std::setprecision(1);
    std::cout << "Cycle slack (us):   " << slack_us << " (" << slack_pct
              << "% of budget)" << std::endl;
    std::cout << std::endl;

    all_results.push_back(std::move(result));

    // Cooldown between frequency tests
    if (g_running) {
      std::cout << "  Cooldown..." << std::endl;
      std::this_thread::sleep_for(std::chrono::seconds(2));
    }
  }

  // -------------------------------------------------------------------------
  // Frequency comparison table
  // -------------------------------------------------------------------------

  if (all_results.size() > 1) {
    std::cout << "========================================" << std::endl;
    std::cout << "         FREQUENCY COMPARISON" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::left << std::setw(10) << "Freq(us)" << std::setw(12)
              << "Jitter_p99" << std::setw(10) << "WKC_err%" << std::setw(12)
              << "Recv_mean" << std::setw(12) << "Send_mean" << std::setw(10)
              << "Slack_us" << std::setw(10) << "Slack_%" << std::endl;

    for (const auto &r : all_results) {
      double processing =
          r.receive.mean() + r.txpdo_read.mean() + r.rxpdo_write.mean() +
          r.send.mean() + r.dc_sync.mean() + r.mailbox.mean();
      double target = static_cast<double>(r.cycle_time_us);
      double slack = target - processing;
      double slack_pct = (slack / target) * 100.0;
      double wkc_pct =
          (100.0 * r.wkc_errors / std::max(1, r.total_cycles));

      std::cout << std::fixed << std::setprecision(1);
      std::cout << std::left << std::setw(10) << r.cycle_time_us
                << std::setw(12) << r.jitter.p99() << std::setw(10)
                << std::setprecision(2) << wkc_pct << std::setw(12)
                << std::setprecision(1) << r.receive.mean() << std::setw(12)
                << r.send.mean() << std::setw(10) << slack << std::setw(10)
                << slack_pct << std::endl;
    }
    std::cout << std::endl;
  }

  // -------------------------------------------------------------------------
  // Recommendations
  // -------------------------------------------------------------------------

  if (!all_results.empty()) {
    std::cout << "========================================" << std::endl;
    std::cout << "         RECOMMENDATIONS" << std::endl;
    std::cout << "========================================" << std::endl;

    // Score each frequency: must pass jitter + WKC + slack thresholds,
    // then pick the fastest one with adequate headroom.
    // Criteria:
    //   Minimum viable: jitter p99 < 10% budget, WKC < 1%, slack > 40%
    //   Recommended:    jitter p99 < 5% budget,  WKC < 0.1%, slack > 60%
    int min_viable = -1;
    int recommended = -1;

    for (const auto &r : all_results) {
      double target = static_cast<double>(r.cycle_time_us);
      double jitter_ratio = r.jitter.p99() / target;
      double wkc_pct =
          (100.0 * r.wkc_errors / std::max(1, r.total_cycles));
      double processing =
          r.receive.mean() + r.txpdo_read.mean() + r.rxpdo_write.mean() +
          r.send.mean() + r.dc_sync.mean() + r.mailbox.mean();
      double slack_pct = ((target - processing) / target) * 100.0;

      if (min_viable < 0 && jitter_ratio < 0.10 && wkc_pct < 1.0 &&
          slack_pct > 40.0) {
        min_viable = r.cycle_time_us;
      }
      if (jitter_ratio < 0.05 && wkc_pct < 0.1 && slack_pct > 60.0) {
        if (recommended < 0)
          recommended = r.cycle_time_us;
      }
    }

    if (min_viable > 0) {
      std::cout << "Minimum viable:  " << min_viable
                << " us (jitter p99 < 10% budget, WKC < 1%, slack > 40%)"
                << std::endl;
    } else {
      std::cout << "Minimum viable:  None met criteria" << std::endl;
    }

    if (recommended > 0) {
      std::cout << "Recommended:     " << recommended
                << " us (jitter p99 < 5% budget, WKC < 0.1%, slack > 60%)"
                << std::endl;
    } else {
      std::cout << "Recommended:     None met criteria (try slower frequencies)"
                << std::endl;
    }
    std::cout << std::endl;
  }

  std::cout << "Benchmark complete." << std::endl;
  return 0;
}
