#include "can_sdk/master.hpp"

#include <chrono>
#include <cmath>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>

static volatile bool running = true;
void sighandler(int) { running = false; }

struct BenchmarkStats {
  double min_latency_us = 1e9;
  double max_latency_us = 0.0;
  double sum_latency_us = 0.0;
  double sum_squared_us = 0.0;
  int success_count = 0;
  int timeout_count = 0;
  int total_iterations = 0;

  void addSample(double latency_us) {
    min_latency_us = std::min(min_latency_us, latency_us);
    max_latency_us = std::max(max_latency_us, latency_us);
    sum_latency_us += latency_us;
    sum_squared_us += latency_us * latency_us;
    success_count++;
    total_iterations++;
  }

  void addTimeout() {
    timeout_count++;
    total_iterations++;
  }

  double getMean() const {
    return success_count > 0 ? sum_latency_us / success_count : 0.0;
  }

  double getStdDev() const {
    if (success_count <= 1)
      return 0.0;
    double mean = getMean();
    double variance =
        (sum_squared_us / success_count) - (mean * mean);
    return std::sqrt(std::max(0.0, variance));
  }

  void print(const std::string &label) const {
    std::cout << "\n=== " << label << " ===" << std::endl;
    std::cout << "Total iterations:  " << total_iterations << std::endl;
    std::cout << "Successful reads:  " << success_count << " ("
              << (100.0 * success_count / total_iterations) << "%)"
              << std::endl;
    std::cout << "Timeouts:          " << timeout_count << " ("
              << (100.0 * timeout_count / total_iterations) << "%)"
              << std::endl;
    if (success_count > 0) {
      std::cout << std::fixed << std::setprecision(2);
      std::cout << "Latency (us):      min=" << min_latency_us
                << " max=" << max_latency_us << " mean=" << getMean()
                << " stddev=" << getStdDev() << std::endl;
    }
  }
};

void runBenchmark(can_sdk::CanMaster &master, uint16_t joint_id,
                  int timeout_us, int num_iterations, BenchmarkStats &stats,
                  std::vector<double> &latencies) {
  for (int i = 0; i < num_iterations && running; i++) {
    // Send motion command
    can_sdk::MotionCommand cmd;
    cmd.position = 0.0;
    cmd.velocity = 0.0;
    cmd.kp = 10.0;
    cmd.kd = 1.0;
    cmd.torque = 0.0;

    auto send_time = std::chrono::steady_clock::now();

    if (!master.sendMotionCommand(joint_id, cmd)) {
      std::cerr << "Failed to send command at iteration " << i << std::endl;
      continue;
    }

    // Receive response with specified timeout
    master.recvAll(timeout_us);

    // Check if we got a response
    auto feedback = master.getMotionFeedback(joint_id);
    auto recv_time = std::chrono::steady_clock::now();

    // Check if feedback is fresh (within 2x timeout)
    auto feedback_age = recv_time - feedback.timestamp;
    auto latency_us = std::chrono::duration<double, std::micro>(
                          recv_time - send_time)
                          .count();

    if (feedback_age < std::chrono::microseconds(timeout_us * 2)) {
      stats.addSample(latency_us);
      latencies.push_back(latency_us);
    } else {
      stats.addTimeout();
    }

    // Small delay between iterations to avoid overwhelming the bus
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
}

int main(int argc, char *argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: sudo " << argv[0] << " <can_interface> [joint_id]"
              << std::endl;
    std::cerr << "Example: sudo " << argv[0] << " can0 1" << std::endl;
    return 1;
  }

  std::signal(SIGINT, sighandler);

  const std::string interface = argv[1];
  const uint16_t joint_id = (argc >= 3) ? std::atoi(argv[2]) : 1;
  const int iterations_per_test = 1000;

  // Initialize CAN master
  can_sdk::CanMaster master;

  std::cout << "=== CAN SDK Latency Benchmark ===" << std::endl;
  std::cout << "Interface: " << interface << std::endl;
  std::cout << "Joint ID:  " << joint_id << std::endl;
  std::cout << "Iterations per timeout test: " << iterations_per_test
            << std::endl;

  if (!master.init(interface)) {
    std::cerr << "Failed to initialize CAN interface" << std::endl;
    return 1;
  }

  master.addJoint(joint_id);

  // Verify actuator is responding
  std::cout << "\nVerifying actuator connection..." << std::endl;
  master.requestMotorStatus(joint_id);
  master.recvAll(5000); // 5ms initial timeout

  auto status = master.getMotorStatus(joint_id);
  auto age = std::chrono::steady_clock::now() - status.timestamp;
  if (age > std::chrono::milliseconds(10)) {
    std::cerr << "Warning: No response from actuator. Check connection."
              << std::endl;
    std::cerr << "Continuing anyway for testing..." << std::endl;
  } else {
    std::cout << "Actuator responding. Initial position: " << status.position
              << " rad" << std::endl;
  }

  // Test different timeout values
  std::vector<int> timeout_values = {0,   10,  20,  50,  100,
                                     200, 500, 1000, 2000, 5000};

  std::vector<BenchmarkStats> all_stats;
  std::vector<std::vector<double>> all_latencies;

  std::cout << "\nStarting benchmark tests..." << std::endl;
  std::cout << "Press Ctrl+C to stop\n" << std::endl;

  for (int timeout_us : timeout_values) {
    if (!running)
      break;

    BenchmarkStats stats;
    std::vector<double> latencies;

    std::cout << "\nTesting timeout = " << timeout_us << " us..." << std::flush;

    runBenchmark(master, joint_id, timeout_us, iterations_per_test, stats,
                 latencies);

    all_stats.push_back(stats);
    all_latencies.push_back(latencies);

    std::cout << " Done. Success rate: "
              << (100.0 * stats.success_count / stats.total_iterations) << "%"
              << std::endl;
  }

  // Print summary
  std::cout << "\n\n========================================" << std::endl;
  std::cout << "         BENCHMARK RESULTS" << std::endl;
  std::cout << "========================================" << std::endl;

  for (size_t i = 0; i < timeout_values.size() && i < all_stats.size(); i++) {
    std::cout << "\n--- Timeout: " << timeout_values[i] << " us ---"
              << std::endl;
    const auto &stats = all_stats[i];
    std::cout << "Success rate: " << std::fixed << std::setprecision(2)
              << (100.0 * stats.success_count / stats.total_iterations) << "%"
              << std::endl;
    if (stats.success_count > 0) {
      std::cout << "Latency: " << std::setprecision(1) << "min=" << stats.min_latency_us
                << "us, max=" << stats.max_latency_us << "us, mean="
                << stats.getMean() << "us, stddev=" << stats.getStdDev()
                << "us" << std::endl;
    }
  }

  // Find optimal timeout
  std::cout << "\n========================================" << std::endl;
  std::cout << "         RECOMMENDATIONS" << std::endl;
  std::cout << "========================================" << std::endl;

  // Find first timeout with >99% success rate
  int optimal_timeout = -1;
  for (size_t i = 0; i < all_stats.size(); i++) {
    double success_rate =
        100.0 * all_stats[i].success_count / all_stats[i].total_iterations;
    if (success_rate >= 99.0) {
      optimal_timeout = timeout_values[i];
      break;
    }
  }

  if (optimal_timeout >= 0) {
    std::cout << "\nOptimal timeout (>99% success): " << optimal_timeout
              << " us" << std::endl;
  } else {
    std::cout << "\nWarning: No timeout achieved >99% success rate!"
              << std::endl;
  }

  // Report minimum observed latency
  double global_min = 1e9;
  for (const auto &stats : all_stats) {
    if (stats.success_count > 0) {
      global_min = std::min(global_min, stats.min_latency_us);
    }
  }
  if (global_min < 1e9) {
    std::cout << "Minimum observed latency: " << std::fixed
              << std::setprecision(1) << global_min << " us" << std::endl;
  }

  // Save detailed results to CSV
  std::ofstream csv("can_latency_benchmark.csv");
  if (csv.is_open()) {
    csv << "timeout_us,iteration,latency_us\n";
    for (size_t i = 0; i < timeout_values.size(); i++) {
      for (size_t j = 0; j < all_latencies[i].size(); j++) {
        csv << timeout_values[i] << "," << j << "," << all_latencies[i][j]
            << "\n";
      }
    }
    csv.close();
    std::cout << "\nDetailed results saved to: can_latency_benchmark.csv"
              << std::endl;
  }

  std::cout << "\nBenchmark complete." << std::endl;
  master.close();

  return 0;
}
