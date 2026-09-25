#pragma once

#include <atomic>
#include <chrono>
#include <csignal>
#include <spdlog/spdlog.h>
#include <string>
#include <sys/mman.h>
#include <thread>
#include <transport/node_context.hpp>

namespace transport {

namespace detail {
inline std::atomic<bool> &running_flag() {
  static std::atomic<bool> running(true);
  return running;
}

inline void signal_handler(int /* signal */) { running_flag() = false; }
} // namespace detail

struct ServiceOptions {
  bool lock_memory = false; // Enable mlockall for real-time operation
};

/**
 * @brief Run a service with standard initialization and shutdown handling.
 *
 * Provides common boilerplate for all services:
 * - Signal handling (SIGINT, SIGTERM)
 * - ROS2 NodeContext initialization
 * - Optional memory locking for real-time operation
 * - Service lifecycle management (init, start, stop)
 * - Main loop with graceful shutdown
 *
 * @tparam ServiceType Service class with init(), start(), and stop() methods
 * @param service_name Name for logging and ROS2 node
 * @param argc Command line argument count
 * @param argv Command line arguments
 * @param options Service configuration options
 * @return Exit code (0 for success, 1 for failure)
 */
template <typename ServiceType>
int run_service(const std::string &service_name, int argc, char **argv,
                const ServiceOptions &options = {}) {
  std::signal(SIGINT, detail::signal_handler);
  std::signal(SIGTERM, detail::signal_handler);
  detail::running_flag() = true;

  spdlog::info("Starting {}", service_name);

  NodeContext::instance().init(service_name, argc, argv);

  ServiceType service;

  if (!service.init()) {
    spdlog::error("Failed to initialize {}", service_name);
    NodeContext::instance().shutdown();
    return 1;
  }

  if (options.lock_memory) {
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
      spdlog::warn(
          "mlockall() failed - may experience page faults in RT loop. "
          "Run with root or CAP_IPC_LOCK capability for best RT performance.");
    } else {
      spdlog::info("Memory locked for real-time operation");
    }
  }

  service.start();
  spdlog::info("{} running. Press Ctrl+C to exit.", service_name);

  while (detail::running_flag() && NodeContext::instance().ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  spdlog::info("Shutting down...");
  service.stop();
  NodeContext::instance().shutdown();
  spdlog::info("{} stopped.", service_name);

  return 0;
}

} // namespace transport
