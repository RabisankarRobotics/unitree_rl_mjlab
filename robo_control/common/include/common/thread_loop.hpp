#pragma once

#include <atomic>
#include <chrono>
#include <cstring>
#include <functional>
#include <pthread.h>
#include <sched.h>
#include <string>
#include <thread>

#include <spdlog/spdlog.h>

#include "common/log_throttle.hpp"

namespace common {

struct RealtimeConfig {
  int frequency_hz = 100;
  int scheduler_policy = SCHED_FIFO; // SCHED_FIFO, SCHED_RR, SCHED_OTHER
  int scheduler_priority = 50;       // 1-99 for RT
  int cpu_affinity = -1;             // -1 for no affinity
};

class ThreadLoop {
public:
  using Callback = std::function<void()>;

  ThreadLoop(const std::string &name, const RealtimeConfig &config,
             Callback callback)
      : name_(name), config_(config), callback_(std::move(callback)),
        running_(false) {}

  ~ThreadLoop() { stop(); }

  void start() {
    if (running_) {
      return;
    }
    running_ = true;
    thread_ = std::thread(&ThreadLoop::loop_function, this);
  }

  void stop() {
    if (!running_) {
      return;
    }
    running_ = false;
    if (thread_.joinable()) {
      thread_.join();
    }
  }

private:
  void loop_function() {
    // Set thread name
    pthread_setname_np(pthread_self(), name_.substr(0, 15).c_str());

    // Configure Real-time settings
    if (config_.scheduler_policy != SCHED_OTHER) {
      struct sched_param param;
      param.sched_priority = config_.scheduler_priority;
      int ret = pthread_setschedparam(pthread_self(), config_.scheduler_policy,
                                      &param);
      if (ret != 0) {
        spdlog::warn("Failed to set RT scheduling for thread '{}': {} ({})",
                     name_, std::strerror(ret), ret);
      } else {
        spdlog::info(
            "Set RT scheduling for thread '{}': policy={}, priority={}", name_,
            config_.scheduler_policy, config_.scheduler_priority);
      }
    }

    // Configure CPU Affinity
    if (config_.cpu_affinity >= 0) {
      cpu_set_t cpuset;
      CPU_ZERO(&cpuset);
      CPU_SET(config_.cpu_affinity, &cpuset);
      int ret =
          pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
      if (ret != 0) {
        spdlog::warn("Failed to set CPU affinity for thread '{}': {} ({})",
                     name_, std::strerror(ret), ret);
      } else {
        spdlog::info("Set CPU affinity for thread '{}' to CPU {}", name_,
                     config_.cpu_affinity);
      }
    }

    auto period = std::chrono::nanoseconds(1000000000 / config_.frequency_hz);
    auto next_wake = std::chrono::steady_clock::now();
    LogThrottle late_warn_throttle(std::chrono::seconds(1));

    while (running_) {
      next_wake += period;

      // Execute the callback
      callback_();

      auto now = std::chrono::steady_clock::now();
      if (now > next_wake) {
        auto late_time_us =
            std::chrono::duration_cast<std::chrono::microseconds>(now -
                                                                  next_wake)
                .count();
        late_warn_throttle.warn("ThreadLoop '{}' is late by: {} us", name_,
                                late_time_us);
      }

      // Sleep until next period
      std::this_thread::sleep_until(next_wake);
    }
  }

  std::string name_;
  RealtimeConfig config_;
  Callback callback_;
  std::atomic<bool> running_;
  std::thread thread_;
};

} // namespace common
