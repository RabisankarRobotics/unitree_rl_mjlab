#pragma once

#include <common/thread_loop.hpp>
#include <power_dist_board_sdk/power_dist_board.hpp>
#include <transport/transport.hpp>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace power_service {

struct PowerServiceConfig {
  int loop_hz;
  std::string device_path;
};

class PowerService {
public:
  PowerService();
  ~PowerService();

  PowerService(const PowerService &) = delete;
  PowerService &operator=(const PowerService &) = delete;

  bool init();
  void start();
  void stop();

private:
  void loadParameters();
  void publishLoop();
  void watchdogLoop();
  void tryConnect();
  void logAlertChanges(uint16_t flags);

  PowerServiceConfig config_;

  std::unique_ptr<power_dist_board_sdk::PowerDistBoard> pdb_;
  std::unique_ptr<transport::Publisher<transport::PowerData>> publisher_;
  std::unique_ptr<common::ThreadLoop> thread_loop_;

  std::thread watchdog_thread_;
  std::atomic<bool> watchdog_running_{false};
  std::mutex watchdog_mutex_;
  std::condition_variable watchdog_cv_;

  std::atomic<bool> pdb_connected_{false};
  std::atomic<bool> needs_reconnect_{true};
  bool warned_not_connected_{false};

  uint16_t last_alert_flags_{0};
  bool last_estop_pressed_{false};
  bool last_shutdown_requested_{false};
};

} // namespace power_service
