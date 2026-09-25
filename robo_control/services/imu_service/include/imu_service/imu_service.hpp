#pragma once

#include <common/thread_loop.hpp>
#include <imu_sdk/bno08x_imu.hpp>
#include <transport/transport.hpp>

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace imu_service {

struct IMUServiceConfig {
  int loop_hz;
  std::string device_path;
};

class IMUService {
public:
  IMUService();
  ~IMUService();

  IMUService(const IMUService &) = delete;
  IMUService &operator=(const IMUService &) = delete;

  bool init();
  void start();
  void stop();

private:
  void loadParameters();
  void publishLoop();
  void watchdogLoop();
  void tryConnect();

  IMUServiceConfig config_;

  std::unique_ptr<imu_sdk::BNO08xIMU> imu_;
  std::unique_ptr<transport::Publisher<transport::IMUData>> publisher_;
  std::unique_ptr<common::ThreadLoop> thread_loop_;

  // Watchdog thread for device monitoring (non-RT)
  std::thread watchdog_thread_;
  std::atomic<bool> watchdog_running_{false};
  std::mutex watchdog_mutex_;
  std::condition_variable watchdog_cv_;

  std::atomic<bool> imu_connected_{false};
  std::atomic<bool> needs_reconnect_{true};
  bool warned_not_connected_{false};
};

} // namespace imu_service
