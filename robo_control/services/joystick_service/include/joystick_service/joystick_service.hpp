#pragma once

#include <common/thread_loop.hpp>
#include <joystick_sdk/joystick.hpp>
#include <transport/transport.hpp>

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace joystick_service {

struct JoystickServiceConfig {
  int loop_hz;
  std::string device_path;
};

class JoystickService {
public:
  JoystickService();
  ~JoystickService();

  JoystickService(const JoystickService &) = delete;
  JoystickService &operator=(const JoystickService &) = delete;

  bool init();
  void start();
  void stop();

private:
  void loadParameters();
  void publishLoop();
  void watchdogLoop();
  void tryConnect();

  JoystickServiceConfig config_;

  std::unique_ptr<joystick_sdk::Joystick> joystick_;
  std::unique_ptr<transport::Publisher<transport::Joystick>> publisher_;
  std::unique_ptr<common::ThreadLoop> thread_loop_;

  // Watchdog thread for device monitoring (non-RT)
  std::thread watchdog_thread_;
  std::atomic<bool> watchdog_running_{false};
  std::mutex watchdog_mutex_;
  std::condition_variable watchdog_cv_;

  std::atomic<bool> joystick_connected_{false};
  std::atomic<bool> needs_reconnect_{true};
  bool warned_not_connected_{false};

  // Store device path after successful connection
  std::string connected_device_path_;
};

} // namespace joystick_service
