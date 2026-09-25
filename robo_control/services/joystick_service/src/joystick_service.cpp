#include "joystick_service/joystick_service.hpp"

#include <filesystem>
#include <spdlog/spdlog.h>
#include <transport/node_context.hpp>
#include <yaml-cpp/yaml.h>

#include "common/config_loader.hpp"

namespace joystick_service {

namespace {
constexpr std::chrono::milliseconds kWatchdogInterval{1000};
} // namespace

JoystickService::JoystickService() {}

JoystickService::~JoystickService() { stop(); }

void JoystickService::loadParameters() {
  auto node = transport::NodeContext::instance().node();

  auto config_path = node->declare_parameter<std::string>("config_path");
  YAML::Node config = YAML::LoadFile(config_path);

  config_.loop_hz = common::get_config_value<int>(config, "loop_hz");
  config_.device_path =
      common::get_config_value<std::string>(config, "device_path", "");
}

bool JoystickService::init() {
  loadParameters();

  publisher_ = std::make_unique<transport::Publisher<transport::Joystick>>(
      transport::JOYSTICK_TOPIC);
  if (!publisher_->is_valid()) {
    spdlog::error("Failed to create joystick publisher");
    return false;
  }

  common::RealtimeConfig rt_config;
  rt_config.frequency_hz = config_.loop_hz;
  rt_config.scheduler_policy = SCHED_OTHER;
  rt_config.scheduler_priority = 0;
  rt_config.cpu_affinity = -1;

  thread_loop_ = std::make_unique<common::ThreadLoop>(
      "JoystickLoop", rt_config, [this]() { publishLoop(); });

  spdlog::info("JoystickService initialized: loop_hz={}", config_.loop_hz);
  return true;
}

void JoystickService::start() {
  // Start watchdog thread for device monitoring
  watchdog_running_.store(true);
  watchdog_thread_ = std::thread([this]() { watchdogLoop(); });

  if (thread_loop_) {
    thread_loop_->start();
    spdlog::info("JoystickService started");
  }
}

void JoystickService::stop() {
  // Stop watchdog thread
  {
    std::lock_guard<std::mutex> lock(watchdog_mutex_);
    watchdog_running_.store(false);
  }
  watchdog_cv_.notify_all();
  if (watchdog_thread_.joinable()) {
    watchdog_thread_.join();
  }

  if (thread_loop_) {
    thread_loop_->stop();
    spdlog::info("JoystickService stopped");
  }
  publisher_.reset();
  joystick_.reset();
}

void JoystickService::watchdogLoop() {
  spdlog::debug("Joystick watchdog thread started");

  while (watchdog_running_.load()) {
    // Check device connection status
    if (joystick_connected_.load()) {
      // Check if device file still exists
      if (!connected_device_path_.empty() &&
          !std::filesystem::exists(connected_device_path_)) {
        spdlog::warn("Joystick disconnected (device removed)");
        joystick_connected_.store(false);
        needs_reconnect_.store(true);
      }
    } else if (needs_reconnect_.load()) {
      // Attempt reconnection
      tryConnect();
    }

    // Wait for next check interval or shutdown
    std::unique_lock<std::mutex> lock(watchdog_mutex_);
    watchdog_cv_.wait_for(lock, kWatchdogInterval,
                          [this]() { return !watchdog_running_.load(); });
  }

  spdlog::debug("Joystick watchdog thread stopped");
}

void JoystickService::tryConnect() {
  try {
    std::unique_ptr<joystick_sdk::Joystick> new_joystick;

    if (config_.device_path.empty()) {
      new_joystick = std::make_unique<joystick_sdk::Joystick>();
      connected_device_path_ = new_joystick->getDevicePath();
    } else {
      new_joystick =
          std::make_unique<joystick_sdk::Joystick>(config_.device_path);
      connected_device_path_ = config_.device_path;
    }

    joystick_ = std::move(new_joystick);
    joystick_connected_.store(true);
    needs_reconnect_.store(false);
    warned_not_connected_ = false;
    spdlog::info("Joystick connected: {}", connected_device_path_);
  } catch (const std::exception &) {
    if (!warned_not_connected_) {
      spdlog::warn("Joystick not connected, waiting for connection...");
      warned_not_connected_ = true;
    }
  }
}

void JoystickService::publishLoop() {
  if (!joystick_connected_.load() || !joystick_) {
    return;
  }

  try {
    joystick_sdk::JoystickState state = joystick_->getState();

    transport::Joystick msg;
    msg.left_stick_x = state.x;
    msg.left_stick_y = state.y;
    msg.right_stick_x = state.rx;
    msg.right_stick_y = state.ry;
    msg.left_trigger = state.lt;
    msg.right_trigger = state.rt;
    msg.buttons = state.buttons;
    msg.dpad_x = state.hat_x;
    msg.dpad_y = state.hat_y;

    publisher_->publish(msg);
  } catch (const std::exception &e) {
    spdlog::warn("Joystick read error: {}", e.what());
    joystick_connected_.store(false);
    needs_reconnect_.store(true);
  }
}

} // namespace joystick_service
