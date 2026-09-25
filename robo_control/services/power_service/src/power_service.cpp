#include "power_service/power_service.hpp"

#include <filesystem>
#include <spdlog/spdlog.h>
#include <transport/node_context.hpp>
#include <yaml-cpp/yaml.h>

#include "common/config_loader.hpp"

namespace power_service {

namespace {
constexpr int kBaudrate = 115200;
constexpr std::chrono::milliseconds kWatchdogInterval{1000};
} // namespace

PowerService::PowerService() {}

PowerService::~PowerService() { stop(); }

void PowerService::loadParameters() {
  auto node = transport::NodeContext::instance().node();

  auto config_path = node->declare_parameter<std::string>("config_path");
  YAML::Node config = YAML::LoadFile(config_path);

  config_.loop_hz = common::get_config_value<int>(config, "loop_hz");
  config_.device_path =
      common::get_config_value<std::string>(config, "device_path");
}

bool PowerService::init() {
  loadParameters();

  publisher_ = std::make_unique<transport::Publisher<transport::PowerData>>(
      transport::POWER_TOPIC);
  if (!publisher_->is_valid()) {
    spdlog::error("Failed to create power data publisher");
    return false;
  }

  common::RealtimeConfig rt_config;
  rt_config.frequency_hz = config_.loop_hz;
  rt_config.scheduler_policy = SCHED_OTHER;
  rt_config.scheduler_priority = 0;
  rt_config.cpu_affinity = -1;

  thread_loop_ = std::make_unique<common::ThreadLoop>(
      "PowerLoop", rt_config, [this]() { publishLoop(); });

  spdlog::info("PowerService initialized: loop_hz={}, device={}",
               config_.loop_hz, config_.device_path);
  return true;
}

void PowerService::start() {
  watchdog_running_.store(true);
  watchdog_thread_ = std::thread([this]() { watchdogLoop(); });

  if (thread_loop_) {
    thread_loop_->start();
    spdlog::info("PowerService started");
  }
}

void PowerService::stop() {
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
    spdlog::info("PowerService stopped");
  }
  publisher_.reset();
  pdb_.reset();
}

void PowerService::watchdogLoop() {
  spdlog::debug("Power watchdog thread started");

  while (watchdog_running_.load()) {
    if (pdb_connected_.load()) {
      if (!std::filesystem::exists(config_.device_path)) {
        spdlog::warn("Power distribution board disconnected (device removed)");
        pdb_connected_.store(false);
        needs_reconnect_.store(true);
      }
    } else if (needs_reconnect_.load()) {
      tryConnect();
    }

    std::unique_lock<std::mutex> lock(watchdog_mutex_);
    watchdog_cv_.wait_for(lock, kWatchdogInterval,
                          [this]() { return !watchdog_running_.load(); });
  }

  spdlog::debug("Power watchdog thread stopped");
}

void PowerService::tryConnect() {
  try {
    auto new_pdb = std::make_unique<power_dist_board_sdk::PowerDistBoard>(
        config_.device_path, kBaudrate);
    spdlog::info("Power distribution board connected: {} @ {} baud",
                 config_.device_path, kBaudrate);

    pdb_ = std::move(new_pdb);
    pdb_connected_.store(true);
    needs_reconnect_.store(false);
    warned_not_connected_ = false;
    last_alert_flags_ = 0;
    last_estop_pressed_ = false;
    last_shutdown_requested_ = false;
  } catch (const std::exception &) {
    if (!warned_not_connected_) {
      spdlog::warn(
          "Power distribution board not connected at {}, waiting for connection...",
          config_.device_path);
      warned_not_connected_ = true;
    }
  }
}

void PowerService::logAlertChanges(uint16_t flags) {
  uint16_t newly_set = flags & ~last_alert_flags_;
  uint16_t newly_cleared = last_alert_flags_ & ~flags;

  for (const auto &e : power_dist_board_sdk::kAlertTable) {
    if (newly_set & e.flag) {
      spdlog::error(
          "Power alert raised: {} - {} (code=0x{:04x}, flags=0x{:04x})",
          e.code, e.description, static_cast<uint16_t>(e.flag), flags);
    }
    if (newly_cleared & e.flag) {
      spdlog::info(
          "Power alert cleared: {} - {} (code=0x{:04x}, flags=0x{:04x})",
          e.code, e.description, static_cast<uint16_t>(e.flag), flags);
    }
  }

  last_alert_flags_ = flags;
}

void PowerService::publishLoop() {
  if (!pdb_connected_.load() || !pdb_) {
    return;
  }

  if (!pdb_->hasTelemetry()) {
    return;
  }

  try {
    auto t = pdb_->telemetry();
    auto a = pdb_->alert();

    transport::PowerData msg;
    msg.temperature_c = t.temperature_c;
    msg.current_ma = t.current_ma;
    msg.voltage_mv = t.voltage_mv;
    msg.estop_pressed = t.estop_pressed;
    msg.shutdown_requested = t.shutdown_requested;
    msg.alert_flags = a.flags;
    msg.alert_message = a.message();

    publisher_->publish(msg);

    logAlertChanges(a.flags);

    if (t.estop_pressed && !last_estop_pressed_) {
      spdlog::error("E-stop pressed");
    } else if (!t.estop_pressed && last_estop_pressed_) {
      spdlog::info("E-stop released");
    }
    last_estop_pressed_ = t.estop_pressed;

    if (t.shutdown_requested && !last_shutdown_requested_) {
      spdlog::warn("Shutdown requested by power distribution board");
    }
    last_shutdown_requested_ = t.shutdown_requested;
  } catch (const std::exception &e) {
    spdlog::warn("Power board read error: {}", e.what());
    pdb_connected_.store(false);
    needs_reconnect_.store(true);
  }
}

} // namespace power_service
