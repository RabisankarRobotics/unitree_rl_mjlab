#include "imu_service/imu_service.hpp"

#include <filesystem>
#include <spdlog/spdlog.h>
#include <transport/node_context.hpp>
#include <yaml-cpp/yaml.h>

#include "common/config_loader.hpp"

namespace imu_service {

namespace {
constexpr int kBaudrate = 3000000;
constexpr uint32_t kGyroReportIntervalUs = 2500;
constexpr uint32_t kAccelReportIntervalUs = 5000;
constexpr uint32_t kRotationReportIntervalUs = 2500;
constexpr std::chrono::milliseconds kWatchdogInterval{1000};
} // namespace

IMUService::IMUService() {}

IMUService::~IMUService() { stop(); }

void IMUService::loadParameters() {
  auto node = transport::NodeContext::instance().node();

  auto config_path = node->declare_parameter<std::string>("config_path");
  YAML::Node config = YAML::LoadFile(config_path);

  config_.loop_hz = common::get_config_value<int>(config, "loop_hz");
  config_.device_path =
      common::get_config_value<std::string>(config, "device_path");
}

bool IMUService::init() {
  loadParameters();

  publisher_ = std::make_unique<transport::Publisher<transport::IMUData>>(
      transport::IMU_DATA_TOPIC);
  if (!publisher_->is_valid()) {
    spdlog::error("Failed to create IMU data publisher");
    return false;
  }

  common::RealtimeConfig rt_config;
  rt_config.frequency_hz = config_.loop_hz;
  rt_config.scheduler_policy = SCHED_FIFO;
  rt_config.scheduler_priority = 60;
  rt_config.cpu_affinity = -1;

  thread_loop_ = std::make_unique<common::ThreadLoop>(
      "IMULoop", rt_config, [this]() { publishLoop(); });

  spdlog::info("IMUService initialized: loop_hz={}", config_.loop_hz);
  return true;
}

void IMUService::start() {
  // Start watchdog thread for device monitoring
  watchdog_running_.store(true);
  watchdog_thread_ = std::thread([this]() { watchdogLoop(); });

  if (thread_loop_) {
    thread_loop_->start();
    spdlog::info("IMUService started");
  }
}

void IMUService::stop() {
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
    spdlog::info("IMUService stopped");
  }
  publisher_.reset();
  imu_.reset();
}

void IMUService::watchdogLoop() {
  spdlog::debug("IMU watchdog thread started");

  while (watchdog_running_.load()) {
    // Check device connection status
    if (imu_connected_.load()) {
      // Check if device file still exists
      if (!std::filesystem::exists(config_.device_path)) {
        spdlog::warn("IMU disconnected (device removed)");
        imu_connected_.store(false);
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

  spdlog::debug("IMU watchdog thread stopped");
}

void IMUService::tryConnect() {
  try {
    auto new_imu =
        std::make_unique<imu_sdk::BNO08xIMU>(config_.device_path, kBaudrate);
    spdlog::info("IMU connected: {} @ {} baud", config_.device_path, kBaudrate);

    new_imu->enableFeature(imu_sdk::BNO_REPORT_GYROSCOPE_CALIBRATED,
                           kGyroReportIntervalUs);
    new_imu->enableFeature(imu_sdk::BNO_REPORT_LINEAR_ACCELERATION,
                           kAccelReportIntervalUs);
    new_imu->enableFeature(imu_sdk::BNO_REPORT_GYRO_INTEGRATED_ROTATION_VECTOR,
                           kRotationReportIntervalUs);

    spdlog::info("IMU features enabled: gyro@{}us, accel@{}us, rotation@{}us",
                 kGyroReportIntervalUs, kAccelReportIntervalUs,
                 kRotationReportIntervalUs);

    // Atomically swap in the new IMU
    imu_ = std::move(new_imu);
    imu_connected_.store(true);
    needs_reconnect_.store(false);
    warned_not_connected_ = false;
  } catch (const std::exception &) {
    if (!warned_not_connected_) {
      spdlog::warn("IMU not connected at {}, waiting for connection...",
                   config_.device_path);
      warned_not_connected_ = true;
    }
  }
}

void IMUService::publishLoop() {
  if (!imu_connected_.load() || !imu_) {
    return;
  }

  try {
    imu_sdk::GyroIntegratedRV rv = imu_->gyroIntegratedRotationVector();
    imu_sdk::Vector3 gyro = imu_->gyroscopeCalibrated();
    imu_sdk::Vector3 accel = imu_->linearAcceleration();

    transport::IMUData msg;
    msg.quaternion = {rv.real, rv.i, rv.j, rv.k};
    msg.gyroscope = {gyro.x, gyro.y, gyro.z};
    msg.linear_acceleration = {accel.x, accel.y, accel.z};

    publisher_->publish(msg);
  } catch (const std::exception &e) {
    spdlog::warn("IMU read error: {}", e.what());
    imu_connected_.store(false);
    needs_reconnect_.store(true);
  }
}

} // namespace imu_service
