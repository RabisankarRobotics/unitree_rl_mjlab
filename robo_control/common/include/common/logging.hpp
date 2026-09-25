#pragma once

#include <filesystem>
#include <memory>
#include <string>

#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

namespace common {

inline std::shared_ptr<spdlog::logger>
init_rotating_logger(const std::string &service_name,
                     const std::string &base_dir = "/var/log/robo") {
  try {
    const auto log_path =
        std::filesystem::path(base_dir) / (service_name + ".log");
    std::filesystem::create_directories(log_path.parent_path());

    auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
        log_path.string(), 100 * 1024 * 1024, 2);
    auto stderr_sink = std::make_shared<spdlog::sinks::stderr_color_sink_mt>();

    auto logger = std::make_shared<spdlog::logger>(
        service_name, spdlog::sinks_init_list{file_sink, stderr_sink});

    logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [%n] %v");
    spdlog::flush_on(spdlog::level::info);
    spdlog::set_default_logger(logger);

    return logger;
  } catch (const std::exception &ex) {
    auto fallback_name = service_name + "_stderr";
    auto logger = spdlog::get(fallback_name);
    if (!logger) {
      logger = spdlog::stderr_color_mt(fallback_name);
    }

    logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [%n] %v");
    spdlog::set_default_logger(logger);
    spdlog::warn("Failed to init file logger: {}. Fallback to stderr.",
                 ex.what());

    return logger;
  }
}

} // namespace common
