#pragma once

#include <chrono>
#include <cstddef>
#include <utility>

#include <spdlog/spdlog.h>

namespace common {

// Time-based log throttle. Holds per-instance state (last emit time and count
// of suppressed calls). Emits at most one message per `period`; when it next
// emits after suppressing, it appends the suppressed count to the message.
class LogThrottle {
public:
  explicit LogThrottle(
      std::chrono::milliseconds period = std::chrono::seconds(1))
      : period_(period) {}

  void set_period(std::chrono::milliseconds period) { period_ = period; }

  template <typename... Args>
  void trace(spdlog::format_string_t<Args...> fmt, Args &&...args) {
    log(spdlog::level::trace, fmt, std::forward<Args>(args)...);
  }

  template <typename... Args>
  void debug(spdlog::format_string_t<Args...> fmt, Args &&...args) {
    log(spdlog::level::debug, fmt, std::forward<Args>(args)...);
  }

  template <typename... Args>
  void info(spdlog::format_string_t<Args...> fmt, Args &&...args) {
    log(spdlog::level::info, fmt, std::forward<Args>(args)...);
  }

  template <typename... Args>
  void warn(spdlog::format_string_t<Args...> fmt, Args &&...args) {
    log(spdlog::level::warn, fmt, std::forward<Args>(args)...);
  }

  template <typename... Args>
  void error(spdlog::format_string_t<Args...> fmt, Args &&...args) {
    log(spdlog::level::err, fmt, std::forward<Args>(args)...);
  }

  template <typename... Args>
  void critical(spdlog::format_string_t<Args...> fmt, Args &&...args) {
    log(spdlog::level::critical, fmt, std::forward<Args>(args)...);
  }

  template <typename... Args>
  void log(spdlog::level::level_enum lvl,
           spdlog::format_string_t<Args...> fmt, Args &&...args) {
    auto now = std::chrono::steady_clock::now();
    if (now - last_log_time_ < period_) {
      ++suppressed_;
      return;
    }
    auto msg = fmt::format(fmt, std::forward<Args>(args)...);
    if (suppressed_ > 0) {
      spdlog::log(lvl, "{} ({} similar messages suppressed)", msg, suppressed_);
    } else {
      spdlog::log(lvl, "{}", msg);
    }
    last_log_time_ = now;
    suppressed_ = 0;
  }

private:
  std::chrono::milliseconds period_;
  std::chrono::steady_clock::time_point last_log_time_{};
  std::size_t suppressed_ = 0;
};

} // namespace common
