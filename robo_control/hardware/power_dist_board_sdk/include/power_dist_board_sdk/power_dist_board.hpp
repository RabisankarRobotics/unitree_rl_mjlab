#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace power_dist_board_sdk {

enum AlertFlag : uint16_t {
  PWR_OL     = 1u << 2,
  BUS_UV     = 1u << 3,
  BUS_OV     = 1u << 4,
  SHNT_UL    = 1u << 5,
  SHNT_OL    = 1u << 6,
  TEMP_ALERT = 1u << 7,
};

struct AlertInfo {
  AlertFlag   flag;
  const char *code;
  const char *description;
};

inline constexpr AlertInfo kAlertTable[] = {
    {PWR_OL,     "PWR_OL",     "power overload"},
    {BUS_UV,     "BUS_UV",     "bus undervoltage"},
    {BUS_OV,     "BUS_OV",     "bus overvoltage"},
    {SHNT_UL,    "SHNT_UL",    "shunt under-limit"},
    {SHNT_OL,    "SHNT_OL",    "shunt over-limit"},
    {TEMP_ALERT, "TEMP_ALERT", "temperature alert"},
};

inline const AlertInfo *alertInfo(AlertFlag f) {
  for (const auto &e : kAlertTable) {
    if (e.flag == f) return &e;
  }
  return nullptr;
}

inline const char *alertCode(AlertFlag f) {
  const auto *info = alertInfo(f);
  return info ? info->code : "UNKNOWN";
}

inline const char *alertDescription(AlertFlag f) {
  const auto *info = alertInfo(f);
  return info ? info->description : "unknown alert";
}

inline std::string formatAlertMessage(uint16_t flags) {
  if (flags == 0) return {};
  std::string out;
  for (const auto &e : kAlertTable) {
    if (flags & e.flag) {
      if (!out.empty()) out += ", ";
      out += e.code;
      out += " (";
      out += e.description;
      out += ")";
    }
  }
  return out;
}

struct Telemetry {
  int32_t  temperature_c{};
  uint16_t current_ma{};
  uint16_t voltage_mv{};
  bool     estop_pressed{};
  bool     shutdown_requested{};
  std::chrono::steady_clock::time_point timestamp{};
};

struct Alert {
  uint16_t flags{};
  std::chrono::steady_clock::time_point timestamp{};

  bool has(AlertFlag f) const { return (flags & static_cast<uint16_t>(f)) != 0; }
  std::string message() const { return formatAlertMessage(flags); }
};

class PowerDistBoard {
public:
  explicit PowerDistBoard(const std::string &device, int baudrate = 115200);
  ~PowerDistBoard();

  PowerDistBoard(const PowerDistBoard &) = delete;
  PowerDistBoard &operator=(const PowerDistBoard &) = delete;

  Telemetry telemetry() const;
  Alert alert() const;
  bool hasTelemetry() const;

  uint64_t telemetryCount() const;
  uint64_t alertCount() const;
  uint64_t crcErrorCount() const;
  uint64_t framingErrorCount() const;

private:
  enum class ParseState {
    WaitSof,
    LenL,
    LenH,
    MsgId,
    Data,
    Crc,
    Eof,
  };

  void readThreadFunc();
  void feedAscii(uint8_t c);
  void feedByte(uint8_t b);
  void resetParser();
  void dispatchFrame();
  void handleTelemetry(const uint8_t *data, uint16_t len);
  void handleAlert(const uint8_t *data, uint16_t len);

  int fd_{-1};
  std::thread read_thread_;
  std::atomic<bool> running_{false};

  mutable std::mutex data_mutex_;
  Telemetry telemetry_{};
  Alert alert_{};
  bool has_telemetry_{false};

  std::atomic<uint64_t> telemetry_count_{0};
  std::atomic<uint64_t> alert_count_{0};
  std::atomic<uint64_t> crc_error_count_{0};
  std::atomic<uint64_t> framing_error_count_{0};

  ParseState state_{ParseState::WaitSof};
  uint8_t  msg_id_{0};
  uint16_t data_len_{0};
  uint16_t data_idx_{0};
  uint8_t  crc_running_{0};
  uint8_t  crc_received_{0};
  uint8_t  data_buf_[64]{};

  // Firmware emits each binary byte as two ASCII hex digits ("11" -> 0x11).
  bool     nibble_pending_{false};
  uint8_t  nibble_high_{0};
  uint16_t waitsof_skip_count_{0};
};

}  // namespace power_dist_board_sdk
