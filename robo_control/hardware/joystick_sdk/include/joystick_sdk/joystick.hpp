#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace joystick_sdk {

enum class DeviceType { Joydev, Evdev };

enum class ButtonName : uint8_t {
  A = 0,
  B = 1,
  X = 2,
  Y = 3,
  LB = 4,
  RB = 5,
  BACK = 6,
  START = 7,
  GUIDE = 8,
  LS = 9,
  RS = 10
};

struct JoystickState {
  float x;
  float y;
  float lt;
  float rx;
  float ry;
  float rt;
  uint32_t buttons;
  int8_t hat_x;
  int8_t hat_y;

  JoystickState()
      : x(0.0f), y(0.0f), lt(0.0f), rx(0.0f), ry(0.0f), rt(0.0f), buttons(0),
        hat_x(0), hat_y(0) {}
};

class Joystick {
public:
  Joystick();
  explicit Joystick(const std::string &device_path);
  ~Joystick();

  Joystick(const Joystick &) = delete;
  Joystick &operator=(const Joystick &) = delete;

  JoystickState getState() const;
  std::string getDevicePath() const;
  std::string getDeviceName() const;
  bool isConnected() const;
  bool isButtonPressed(ButtonName button) const;
  std::vector<ButtonName> getPressedButtons() const;

private:
  struct JoystickDeviceInfo {
    std::string path;
    DeviceType type;
  };

  struct AxisRange {
    int32_t min = -32768;
    int32_t max = 32767;
    bool has_range = false;
  };

  static std::optional<JoystickDeviceInfo> detectJoystick();
  static DeviceType inferDeviceType(const std::string &device_path);
  void initialize(const std::string &device_path, DeviceType type);
  void readThreadFunc();
  bool readEvent();
  bool readEvdevEvent();
  bool readJoydevEvent();
  static bool isEvdevGamepad(int fd);
  static bool testBit(const unsigned long *array, size_t bit);
  float normalizeAxisValue(int code, int value) const;
  void cacheAxisRange(int fd, int code);

  int device_fd_;
  std::string device_path_;
  std::string device_name_;
  std::thread read_thread_;
  std::atomic<bool> running_;
  mutable std::mutex data_mutex_;
  JoystickState state_;
  uint8_t num_axes_;
  uint8_t num_buttons_;
  DeviceType device_type_;
  std::unordered_map<int, AxisRange> axis_ranges_;
};

} // namespace joystick_sdk
