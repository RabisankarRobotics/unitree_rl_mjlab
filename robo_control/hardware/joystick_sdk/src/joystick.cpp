#include "joystick_sdk/joystick.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <fcntl.h>
#include <linux/input.h>
#include <linux/joystick.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cstring>
#include <filesystem>
#include <optional>
#include <stdexcept>

namespace joystick_sdk {

namespace {
constexpr int kMaxJoydevDevices = 10;
constexpr int kMaxEvdevDevices = 64;

std::optional<uint8_t> mapEvdevButton(uint16_t code) {
  switch (code) {
  case BTN_SOUTH:
    return static_cast<uint8_t>(ButtonName::A);
  case BTN_EAST:
    return static_cast<uint8_t>(ButtonName::B);
  case BTN_NORTH:
    return static_cast<uint8_t>(ButtonName::Y);
  case BTN_WEST:
    return static_cast<uint8_t>(ButtonName::X);
  case BTN_TL:
    return static_cast<uint8_t>(ButtonName::LB);
  case BTN_TR:
    return static_cast<uint8_t>(ButtonName::RB);
  case BTN_SELECT:
  case BTN_BACK:
    return static_cast<uint8_t>(ButtonName::BACK);
  case BTN_START:
    return static_cast<uint8_t>(ButtonName::START);
  case BTN_MODE:
    return static_cast<uint8_t>(ButtonName::GUIDE);
  case BTN_THUMBL:
    return static_cast<uint8_t>(ButtonName::LS);
  case BTN_THUMBR:
    return static_cast<uint8_t>(ButtonName::RS);
  default:
    return std::nullopt;
  }
}
} // namespace

Joystick::Joystick()
    : device_fd_(-1), running_(false), num_axes_(0), num_buttons_(0),
      device_type_(DeviceType::Joydev) {
  auto device_info = detectJoystick();
  if (!device_info) {
    throw std::runtime_error("No joystick device found");
  }
  initialize(device_info->path, device_info->type);
}

Joystick::Joystick(const std::string &device_path)
    : device_fd_(-1), running_(false), num_axes_(0), num_buttons_(0),
      device_type_(DeviceType::Joydev) {
  initialize(device_path, inferDeviceType(device_path));
}

Joystick::~Joystick() {
  running_ = false;
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
  if (device_fd_ >= 0) {
    close(device_fd_);
  }
}

JoystickState Joystick::getState() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return state_;
}

std::string Joystick::getDevicePath() const { return device_path_; }

std::string Joystick::getDeviceName() const { return device_name_; }

bool Joystick::isConnected() const { return device_fd_ >= 0 && running_; }

bool Joystick::isButtonPressed(ButtonName button) const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return (state_.buttons & (1 << static_cast<uint8_t>(button))) != 0;
}

std::vector<ButtonName> Joystick::getPressedButtons() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  std::vector<ButtonName> pressed;

  if (state_.buttons & (1 << static_cast<uint8_t>(ButtonName::A)))
    pressed.push_back(ButtonName::A);
  if (state_.buttons & (1 << static_cast<uint8_t>(ButtonName::B)))
    pressed.push_back(ButtonName::B);
  if (state_.buttons & (1 << static_cast<uint8_t>(ButtonName::X)))
    pressed.push_back(ButtonName::X);
  if (state_.buttons & (1 << static_cast<uint8_t>(ButtonName::Y)))
    pressed.push_back(ButtonName::Y);
  if (state_.buttons & (1 << static_cast<uint8_t>(ButtonName::LB)))
    pressed.push_back(ButtonName::LB);
  if (state_.buttons & (1 << static_cast<uint8_t>(ButtonName::RB)))
    pressed.push_back(ButtonName::RB);
  if (state_.buttons & (1 << static_cast<uint8_t>(ButtonName::BACK)))
    pressed.push_back(ButtonName::BACK);
  if (state_.buttons & (1 << static_cast<uint8_t>(ButtonName::START)))
    pressed.push_back(ButtonName::START);
  if (state_.buttons & (1 << static_cast<uint8_t>(ButtonName::GUIDE)))
    pressed.push_back(ButtonName::GUIDE);
  if (state_.buttons & (1 << static_cast<uint8_t>(ButtonName::LS)))
    pressed.push_back(ButtonName::LS);
  if (state_.buttons & (1 << static_cast<uint8_t>(ButtonName::RS)))
    pressed.push_back(ButtonName::RS);

  return pressed;
}

std::optional<Joystick::JoystickDeviceInfo> Joystick::detectJoystick() {
  for (int i = 0; i < kMaxJoydevDevices; ++i) {
    std::string device_path = "/dev/input/js" + std::to_string(i);
    if (std::filesystem::exists(device_path)) {
      int fd = open(device_path.c_str(), O_RDONLY | O_NONBLOCK);
      if (fd >= 0) {
        close(fd);
        return JoystickDeviceInfo{device_path, DeviceType::Joydev};
      }
    }
  }

  for (int i = 0; i < kMaxEvdevDevices; ++i) {
    std::string device_path = "/dev/input/event" + std::to_string(i);
    if (!std::filesystem::exists(device_path)) {
      continue;
    }

    int fd = open(device_path.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd < 0) {
      continue;
    }

    bool is_gamepad = isEvdevGamepad(fd);
    close(fd);

    if (is_gamepad) {
      return JoystickDeviceInfo{device_path, DeviceType::Evdev};
    }
  }

  return std::nullopt;
}

DeviceType Joystick::inferDeviceType(const std::string &device_path) {
  if (device_path.find("event") != std::string::npos) {
    return DeviceType::Evdev;
  }

  return DeviceType::Joydev;
}

void Joystick::initialize(const std::string &device_path, DeviceType type) {
  device_path_ = device_path;
  device_type_ = type;
  axis_ranges_.clear();

  device_fd_ = open(device_path.c_str(), O_RDONLY | O_NONBLOCK);
  if (device_fd_ < 0) {
    throw std::runtime_error("Failed to open joystick device: " + device_path);
  }

  char name[128];
  if (device_type_ == DeviceType::Joydev) {
    if (ioctl(device_fd_, JSIOCGNAME(sizeof(name)), name) >= 0) {
      device_name_ = std::string(name);
    } else {
      device_name_ = "Unknown Joystick";
    }

    if (ioctl(device_fd_, JSIOCGAXES, &num_axes_) < 0) {
      num_axes_ = 0;
    }

    if (ioctl(device_fd_, JSIOCGBUTTONS, &num_buttons_) < 0) {
      num_buttons_ = 0;
    }
  } else {
    if (ioctl(device_fd_, EVIOCGNAME(sizeof(name)), name) >= 0) {
      device_name_ = std::string(name);
    } else {
      device_name_ = "Unknown Joystick";
    }

    num_axes_ = 0;
    num_buttons_ = 0;

    cacheAxisRange(device_fd_, ABS_X);
    cacheAxisRange(device_fd_, ABS_Y);
    cacheAxisRange(device_fd_, ABS_Z);
    cacheAxisRange(device_fd_, ABS_RX);
    cacheAxisRange(device_fd_, ABS_RY);
    cacheAxisRange(device_fd_, ABS_RZ);
    cacheAxisRange(device_fd_, ABS_HAT0X);
    cacheAxisRange(device_fd_, ABS_HAT0Y);
  }

  running_ = true;
  read_thread_ = std::thread(&Joystick::readThreadFunc, this);
}

void Joystick::readThreadFunc() {
  while (running_) {
    if (!readEvent()) {
      if (running_) {
        usleep(1000);
      }
    }
  }
}

bool Joystick::readEvent() {
  if (device_type_ == DeviceType::Evdev) {
    return readEvdevEvent();
  }

  return readJoydevEvent();
}

bool Joystick::readEvdevEvent() {
  struct input_event event;
  ssize_t bytes = read(device_fd_, &event, sizeof(event));

  if (bytes != sizeof(event)) {
    if (bytes < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      return false;
    }
    return false;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);

  if (event.type == EV_ABS) {
    switch (event.code) {
    case ABS_X:
      state_.x = normalizeAxisValue(ABS_X, event.value);
      break;
    case ABS_Y:
      state_.y = normalizeAxisValue(ABS_Y, event.value);
      break;
    case ABS_Z:
      state_.lt = normalizeAxisValue(ABS_Z, event.value);
      break;
    case ABS_RX:
      state_.rx = normalizeAxisValue(ABS_RX, event.value);
      break;
    case ABS_RY:
      state_.ry = normalizeAxisValue(ABS_RY, event.value);
      break;
    case ABS_RZ:
      state_.rt = normalizeAxisValue(ABS_RZ, event.value);
      break;
    case ABS_HAT0X:
      state_.hat_x = (event.value < 0) ? -1 : (event.value > 0 ? 1 : 0);
      break;
    case ABS_HAT0Y:
      state_.hat_y = (event.value < 0) ? -1 : (event.value > 0 ? 1 : 0);
      break;
    default:
      break;
    }
  } else if (event.type == EV_KEY) {
    auto mapped = mapEvdevButton(event.code);
    if (mapped && *mapped < 32) {
      if (event.value) {
        state_.buttons |= (1 << *mapped);
      } else {
        state_.buttons &= ~(1 << *mapped);
      }
    }
  }

  return true;
}

bool Joystick::readJoydevEvent() {
  struct js_event event;
  ssize_t bytes = read(device_fd_, &event, sizeof(event));

  if (bytes != sizeof(event)) {
    if (bytes < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      return false;
    }
    return false;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);

  if (event.type & JS_EVENT_AXIS) {
    float value = event.value / 32767.0f;

    switch (event.number) {
    case 0:
      state_.x = value;
      break;
    case 1:
      state_.y = value;
      break;
    case 2:
      state_.lt = value;
      break;
    case 3:
      state_.rx = value;
      break;
    case 4:
      state_.ry = value;
      break;
    case 5:
      state_.rt = value;
      break;
    case 6:
      state_.hat_x = (event.value < -16384)  ? -1
                     : (event.value > 16384) ? 1
                                             : 0;
      break;
    case 7:
      state_.hat_y = (event.value < -16384)  ? -1
                     : (event.value > 16384) ? 1
                                             : 0;
      break;
    default:
      break;
    }
  } else if (event.type & JS_EVENT_BUTTON) {
    if (event.number < 32) {
      if (event.value) {
        state_.buttons |= (1 << event.number);
      } else {
        state_.buttons &= ~(1 << event.number);
      }
    }
  }

  return true;
}

bool Joystick::isEvdevGamepad(int fd) {
  constexpr size_t bits_per_long = sizeof(unsigned long) * 8;
  std::array<unsigned long, (EV_MAX / bits_per_long) + 1> ev_bits{};

  if (ioctl(fd, EVIOCGBIT(0, ev_bits.size() * sizeof(unsigned long)),
            ev_bits.data()) < 0) {
    return false;
  }

  if (!testBit(ev_bits.data(), EV_KEY) || !testBit(ev_bits.data(), EV_ABS)) {
    return false;
  }

  std::array<unsigned long, (KEY_MAX / bits_per_long) + 1> key_bits{};
  if (ioctl(fd, EVIOCGBIT(EV_KEY, key_bits.size() * sizeof(unsigned long)),
            key_bits.data()) < 0) {
    return false;
  }

  const int button_candidates[] = {BTN_GAMEPAD, BTN_SOUTH, BTN_EAST, BTN_NORTH,
                                   BTN_WEST,    BTN_TL,    BTN_TR};
  bool has_button = false;
  for (int code : button_candidates) {
    if (testBit(key_bits.data(), code)) {
      has_button = true;
      break;
    }
  }

  std::array<unsigned long, (ABS_MAX / bits_per_long) + 1> abs_bits{};
  if (ioctl(fd, EVIOCGBIT(EV_ABS, abs_bits.size() * sizeof(unsigned long)),
            abs_bits.data()) < 0) {
    return false;
  }

  bool has_axes =
      testBit(abs_bits.data(), ABS_X) && testBit(abs_bits.data(), ABS_Y);

  return has_button && has_axes;
}

bool Joystick::testBit(const unsigned long *array, size_t bit) {
  constexpr size_t bits_per_long = sizeof(unsigned long) * 8;
  return (array[bit / bits_per_long] >> (bit % bits_per_long)) & 1UL;
}

float Joystick::normalizeAxisValue(int code, int value) const {
  auto it = axis_ranges_.find(code);
  int32_t min = -32768;
  int32_t max = 32767;
  if (it != axis_ranges_.end() && it->second.has_range) {
    min = it->second.min;
    max = it->second.max;
  }

  if (max == min) {
    return 0.0f;
  }

  float normalized =
      static_cast<float>(value - min) / static_cast<float>(max - min);
  normalized = (normalized * 2.0f) - 1.0f;
  return std::clamp(normalized, -1.0f, 1.0f);
}

void Joystick::cacheAxisRange(int fd, int code) {
  struct input_absinfo abs_info{};
  if (ioctl(fd, EVIOCGABS(code), &abs_info) == 0) {
    axis_ranges_[code] = AxisRange{abs_info.minimum, abs_info.maximum, true};
  }
}

} // namespace joystick_sdk
