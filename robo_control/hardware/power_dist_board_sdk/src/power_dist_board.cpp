#include "power_dist_board_sdk/power_dist_board.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>

#include "crc8_table.hpp"

namespace power_dist_board_sdk {

namespace {

constexpr uint8_t kSof = 0x11;
constexpr uint8_t kEof = 0x22;
constexpr uint8_t kMsgTelemetry = 0x00;
constexpr uint8_t kMsgAlert = 0x01;
constexpr uint16_t kMaxDataLen = 64;
constexpr uint16_t kTelemetryDataLen = 10;
constexpr uint16_t kAlertDataLen = 2;

speed_t toBaud(int baudrate) {
  switch (baudrate) {
    case 9600:    return B9600;
    case 19200:   return B19200;
    case 38400:   return B38400;
    case 57600:   return B57600;
    case 115200:  return B115200;
    case 230400:  return B230400;
    case 460800:  return B460800;
    case 921600:  return B921600;
    default:      return 0;
  }
}

uint16_t readU16LE(const uint8_t *p) {
  return static_cast<uint16_t>(p[0]) |
         (static_cast<uint16_t>(p[1]) << 8);
}

int32_t readI32LE(const uint8_t *p) {
  uint32_t u = static_cast<uint32_t>(p[0]) |
               (static_cast<uint32_t>(p[1]) << 8) |
               (static_cast<uint32_t>(p[2]) << 16) |
               (static_cast<uint32_t>(p[3]) << 24);
  int32_t s;
  std::memcpy(&s, &u, sizeof(s));
  return s;
}

}  // namespace

PowerDistBoard::PowerDistBoard(const std::string &device, int baudrate) {
  speed_t baud = toBaud(baudrate);
  if (baud == 0) {
    throw std::runtime_error("PowerDistBoard: unsupported baudrate " +
                             std::to_string(baudrate));
  }

  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY);
  if (fd_ < 0) {
    throw std::runtime_error("PowerDistBoard: failed to open " + device + ": " +
                             std::strerror(errno));
  }

  termios tty{};
  if (::tcgetattr(fd_, &tty) != 0) {
    int err = errno;
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error("PowerDistBoard: tcgetattr failed: " +
                             std::string(std::strerror(err)));
  }

  ::cfsetospeed(&tty, baud);
  ::cfsetispeed(&tty, baud);

  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag |= CREAD | CLOCAL;
  tty.c_cflag &= ~CRTSCTS;

  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG | IEXTEN);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;

  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 1;

  if (::tcsetattr(fd_, TCSANOW, &tty) != 0) {
    int err = errno;
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error("PowerDistBoard: tcsetattr failed: " +
                             std::string(std::strerror(err)));
  }

  ::tcflush(fd_, TCIOFLUSH);

  running_ = true;
  read_thread_ = std::thread(&PowerDistBoard::readThreadFunc, this);
}

PowerDistBoard::~PowerDistBoard() {
  running_ = false;
  if (read_thread_.joinable()) read_thread_.join();
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

Telemetry PowerDistBoard::telemetry() const {
  std::lock_guard<std::mutex> lk(data_mutex_);
  return telemetry_;
}

Alert PowerDistBoard::alert() const {
  std::lock_guard<std::mutex> lk(data_mutex_);
  return alert_;
}

bool PowerDistBoard::hasTelemetry() const {
  std::lock_guard<std::mutex> lk(data_mutex_);
  return has_telemetry_;
}

uint64_t PowerDistBoard::telemetryCount() const     { return telemetry_count_.load(std::memory_order_relaxed); }
uint64_t PowerDistBoard::alertCount() const         { return alert_count_.load(std::memory_order_relaxed); }
uint64_t PowerDistBoard::crcErrorCount() const      { return crc_error_count_.load(std::memory_order_relaxed); }
uint64_t PowerDistBoard::framingErrorCount() const  { return framing_error_count_.load(std::memory_order_relaxed); }

void PowerDistBoard::readThreadFunc() {
  uint8_t chunk[128];
  while (running_.load(std::memory_order_relaxed)) {
    ssize_t n = ::read(fd_, chunk, sizeof(chunk));
    if (n > 0) {
      for (ssize_t i = 0; i < n; ++i) feedAscii(chunk[i]);
    } else if (n < 0 && errno != EINTR && errno != EAGAIN) {
      break;
    }
  }
}

void PowerDistBoard::feedAscii(uint8_t c) {
  uint8_t nibble;
  if (c >= '0' && c <= '9')      nibble = static_cast<uint8_t>(c - '0');
  else if (c >= 'A' && c <= 'F') nibble = static_cast<uint8_t>(c - 'A' + 10);
  else if (c >= 'a' && c <= 'f') nibble = static_cast<uint8_t>(c - 'a' + 10);
  else                           return;

  if (!nibble_pending_) {
    nibble_high_ = nibble;
    nibble_pending_ = true;
    return;
  }

  uint8_t byte = static_cast<uint8_t>((nibble_high_ << 4) | nibble);
  nibble_pending_ = false;

  if (state_ != ParseState::WaitSof) {
    feedByte(byte);
    return;
  }

  if (byte == kSof) {
    waitsof_skip_count_ = 0;
    feedByte(byte);
    return;
  }

  // Recover from opening the stream mid-byte: shift nibble alignment by one
  // after a frame's worth of bytes failed to land on a SOF.
  if (++waitsof_skip_count_ >= 16) {
    nibble_high_ = nibble;
    nibble_pending_ = true;
    waitsof_skip_count_ = 0;
  }
}

void PowerDistBoard::resetParser() {
  state_ = ParseState::WaitSof;
  data_len_ = 0;
  data_idx_ = 0;
  crc_running_ = 0;
}

void PowerDistBoard::feedByte(uint8_t b) {
  switch (state_) {
    case ParseState::WaitSof:
      if (b == kSof) state_ = ParseState::LenL;
      break;

    case ParseState::LenL:
      data_len_ = b;
      state_ = ParseState::LenH;
      break;

    case ParseState::LenH:
      data_len_ |= static_cast<uint16_t>(b) << 8;
      if (data_len_ > kMaxDataLen) {
        framing_error_count_.fetch_add(1, std::memory_order_relaxed);
        resetParser();
      } else {
        state_ = ParseState::MsgId;
      }
      break;

    case ParseState::MsgId:
      msg_id_ = b;
      data_idx_ = 0;
      state_ = (data_len_ == 0) ? ParseState::Crc : ParseState::Data;
      break;

    case ParseState::Data:
      data_buf_[data_idx_++] = b;
      crc_running_ = crc8Update(crc_running_, b);
      if (data_idx_ == data_len_) state_ = ParseState::Crc;
      break;

    case ParseState::Crc:
      crc_received_ = b;
      state_ = ParseState::Eof;
      break;

    case ParseState::Eof:
      if (b == kEof && crc_received_ == crc_running_) {
        dispatchFrame();
      } else if (b != kEof) {
        framing_error_count_.fetch_add(1, std::memory_order_relaxed);
      } else {
        crc_error_count_.fetch_add(1, std::memory_order_relaxed);
      }
      resetParser();
      break;
  }
}

void PowerDistBoard::dispatchFrame() {
  switch (msg_id_) {
    case kMsgTelemetry:
      handleTelemetry(data_buf_, data_len_);
      break;
    case kMsgAlert:
      handleAlert(data_buf_, data_len_);
      break;
    default:
      framing_error_count_.fetch_add(1, std::memory_order_relaxed);
      break;
  }
}

void PowerDistBoard::handleTelemetry(const uint8_t *data, uint16_t len) {
  if (len != kTelemetryDataLen) {
    framing_error_count_.fetch_add(1, std::memory_order_relaxed);
    return;
  }

  Telemetry t;
  t.temperature_c      = readI32LE(data + 0);
  t.current_ma         = readU16LE(data + 4);
  t.voltage_mv         = readU16LE(data + 6);
  t.estop_pressed      = (data[8] != 0);
  t.shutdown_requested = (data[9] != 0);
  t.timestamp          = std::chrono::steady_clock::now();

  {
    std::lock_guard<std::mutex> lk(data_mutex_);
    telemetry_ = t;
    has_telemetry_ = true;
  }
  telemetry_count_.fetch_add(1, std::memory_order_relaxed);
}

void PowerDistBoard::handleAlert(const uint8_t *data, uint16_t len) {
  if (len != kAlertDataLen) {
    framing_error_count_.fetch_add(1, std::memory_order_relaxed);
    return;
  }

  Alert a;
  a.flags     = readU16LE(data);
  a.timestamp = std::chrono::steady_clock::now();

  {
    std::lock_guard<std::mutex> lk(data_mutex_);
    alert_ = a;
  }
  alert_count_.fetch_add(1, std::memory_order_relaxed);
}

}  // namespace power_dist_board_sdk
