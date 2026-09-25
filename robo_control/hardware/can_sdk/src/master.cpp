#include "can_sdk/master.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <iostream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

namespace can_sdk {

namespace {

// Constants for unit conversion
constexpr float kTorqueConstant = 2.4f;
constexpr float kPositionRange[] = {-12.5f, 12.5f};
constexpr float kVelocityRange[] = {-45.0f, 45.0f};
constexpr float kTorqueRange[] = {-24.0f, 24.0f};
constexpr float kKpRange[] = {0.0f, 500.0f};
constexpr float kKdRange[] = {0.0f, 50.0f};

// Protocol IDs
constexpr uint16_t kBaseCommandId = 0x140;
constexpr uint16_t kBaseStatusId = 0x240;
constexpr uint16_t kBaseMotionCommandId = 0x400;
constexpr uint16_t kBaseMotionReplyId = 0x500;
constexpr uint8_t kMotorStatus2Command = 0x9C;

inline uint16_t floatToUint(double value, const float range[], unsigned bits) {
  const auto maxRaw = (1u << bits) - 1u;
  const double clamped = std::clamp(value, static_cast<double>(range[0]),
                                    static_cast<double>(range[1]));
  const double span = range[1] - range[0];
  const double scaled =
      (clamped - range[0]) * static_cast<double>(maxRaw) / span;
  const auto rounded = static_cast<long long>(std::llround(scaled));
  if (rounded < 0)
    return 0;
  if (rounded > static_cast<long long>(maxRaw))
    return static_cast<uint16_t>(maxRaw);
  return static_cast<uint16_t>(rounded);
}

inline double uintToFloat(uint16_t raw, const float range[], unsigned bits) {
  const auto maxRaw = (1u << bits) - 1u;
  const double span = range[1] - range[0];
  const double scaled =
      static_cast<double>(raw) * span / static_cast<double>(maxRaw);
  return scaled + range[0];
}

} // namespace

CanMaster::CanMaster() = default;

CanMaster::~CanMaster() { close(); }

bool CanMaster::init(const std::string &interface_name) {
  interface_name_ = interface_name;

  socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    std::cerr << "Error creating CAN socket: " << std::strerror(errno)
              << std::endl;
    return false;
  }

  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';
  if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
    std::cerr << "Error getting interface index: " << std::strerror(errno)
              << std::endl;
    ::close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(socket_fd_, reinterpret_cast<struct sockaddr *>(&addr),
           sizeof(addr)) < 0) {
    std::cerr << "Error binding CAN socket: " << std::strerror(errno)
              << std::endl;
    ::close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  return true;
}

void CanMaster::close() {
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
}

void CanMaster::addJoint(uint16_t joint_id) {
  if (joint_id == 0 || joint_id > kMaxActuators) {
    std::cerr << "Joint id out of range (1-64): " << joint_id << std::endl;
    return;
  }
  joints_[joint_id].registered = true;
}

bool CanMaster::sendMotionCommand(uint16_t joint_id,
                                  const MotionCommand &command) {
  if (socket_fd_ < 0)
    return false;

  const uint16_t positionRaw =
      floatToUint(command.position, kPositionRange, 16);
  const uint16_t velocityRaw =
      floatToUint(command.velocity, kVelocityRange, 12);
  const uint16_t torqueRaw = floatToUint(command.torque, kTorqueRange, 12);
  const uint16_t kpRaw = floatToUint(command.kp, kKpRange, 12);
  const uint16_t kdRaw = floatToUint(command.kd, kKdRange, 12);

  struct can_frame frame;
  frame.can_id = (kBaseMotionCommandId + joint_id) & CAN_SFF_MASK;
  frame.can_dlc = 8;
  frame.data[0] = static_cast<uint8_t>((positionRaw >> 8) & 0xFF);
  frame.data[1] = static_cast<uint8_t>(positionRaw & 0xFF);
  frame.data[2] = static_cast<uint8_t>((velocityRaw >> 4) & 0xFF);
  frame.data[3] =
      static_cast<uint8_t>(((velocityRaw & 0xF) << 4) | ((kpRaw >> 8) & 0xF));
  frame.data[4] = static_cast<uint8_t>(kpRaw & 0xFF);
  frame.data[5] = static_cast<uint8_t>((kdRaw >> 4) & 0xFF);
  frame.data[6] =
      static_cast<uint8_t>(((kdRaw & 0xF) << 4) | ((torqueRaw >> 8) & 0xF));
  frame.data[7] = static_cast<uint8_t>(torqueRaw & 0xFF);

  ssize_t nbytes = write(socket_fd_, &frame, sizeof(frame));
  return nbytes == sizeof(frame);
}

bool CanMaster::requestMotorStatus(uint16_t joint_id) {
  if (socket_fd_ < 0)
    return false;

  struct can_frame frame;
  frame.can_id = (kBaseCommandId + joint_id) & CAN_SFF_MASK;
  frame.can_dlc = 8;
  std::memset(frame.data, 0, 8);
  frame.data[0] = kMotorStatus2Command;

  ssize_t nbytes = write(socket_fd_, &frame, sizeof(frame));
  return nbytes == sizeof(frame);
}

bool CanMaster::isDataAvailable(int timeout_us) {
  if (socket_fd_ < 0)
    return false;

  fd_set read_fds;
  FD_ZERO(&read_fds);
  FD_SET(socket_fd_, &read_fds);

  struct timeval timeout;
  timeout.tv_sec = timeout_us / 1000000;
  timeout.tv_usec = timeout_us % 1000000;

  int result = select(socket_fd_ + 1, &read_fds, nullptr, nullptr, &timeout);
  return (result > 0);
}

void CanMaster::recvAll(int timeout_us) {
  while (isDataAvailable(timeout_us)) {
    struct can_frame frame;
    ssize_t nbytes = read(socket_fd_, &frame, sizeof(frame));
    if (nbytes == sizeof(frame)) {
      processFrame(frame.can_id, frame.data, frame.can_dlc);
    } else {
      break;
    }
  }
}

void CanMaster::processFrame(uint32_t can_id, const uint8_t *data,
                             uint8_t dlc) {
  const uint16_t id = can_id & CAN_SFF_MASK;
  const auto now = std::chrono::steady_clock::now();

  // Motion feedback (0x500 + joint_id)
  if (id >= kBaseMotionReplyId && id < kBaseMotionReplyId + kMaxActuators) {
    const uint16_t joint_id = id - kBaseMotionReplyId;

    if (joint_id <= kMaxActuators && joints_[joint_id].registered) {
      if (dlc < 6)
        return;

      const uint16_t positionRaw = (static_cast<uint16_t>(data[1]) << 8) |
                                   static_cast<uint16_t>(data[2]);
      const uint16_t velocityRaw =
          (static_cast<uint16_t>(data[3]) << 4) |
          ((static_cast<uint16_t>(data[4]) >> 4) & 0x0F);
      const uint16_t torqueRaw =
          ((static_cast<uint16_t>(data[4]) & 0x0F) << 8) |
          static_cast<uint16_t>(data[5]);

      auto &joint = joints_[joint_id];
      joint.feedback.position = uintToFloat(positionRaw, kPositionRange, 16);
      joint.feedback.velocity = uintToFloat(velocityRaw, kVelocityRange, 12);
      joint.feedback.torque = uintToFloat(torqueRaw, kTorqueRange, 12);
      joint.feedback.timestamp = now;
    }
    return;
  }

  // Motor status (0x240 + joint_id)
  if (id >= kBaseStatusId && id < kBaseStatusId + kMaxActuators) {
    const uint16_t joint_id = id - kBaseStatusId;

    if (joint_id <= kMaxActuators && joints_[joint_id].registered) {
      if (dlc < 8)
        return;

      const int16_t iqRaw =
          static_cast<int16_t>(static_cast<uint16_t>(data[2]) |
                               (static_cast<uint16_t>(data[3]) << 8));
      const int16_t speedRaw =
          static_cast<int16_t>(static_cast<uint16_t>(data[4]) |
                               (static_cast<uint16_t>(data[5]) << 8));
      const int16_t angleRaw =
          static_cast<int16_t>(static_cast<uint16_t>(data[6]) |
                               (static_cast<uint16_t>(data[7]) << 8));
      const int8_t tempRaw = static_cast<int8_t>(data[1]);

      auto &joint = joints_[joint_id];
      joint.status.position = static_cast<double>(angleRaw) * (M_PI / 180.0);
      joint.status.velocity = static_cast<double>(speedRaw) * (M_PI / 180.0);
      joint.status.torque =
          (static_cast<double>(iqRaw) * 0.01) * kTorqueConstant;
      joint.status.temperature = static_cast<double>(tempRaw);
      joint.status.timestamp = now;
    }
  }
}

MotionFeedback CanMaster::getMotionFeedback(uint16_t joint_id) const {
  if (joint_id == 0 || joint_id > kMaxActuators) {
    return {};
  }
  return joints_[joint_id].feedback;
}

MotorStatus CanMaster::getMotorStatus(uint16_t joint_id) const {
  if (joint_id == 0 || joint_id > kMaxActuators) {
    return {};
  }
  return joints_[joint_id].status;
}

bool CanMaster::isMotionFeedbackStale(uint16_t joint_id,
                                      int max_stale_us) const {
  if (joint_id == 0 || joint_id > kMaxActuators) {
    return true;
  }
  return std::chrono::steady_clock::now() -
             joints_[joint_id].feedback.timestamp >
         std::chrono::microseconds(max_stale_us);
}

bool CanMaster::isMotorStatusStale(uint16_t joint_id, int max_stale_us) const {
  if (joint_id == 0 || joint_id > kMaxActuators) {
    return true;
  }
  return std::chrono::steady_clock::now() - joints_[joint_id].status.timestamp >
         std::chrono::microseconds(max_stale_us);
}

} // namespace can_sdk
