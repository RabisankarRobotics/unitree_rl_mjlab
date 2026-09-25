#pragma once

#include "can_sdk/types.hpp"

#include <array>
#include <cstdint>
#include <string>

namespace can_sdk {

constexpr std::size_t kMaxActuators = 64;

class CanMaster {
public:
  CanMaster();
  ~CanMaster();

  // Delete copy constructor and assignment operator
  CanMaster(const CanMaster &) = delete;
  CanMaster &operator=(const CanMaster &) = delete;

  bool init(const std::string &interface_name);
  void close();

  void addJoint(uint16_t joint_id);

  bool sendMotionCommand(uint16_t joint_id, const MotionCommand &command);
  bool requestMotorStatus(uint16_t joint_id);

  // Receive ALL pending frames (uses select() with timeout)
  void recvAll(int timeout_us = 500);

  // Get cached feedback/status (after calling recvAll)
  MotionFeedback getMotionFeedback(uint16_t joint_id) const;
  MotorStatus getMotorStatus(uint16_t joint_id) const;

  // Helper functions
  bool isMotionFeedbackStale(uint16_t joint_id, int max_stale_us) const;
  bool isMotorStatusStale(uint16_t joint_id, int max_stale_us) const;

private:
  struct JointData {
    bool registered = false;
    MotionFeedback feedback;
    MotorStatus status;
  };

  bool isDataAvailable(int timeout_us);
  void processFrame(uint32_t can_id, const uint8_t *data, uint8_t dlc);

  std::string interface_name_;
  int socket_fd_ = -1;
  std::array<JointData, kMaxActuators + 1> joints_{};
};

} // namespace can_sdk
