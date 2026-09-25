#pragma once

#include <array>
#include <cstddef>
#include <deque>
#include <vector>

namespace policy_service {

struct Observations {
  std::array<float, 3> vel_cmd{};       // Velocity command from joystick
  std::array<float, 3> ang_vel{};       // Angular velocity from IMU gyroscope
  std::array<float, 3> proj_gravity{};  // Projected gravity in body frame
  std::vector<float> joint_pos;         // Joint positions offset from default
  std::vector<float> joint_vel;         // Joint velocities
};

// Per-term circular buffer that maintains H timesteps of history.
// Flattens to Isaac Lab layout: [term0_oldest..term0_newest,
//                                term1_oldest..term1_newest, ...]
class ObservationHistory {
public:
  ObservationHistory(int history_length, int num_terms)
      : history_length_(history_length), buffers_(num_terms),
        initialized_(false) {}

  // Push one timestep of observation terms into the history buffers.
  // obs_terms[i] is the vector of floats for term i at the current timestep.
  void push(const std::vector<std::vector<float>> &obs_terms) {
    if (!initialized_) {
      // Fill each buffer with H copies of the first observation
      for (std::size_t i = 0; i < obs_terms.size(); ++i) {
        for (int h = 0; h < history_length_; ++h) {
          buffers_[i].push_back(obs_terms[i]);
        }
      }
      initialized_ = true;
    } else {
      for (std::size_t i = 0; i < obs_terms.size(); ++i) {
        buffers_[i].push_back(obs_terms[i]);
        if (static_cast<int>(buffers_[i].size()) > history_length_) {
          buffers_[i].pop_front();
        }
      }
    }
  }

  // Flatten all buffers into a single vector:
  // [term0_oldest..term0_newest, term1_oldest..term1_newest, ...]
  std::vector<float> flatten() const {
    std::vector<float> result;
    result.reserve(total_size());
    for (const auto &buf : buffers_) {
      for (const auto &frame : buf) {
        result.insert(result.end(), frame.begin(), frame.end());
      }
    }
    return result;
  }

  void reset() {
    for (auto &buf : buffers_) {
      buf.clear();
    }
    initialized_ = false;
  }

  int history_length() const { return history_length_; }

private:
  std::size_t total_size() const {
    std::size_t size = 0;
    for (const auto &buf : buffers_) {
      for (const auto &frame : buf) {
        size += frame.size();
      }
    }
    return size;
  }

  int history_length_;
  std::vector<std::deque<std::vector<float>>> buffers_;
  bool initialized_;
};

// Compute observation vector size for a single timestep
inline std::size_t computeSingleObservationSize(int num_joints) {
  return 9 + static_cast<std::size_t>(num_joints) * 3;
}

// Compute total observation size including history
inline std::size_t computeObservationSize(int num_joints, int history_length) {
  return computeSingleObservationSize(num_joints) * history_length;
}

} // namespace policy_service
