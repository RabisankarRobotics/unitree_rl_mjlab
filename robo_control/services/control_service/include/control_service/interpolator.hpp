#pragma once

namespace control_service {

class JointInterpolator {
public:
  JointInterpolator() = default;

  void reset(double current_position) {
    current_position_ = current_position;
    target_position_ = current_position;
    step_size_ = 0.0;
    steps_remaining_ = 0;
  }

  void setTarget(double target_position, int steps) {
    if (steps <= 0) {
      // Immediate jump if steps is 0 (should not happen in normal operation)
      current_position_ = target_position;
      step_size_ = 0.0;
      steps_remaining_ = 0;
    } else {
      target_position_ = target_position;
      steps_remaining_ = steps;
      step_size_ = (target_position - current_position_) / steps;
    }
  }

  double process() {
    if (steps_remaining_ > 0) {
      steps_remaining_--;
      if (steps_remaining_ == 0) {
        current_position_ = target_position_; // Snap to exact target
      } else {
        current_position_ += step_size_;
      }
    }
    return current_position_;
  }

  bool isComplete() const { return steps_remaining_ == 0; }

  double getCurrentPosition() const { return current_position_; }

private:
  double current_position_ = 0.0;
  double target_position_ = 0.0;
  double step_size_ = 0.0;
  int steps_remaining_ = 0;
};

} // namespace control_service
