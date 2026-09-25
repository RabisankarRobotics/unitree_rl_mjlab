#pragma once

#include <Eigen/Dense>
#include <cstddef>

namespace transmission {

// Transmission interface.
//
// Two call styles:
//   *_into(...)  — writes into a caller-provided output buffer. No allocation.
//                  Pre-size `out` to dof() and reuse across calls on hot paths.
//   value-returning overloads  — allocate a VectorXd and delegate to *_into.
//                  Use for tests and scripts; avoid on the 1 kHz control loop.
class TransmissionBase {
 public:
  virtual ~TransmissionBase() = default;

  virtual std::size_t dof() const = 0;

  // Hot-path, no-alloc virtuals. `out` must be size dof().
  virtual void position_forward_into(
      const Eigen::Ref<const Eigen::VectorXd>& q_joint,
      Eigen::Ref<Eigen::VectorXd> out) = 0;
  virtual void velocity_forward_into(
      const Eigen::Ref<const Eigen::VectorXd>& q_joint,
      const Eigen::Ref<const Eigen::VectorXd>& dq_joint,
      Eigen::Ref<Eigen::VectorXd> out) = 0;
  virtual void torque_forward_into(
      const Eigen::Ref<const Eigen::VectorXd>& q_joint,
      const Eigen::Ref<const Eigen::VectorXd>& tau_joint,
      Eigen::Ref<Eigen::VectorXd> out) = 0;

  virtual void position_inverse_into(
      const Eigen::Ref<const Eigen::VectorXd>& q_motor,
      const Eigen::Ref<const Eigen::VectorXd>& q_joint_init,
      Eigen::Ref<Eigen::VectorXd> out) = 0;
  virtual void velocity_inverse_into(
      const Eigen::Ref<const Eigen::VectorXd>& q_joint,
      const Eigen::Ref<const Eigen::VectorXd>& dq_motor,
      Eigen::Ref<Eigen::VectorXd> out) = 0;
  virtual void torque_inverse_into(
      const Eigen::Ref<const Eigen::VectorXd>& q_joint,
      const Eigen::Ref<const Eigen::VectorXd>& tau_motor,
      Eigen::Ref<Eigen::VectorXd> out) = 0;

  // Ergonomic value-returning overloads; allocate and delegate.
  Eigen::VectorXd position_forward(const Eigen::VectorXd& q_joint) {
    Eigen::VectorXd out(dof());
    position_forward_into(q_joint, out);
    return out;
  }
  Eigen::VectorXd velocity_forward(const Eigen::VectorXd& q_joint,
                                   const Eigen::VectorXd& dq_joint) {
    Eigen::VectorXd out(dof());
    velocity_forward_into(q_joint, dq_joint, out);
    return out;
  }
  Eigen::VectorXd torque_forward(const Eigen::VectorXd& q_joint,
                                 const Eigen::VectorXd& tau_joint) {
    Eigen::VectorXd out(dof());
    torque_forward_into(q_joint, tau_joint, out);
    return out;
  }
  Eigen::VectorXd position_inverse(const Eigen::VectorXd& q_motor,
                                   const Eigen::VectorXd& q_joint_init) {
    Eigen::VectorXd out(dof());
    position_inverse_into(q_motor, q_joint_init, out);
    return out;
  }
  Eigen::VectorXd velocity_inverse(const Eigen::VectorXd& q_joint,
                                   const Eigen::VectorXd& dq_motor) {
    Eigen::VectorXd out(dof());
    velocity_inverse_into(q_joint, dq_motor, out);
    return out;
  }
  Eigen::VectorXd torque_inverse(const Eigen::VectorXd& q_joint,
                                 const Eigen::VectorXd& tau_motor) {
    Eigen::VectorXd out(dof());
    torque_inverse_into(q_joint, tau_motor, out);
    return out;
  }
};

}  // namespace transmission
