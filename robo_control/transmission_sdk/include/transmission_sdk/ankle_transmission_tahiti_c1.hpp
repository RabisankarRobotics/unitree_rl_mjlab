#pragma once

#include "transmission_sdk/rotation_utils.hpp"
#include "transmission_sdk/transmission_base.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace transmission {

// Geometric config for the Tahiti C1 ankle
struct AnkleTahitiC1Config {
  // Serial chain
  Eigen::Vector3d pitch_joint_pos{0.0, 0.0503273, -0.356224};
  double pitch_joint_quat[4] = {0.5, -0.5, -0.5, 0.5};  // w, x, y, z
  Eigen::Vector3d foot_pos{0.0, 0.025, 0.0};
  double foot_quat[4] = {0.707107, 0.0, 0.707107, 0.0};

  Eigen::Vector3d closure_site_A{-0.02875, -0.01, -0.045};
  Eigen::Vector3d closure_site_B{0.03075, -0.01, -0.045};

  // Motor A four-bar
  Eigen::Vector3d motor_a_pos{0.0, 0.00442358, -0.0698882};
  double motor_a_quat[4] = {0.693464, 0.718103, -0.042156, 0.0407096};
  double motor_a_arm_length = 0.031000;
  double motor_a_arm_z_offset = 0.011000;
  double motor_a_arm_angle_at_zero = -2.993017;
  double motor_a_link_length = 0.293045;

  // Motor B four-bar
  Eigen::Vector3d motor_b_pos{0.0, 0.00556575, -0.159903};
  double motor_b_quat[4] = {0.694631, 0.719311, 0.0064348, -0.00621402};
  double motor_b_arm_length = 0.031000;
  double motor_b_arm_z_offset = 0.011000;
  double motor_b_arm_angle_at_zero = -0.216418;
  double motor_b_link_length = 0.204016;
};

// Analytical transmission for the Tahiti C1 parallel ankle.
//
// Joint space:  q = [pitch, roll]             (serial chain)
// Motor space:  q_m = [motorA, motorB]        (two four-bar driven motors)
class AnkleTransmissionTahitiC1 : public TransmissionBase {
 public:
  using Config = AnkleTahitiC1Config;

  AnkleTransmissionTahitiC1() : AnkleTransmissionTahitiC1(Config{}) {}

  explicit AnkleTransmissionTahitiC1(const Config& cfg) : cfg_(cfg) {
    R1_ = quat_to_rotmat(cfg_.pitch_joint_quat[0], cfg_.pitch_joint_quat[1],
                         cfg_.pitch_joint_quat[2], cfg_.pitch_joint_quat[3]);
    t1_ = cfg_.pitch_joint_pos;
    R2_ = quat_to_rotmat(cfg_.foot_quat[0], cfg_.foot_quat[1],
                         cfg_.foot_quat[2], cfg_.foot_quat[3]);
    t2_ = cfg_.foot_pos;

    R_ma_ = quat_to_rotmat(cfg_.motor_a_quat[0], cfg_.motor_a_quat[1],
                           cfg_.motor_a_quat[2], cfg_.motor_a_quat[3]);
    t_ma_ = cfg_.motor_a_pos;
    arm_at_zero_A_ << cfg_.motor_a_arm_length * std::cos(cfg_.motor_a_arm_angle_at_zero),
                      cfg_.motor_a_arm_length * std::sin(cfg_.motor_a_arm_angle_at_zero),
                      cfg_.motor_a_arm_z_offset;

    R_mb_ = quat_to_rotmat(cfg_.motor_b_quat[0], cfg_.motor_b_quat[1],
                           cfg_.motor_b_quat[2], cfg_.motor_b_quat[3]);
    t_mb_ = cfg_.motor_b_pos;
    arm_at_zero_B_ << cfg_.motor_b_arm_length * std::cos(cfg_.motor_b_arm_angle_at_zero),
                      cfg_.motor_b_arm_length * std::sin(cfg_.motor_b_arm_angle_at_zero),
                      cfg_.motor_b_arm_z_offset;
  }

  std::size_t dof() const override { return 2; }

  void position_forward_into(
      const Eigen::Ref<const Eigen::VectorXd>& q_joint,
      Eigen::Ref<Eigen::VectorXd> out) override {
    check_dof(q_joint, "position_forward");
    check_dof(out, "position_forward");
    Eigen::Vector3d b_A, b_B;
    Eigen::Matrix<double, 3, 2> Js_A, Js_B;
    serial_fk(q_joint(0), q_joint(1), b_A, b_B, Js_A, Js_B);
    out(0) = solve_motor_a(b_A);
    out(1) = solve_motor_b(b_B);
  }

  void velocity_forward_into(
      const Eigen::Ref<const Eigen::VectorXd>& q_joint,
      const Eigen::Ref<const Eigen::VectorXd>& dq_joint,
      Eigen::Ref<Eigen::VectorXd> out) override {
    check_dof(q_joint, "velocity_forward");
    check_dof(dq_joint, "velocity_forward");
    check_dof(out, "velocity_forward");
    Eigen::Vector2d q_motor;
    Eigen::Matrix2d J;
    compute_jacobian(Eigen::Vector2d(q_joint), q_motor, J);
    out.noalias() = J * dq_joint;
  }

  void torque_forward_into(
      const Eigen::Ref<const Eigen::VectorXd>& q_joint,
      const Eigen::Ref<const Eigen::VectorXd>& tau_joint,
      Eigen::Ref<Eigen::VectorXd> out) override {
    check_dof(q_joint, "torque_forward");
    check_dof(tau_joint, "torque_forward");
    check_dof(out, "torque_forward");
    Eigen::Vector2d q_motor;
    Eigen::Matrix2d J;
    compute_jacobian(Eigen::Vector2d(q_joint), q_motor, J);
    // tau_m = J^-T tau_s. Closed-form 2x2 inverse: no heap allocation.
    out.noalias() = J.transpose().inverse() * tau_joint;
  }

  void position_inverse_into(
      const Eigen::Ref<const Eigen::VectorXd>& q_motor,
      const Eigen::Ref<const Eigen::VectorXd>& q_joint_init,
      Eigen::Ref<Eigen::VectorXd> out) override {
    check_dof(q_motor, "position_inverse");
    check_dof(out, "position_inverse");
    Eigen::Vector2d q_s = Eigen::Vector2d::Zero();
    if (q_joint_init.size() == 2) {
      q_s = q_joint_init;
    }

    Eigen::Vector2d q_m_pred;
    Eigen::Matrix2d J;
    const Eigen::Vector2d q_m_target(q_motor);
    for (int i = 0; i < kMaxIter; ++i) {
      compute_jacobian(q_s, q_m_pred, J);
      const Eigen::Vector2d err = q_m_pred - q_m_target;
      if (err.cwiseAbs().maxCoeff() < kTol) {
        out = q_s;
        return;
      }
      q_s.noalias() -= J.inverse() * err;
    }
    std::ostringstream os;
    os << "AnkleTransmissionTahitiC1::position_inverse did not converge after "
       << kMaxIter << " iterations";
    throw std::runtime_error(os.str());
  }

  void velocity_inverse_into(
      const Eigen::Ref<const Eigen::VectorXd>& q_joint,
      const Eigen::Ref<const Eigen::VectorXd>& dq_motor,
      Eigen::Ref<Eigen::VectorXd> out) override {
    check_dof(q_joint, "velocity_inverse");
    check_dof(dq_motor, "velocity_inverse");
    check_dof(out, "velocity_inverse");
    Eigen::Vector2d q_motor;
    Eigen::Matrix2d J;
    compute_jacobian(Eigen::Vector2d(q_joint), q_motor, J);
    out.noalias() = J.inverse() * dq_motor;
  }

  void torque_inverse_into(
      const Eigen::Ref<const Eigen::VectorXd>& q_joint,
      const Eigen::Ref<const Eigen::VectorXd>& tau_motor,
      Eigen::Ref<Eigen::VectorXd> out) override {
    check_dof(q_joint, "torque_inverse");
    check_dof(tau_motor, "torque_inverse");
    check_dof(out, "torque_inverse");
    Eigen::Vector2d q_motor;
    Eigen::Matrix2d J;
    compute_jacobian(Eigen::Vector2d(q_joint), q_motor, J);
    out.noalias() = J.transpose() * tau_motor;
  }

 private:
  static constexpr double kTol = 1e-12;
  static constexpr int kMaxIter = 20;

  Config cfg_;
  Eigen::Matrix3d R1_, R2_, R_ma_, R_mb_;
  Eigen::Vector3d t1_, t2_, t_ma_, t_mb_;
  Eigen::Vector3d arm_at_zero_A_, arm_at_zero_B_;

  template <typename Derived>
  static void check_dof(const Eigen::EigenBase<Derived>& v, const char* where) {
    if (v.size() != 2) {
      throw std::invalid_argument(std::string("AnkleTransmissionTahitiC1::") +
                                  where + ": expected 2-vector");
    }
  }

  void serial_fk(double q_pitch, double q_roll,
                 Eigen::Vector3d& b_A, Eigen::Vector3d& b_B,
                 Eigen::Matrix<double, 3, 2>& Js_A,
                 Eigen::Matrix<double, 3, 2>& Js_B) const {
    const Eigen::Matrix3d Rp = Rz(q_pitch);
    const Eigen::Matrix3d Rr = Rz(q_roll);
    const Eigen::Matrix3d dRp = dRz(q_pitch);
    const Eigen::Matrix3d dRr = dRz(q_roll);

    const Eigen::Matrix3d R1_Rp = R1_ * Rp;
    const Eigen::Matrix3d R1_dRp = R1_ * dRp;
    const Eigen::Matrix3d R2_Rr = R2_ * Rr;
    const Eigen::Matrix3d R2_dRr = R2_ * dRr;

    {
      const Eigen::Vector3d inner = R2_Rr * cfg_.closure_site_A + t2_;
      b_A = R1_Rp * inner + t1_;
      Js_A.col(0) = R1_dRp * inner;
      Js_A.col(1) = R1_Rp * R2_dRr * cfg_.closure_site_A;
    }
    {
      const Eigen::Vector3d inner = R2_Rr * cfg_.closure_site_B + t2_;
      b_B = R1_Rp * inner + t1_;
      Js_B.col(0) = R1_dRp * inner;
      Js_B.col(1) = R1_Rp * R2_dRr * cfg_.closure_site_B;
    }
  }

  double solve_motor_a(const Eigen::Vector3d& b_A) const {
    const double l1 = cfg_.motor_a_arm_length;
    const double l2 = cfg_.motor_a_link_length;
    const double z_arm = cfg_.motor_a_arm_z_offset;
    const double phi0 = cfg_.motor_a_arm_angle_at_zero;

    const Eigen::Vector3d v = R_ma_.transpose() * (b_A - t_ma_);
    const double r_v = std::hypot(v.x(), v.y());
    const double theta_v = std::atan2(v.y(), v.x());
    const double dz = z_arm - v.z();

    const double cos_gamma =
        (l1 * l1 + r_v * r_v + dz * dz - l2 * l2) / (2.0 * l1 * r_v);
    if (std::abs(cos_gamma) > 1.0) {
      std::ostringstream os;
      os << "Motor A outside feasible workspace (cos_gamma=" << cos_gamma << ")";
      throw std::runtime_error(os.str());
    }
    return -std::acos(cos_gamma) - phi0 + theta_v;
  }

  double solve_motor_b(const Eigen::Vector3d& b_B) const {
    const double l1 = cfg_.motor_b_arm_length;
    const double l2 = cfg_.motor_b_link_length;
    const double z_arm = cfg_.motor_b_arm_z_offset;
    const double phi0 = cfg_.motor_b_arm_angle_at_zero;

    const Eigen::Vector3d v = R_mb_.transpose() * (b_B - t_mb_);
    const double r_v = std::hypot(v.x(), v.y());
    const double theta_v = std::atan2(v.y(), v.x());
    const double dz = z_arm - v.z();

    const double cos_delta =
        (l1 * l1 + r_v * r_v + dz * dz - l2 * l2) / (2.0 * l1 * r_v);
    if (std::abs(cos_delta) > 1.0) {
      std::ostringstream os;
      os << "Motor B outside feasible workspace (cos_delta=" << cos_delta << ")";
      throw std::runtime_error(os.str());
    }
    return std::acos(cos_delta) - phi0 + theta_v;
  }

  // Computes motor angles and actuation Jacobian J = dq_m/dq_s at q_joint.
  void compute_jacobian(const Eigen::Vector2d& q_joint,
                        Eigen::Vector2d& q_motor,
                        Eigen::Matrix2d& J) const {
    Eigen::Vector3d b_A, b_B;
    Eigen::Matrix<double, 3, 2> Js_A, Js_B;
    serial_fk(q_joint(0), q_joint(1), b_A, b_B, Js_A, Js_B);

    const double qA = solve_motor_a(b_A);
    const double qB = solve_motor_b(b_B);
    q_motor << qA, qB;

    // Motor A row
    {
      const Eigen::Vector3d v_A = R_ma_.transpose() * (b_A - t_ma_);
      const Eigen::Matrix3d Rz_negA = Rz(-qA);
      const Eigen::Vector3d b_ma = Rz_negA * v_A;
      const Eigen::Vector3d diff_A = arm_at_zero_A_ - b_ma;

      const Eigen::Vector3d db_ma_dqA = -dRz(-qA) * v_A;
      const Eigen::Matrix<double, 3, 2> db_ma_dqs =
          Rz_negA * R_ma_.transpose() * Js_A;

      const double denom = diff_A.dot(db_ma_dqA);
      J.row(0) = -(diff_A.transpose() * db_ma_dqs) / denom;
    }

    // Motor B row
    {
      const Eigen::Vector3d v_B = R_mb_.transpose() * (b_B - t_mb_);
      const Eigen::Matrix3d Rz_negB = Rz(-qB);
      const Eigen::Vector3d b_mb = Rz_negB * v_B;
      const Eigen::Vector3d diff_B = arm_at_zero_B_ - b_mb;

      const Eigen::Vector3d db_mb_dqB = -dRz(-qB) * v_B;
      const Eigen::Matrix<double, 3, 2> db_mb_dqs =
          Rz_negB * R_mb_.transpose() * Js_B;

      const double denom = diff_B.dot(db_mb_dqB);
      J.row(1) = -(diff_B.transpose() * db_mb_dqs) / denom;
    }
  }
};

}  // namespace transmission
