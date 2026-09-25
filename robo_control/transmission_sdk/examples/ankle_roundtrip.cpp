#include "transmission_sdk/ankle_transmission_tahiti_c1.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <cstdio>
#include <stdexcept>

// Verifies the analytical ankle transmission:
//   1. position round-trip: inverse(forward(q)) == q
//   2. Jacobian matches numerical finite-difference of position_forward
//   3. velocity/torque duality: velocity_inverse(forward) and torque pair round-trip
//
// Exits non-zero on any failure.

namespace {

using transmission::AnkleTransmissionTahitiC1;

int check(bool cond, const char* msg) {
  if (!cond) {
    std::fprintf(stderr, "FAIL: %s\n", msg);
    return 1;
  }
  return 0;
}

Eigen::Matrix2d finite_diff_jacobian(AnkleTransmissionTahitiC1& tx,
                                     const Eigen::VectorXd& q,
                                     double eps = 1e-6) {
  Eigen::Matrix2d J;
  for (int i = 0; i < 2; ++i) {
    Eigen::VectorXd qp = q;
    Eigen::VectorXd qm = q;
    qp(i) += eps;
    qm(i) -= eps;
    Eigen::VectorXd fp = tx.position_forward(qp);
    Eigen::VectorXd fm = tx.position_forward(qm);
    J.col(i) = (fp - fm) / (2.0 * eps);
  }
  return J;
}

}  // namespace

int main() {
  AnkleTransmissionTahitiC1 tx;

  int failures = 0;
  double max_pos_err = 0.0;
  double max_jac_err = 0.0;
  double max_vel_err = 0.0;
  double max_trq_err = 0.0;

  // Range kept inside the mechanism's feasible workspace — corners of a larger
  // square are physically unreachable (the arcsin in the four-bar goes
  // imaginary). 0.22 rad per axis leaves a comfortable margin.
  constexpr int N = 9;
  constexpr double kRange = 0.22;  // rad
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < N; ++j) {
      const double pitch = -kRange + 2.0 * kRange * i / (N - 1);
      const double roll = -kRange + 2.0 * kRange * j / (N - 1);
      Eigen::VectorXd q(2);
      q << pitch, roll;

      Eigen::VectorXd q_m;
      try {
        q_m = tx.position_forward(q);
      } catch (const std::exception& e) {
        std::fprintf(stderr, "position_forward threw at (%.3f,%.3f): %s\n",
                     pitch, roll, e.what());
        ++failures;
        continue;
      }

      Eigen::VectorXd q_init = Eigen::VectorXd::Zero(2);
      Eigen::VectorXd q_back;
      try {
        q_back = tx.position_inverse(q_m, q_init);
      } catch (const std::exception& e) {
        std::fprintf(stderr, "position_inverse threw at (%.3f,%.3f): %s\n",
                     pitch, roll, e.what());
        ++failures;
        continue;
      }
      max_pos_err = std::max(max_pos_err, (q_back - q).cwiseAbs().maxCoeff());

      // Jacobian check via finite differences of position_forward.
      // J_analytic compared via velocity_forward applied to unit vectors.
      Eigen::VectorXd e0(2), e1(2);
      e0 << 1.0, 0.0;
      e1 << 0.0, 1.0;
      Eigen::Matrix2d J_analytic;
      J_analytic.col(0) = tx.velocity_forward(q, e0);
      J_analytic.col(1) = tx.velocity_forward(q, e1);
      Eigen::Matrix2d J_fd = finite_diff_jacobian(tx, q);
      max_jac_err =
          std::max(max_jac_err, (J_analytic - J_fd).cwiseAbs().maxCoeff());

      // Velocity duality: dq_m = J dq_s ; velocity_inverse(J dq_s) == dq_s.
      Eigen::VectorXd dq_s(2);
      dq_s << 0.7, -0.4;
      Eigen::VectorXd dq_m = tx.velocity_forward(q, dq_s);
      Eigen::VectorXd dq_s_back = tx.velocity_inverse(q, dq_m);
      max_vel_err =
          std::max(max_vel_err, (dq_s_back - dq_s).cwiseAbs().maxCoeff());

      // Torque duality: tau_s = J^T tau_m ; torque_forward should invert that.
      Eigen::VectorXd tau_m(2);
      tau_m << 1.3, -2.1;
      Eigen::VectorXd tau_s = tx.torque_inverse(q, tau_m);
      Eigen::VectorXd tau_m_back = tx.torque_forward(q, tau_s);
      max_trq_err =
          std::max(max_trq_err, (tau_m_back - tau_m).cwiseAbs().maxCoeff());
    }
  }

  std::printf("ankle roundtrip results over %dx%d grid in [-%.2f, %.2f]:\n",
              N, N, kRange, kRange);
  std::printf("  max position round-trip error : %.3e\n", max_pos_err);
  std::printf("  max |J_analytic - J_fd|       : %.3e\n", max_jac_err);
  std::printf("  max velocity round-trip error : %.3e\n", max_vel_err);
  std::printf("  max torque   round-trip error : %.3e\n", max_trq_err);

  failures += check(max_pos_err < 1e-9, "position round-trip > 1e-9");
  failures += check(max_jac_err < 1e-5, "jacobian vs finite-diff > 1e-5");
  failures += check(max_vel_err < 1e-10, "velocity round-trip > 1e-10");
  failures += check(max_trq_err < 1e-10, "torque round-trip > 1e-10");

  // Cross-check against Python reference values from ankle_transmission.py.
  struct Ref {
    double pitch, roll, qA, qB;
  };
  const Ref refs[] = {
      {+0.000, +0.000, -0.000022665, +0.000008224},
      {+0.100, +0.000, -0.151518269, +0.151255199},
      {+0.000, +0.100, -0.092786602, -0.105278595},
      {+0.100, +0.100, -0.241495494, +0.050634230},
      {-0.100, +0.100, +0.056396954, -0.265933545},
      {+0.150, -0.150, -0.086689426, +0.368970591},
  };
  double max_ref_err = 0.0;
  for (const auto& r : refs) {
    Eigen::VectorXd q(2);
    q << r.pitch, r.roll;
    Eigen::VectorXd qm = tx.position_forward(q);
    max_ref_err = std::max(max_ref_err, std::abs(qm(0) - r.qA));
    max_ref_err = std::max(max_ref_err, std::abs(qm(1) - r.qB));
  }
  std::printf("  max error vs python reference : %.3e\n", max_ref_err);
  failures += check(max_ref_err < 1e-8, "C++ vs python reference > 1e-8");

  if (failures) {
    std::fprintf(stderr, "%d checks failed\n", failures);
    return 1;
  }
  std::printf("all checks passed\n");
  return 0;
}
