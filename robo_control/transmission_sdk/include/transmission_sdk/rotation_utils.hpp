#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace transmission {

inline Eigen::Matrix3d quat_to_rotmat(double w, double x, double y, double z) {
  Eigen::Matrix3d R;
  R << 1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y),
       2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x),
       2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y);
  return R;
}

inline Eigen::Matrix3d Rz(double theta) {
  const double c = std::cos(theta);
  const double s = std::sin(theta);
  Eigen::Matrix3d R;
  R << c, -s, 0.0,
       s,  c, 0.0,
       0.0, 0.0, 1.0;
  return R;
}

inline Eigen::Matrix3d dRz(double theta) {
  const double c = std::cos(theta);
  const double s = std::sin(theta);
  Eigen::Matrix3d D;
  D << -s, -c, 0.0,
        c, -s, 0.0,
        0.0, 0.0, 0.0;
  return D;
}

}  // namespace transmission
