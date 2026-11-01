#include "open_apero_core/common/utils.h"

#include <cmath>

namespace open_apero {

bool isUnitQuaternion(const Eigen::Quaterniond& q, double tolerance) {
  const bool finite = std::isfinite(q.w()) && std::isfinite(q.x()) && std::isfinite(q.y()) && std::isfinite(q.z());
  return finite && std::abs(q.norm() - 1.0) <= tolerance;
}

double yawFromQuaternion(const Eigen::Quaterniond& q) {
  return atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

Eigen::Quaterniond quaternionFromYaw(double yaw) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
}

Eigen::Quaterniond quaternionFromEulerAnglesZyx(const Eigen::Vector3d& zyx) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(zyx(0), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(zyx(1), Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(zyx(2), Eigen::Vector3d::UnitX()));
}

// Eigen's eulerAngles(2, 1, 0) without its fold of z into [0, pi], which turns a negative yaw into the equivalent
// (z + pi, +-pi - y, x +- pi) with pitch outside [-pi/2, pi/2].
Eigen::Vector3d eulerAnglesZyxFromQuaternion(const Eigen::Quaterniond& q) {
  const Eigen::Matrix3d r = q.toRotationMatrix();
  const double cos_pitch = std::hypot(r(2, 2), r(2, 1));

  // Gimbal lock: yaw and roll turn about the same axis and only their combination is defined; report it as yaw with
  // roll 0. That moves the rotation by at most cos_pitch, so the threshold bounds the error while still catching exact
  // lock, where cos_pitch is rounding noise (~1e-16).
  if (cos_pitch < 1e-12) {
    return {std::atan2(-r(0, 1), r(1, 1)), std::copysign(M_PI / 2.0, -r(2, 0)), 0.0};
  }

  // Roll from the yaw-derotated frame, whose entries stay well conditioned near gimbal lock: there it absorbs the error
  // of yaw, so the angles still recompose q to rounding.
  const double yaw = yawFromQuaternion(q);  // atan2(r(1, 0), r(0, 0))
  const double sin_yaw = std::sin(yaw);
  const double cos_yaw = std::cos(yaw);
  const double roll = std::atan2(sin_yaw * r(0, 2) - cos_yaw * r(1, 2), cos_yaw * r(1, 1) - sin_yaw * r(0, 1));
  return {yaw, std::atan2(-r(2, 0), cos_pitch), roll};
}

Eigen::Vector2d yawPitchFromDirection(const Eigen::Vector3d& direction) {
  return {std::atan2(direction.y(), direction.x()), std::atan2(-direction.z(), direction.head<2>().norm())};
}

Eigen::Vector2d yawPitchFromQuaternion(const Eigen::Quaterniond& q) {
  return yawPitchFromDirection(q * Eigen::Vector3d::UnitX());
}

bool isSamePosition(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
  return (a - b).squaredNorm() <= kEpsilonSquared;
}

bool isSameOrientation(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b) {
  return std::abs(a.dot(b)) > 1.0 - kEpsilon;
}

bool boxesOverlap(const Eigen::Vector3d& min_a, const Eigen::Vector3d& max_a, const Eigen::Vector3d& min_b, const Eigen::Vector3d& max_b) {
  if (min_a.hasNaN() || max_a.hasNaN() || min_b.hasNaN() || max_b.hasNaN()) {
    return false;
  }
  // AABBs overlap only when their intervals overlap on every axis.
  return (min_a.array() < max_b.array()).all() && (min_b.array() < max_a.array()).all();
}

double angleScaled(double angle) {
  angle = std::fmod(angle, 2.0 * M_PI);
  return angle + 2.0 * M_PI * (angle < 0);
}

double angleDifference(double angle1, double angle2) {
  double angle = std::fmod(angle2 - angle1, 2.0 * M_PI);
  if (angle < -M_PI) {
    angle += 2.0 * M_PI;
  } else if (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  return angle;
}

double rateStep(double remaining, double max_step, double min_last_step) {
  if (std::abs(remaining) <= max_step * (1.0 + kEpsilon)) {
    return remaining;
  }
  if (std::abs(remaining) - max_step < min_last_step) {
    return remaining / 2.0;
  }
  return std::copysign(max_step, remaining);
}

}  // namespace open_apero
