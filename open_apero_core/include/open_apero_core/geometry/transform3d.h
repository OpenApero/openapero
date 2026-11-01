#pragma once

/**
 * @file transform3d.h
 * @brief A rigid 3D transform consisting of a position and orientation.
 */

#include <algorithm>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "open_apero_core/common/typedef.h"

namespace open_apero {

/// @brief Convert the quaternion dot-product epsilon used by isSameOrientation to an angle [rad].
inline double orientationDotEpsilonToAngleThreshold(double orientation_dot_epsilon) {
  const double cos_half_angle = std::max(-1.0, std::min(1.0, 1.0 - orientation_dot_epsilon));
  return 2.0 * std::acos(cos_half_angle);
}

/// @brief Angle of the shortest rotation between two orientations, in [0, pi]. A quaternion and its negation are one
/// orientation.
inline double geodesicAngle(const Eigen::Quaterniond& from, const Eigen::Quaterniond& to) {
  const double angle = std::abs(Eigen::AngleAxisd(from.inverse() * to).angle());
  return std::min(angle, 2.0 * M_PI - angle);
}

/// @brief A rigid 3D transform consisting of a position and orientation.
struct Transform3d {
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;

  /// @brief Transform a point from this frame to the parent frame.
  Eigen::Vector3d operator*(const Eigen::Vector3d& point) const { return position + orientation * point; }

  /// @brief Transform an orientation from this frame to the parent frame.
  Eigen::Quaterniond operator*(const Eigen::Quaterniond& quat) const { return orientation * quat; }

  /// @brief Transform another transform from this frame to the parent frame.
  Transform3d operator*(const Transform3d& other) const {
    return Transform3d{position + orientation * other.position, orientation * other.orientation};
  }

  /// @brief Return the inverse of this transform.
  Transform3d inverse() const {
    const Eigen::Quaterniond q_inv = orientation.inverse();
    return Transform3d{q_inv * (-position), q_inv};
  }

  /// @brief Check if position matches another transform.
  bool isSamePosition(const Transform3d& other, double position_epsilon = kEpsilon) const {
    return (position - other.position).squaredNorm() <= position_epsilon * position_epsilon;
  }

  /// @brief Check if orientation matches another transform (handles q == -q equivalence).
  bool isSameOrientation(const Transform3d& other, double orientation_dot_epsilon = kEpsilon) const {
    return std::abs(orientation.dot(other.orientation)) > 1.0 - orientation_dot_epsilon;
  }

  /// @brief Check if both position and orientation match another transform.
  bool isSamePose(const Transform3d& other, double position_epsilon = kEpsilon, double orientation_dot_epsilon = kEpsilon) const {
    return isSamePosition(other, position_epsilon) && isSameOrientation(other, orientation_dot_epsilon);
  }

  /// @brief Check if the target pose has been reached from this transform within thresholds.
  /// A threshold near zero disables that check dimension.
  bool isPoseReached(const Transform3d& target, double pos_threshold, double angle_threshold) const {
    if (pos_threshold >= kEpsilon && (position - target.position).squaredNorm() >= pos_threshold * pos_threshold) {
      return false;
    }
    if (angle_threshold < kEpsilon) {
      return true;
    }
    double angle = Eigen::AngleAxisd(orientation.inverse() * target.orientation).angle();
    return std::abs(angle) < angle_threshold;
  }
};

}  // namespace open_apero
