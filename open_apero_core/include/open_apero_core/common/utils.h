#pragma once

/**
 * @file utils.h
 * @brief Utility functions for angle conversions and quaternion operations.
 *
 * Provides helper functions for converting between quaternions and Euler angles, and angle arithmetic.
 */

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "open_apero_core/common/typedef.h"

namespace open_apero {

/**
 * @brief Whether a quaternion is a valid rotation: finite and unit-norm.
 *
 * Reject failing values at the boundary rather than normalizing them. The tolerance covers
 * representation rounding only (float32 payloads deviate by about 5e-8).
 *
 * @param q Quaternion to check.
 * @param tolerance Permitted |norm - 1|.
 * @return True if @p q can be used as an orientation as-is.
 */
bool isUnitQuaternion(const Eigen::Quaterniond& q, double tolerance = 1e-7);

/**
 * @brief Extract the yaw angle from a quaternion.
 * @param q Input quaternion.
 * @return Yaw angle in radians.
 */
double yawFromQuaternion(const Eigen::Quaterniond& q);

/**
 * @brief Create a quaternion representing a pure yaw rotation.
 * @param yaw Yaw angle in radians.
 * @return Quaternion representing the yaw rotation.
 */
Eigen::Quaterniond quaternionFromYaw(double yaw);

/**
 * @brief Create a quaternion from Euler angles in ZYX order.
 * @param zyx Euler angles as (z, y, x) vector in radians.
 * @return The corresponding quaternion.
 */
Eigen::Quaterniond quaternionFromEulerAnglesZyx(const Eigen::Vector3d& zyx);

/**
 * @brief Extract Euler angles in ZYX order from a unit quaternion.
 *
 * Returns z and x in [-pi, pi], y in [-pi/2, pi/2]. Away from gimbal lock z equals yawFromQuaternion(q). At gimbal
 * lock (cos(y) < 1e-12, reported as |y| = pi/2) only z - x (y = pi/2) or z + x (y = -pi/2) is defined: x is reported
 * as 0 and z carries that combination. The angles recompose @p q through quaternionFromEulerAnglesZyx to rounding.
 *
 * @param q Input quaternion; must be unit (see isUnitQuaternion).
 * @return Euler angles as (z, y, x) vector in radians.
 */
Eigen::Vector3d eulerAnglesZyxFromQuaternion(const Eigen::Quaterniond& q);

/**
 * @brief ZYX yaw and pitch of the roll-free frame looking along a direction.
 *
 * Pitch is positive-down, hence the negated z: a direction pointing upwards yields a
 * negative pitch. The direction need not be normalized.
 *
 * @param direction Direction in the reference frame; need not be normalized.
 * @return Angles as (yaw, pitch) vector in radians.
 */
Eigen::Vector2d yawPitchFromDirection(const Eigen::Vector3d& direction);

/**
 * @brief Extract the yaw and pitch of a quaternion from its forward (+x) axis.
 *
 * Roll-invariant: only the forward axis enters. Equals the ZYX yaw and pitch whenever
 * |pitch| < pi/2.
 *
 * @param q Input quaternion.
 * @return Angles as (yaw, pitch) vector in radians.
 */
Eigen::Vector2d yawPitchFromQuaternion(const Eigen::Quaterniond& q);

/**
 * @brief Wrap an angle to the range [0, 2*pi).
 * @param angle Input angle in radians.
 * @return The wrapped angle in [0, 2*pi).
 */
double angleScaled(double angle);

/**
 * @brief Compute the minimum signed angular difference to turn angle1 into angle2.
 * @param angle1 Starting angle in radians.
 * @param angle2 Target angle in radians.
 * @return The signed angular difference in radians (can be positive or negative).
 */
double angleDifference(double angle1, double angle2);

/**
 * @brief The next step toward @p remaining: all of it once it fits in @p max_step (relative kEpsilon slack), half of it
 * when a full step would leave a last step below @p min_last_step, else a full @p max_step.
 * Precondition: max_step > 2 * min_last_step.
 */
double rateStep(double remaining, double max_step, double min_last_step);

/**
 * @brief Check whether two 3D positions are approximately equal.
 * @param a First position.
 * @param b Second position.
 * @return True if the squared distance is within kEpsilonSquared.
 */
bool isSamePosition(const Eigen::Vector3d& a, const Eigen::Vector3d& b);

/**
 * @brief Check whether two quaternion orientations are approximately equal.
 *
 * Uses the quaternion dot product to handle q == -q equivalence.
 * @param a First orientation.
 * @param b Second orientation.
 * @return True if the orientations are within kEpsilon of each other.
 */
bool isSameOrientation(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b);

/**
 * @brief Whether two axis-aligned boxes overlap; touching faces do not count.
 *
 * False when either box has a NaN corner.
 */
bool boxesOverlap(const Eigen::Vector3d& min_a, const Eigen::Vector3d& max_a, const Eigen::Vector3d& min_b, const Eigen::Vector3d& max_b);

}  // namespace open_apero
