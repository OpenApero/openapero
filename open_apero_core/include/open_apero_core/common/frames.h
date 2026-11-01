#pragma once

/**
 * @file frames.h
 * @brief Shared frame-convention constants and conversions.
 */

#include <Eigen/Geometry>

#include "open_apero_core/geometry/transform3d.h"

namespace open_apero {

/**
 * @brief Fixed rotation from the sensor frame to its optical frame (+z along the optical axis).
 *
 * The single definition: the settle gate and every map reporting optical frames must use it,
 * or their transforms will never compare equal.
 */
inline const Eigen::Quaterniond& sensorToOpticalOrientation() {
  static const Eigen::Quaterniond kSensorToOptical(0.5, -0.5, 0.5, -0.5);
  return kSensorToOptical;
}

/**
 * @brief Convert a sensor-frame transform to the corresponding optical-frame
 *        transform (same origin, orientation rotated into the optical frame).
 */
inline Transform3d sensorToOpticalFrame(const Transform3d& sensor) {
  return Transform3d{sensor.position, sensor.orientation * sensorToOpticalOrientation()};
}

/**
 * @brief Inverse of sensorToOpticalFrame: bring a reported optical frame back to
 *        the sensor frame the planner reasons in.
 *
 * The origin is shared; only the absolute orientation differs between the two frames.
 */
inline Transform3d opticalToSensorFrame(const Transform3d& optical) {
  return Transform3d{optical.position, optical.orientation * sensorToOpticalOrientation().inverse()};
}

}  // namespace open_apero
