#pragma once

/**
 * @file geometric_sphere.h
 * @brief Geometric sphere primitive used for collision checking.
 */

#include <limits>
#include <vector>

#include <Eigen/Core>

namespace open_apero {

/**
 * @brief A sphere defined by center, radius, and an optional safety margin.
 *
 * Used for collision checking. Both center and radius must be explicitly set;
 * the radius defaults to NaN to indicate it has not been initialized.
 */
struct GeometricSphere {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// @brief Center position of the sphere. Must be provided.
  Eigen::Vector3d center;

  /// @brief Radius of the sphere. Defaults to NaN to indicate uninitialized state.
  double radius = std::numeric_limits<double>::quiet_NaN();

  /// @brief Additional safety margin around the sphere. Defaults to 0.
  double safe_margin = 0.0;
};

/// @brief A collection of geometric spheres.
using GeometricSpheres = std::vector<GeometricSphere>;

}  // namespace open_apero
