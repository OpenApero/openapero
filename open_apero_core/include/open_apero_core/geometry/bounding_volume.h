#pragma once

/**
 * @file bounding_volume.h
 * @brief Axis-aligned bounding volume with optional Z-axis rotation.
 *
 * Provides a simple box-shaped bounding volume used to constrain the
 * exploration space. Supports containment queries and visualization.
 */

#include <functional>
#include <string>
#include <utility>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "open_apero_core/plugin/plugin.h"
#include "open_apero_core/visualization/visualization_markers.h"

namespace open_apero {

/**
 * @brief Axis-aligned bounding box with optional Z-rotation for constraining the planning space.
 *
 * Not a factory-created plugin: owners (Evaluator, SimpleRobot) construct it directly and
 * configure it via PluginFactory::configure(). Diagnostics go to an optional callback.
 */
struct BoundingVolume : public Configurable {
  /// @brief Callback used to surface human-readable setup diagnostics.
  using InfoLogger = std::function<void(const std::string&)>;

  BoundingVolume() = default;

  ~BoundingVolume() override = default;

  /**
   * @brief Install a callback for diagnostic messages (e.g. forwarding to the
   *        owning agent's printInfo). Optional; unset means messages are dropped.
   */
  void setInfoLogger(InfoLogger info_logger) { info_logger_ = std::move(info_logger); }

  /**
   * @brief Populate the bounding volume parameters from a parameter map.
   * @param param_map Pointer to the parameter map containing volume extents and rotation.
   */
  void setupFromParamMap(Configurable::ParamMap* param_map) override;

  /**
   * @brief Check whether a point lies inside the bounding volume.
   * @param point The 3D point to test for containment.
   * @return True if the bounding volume is set up and the point is inside it.
   */
  bool contains(const Eigen::Vector3d& point);

  /**
   * @brief Generate an orange world-frame wireframe for the bounding volume.
   * @param markers Pointer to the marker collection to populate.
   */
  void visualize(VisualizationMarkers* markers);

  /// @brief Whether the bounding volume has been initialized.
  bool is_setup = false;

  /// @brief Axis-aligned extents in meters and rotation around Z axis in degrees.
  double x_min, x_max, y_min, y_max, z_min, z_max, rotation;

  /// @brief Quaternion representation of the Z-axis rotation.
  Eigen::Quaterniond rotation_quat;

 private:
  InfoLogger info_logger_;
};

}  // namespace open_apero
