#pragma once

/**
 * @file lidar_model.h
 * @brief Base lidar sensor model for computing visible voxels and field-of-view boundaries.
 */

#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "open_apero_core/geometry/transform3d.h"
#include "open_apero_core/graph/graph_node.h"
#include "open_apero_core/graph/tree_node.h"
#include "open_apero_core/plugin/sensor_model/sensor_model.h"
#include "open_apero_core/visualization/visualization_markers.h"

namespace open_apero {
namespace sensor_model {

/**
 * @brief Lidar model for visibility and view-boundary queries.
 */
class LidarModel : public SensorModel {
 public:
  /** @brief Constructs a LidarModel and binds it to the agent. */
  explicit LidarModel(AgentInterface& agent);  // NOLINT(google-runtime-references) non-const ref to agent is intentional

  virtual ~LidarModel() = default;

  /** @brief Visualizes the lidar scan volume boundaries at the node's sensor pose. */
  void visualizeSensorView(VisualizationMarkers* markers, const Node* node) override;

  /** @brief Loads lidar parameters (FOV, resolution) from the config map. */
  void setupFromParamMap(Configurable::ParamMap* param_map) override;

  /**
   * @brief Validates that FOV and resolution parameters are positive.
   * @param error_message Populated with a description of the first validation failure.
   * @return true if all parameters are valid.
   */
  bool checkParamsValid(std::string* error_message) override;

  /**
   * @brief Returns the axis-aligned bounding box of the lidar scan volume; a scan of at least half a
   *        turn is bounded as from a level sensor (ray_length horizontally around it).
   * @param min Output minimum corner.
   * @param max Output maximum corner.
   * @param sensor_transform Sensor transform in world frame.
   */
  void getSensorViewBoundary(Eigen::Vector3d& min, Eigen::Vector3d& max, const Transform3d& sensor_transform) override;

 protected:
  double p_field_of_view_x_ = 2.0 * M_PI;  ///< Total field of view [rad], symmetric about the facing direction; configured in degrees.
  double p_field_of_view_y_ = 30.0 * M_PI / 180.0;  ///< Vertical field of view [rad]; configured in degrees.
  int p_resolution_x_ = 1024;
  int p_resolution_y_ = 128;

  /**
   * @brief Computes the ray direction for a scan point at relative angular coordinates.
   * @param result Output direction vector (normalized).
   * @param relative_x Relative horizontal position in [0, 1].
   * @param relative_y Relative vertical position in [0, 1].
   */
  void getDirectionVector(Eigen::Vector3d* result, double relative_x, double relative_y);
};

}  // namespace sensor_model
}  // namespace open_apero
