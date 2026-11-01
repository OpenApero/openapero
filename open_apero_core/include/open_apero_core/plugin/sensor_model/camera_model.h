#pragma once

/**
 * @file camera_model.h
 * @brief Base camera sensor model for computing visible voxels and field-of-view boundaries.
 */

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
 * @brief Camera model for visibility and view-boundary queries.
 */
class CameraModel : public SensorModel {
 public:
  /** @brief Constructs a CameraModel and binds it to the agent. */
  explicit CameraModel(AgentInterface& agent);  // NOLINT(google-runtime-references) non-const ref to agent is intentional

  virtual ~CameraModel() = default;

  /** @brief Visualizes the camera frustum boundaries at the node's sensor pose. */
  void visualizeSensorView(VisualizationMarkers* markers, const Node* node) override;

  /**
   * @brief Loads camera parameters (focal length, resolution, observation boundary) from the config map.
   * @throws std::runtime_error if observation_boundary is neither "spherical" nor "planar".
   */
  void setupFromParamMap(Configurable::ParamMap* param_map) override;

  /**
   * @brief Validates that focal length and resolution parameters are positive.
   * @param error_message Populated with a description of the first validation failure.
   * @return true if all parameters are valid.
   */
  bool checkParamsValid(std::string* error_message) override;

  /**
   * @brief Returns the axis-aligned bounding box of the camera's view, every ray out to the observation boundary; exact for
   *        any orientation.
   * @param min Output minimum corner.
   * @param max Output maximum corner.
   * @param sensor_transform Sensor transform in world frame.
   */
  void getSensorViewBoundary(Eigen::Vector3d& min, Eigen::Vector3d& max, const Transform3d& sensor_transform) override;

  /**
   * @brief Whether a world point falls within the observation boundary and inside the frustum.
   *
   * Pinhole test against the same range and field of view the ray caster uses,
   * grown by @p margin in every direction, so it never rejects a point the ray
   * caster would have reported.
   */
  bool pointInSensorView(const Eigen::Vector3d& point, const Transform3d& sensor_transform, double margin = 0.0) const override;

 protected:
  /** @brief Surface that ends the view; the names and spelling are Voxblox's `observation_boundary`. */
  enum class ObservationBoundary {
    kSpherical,  ///< The sphere of radius ray_length around the sensor (default).
    kPlanar,     ///< The plane at camera depth ray_length, as a depth camera's max range.
  };

  // parameters
  double p_focal_length_ = 320.0;
  int p_resolution_x_ = 640;
  int p_resolution_y_ = 480;
  ObservationBoundary p_observation_boundary_ = ObservationBoundary::kSpherical;

  // constants, derived in setupFromParamMap
  double c_field_of_view_x_ = 0.0;
  double c_field_of_view_y_ = 0.0;
  double c_tan_half_field_of_view_x_ = 0.0;
  double c_tan_half_field_of_view_y_ = 0.0;
  /// Converts a distance measured perpendicular to a frustum side plane into the
  /// lateral offset it allows at a given depth. See pointInSensorView.
  double c_sec_half_field_of_view_x_ = 1.0;
  double c_sec_half_field_of_view_y_ = 1.0;

  /**
   * @brief Computes the ray direction for a pixel at relative image coordinates.
   * @param result Output direction vector (normalized).
   * @param relative_x Relative x position in [0, 1].
   * @param relative_y Relative y position in [0, 1].
   */
  void getDirectionVector(Eigen::Vector3d* result, double relative_x, double relative_y);

  /**
   * @brief Distance along a ray at which the view ends.
   * @param camera_direction Unit ray direction in the camera frame, as getDirectionVector returns it.
   * @return ray_length, or for a planar boundary the distance at which the ray reaches camera depth ray_length.
   */
  double getRayLength(const Eigen::Vector3d& camera_direction) const;

  /**
   * @brief Extent of the observation boundary along one image axis: the arc ray_length * field_of_view on the sphere,
   *        or the far face's ray_length * resolution / focal_length on the plane.
   * @param field_of_view The axis's field of view [rad].
   * @param resolution The axis's resolution [px].
   */
  double getObservationBoundaryWidth(double field_of_view, int resolution) const;
};

}  // namespace sensor_model
}  // namespace open_apero
