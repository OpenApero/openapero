#pragma once

/**
 * @file simple_robot.h
 * @brief Simple robot model using geometric collision and contact spheres.
 */

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "open_apero_core/agent/agent_interface.h"
#include "open_apero_core/common/seeding.h"
#include "open_apero_core/common/typedef.h"
#include "open_apero_core/common/utils.h"
#include "open_apero_core/geometry/bounding_volume.h"
#include "open_apero_core/geometry/geometric_sphere.h"
#include "open_apero_core/geometry/transform3d.h"
#include "open_apero_core/plugin/map/tsdf_map.h"
#include "open_apero_core/plugin/plugin_registry.h"
#include "open_apero_core/plugin/robot_model/robot_model.h"
#include "open_apero_core/state/robot_state.h"
#include "open_apero_core/visualization/visualization_markers.h"

namespace open_apero {
namespace robot_model {

/**
 * @brief Robot model defined by geometric collision and contact spheres for traversability checking.
 */
class SimpleRobot : public RobotModel {
 public:
  /** @brief Constructs SimpleRobot and binds it to the agent. */
  explicit SimpleRobot(AgentInterface& agent);
  virtual ~SimpleRobot() = default;

  /** @brief Loads collision geometry, bounds, and traversability settings. */
  void setupFromParamMap(Configurable::ParamMap* param_map) override;

  /** @brief Validates that all required parameters are set and consistent. */
  bool checkParamsValid(std::string* error_message) override;

  /** @brief Reseeds the sampling RNG and clears the mapped region for a new episode. */
  void resetEpisode() override;

  /** @brief Refreshes the cached mapped region that restricts where samples are drawn. */
  void update() override;

  /**
   * @brief Transforms collision or contact spheres into the world frame for a given robot state.
   * @param robot_state        The current robot pose.
   * @param spheres            Output list of spheres in world coordinates.
   * @param use_contact_spheres If true, returns contact spheres; otherwise returns collision spheres.
   * @return true on success.
   */
  virtual bool getGeometricSpheresWorldFrame(const RobotState* robot_state, GeometricSpheres& spheres, bool use_contact_spheres) const;

  /** @brief Returns collision spheres in the world frame. */
  bool getCollisionSpheresWorldFrame(const RobotState* robot_state, GeometricSpheres& collision_spheres) const {
    return getGeometricSpheresWorldFrame(robot_state, collision_spheres, false);
  }

  /** @brief Returns contact spheres in the world frame. */
  bool getContactSpheresWorldFrame(const RobotState* robot_state, GeometricSpheres& contact_spheres) const {
    return getGeometricSpheresWorldFrame(robot_state, contact_spheres, true);
  }

  /**
   * @brief Box over every collision and contact sphere as placed at @p state, each grown by its safe margin.
   *
   * False when a placement fails or the robot has no spheres.
   */
  bool getBodyBounds(const RobotState& state, Eigen::Vector3d* min, Eigen::Vector3d* max) const override;

  bool samplesHeight() const override { return p_sample_z_; }

  /**
   * @brief Samples a position within the bounding volume, optionally narrowed to the mapped region.
   *
   * With restrict_sampling_to_mapped_region every sampled axis is confined to the intersection
   * of the bounding volume and the mapped region; z is sampled only when sample_z is set.
   */
  bool samplePosition(Eigen::Vector3d* pos, double z_default) const override;

  bool samplePositionNear(const Eigen::Vector3d& center, double max_distance_3d, double max_distance_xy, Eigen::Vector3d* pos,
                          double z_default) const override;

  /**
   * @brief Plans a feasible trajectory from start to goal via a straight-line steer.
   *
   * Returns false if the steer produces no motion or is untraversable. Subclasses
   * may override to add a fallback strategy (see SimpleFallbackRobot).
   */
  bool planTrajectory(const RobotState& start, const RobotState& goal, Map* map, RobotTrajectory* trajectory) override;

  /**
   * @brief Smallest relative margin any collision sphere keeps (see getSphereClearance).
   *
   * Contact spheres are excluded. 1.0 when no map, no distance field or no sphere constrains it.
   */
  double getStateClearance(const RobotState& state, Map* map) const override;

  Traversability placeRobotAtSensorPose(RobotState* robot_state, const Transform3d& sensor_transform, Map* map = nullptr) const override;

  /**
   * @brief Steers from start toward goal using position and yaw stepping.
   *
   * The robot rotates in-place to face the travel direction, translates at v_max,
   * then rotates to match the goal orientation.
   *
   * Precondition: @p start and @p goal are level (zero pitch). Each step rebuilds the orientation
   * from yaw alone, so a pitched start loses its pitch and a pitched goal throws std::runtime_error.
   *
   * @param start The start robot state.
   * @param goal  The goal robot state.
   * @param map Optional planner map for map-aware trajectory generation.
   * @return A dense trajectory of robot states advancing by dt per step.
   */
  RobotTrajectory steer(const RobotState& start, const RobotState& goal, Map* map = nullptr) const override;

 protected:
  /**
   * @brief Raw sphere collision against the TSDF map plus bounding-volume and hit-region checks.
   *
   * kUntraversable outside the bounding volume, inside a hit region, or when any collision sphere
   * collides or any contact sphere is off the ground. Otherwise kTraversable once
   * min_observed_spheres spheres were observed passing, else kPotentiallyTraversable.
   */
  Traversability getStateTraversabilityRaw(const RobotState& state, Map* map) const override;

  /**
   * @brief Returns whether traversability checking depends on the robot's yaw orientation.
   * @return true if collision geometry is yaw-dependent.
   */
  bool isTraversabilityOrientationRelated() const override;

  /**
   * @brief Deduces whether traversability depends on yaw and names the evidence.
   *
   * The default inspects the collision and contact sphere offsets.
   */
  virtual bool deduceCollisionCheckYaw(std::string* source) const;

  /**
   * @brief Applies the optional collision_check_yaw override to the deduction.
   *
   * An explicit value that disagrees with the deduction is honored, with a warning.
   */
  void configureCollisionCheckYaw(Configurable::ParamMap* param_map);

  /**
   * @brief Returns false if the state violates bounding volume or hit region constraints.
   */
  bool passesPlannerConstraints(const RobotState& state, Map* map) const;

  /**
   * @brief Relative margin one placed sphere keeps, in [clearance_margin_eps, 1.0].
   *
   * `clamp((esdf_distance - radius) / safe_margin, clearance_margin_eps, 1.0)`; 1.0 without a
   * safe_margin or a distance value.
   *
   * @param sphere World-frame sphere to measure.
   * @param map    Distance field to measure against; must not be null.
   */
  double getSphereClearance(const GeometricSphere& sphere, map::TsdfMap* map) const;

  /**
   * @brief Renders collision (gray, `collision_sphere`) and contact (white, `contact_sphere`)
   *        spheres along a trajectory.
   *
   * Safe margins are drawn as outer shells (`collision_sphere_safe_margin`) colored by the margin
   * kept (gray without a distance field). Bodies are drawn every collision_sphere_visualization_stride
   * states, endpoints always.
   */
  void getVisualizationImpl(const RobotTrajectory& trajectory, VisualizationMarkers* markers) const override;

  static PluginRegistry::Registration<SimpleRobot> registration;
  std::unique_ptr<BoundingVolume> bounding_volume_;

  bool p_sample_z_{true};

  // Narrows the sampling distribution to the mapped part of the bounding volume; the bounding
  // volume itself stays the constraint (passesPlannerConstraints).
  bool p_restrict_sampling_to_mapped_region_{false};

  /** @brief Whether update() has seen a mapped region this planning cycle. */
  bool hasMappedRegion() const { return !mapped_region_min_.hasNaN(); }

  /** @brief Marks the mapped region unknown, so sampling falls back to the bounding volume. */
  void clearMappedRegion();

  /**
   * @brief Computes the bounds used by samplePosition for the current cycle.
   *
   * @param sampling_min Output lower corner.
   * @param sampling_max Output upper corner.
   * @return false only before setup has created the bounding volume; an inactive volume yields
   *         its (zero) extents.
   */
  bool getSamplingBounds(Eigen::Vector3d* sampling_min, Eigen::Vector3d* sampling_max) const;

  /** @brief Adds static and effective sampling-volume markers. */
  void addSamplingBoundsVisualization(VisualizationMarkers* markers) const;

  // Mapped region cached by update() for this cycle; written only by update() and resetEpisode().
  // NaN means no region yet.
  Eigen::Vector3d mapped_region_min_{Eigen::Vector3d::Constant(kDoubleNan)};
  Eigen::Vector3d mapped_region_max_{Eigen::Vector3d::Constant(kDoubleNan)};

  GeometricSpheres p_collision_spheres_;
  GeometricSpheres p_contact_spheres_;
  bool p_collision_check_yaw_{false};

  // Spheres that must be observed passing for kTraversable rather than kPotentiallyTraversable;
  // negative means every sphere.
  int p_min_observed_spheres_{-1};

  /// Draw the body every Nth trajectory state (endpoints regardless). 1 draws them all.
  int p_collision_sphere_visualization_stride_{1};

  // Reproducible RNG for random position sampling. Mutable because sampling
  // happens from const query methods.
  mutable SeededRng rng_;
};

}  // namespace robot_model
}  // namespace open_apero
