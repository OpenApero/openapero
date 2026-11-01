#pragma once

/**
 * @file robot_model.h
 * @brief Abstract robot interface for traversability checking and trajectory generation.
 *
 * Subclasses implement collision/traversability semantics against a particular map type
 * (e.g. SimpleRobot against TsdfMap) and define how to steer or plan between two states.
 */

#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "open_apero_core/agent/agent_interface.h"
#include "open_apero_core/common/typedef.h"
#include "open_apero_core/common/utils.h"
#include "open_apero_core/geometry/geometry_primitives.h"
#include "open_apero_core/geometry/transform3d.h"
#include "open_apero_core/graph/node.h"
#include "open_apero_core/plugin/map/map.h"
#include "open_apero_core/plugin/plugin.h"
#include "open_apero_core/plugin/plugin_registry.h"
#include "open_apero_core/state/robot_state.h"
#include "open_apero_core/visualization/visualization_markers.h"

namespace open_apero {

/**
 * @brief Whether checkOrientations enforces the stationing rule at a position.
 *
 * kExemptBootstrap is only for the planning start node the robot already occupies; every sampled
 * position uses kEnforce, the default.
 */
enum class StationingCheck {
  kEnforce,
  kExemptBootstrap,
};

/**
 * @brief Abstract robot interface: kinematic parameters and traversability/planning hooks.
 *
 * Stores velocity limits, time step and the sensor-to-base transform. Subclasses define what
 * "traversable" means for their body and which map type they query (SimpleRobot: TsdfMap).
 */
class RobotModel : public Plugin {
 public:
  /** @brief Constructs RobotModel and binds it to the agent. */
  explicit RobotModel(AgentInterface& agent);
  virtual ~RobotModel() = default;

  /** @brief Loads kinematic, sensor-mount, and traversability settings. */
  void setupFromParamMap(Configurable::ParamMap* param_map) override;

  /** @brief Validates that all required base parameters are set. */
  bool checkParamsValid(std::string* error_message) override;

  /** @brief Reset per-episode robot state. */
  void resetEpisode() override {}

  /**
   * @brief Refresh per-planning-cycle robot state, the robot-side peer of Layer::update().
   *
   * Called once per cycle, after the executed edge's map integration is visible and before any
   * layer updates, expands or plans against this robot. Map-derived caches are refreshed here.
   */
  virtual void update() {}

  /** @brief Returns the planner's map. */
  Map* getMap() const;

  /** @brief Maximum linear velocity (m/s). */
  double getMaxLinearVelocity() const { return p_max_linear_velocity_; }

  /** @brief Maximum yaw rate (rad/s). */
  double getMaxYawRate() const { return p_max_yaw_rate_; }

  /** @brief Time step between consecutive trajectory states (s). */
  double getTimeStep() const { return p_time_step_; }

  /** @brief Sensor-to-base transform (base pose expressed in the sensor frame). */
  const Transform3d& getBaseInSensorTransform() const { return p_base_in_sensor_transform_; }

  /** @brief Override the sensor-to-base transform (e.g. when measured from TF at runtime). */
  void setBaseInSensorTransform(const Transform3d& base_in_sensor_transform) { p_base_in_sensor_transform_ = base_in_sensor_transform; }

  /** @brief Sensor mounting type (kFixed, kFreePitch, kUnconstrained). */
  SensorMountType getSensorMountType() const { return p_sensor_mount_type_; }

  /**
   * @brief Lowest pitch this robot's gaze may be planned at [rad], positive down.
   *
   * Every consumer that chooses a pitch stays within [getGazePitchMin, getGazePitchMax];
   * the default band is the full [-pi/2, pi/2].
   */
  double getGazePitchMin() const { return p_gaze_pitch_min_; }

  /** @brief Highest pitch this robot's gaze may be planned at [rad], positive down. */
  double getGazePitchMax() const { return p_gaze_pitch_max_; }

  /**
   * @brief Updates registered hit regions after a failed motion.
   *
   * The map owns the storage; the robot owns the policy (removing stale regions and adding
   * newly blocked ones). The default does nothing.
   *
   * @param target_state The robot's target/planned state.
   * @param stopped_state The state where the robot actually stopped.
   * @param hit_regions Registered hit regions to update in place.
   */
  virtual void updateHitRegions(const RobotState& target_state, const RobotState& stopped_state,
                                GeometryPrimitiveCollection* hit_regions) const;

  /**
   * @brief Tests whether this robot state intersects registered hit regions.
   *
   * The default reports no intersection; robot models override it with their body geometry.
   *
   * @param robot_state The robot state to test.
   * @param hit_regions Registered blocked regions.
   * @return true if the state should be considered blocked by hit regions.
   */
  virtual bool isInsideHitRegions(const RobotState& robot_state, const GeometryPrimitiveCollection& hit_regions) const;

  /**
   * @brief Returns whether a robot state is traversable in the given map.
   *
   * Wraps getStateTraversabilityRaw. kPotentiallyTraversable counts as traversable only within
   * safe_radius of the initial sensor position.
   *
   * @param state RobotModel state to check.
   * @param map Map used for environment queries.
   * @return true if the state is considered traversable.
   */
  bool isStateTraversable(const RobotState& state, Map* map) const;

  /**
   * @brief Axis-aligned bounds of the collision geometry at @p state, safety margins included.
   *
   * Everything the traversability and clearance verdicts at @p state read from the map lies inside. Default: no
   * bounds (false); a caller then treats every map update as relevant.
   */
  virtual bool getBodyBounds(const RobotState& state, Eigen::Vector3d* min, Eigen::Vector3d* max) const;

  /** @brief Whether sampled positions vary in height; false for a robot that plans in a plane. Default: true. */
  virtual bool samplesHeight() const { return true; }

  /**
   * @brief Returns how much of the robot's preferred obstacle margin a state keeps.
   *
   * Relative, not a distance: 1.0 means the whole body keeps its preferred margin, lower values
   * mean the margin is partly used up. Independent of traversability. The default reports 1.0
   * (a robot that declares no margin).
   *
   * @param state RobotModel state to measure.
   * @param map   Map used for environment queries.
   * @return Relative clearance in [clearance_margin_eps, 1.0].
   */
  virtual double getStateClearance(const RobotState& state, Map* map) const;

  /**
   * @brief Returns whether a state is traversable and keeps the robot's full preferred margin.
   *
   * Applied to stationing positions (nodes); trajectories are held to isStateTraversable only.
   *
   * @param state RobotModel state to check.
   * @param map   Map used for environment queries.
   * @return true if the state is traversable and at full clearance.
   */
  bool isStateSafe(const RobotState& state, Map* map) const;

  /**
   * @brief Floor of the relative clearance scale (0 < eps <= 1).
   *
   * getStateClearance never reports less, so 1/clearance stays finite.
   */
  double getClearanceMarginEps() const { return p_clearance_margin_eps_; }

  /**
   * @brief Returns whether the robot may be stationed (stop, rotate, observe) at a position.
   *
   * Asked of a bare position, independently of traversability; trajectories are never held to
   * it. checkOrientations enforces it at node creation, and a generator may re-ask it when it
   * refreshes existing clusters. The default admits everywhere.
   *
   * @param position Sensor position in world frame.
   * @param map      Map used for environment queries.
   * @return true if a node may be placed and kept at @p position.
   */
  virtual bool isStationingPositionAdmissible(const Eigen::Vector3d& position, Map* map) const;

  /**
   * @brief True iff @p state lies within safe_radius of the initial sensor position.
   *
   * The predicate isStateTraversable uses; the reference is
   * AgentInterface::getInitialSensorPosition, not the current pose. False when safe_radius <= 0.
   */
  bool isWithinSafeRadius(const RobotState& state) const;

  /** @brief Configured safe radius around the initial sensor position [m], 0 if unused. */
  double getSafeRadius() const { return p_safe_radius_; }

  /**
   * @brief Returns whether every state in a trajectory is traversable.
   * @param trajectory RobotModel trajectory to check.
   * @param map Map used for environment queries.
   * @return true if all states are traversable.
   */
  virtual bool isTrajectoryTraversable(const RobotTrajectory& trajectory, Map* map) const;

  /**
   * @brief Builds robot states and checks traversability for a list of orientation candidates.
   *
   * The check is isStateSafe, or isStateTraversable without the stationing rule under
   * kExemptBootstrap. At a position failing isStationingPositionAdmissible every candidate is
   * untraversable. @p states is filled for every candidate whatever the verdict. A kTraversable or
   * kUntraversable placement is the verdict without a map query; otherwise an orientation-independent
   * body reuses the first candidate's verdict.
   *
   * @param position        Sensor position in world frame.
   * @param candidates      Candidate orientation quaternions (e.g. from sampleOrientations).
   * @param states          Output robot states, one per candidate.
   * @param is_traversable  Output per-candidate traversability (true = traversable).
   * @param stationing      Whether to enforce the stationing rule; see StationingCheck.
   */
  void checkOrientations(const Eigen::Vector3d& position, const std::vector<Eigen::Quaterniond>& candidates,
                         std::vector<RobotState>* states, std::vector<bool>* is_traversable,
                         StationingCheck stationing = StationingCheck::kEnforce) const;

  /**
   * @brief Samples a random position within the robot's operating bounds.
   *
   * The default returns false: the base class has no sampling bounds. SimpleRobot samples within its configured
   * bounds and draws z only when its `sample_z` is true; otherwise z is @p z_default.
   *
   * @param pos       Output sampled position.
   * @param z_default Z value used when the robot is configured for 2D sampling.
   * @return true if a position was sampled; false if no bounds are configured.
   */
  virtual bool samplePosition(Eigen::Vector3d* pos, double z_default) const;

  /**
   * @brief Samples a random position in the neighborhood of a point of interest.
   *
   * Same bounds and z convention as samplePosition, further restricted to within
   * @p max_distance_3d of @p center in 3D and @p max_distance_xy horizontally (both apply).
   * The default returns false, like samplePosition().
   *
   * @param center          Point to sample around, in world frame.
   * @param max_distance_3d Maximum 3D distance from @p center.
   * @param max_distance_xy Maximum horizontal distance from @p center.
   * @param pos             Output sampled position.
   * @param z_default       Z value used when the robot is configured for 2D sampling.
   * @return true if a position was sampled; false if no bounds are configured or
   *         the neighborhood holds no admissible position.
   */
  virtual bool samplePositionNear(const Eigen::Vector3d& center, double max_distance_3d, double max_distance_xy, Eigen::Vector3d* pos,
                                  double z_default) const;

  /**
   * @brief Populates @p candidates with the orientations this robot considers at a position.
   *
   * The default yields `num_orientation_candidates` evenly spaced yaws. Performs no map
   * queries; pair it with checkOrientations.
   *
   * @param position   Sensor position in world frame.
   * @param candidates Output list of candidate orientation quaternions.
   */
  virtual void sampleOrientations(const Eigen::Vector3d& position, std::vector<Eigen::Quaterniond>* candidates) const;

  /**
   * @brief Gives the nodes of one same-position cluster gain anchors they share.
   *
   * RRAG calls it once per new cluster, after the cluster's nodes exist and before they are
   * evaluated, with one node per candidate in sampleOrientations order. An override appends the
   * anchors to the nodes' gain_anchors, so an evaluator can score an anchor once for every node
   * holding it. The default attaches none.
   *
   * @param position Sensor position of the cluster in world frame.
   * @param cluster  The cluster's nodes.
   */
  virtual void attachSharedGainAnchors(const Eigen::Vector3d& position, const NodeClusterPtrs& cluster) const;

  /**
   * @brief Returns edges connecting angularly adjacent states within a same-position cluster.
   *
   * Each pair (i, j) is an in-place rotation edge. The default is the yaw ring in both
   * directions; robots sampling more DOFs (e.g. pitch) override it.
   *
   * @param states RobotModel states from a single cluster (size must equal num_orientation_candidates).
   * @return Edge list for in-place rotation edges within the cluster.
   */
  virtual EdgeList getInplaceAdjacency(const std::vector<RobotState>& states) const;

  /**
   * @brief Returns edges between two clusters, scored against a planned path's own end directions.
   *
   * Without a planned path, pass the straight-line chord as both directions. The default
   * connects the single best-aligned pair and ignores @p connect_enclosing_cell.
   *
   * @param from_states            RobotModel states of the source cluster.
   * @param to_states              RobotModel states of the target cluster.
   * @param departure_direction    World-frame direction the path leaves @p from_states along.
   * @param arrival_direction      World-frame direction the path arrives at @p to_states along.
   * @param connect_enclosing_cell Whether to connect every candidate bracketing the direction.
   * @return Edge list of (from_index, to_index) pairs to connect; empty if either cluster is empty.
   */
  virtual EdgeList getTranslationAdjacency(const std::vector<RobotState>& from_states, const std::vector<RobotState>& to_states,
                                           const Eigen::Vector3d& departure_direction, const Eigen::Vector3d& arrival_direction,
                                           bool connect_enclosing_cell) const;

  /**
   * @brief Plans a collision-free translation trajectory from start to goal.
   *
   * Precondition: start and goal differ in position (in-place rotations go through
   * planRotationTrajectory); implementations may throw otherwise. The result satisfies all
   * planner constraints (bounding volume, hit regions).
   *
   * @param start      Start robot state.
   * @param goal       Goal robot state.
   * @param map        Map used for collision and traversability queries.
   * @param trajectory Output trajectory, valid only when the function returns true.
   * @return true if a feasible trajectory was found, false otherwise.
   */
  virtual bool planTrajectory(const RobotState& start, const RobotState& goal, Map* map, RobotTrajectory* trajectory) = 0;

  /**
   * @brief Plans a rotation-only trajectory connecting two same-position states.
   *
   * The default steers and, when traversability depends on orientation, validates the result.
   *
   * @param start      Start robot state.
   * @param goal       Goal robot state (same position, different orientation).
   * @param map        Map used for collision and traversability queries.
   * @param trajectory Output trajectory, valid only when the function returns true.
   * @return true if a feasible rotation trajectory was found, false otherwise.
   */
  virtual bool planRotationTrajectory(const RobotState& start, const RobotState& goal, Map* map, RobotTrajectory* trajectory);

  /**
   * @brief Steers from start toward goal, returning a kinematically feasible trajectory.
   *
   * Precondition: the sensor poses differ (isSameSensorPose at default kEpsilon); coincident
   * poses throw. The result therefore has at least 2 states.
   *
   * Postcondition: the last state is exactly @p goal's sensor pose; consecutive states differ at the
   * default pose tolerance; no step exceeds max_linear_velocity or max_yaw_rate times time_step.
   *
   * @param start The start robot state.
   * @param goal  The goal robot state (must differ from start in sensor pose).
   * @param map Optional planner map for map-aware trajectory generation.
   * @return A dense trajectory of robot states advancing by dt per step.
   */
  virtual RobotTrajectory steer(const RobotState& start, const RobotState& goal, Map* map = nullptr) const = 0;

  /**
   * @brief Places the robot so its sensor is at the given pose; @p robot_state is always written.
   *
   * kTraversable / kUntraversable vouch for the isStateTraversable verdict, including every
   * planner constraint it applies (hit regions, bounding volume); kUnset makes no claim and the
   * caller must ask isStateTraversable. Never returns kPotentiallyTraversable.
   *
   * @param robot_state Output robot state to populate (always written).
   * @param sensor_transform Sensor transform in world frame.
   * @param map Optional planner map for map-aware placement and vouching.
   * @return Placement-layer traversability verdict (see above).
   */
  virtual Traversability placeRobotAtSensorPose(RobotState* robot_state, const Transform3d& sensor_transform, Map* map = nullptr) const = 0;

  /**
   * @brief Generates visualization markers for the robot's collision model along a trajectory.
   *
   * Checks p_visualize_ and dispatches to getVisualizationImpl. Subclasses override
   * getVisualizationImpl to render their specific collision geometry.
   *
   * @param trajectory The trajectory whose robot poses to visualize.
   * @param markers    Output marker container.
   */
  void getVisualization(const RobotTrajectory& trajectory, VisualizationMarkers* markers) const {
    if (p_visualize_) {
      getVisualizationImpl(trajectory, markers);
    }
  }

  /**
   * @brief Hook called by the agent on the selected edge trajectory immediately before execution.
   *
   * May rewrite intermediate states (1..N-2), never front() or back(); the agent recomputes the
   * edge cost afterwards. Only the first @p num_committed_states are executed before the next
   * call, so phase carried across calls advances by those, not by the whole trajectory.
   *
   * @param trajectory          Trajectory to refine in-place.
   * @param num_committed_states Leading states executed before refinement.
   */
  virtual void refineBeforeExecute(RobotTrajectory& trajectory, int num_committed_states);

  /**
   * @brief Replaces the trailing rotation with one toward @p goal_state's orientation.
   *
   * Keeps the trajectory up to its first state at the final position, then steers the rotation
   * to @p goal_state. Refuses a result with fewer than 2 states (a zero-motion edge).
   *
   * @param trajectory Trajectory to modify in place. Must be non-null and non-empty.
   * @param goal_state Desired final robot state (position is ignored; only orientation is used).
   * @return True on success. False if the result would have fewer than 2 states; the
   *         trajectory is left unchanged.
   */
  virtual bool retargetTrajectoryTail(RobotTrajectory* trajectory, const RobotState& goal_state);

  /**
   * @brief Reorients the trajectory's leading rotation toward @p goal_state.
   *
   * Implemented by reversing the trajectory, calling retargetTrajectoryTail, then reversing back.
   *
   * @param trajectory Trajectory to modify in place. Must be non-null and non-empty.
   * @param goal_state Desired initial robot state (position is ignored; only orientation is used).
   * @return True on success. False if the result would have fewer than 2 states; the
   *         trajectory is left unchanged (in its original order).
   */
  virtual bool retargetTrajectoryHead(RobotTrajectory* trajectory, const RobotState& goal_state);

  /**
   * @brief Re-aims both end rotations of an already validated trajectory at new endpoint states.
   *
   * Precondition: @p trajectory is validated against the current map and @p start_state /
   * @p goal_state share its first and last positions (throws otherwise). All or nothing; an
   * orientation-dependent robot re-checks the result against the map.
   *
   * @param trajectory  Trajectory to retarget in place. Must be non-null and non-empty.
   * @param start_state Desired first state (its position must match the trajectory's).
   * @param goal_state  Desired last state (its position must match the trajectory's).
   * @return True on success, with @p trajectory replaced by the retargeted result.
   *         False on refusal, with @p trajectory unchanged.
   */
  bool retargetTrajectoryEndpoints(RobotTrajectory* trajectory, const RobotState& start_state, const RobotState& goal_state);

 protected:
  /**
   * @brief Override to render robot-specific collision geometry (spheres, footprint polygons, etc.).
   *
   * Called by getVisualization when p_visualize_ is true. Default is a no-op.
   */
  virtual void getVisualizationImpl(const RobotTrajectory& /*trajectory*/, VisualizationMarkers* /*markers*/) const {}

  /**
   * @brief Raw traversability of a robot state — subclasses implement collision/footprint here.
   *
   * Should NOT apply the safe_radius optimism — that is layered on by
   * the public isStateTraversable wrapper so every call site sees consistent semantics.
   */
  virtual Traversability getStateTraversabilityRaw(const RobotState& state, Map* map) const = 0;

  /**
   * @brief Returns whether traversability checking depends on the robot's yaw orientation.
   * @return true if collision geometry is orientation-dependent.
   */
  virtual bool isTraversabilityOrientationRelated() const = 0;

  bool p_visualize_{true};

  /// Optimism applied on top of kPotentiallyTraversable map results.
  double p_safe_radius_{0.0};  ///< Trust positions within this distance from the initial sensor position.

  double p_clearance_margin_eps_{1e-4};  ///< Floor of the relative clearance scale; keeps 1/clearance finite.

  double p_max_linear_velocity_{1.0};     ///< m/s, maximum linear velocity.
  double p_max_yaw_rate_{M_PI / 2.0};     ///< rad/s, maximum yaw rate.
  double p_time_step_{0.05};              ///< s, time step between consecutive trajectory states.
  int p_num_orientation_candidates_{12};  ///< Number of orientation candidates the robot enumerates per position.

  Transform3d p_base_in_sensor_transform_{Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()};
  SensorMountType p_sensor_mount_type_{SensorMountType::kFixed};

  double p_gaze_pitch_min_{-M_PI_2};  ///< rad, lowest plannable gaze pitch; negative looks up.
  double p_gaze_pitch_max_{M_PI_2};   ///< rad, highest plannable gaze pitch; positive looks down.
};

}  // namespace open_apero
