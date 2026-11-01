#pragma once

/**
 * @file robot_state.h
 * @brief Robot state and trajectory types for the exploration planner.
 *
 * Defines the RobotState class that encapsulates base and sensor poses at a
 * given time, along with the RobotTrajectory type and utility functions for
 * trajectory manipulation.
 */

#include <string>
#include <vector>

#include <Eigen/Eigen>

#include "open_apero_core/common/typedef.h"
#include "open_apero_core/geometry/transform3d.h"

namespace open_apero {

/**
 * @brief Represents the full robot state including base and sensor poses at a point in time.
 *
 * Stores the base frame and sensor frame positions and orientations in the
 * world frame, along with a timestamp. Provides coordinate transformations
 * and comparison utilities.
 */
class RobotState {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// @brief Default constructor.
  RobotState();

  /**
   * @brief Construct a RobotState with full pose specification.
   * @param time_from_start Time offset from trajectory start.
   * @param base_transform Base frame transform in world coordinates.
   * @param sensor_transform Sensor frame transform in world coordinates.
   */
  RobotState(double time_from_start, const Transform3d& base_transform, const Transform3d& sensor_transform);
  RobotState(const RobotState&) = default;
  RobotState& operator=(const RobotState&) = default;
  ~RobotState() = default;

  /**
   * @brief Check if sensor pose (position and orientation) matches another state.
   * @param other The other robot state to compare.
   * @param position_epsilon Position tolerance in meters.
   * @param orientation_dot_epsilon Quaternion dot-product tolerance.
   * @return True if sensor poses are the same.
   */
  bool isSameSensorPose(const RobotState& other, double position_epsilon = kEpsilon, double orientation_dot_epsilon = kEpsilon) const;

  /**
   * @brief Check if sensor position matches another state.
   * @param other The other robot state to compare.
   * @param position_epsilon Position tolerance in meters.
   * @return True if sensor positions are the same.
   */
  bool isSameSensorPosition(const RobotState& other, double position_epsilon = kEpsilon) const;

  /**
   * @brief Check if sensor orientation matches another state.
   * @param other The other robot state to compare.
   * @param orientation_dot_epsilon Quaternion dot-product tolerance.
   * @return True if sensor orientations are the same.
   */
  bool isSameSensorOrientation(const RobotState& other, double orientation_dot_epsilon = kEpsilon) const;

  /**
   * @brief Check if the sensor pose has been reached given position and angle thresholds.
   *
   * Symmetric in the two states, so either may be the target (Agent calls it on the current state with the target
   * waypoint as @p current). A threshold below kEpsilon skips that check.
   *
   * @param current The other robot state.
   * @param pos_threshold Position tolerance in meters.
   * @param angle_threshold Orientation tolerance in radians.
   * @return True if the sensor pose is within the specified thresholds.
   */
  bool isSensorPoseReached(const RobotState& current, double pos_threshold, double angle_threshold) const;

  /**
   * @brief Get the time offset from trajectory start.
   * @return Time from start in seconds.
   */
  double getTimeFromStart() const;

  /**
   * @brief Get the base frame position in world coordinates.
   * @return Reference to the base position vector.
   */
  const Eigen::Vector3d& getBasePosition() const;

  /**
   * @brief Get the base frame orientation in world coordinates.
   * @return Reference to the base orientation quaternion.
   */
  const Eigen::Quaterniond& getBaseOrientation() const;

  /**
   * @brief Get the base frame transform in world coordinates.
   * @return Const reference to the base transform.
   */
  const Transform3d& getBaseTransform() const;

  /**
   * @brief Get the sensor frame position in world coordinates.
   * @return Reference to the sensor position vector.
   */
  const Eigen::Vector3d& getSensorPosition() const;

  /**
   * @brief Get the sensor frame orientation in world coordinates.
   * @return Reference to the sensor orientation quaternion.
   */
  const Eigen::Quaterniond& getSensorOrientation() const;

  /**
   * @brief Get the sensor frame transform in world coordinates.
   * @return Const reference to the sensor transform.
   */
  const Transform3d& getSensorTransform() const;

  /**
   * @brief Get the joint positions.
   * @return Const reference to the joint position vector. Empty if unset.
   */
  const Eigen::VectorXd& getJointPosition() const;

  /**
   * @brief Get the sensor yaw angle.
   * @return Yaw angle in radians.
   */
  double getSensorYaw() const;

  /**
   * @brief Get sensor Euler angles in ZYX order.
   * @return Euler angles as (z, y, x) vector.
   */
  Eigen::Vector3d getSensorEulerAnglesZyx() const;

  /**
   * @brief Set the time offset from trajectory start.
   * @param time_from_start Time from start in seconds.
   */
  void setTimeFromStart(double time_from_start);

  /**
   * @brief Set the base frame position.
   * @param base_position Base position in world coordinates.
   */
  void setBasePosition(const Eigen::Vector3d& base_position);

  /**
   * @brief Set the base frame orientation.
   * @param base_orientation Base orientation in world coordinates.
   */
  void setBaseOrientation(const Eigen::Quaterniond& base_orientation);

  /**
   * @brief Set the base frame transform in world coordinates.
   * @param base_transform Base transform in world coordinates.
   */
  void setBaseTransform(const Transform3d& base_transform);

  /**
   * @brief Set the sensor frame position.
   * @param sensor_position Sensor position in world coordinates.
   */
  void setSensorPosition(const Eigen::Vector3d& sensor_position);

  /**
   * @brief Set the sensor frame orientation.
   * @param sensor_orientation Sensor orientation in world coordinates.
   */
  void setSensorOrientation(const Eigen::Quaterniond& sensor_orientation);

  /**
   * @brief Set the sensor frame transform in world coordinates.
   * @param sensor_transform Sensor transform in world coordinates.
   */
  void setSensorTransform(const Transform3d& sensor_transform);

  /**
   * @brief Set the joint positions.
   * @param joint_position Joint position vector of arbitrary dimension.
   */
  void setJointPosition(const Eigen::VectorXd& joint_position);

  /**
   * @brief Get a string representation of this robot state.
   * @return Formatted string with position and orientation information.
   */
  std::string toString() const;

 private:
  /// Time relative to the start of the containing trajectory [s].
  double time_from_start_;

  Transform3d transform_W_B_;
  Transform3d transform_W_S_;
  Eigen::VectorXd joint_position_;
};

/// @brief A sequence of robot states forming a trajectory, with Eigen-aligned allocation.
using RobotTrajectory = std::vector<RobotState, Eigen::aligned_allocator<RobotState>>;

/**
 * @brief Reverse the order of states in a trajectory.
 * @param trajectory The trajectory to reverse.
 * @return A new trajectory with states in reverse order.
 */
RobotTrajectory reverseTrajectory(const RobotTrajectory& trajectory);

/**
 * @brief World-frame displacement along which the trajectory leaves its start (any in-place prefix skipped).
 * @param trajectory The trajectory to inspect.
 * @return Raw, non-normalized displacement from the first state to the first state at a different position.
 * @throws std::invalid_argument if the trajectory is empty or never leaves its start position.
 */
Eigen::Vector3d trajectoryDepartureDelta(const RobotTrajectory& trajectory);

/**
 * @brief World-frame displacement along which the trajectory arrives at its end (any in-place suffix skipped).
 * @param trajectory The trajectory to inspect.
 * @return Raw, non-normalized displacement from the last state at a different position to the final state.
 * @throws std::invalid_argument if the trajectory is empty or never leaves its end position.
 */
Eigen::Vector3d trajectoryArrivalDelta(const RobotTrajectory& trajectory);

/**
 * @brief Index of the state whose sensor orientation best aligns with a yaw and pitch.
 *
 * Minimizes |angleDifference(state_yaw, yaw)| + |state_pitch - pitch|, keeping the first
 * of equal-cost states; pitch is positive-down. For pure-yaw candidates the pitch term is
 * a constant offset, so this reduces to the yaw argmin.
 *
 * @param states Candidate states; must be non-empty.
 * @param yaw    Target yaw [rad].
 * @param pitch  Target pitch [rad], positive down.
 * @return Index into @p states of the best-aligned state.
 */
int bestAlignedState(const std::vector<RobotState>& states, double yaw, double pitch);

/**
 * @brief Concatenate two trajectories end-to-end at a shared junction state.
 *
 * rhs.front() duplicates lhs.back() and is dropped; the remaining rhs times are offset by lhs's
 * final time. An empty side returns the other unchanged.
 *
 * @param lhs The first trajectory.
 * @param rhs The second trajectory to append.
 * @return A new concatenated trajectory.
 * @throws std::runtime_error unless lhs.back() and rhs.front() are the same sensor pose.
 */
RobotTrajectory concatenateTrajectory(const RobotTrajectory& lhs, const RobotTrajectory& rhs);

/**
 * @brief Compute the total arc length of a trajectory.
 * @param trajectory The trajectory to measure.
 * @return Sum of Euclidean distances between consecutive sensor positions [m].
 */
double getTrajectoryLength(const RobotTrajectory& trajectory);

/**
 * @brief Assign time_from_start to every state using a fixed time step.
 *
 * The first state is set to t = 0. Each subsequent state receives
 * t = i * dt.
 *
 * @param trajectory The trajectory whose state times will be updated in-place.
 * @param dt Time step between consecutive states [s]. Must be positive.
 */
void setUniformTrajectoryTimes(RobotTrajectory& trajectory, double dt);

/**
 * @brief Append a state unless it repeats (default pose tolerance) the pose the trajectory ends at.
 *
 * Steering loops use this instead of push_back so a split never yields a zero-motion edge.
 *
 * @param trajectory The trajectory to append to; an empty one always takes the state.
 * @param state The state to record.
 */
void appendMotionState(RobotTrajectory* trajectory, const RobotState& state);

/**
 * @brief Make final_state the last state of a trajectory built with appendMotionState().
 *
 * A last state within the pose tolerance of @p final_state is overwritten, not duplicated, unless
 * it is the only (start) state, so the trajectory ends exactly at @p final_state.
 *
 * @param trajectory The trajectory to close, modified in-place. Must not be empty.
 * @param final_state The state the trajectory must end at.
 * @throws std::logic_error if the result still contains a zero-motion step.
 */
void closeTrajectoryAtState(RobotTrajectory* trajectory, const RobotState& final_state);

}  // namespace open_apero
