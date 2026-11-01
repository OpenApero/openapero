#pragma once

/**
 * @file observed_viewpoints.h
 * @brief The sensor poses an episode observed from, indexed for "already looked from here" queries.
 */

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "open_apero_core/geometry/transform3d.h"
#include "open_apero_core/graph/kdtree.h"

namespace open_apero {

/**
 * @brief Whether a pose was already observed from: within a position and a geodesic angle threshold of a stored viewpoint.
 *
 * Every added pose is stored, whatever the thresholds; the thresholds decide only the answers. While
 * enabled, a pose is observed exactly when some stored viewpoint lies closer than the position
 * threshold to its position and closer than the angle threshold to its orientation. Position and
 * orientation must match together, and orientation is one rotation, so a yaw and a pitch each
 * inside the angle threshold can still put a pose outside it. While disabled, isObserved() and
 * getViewpointIndicesNear() find nothing.
 *
 * Not copyable: the KD-tree refers to the stored viewpoints by address.
 */
class ObservedViewpoints {
 public:
  /** @brief Disabled (zero thresholds), with nothing stored. */
  ObservedViewpoints();

  ObservedViewpoints(const ObservedViewpoints&) = delete;
  ObservedViewpoints& operator=(const ObservedViewpoints&) = delete;

  /**
   * @brief Whether a threshold pair is one setThresholds() accepts.
   *
   * Valid: both zero (disabled), or a position threshold above 0.01 m and a positive angle threshold.
   * One threshold alone would match on position alone or on orientation alone, and
   * Generator::findNearbyNodesInIndex reads a radius of at most 0.01 as unbounded.
   *
   * @param position_threshold Position threshold [m].
   * @param angle_threshold Geodesic orientation threshold [rad].
   * @param why Optional; set to the reason when the pair is invalid.
   * @return Whether the pair is valid.
   */
  static bool isValidThresholdPair(double position_threshold, double angle_threshold, std::string* why = nullptr);

  /**
   * @brief Sets both thresholds; the stored viewpoints stay, and the answers over them use the new thresholds.
   *
   * Zero thresholds disable it: isObserved() and getViewpointIndicesNear() then find nothing.
   *
   * @param position_threshold Position distance an observed pose lies below [m].
   * @param angle_threshold Geodesic orientation angle an observed pose lies below [rad].
   * @throws std::invalid_argument when isValidThresholdPair() rejects the pair; nothing then changes.
   */
  void setThresholds(double position_threshold, double angle_threshold);

  /** @brief Whether both thresholds are positive. */
  bool isEnabled() const { return position_threshold_ > 0.0 && angle_threshold_ > 0.0; }

  /** @brief Position threshold [m]. */
  double getPositionThreshold() const { return position_threshold_; }

  /** @brief Geodesic orientation threshold [rad]. */
  double getAngleThreshold() const { return angle_threshold_; }

  /**
   * @brief Stores the pose, enabled or not.
   * @param sensor_pose Sensor-frame pose of an integrated observation.
   */
  void add(const Transform3d& sensor_pose);

  /**
   * @brief Whether a stored viewpoint lies closer than both thresholds to this pose.
   * @param position Sensor position of the pose to test.
   * @param orientation Sensor orientation of the pose to test.
   * @return Always false while disabled.
   */
  bool isObserved(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) const;

  /**
   * @brief isObserved() with @p angle_threshold in place of getAngleThreshold(); the position threshold stays.
   * @param position Sensor position of the pose to test.
   * @param orientation Sensor orientation of the pose to test.
   * @param angle_threshold Geodesic orientation angle an observed pose lies below [rad].
   * @return Always false while disabled.
   */
  bool isObserved(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation, double angle_threshold) const;

  /**
   * @brief Indices of the stored viewpoints closer than @p radius to @p position, whatever their orientation.
   * @param position Sensor position to search around.
   * @param radius Search radius [m].
   * @param indices Replaced with the matches, ascending, each below size(); empty while disabled.
   */
  void getViewpointIndicesNear(const Eigen::Vector3d& position, double radius, std::vector<std::size_t>* indices) const;

  /**
   * @brief Number of stored viewpoints.
   *
   * Append-only between clear() calls, so an index below a count read earlier keeps its viewpoint.
   */
  std::size_t size() const { return data_.points.size(); }

  /**
   * @brief One stored viewpoint, in insertion order.
   * @param index Below size().
   * @return The viewpoint's sensor pose.
   */
  Transform3d getViewpoint(std::size_t index) const;

  /** @brief Drops every stored viewpoint; the thresholds stay. */
  void clear();

 private:
  /// @brief Position threshold [m]; zero disables it.
  double position_threshold_ = 0.0;

  /// @brief Geodesic orientation threshold [rad]; zero disables it.
  double angle_threshold_ = 0.0;

  /// @brief The stored viewpoints `kdtree_` indexes.
  ViewpointKdTreeData data_;

  /// @brief Index over `data_`, rebuilt whenever it is emptied.
  std::unique_ptr<ViewpointKdTree> kdtree_;
};

}  // namespace open_apero
