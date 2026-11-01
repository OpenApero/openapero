#pragma once

/**
 * @file history_data.h
 * @brief Stores what an episode did: where the robot went, and where it looked from.
 *
 * Where it looked from is one record, the observed viewpoints, which a plugin may query through
 * getObservedViewpoints(); no shipped plugin does.
 */

#include <limits>
#include <vector>

#include "open_apero_core/geometry/transform3d.h"
#include "open_apero_core/graph/edge.h"
#include "open_apero_core/state/observed_viewpoints.h"
#include "open_apero_core/state/robot_state.h"
#include "open_apero_core/visualization/visualization_markers.h"

namespace open_apero {

/**
 * @brief The episode's record of executed motion and of the poses it observed from.
 *
 * Two independent records: the traveled trajectory (visited-node endpoints, dense path,
 * accumulated cost) and the observed viewpoints, every sensor pose the map integrated a frame
 * from, exposed as getObservedViewpoints().
 */
class HistoryData {
 public:
  HistoryData() = default;

  /**
   * @brief Reset all stored history data. The observed-viewpoint thresholds stay.
   */
  void resetEpisode();

  /**
   * @brief Get the number of stored past nodes.
   * @return The count of past nodes.
   */
  int size() const { return past_nodes_.size(); }

  /**
   * @brief Add a visited trajectory and its associated cost to the history.
   * @param edge The edge representing the visited trajectory and its associated cost.
   */
  void addVisitedEdge(const Edge& edge);

  /**
   * @brief Record a sensor pose the map has integrated a frame from.
   *
   * Fed from the map's own report of the frames it consumed, not from what the planner
   * commanded, so it means "the scene was seen from here". Stored in getObservedViewpoints()
   * unless it is the same pose as the last one stored there.
   *
   * @param sensor_pose Sensor-frame pose of an integrated observation.
   */
  void addObservedPose(const Transform3d& sensor_pose);

  /**
   * @brief The episode's one record of the sensor poses it observed from, oldest first, fed by addObservedPose().
   *
   * Append-only between resetEpisode() calls, whatever its thresholds; consumers may hold a count
   * of entries read.
   *
   * @return Const reference to the observed viewpoints.
   */
  const ObservedViewpoints& getObservedViewpoints() const { return observed_viewpoints_; }

  /**
   * @brief The same record, for configuring its thresholds.
   * @return Reference to the observed viewpoints.
   */
  ObservedViewpoints& getObservedViewpoints() { return observed_viewpoints_; }

  /**
   * @brief Generate visualization markers for the history data.
   * @param markers Pointer to the marker collection to populate.
   * @param total_cost_budget Maximum cost budget for coloring the visualization.
   */
  void getVisualization(VisualizationMarkers* markers, double total_cost_budget = std::numeric_limits<double>::infinity());

 protected:
  /// @brief Past robot states at visited nodes.
  RobotTrajectory past_nodes_;

  /// @brief Full past trajectory including intermediate points.
  RobotTrajectory past_trajectory_;

  /// @brief Cumulative cost at each state of past_trajectory_ (same length).
  std::vector<double> past_costs_;

  /// @brief Sensor poses the map integrated a frame from, oldest first, no pose twice in a row. The only
  /// record that says the scene was actually looked at from a pose rather than merely passed through.
  ObservedViewpoints observed_viewpoints_;
};

}  // namespace open_apero
