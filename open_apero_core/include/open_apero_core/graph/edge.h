#pragma once

/**
 * @file edge.h
 * @brief Data structure representing an edge connecting two nodes in a planning graph or tree.
 */

#include "open_apero_core/common/typedef.h"
#include "open_apero_core/state/robot_state.h"

namespace open_apero {

/**
 * @brief Stores edge data including trajectory, cost, and traversability information.
 *
 * An edge connects two nodes and carries the robot trajectory between them,
 * the associated traversal cost, and traversability status.
 */
struct Edge {
  /// @brief All trajectory points along the edge.
  RobotTrajectory trajectory;

  /// @brief Traversal cost associated with this edge.
  double cost = 0.0;

  /// @brief Whether this edge is traversable. Defaults to false: an edge never assigned is pruned
  /// on the next maintenance pass.
  bool is_traversable = false;

  /// @brief Whether the next path planner considers this edge.
  /// Excluded edges are visualized as semi-transparent.
  bool is_considered = false;

  /// @brief Produced by a Backtracker; backtrackers do not record such edges
  /// (Backtracker::recordExecutedEdge), and a platform may exempt them from its own
  /// pre-flight checks.
  bool is_backtrack = false;
};

}  // namespace open_apero
