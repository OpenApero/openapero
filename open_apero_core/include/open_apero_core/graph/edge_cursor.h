#pragma once

/**
 * @file edge_cursor.h
 * @brief Cursor over the waypoints of the edge being executed.
 */

#include <stdexcept>

#include "open_apero_core/graph/edge.h"

namespace open_apero {

/**
 * @brief Cursor holding the active edge and current waypoint index.
 *        Used so all trajectory-progress operations are in one place.
 *
 * The agent creates one per dispatched edge and hands it to Agent::sendMovement(), which advances it past the
 * waypoints it commanded.
 */
struct EdgeCursor {
  /// @brief The edge being executed; not owned, never null.
  Edge* edge = nullptr;
  /// Index of the waypoint the robot is heading to: 0 (the edge's start state) until sendMovement() advances the cursor.
  int waypoint_idx = 0;

  /// @brief Starts at waypoint 0 of @p e. @throws std::invalid_argument if @p e is null.
  explicit EdgeCursor(Edge* e) : edge(e), waypoint_idx(0) {
    if (!edge) {
      throw std::invalid_argument("EdgeCursor cannot be initialized with a null edge");
    }
  }

  /// @brief Moves to the next waypoint: one more waypoint has been commanded.
  void advance() { ++waypoint_idx; }
  /// @brief Jumps to the last waypoint: the whole remaining trajectory has been commanded.
  void finish() { waypoint_idx = edge->trajectory.size() - 1; }

  /// @brief True once waypoint_idx has reached the edge's last state.
  bool isFinished() const { return edge && waypoint_idx >= static_cast<int>(edge->trajectory.size() - 1); }

  /// Previous waypoint (where the robot currently is). Precondition: waypoint_idx >= 1.
  const RobotState& from() const { return edge->trajectory[waypoint_idx - 1]; }
  /// Next waypoint (where the robot is heading).
  const RobotState& to() const { return edge->trajectory[waypoint_idx]; }

  /// Returns the remaining trajectory from the current waypoint_idx.
  RobotTrajectory getTailTrajectory() const {
    if (!edge || waypoint_idx >= static_cast<int>(edge->trajectory.size())) {
      return {};
    }
    return RobotTrajectory(edge->trajectory.begin() + waypoint_idx, edge->trajectory.end());
  }
};

}  // namespace open_apero