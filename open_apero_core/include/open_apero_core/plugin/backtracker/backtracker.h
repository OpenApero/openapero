#pragma once

/**
 * @file backtracker.h
 * @brief Defines the Backtracker base class: a recovery-trajectory strategy.
 *
 * A Backtracker is a pure producer of a recovery RobotTrajectory from the
 * robot's current state. It owns no nodes, no edges, no graph: it just
 * computes a trajectory. The BacktrackingLayer is the agent-facing facade
 * that wraps a Backtracker and turns its trajectory into a Node+Edge step.
 */

#include <string>
#include <vector>

#include <Eigen/Core>

#include "open_apero_core/common/typedef.h"
#include "open_apero_core/graph/edge.h"
#include "open_apero_core/logging/metrics.h"
#include "open_apero_core/plugin/plugin.h"
#include "open_apero_core/plugin/plugin_registry.h"
#include "open_apero_core/state/robot_state.h"
#include "open_apero_core/visualization/visualization_markers.h"

namespace open_apero {

/** @brief One candidate a strategy evaluated for its last decision. */
struct ProbeSample {
  Eigen::Vector3d position;  ///< World-frame sensor position of the sample.
  bool traversable;          ///< Whether the robot is collision-free there.
  bool counted;              ///< Whether the sample contributed to its branch's count.
  /// 0 traversable; a nonzero value is a strategy-defined reason the sample was rejected or skipped.
  int reason;
  int branch;  ///< Which alternative of the decision the sample argues for.
  int ray;     ///< Direction index within that alternative.
  int step;    ///< Sample index along the ray.
};

/**
 * @brief Abstract base for backtracking strategies that produce recovery trajectories.
 */
class Backtracker : public Plugin {
 public:
  explicit Backtracker(AgentInterface& agent) : Plugin(agent) {}

  /** @brief Resets per-episode backtracker state. Default no-op. */
  void resetEpisode() override {}

  /**
   * @brief Whether the backtracker has finished its initial sweep (e.g. a one-time rotation at
   *        episode start). The owning layer consults this to decide whether to bump its priority.
   */
  virtual bool isInitialExplorationDone() { return true; }

  /**
   * @brief Record an executed edge for use in future recovery trajectories.
   *        Called by the owning layer in onStateReached whenever a non-backtrack
   *        edge has just been executed.
   */
  virtual bool recordExecutedEdge(const Edge& /*edge*/) { return true; }

  /**
   * @brief Observe the agent's reached state even when this backtracker does not win this cycle.
   *
   * The owning BacktrackingLayer calls this from onStateReached so stateful
   * backtrackers can clear per-recovery state once normal planning resumes.
   */
  virtual void onStateReached(const RobotState& /*state*/) {}

  /**
   * @brief Append this backtracker's debug markers for the current recovery state. Default no-op.
   */
  virtual void getVisualization(VisualizationMarkers* /*markers*/) const {}

  /**
   * @brief Register this strategy's per-cycle columns; the owning layer's prefix applies. Default none.
   *
   * Registered pointers must outlive the performance log.
   */
  virtual void declareMetrics(MetricsFields& /*fields*/) const {}

  /** @brief Zero the columns declareMetrics registered. Default no-op. */
  virtual void resetCycleMetrics() {}

  /** @brief Samples behind the most recent decision. Default: a strategy that keeps none. */
  virtual const std::vector<ProbeSample>& lastProbeSamples() const {
    static const std::vector<ProbeSample> kNone;
    return kNone;
  }

  /**
   * @brief Build a recovery trajectory from the robot's current state.
   *
   * Strategy hook implemented by concrete backtrackers (Reverse / RotateInPlace
   * / RotateReverse). Returns false if no recovery is currently possible
   * (e.g. Reverse with an empty queue).
   *
   * @param state Robot's current state (start of the recovery trajectory).
   * @param out   Output trajectory; first waypoint must be at time 0 and
   *              size() must be >= 2 on success.
   * @return true if a valid trajectory was produced.
   */
  virtual bool buildTrajectory(const RobotState& state, RobotTrajectory* out) = 0;
};

}  // namespace open_apero
