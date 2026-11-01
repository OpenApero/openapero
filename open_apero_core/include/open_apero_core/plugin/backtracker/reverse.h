#pragma once

/**
 * @file reverse.h
 * @brief Backtracker that produces recovery trajectories by reversing previously
 *        executed forward segments.
 */

#include <deque>
#include <string>

#include "open_apero_core/common/typedef.h"
#include "open_apero_core/plugin/backtracker/backtracker.h"
#include "open_apero_core/state/robot_state.h"

namespace open_apero {
namespace backtracker {

/**
 * @brief The executed trajectories a reversing backtracker replays: at most a capacity of them,
 *        newest last, the oldest evicted first.
 */
class ExecutedTrajectoryQueue {
 public:
  /**
   * @brief Records the trajectory of an executed edge; a backtracker's own edge is skipped.
   * @param edge     The executed edge.
   * @param capacity Maximum number of trajectories held (> 0).
   * @param error    Set when the edge is refused.
   * @return False, recording nothing, when the trajectory has fewer than 2 states.
   */
  bool record(const Edge& edge, int capacity, std::string* error);

  /**
   * @brief Removes the newest trajectory and returns it reversed.
   * @return False, leaving @p out untouched, when the queue is empty.
   */
  bool popReversed(RobotTrajectory* out);

  bool empty() const { return trajectories_.empty(); }

  void clear() { trajectories_.clear(); }

 private:
  std::deque<RobotTrajectory> trajectories_;
};

/**
 * @brief Replays previously executed trajectory segments in reverse order.
 *
 * Maintains a fixed-size buffer of recent forward trajectories (oldest segment
 * evicted first when full). Each buildTrajectory call pops the most recent
 * segment and returns its reverse.
 */
class Reverse : public Backtracker {
 public:
  explicit Reverse(AgentInterface& agent);

  void resetEpisode() override { queue_.clear(); }

  bool recordExecutedEdge(const Edge& edge) override;

  bool buildTrajectory(const RobotState& state, RobotTrajectory* out) override;

  void setupFromParamMap(Configurable::ParamMap* param_map) override;

  bool checkParamsValid(std::string* error_message) override;

 protected:
  static PluginRegistry::Registration<Reverse> registration;

  int p_queue_size_ = 100;
  ExecutedTrajectoryQueue queue_;
};

}  // namespace backtracker
}  // namespace open_apero
