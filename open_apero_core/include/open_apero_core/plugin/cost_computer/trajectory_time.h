#pragma once

/**
 * @file trajectory_time.h
 * @brief Cost computer that uses the execution time of a trajectory segment as cost.
 */

#include "open_apero_core/graph/edge.h"
#include "open_apero_core/plugin/cost_computer/cost_computer.h"

namespace open_apero {
namespace cost_computer {

/**
 * @brief Cost computer that computes the execution time of a single trajectory segment.
 *
 * The cost of an edge is set to the time required to execute the trajectory
 * associated with that edge. This is useful for time-optimal planning.
 */
class TrajectoryTime : public CostComputer {
 public:
  /**
   * @brief Construct a TrajectoryTime cost computer.
   * @param agent Reference to the parent agent interface.
   */
  explicit TrajectoryTime(AgentInterface& agent);  // NOLINT(google-runtime-references) non-const ref to agent is intentional

  /**
   * @brief Compute the execution time cost for a given edge.
   * @param edge Pointer to the edge whose cost is computed.
   * @return True on success.
   */
  bool computeCost(Edge* edge) override;

  /**
   * @brief The larger of the sensor distance over max_linear_velocity and the yaw difference over max_yaw_rate.
   *
   * Holds for a trajectory whose every step keeps those limits, as RobotModel::steer() promises.
   */
  double getCostLowerBound(const RobotState& start, const RobotState& goal) override;

  /**
   * @brief Initialize parameters from the parameter map.
   * @param param_map Pointer to the plugin parameter map.
   */
  void setupFromParamMap(Configurable::ParamMap* param_map) override;

 protected:
  static PluginRegistry::Registration<TrajectoryTime> registration;
};

}  // namespace cost_computer
}  // namespace open_apero
