#pragma once

/**
 * @file trajectory_length.h
 * @brief Cost computer that uses the Euclidean path length of a trajectory segment as cost.
 */

#include "open_apero_core/graph/edge.h"
#include "open_apero_core/plugin/cost_computer/cost_computer.h"

namespace open_apero {
namespace cost_computer {

/**
 * @brief Cost computer that computes the traveled distance of a trajectory segment.
 *
 * The cost of an edge is set to the total Euclidean path length of the trajectory
 * associated with that edge. This is useful for planners that seek to minimize
 * total distance traveled.
 */
class TrajectoryLength : public CostComputer {
 public:
  /**
   * @brief Construct a TrajectoryLength cost computer.
   * @param agent Reference to the parent agent interface.
   */
  explicit TrajectoryLength(AgentInterface& agent);  // NOLINT(google-runtime-references) non-const ref to agent is intentional

  /**
   * @brief Initialize parameters from the parameter map.
   * @param param_map Pointer to the plugin parameter map.
   */
  void setupFromParamMap(Configurable::ParamMap* param_map) override;

  /**
   * @brief Compute the path length cost for a given edge.
   * @param edge Pointer to the edge whose cost is computed.
   * @return True on success.
   */
  bool computeCost(Edge* edge) override;

  /** @brief The straight-line distance between the sensor positions. */
  double getCostLowerBound(const RobotState& start, const RobotState& goal) override;

 protected:
  static PluginRegistry::Registration<TrajectoryLength> registration;
};

}  // namespace cost_computer
}  // namespace open_apero
