#pragma once

/**
 * @file cost_computer.h
 * @brief Defines the CostComputer base class for computing edge traversal costs.
 */

#include "open_apero_core/graph/edge.h"
#include "open_apero_core/plugin/plugin.h"
#include "open_apero_core/plugin/plugin_registry.h"

namespace open_apero {

/**
 * @brief Abstract base class for computing the cost of traversing an edge.
 *
 * Implementations define the cost metric used to evaluate edges in the
 * planning graph (e.g., distance, time, energy).
 */
class CostComputer : public Plugin {
 public:
  /**
   * @brief Construct a CostComputer associated with the given agent.
   * @param agent Reference to the agent interface.
   */
  explicit CostComputer(AgentInterface& agent) : Plugin(agent) {}

  /**
   * @brief Reset per-episode cost computer state.
   */
  void resetEpisode() override {}

  /**
   * @brief Compute the cost of traversing the given edge.
   * @param edge Pointer to the edge whose cost should be computed.
   * @return True if the cost was computed successfully.
   */
  virtual bool computeCost(Edge* edge) = 0;

  /**
   * @brief A cost no edge the robot plans from @p start to @p goal can undercut.
   *
   * Lets a search skip planning an edge that cannot win. Default: 0, which skips nothing.
   */
  virtual double getCostLowerBound(const RobotState& /*start*/, const RobotState& /*goal*/) { return 0.0; }
};

}  // namespace open_apero
