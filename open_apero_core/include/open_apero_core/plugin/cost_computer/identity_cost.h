#pragma once

/**
 * @file identity_cost.h
 * @brief Cost computer that assigns identity (unit) cost to all edges.
 */

#include "open_apero_core/graph/edge.h"
#include "open_apero_core/plugin/cost_computer/cost_computer.h"

namespace open_apero {
namespace cost_computer {

/**
 * @brief Prices every edge at 1: path cost and the budget count edges.
 */
class IdentityCost : public CostComputer {
 public:
  /**
   * @brief Construct an IdentityCost cost computer.
   * @param agent Reference to the parent agent interface.
   */
  explicit IdentityCost(AgentInterface& agent);  // NOLINT(google-runtime-references) non-const ref to agent is intentional

  /**
   * @brief Compute the cost for a given edge: 1, or 0 for a trajectory of fewer than 2 states.
   * @param edge Pointer to the edge whose cost is computed.
   * @return False for a trajectory of fewer than 2 states.
   */
  bool computeCost(Edge* edge) override;

 protected:
  /**
   * @brief Initialize parameters from the parameter map.
   * @param param_map Pointer to the plugin parameter map.
   */
  void setupFromParamMap(Configurable::ParamMap* param_map) override;

  static PluginRegistry::Registration<IdentityCost> registration;
};

}  // namespace cost_computer
}  // namespace open_apero
