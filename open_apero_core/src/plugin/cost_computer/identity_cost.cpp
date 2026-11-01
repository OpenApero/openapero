#include "open_apero_core/plugin/cost_computer/identity_cost.h"

namespace open_apero {
namespace cost_computer {

PluginRegistry::Registration<IdentityCost> IdentityCost::registration("IdentityCost");

IdentityCost::IdentityCost(AgentInterface& agent) : CostComputer(agent) {}

void IdentityCost::setupFromParamMap(Configurable::ParamMap* /*param_map*/) {}

bool IdentityCost::computeCost(Edge* edge) {
  if (edge->trajectory.size() < 2) {
    edge->cost = 0.0;
    return false;
  }
  edge->cost = 1.0;
  return true;
}

}  // namespace cost_computer
}  // namespace open_apero
