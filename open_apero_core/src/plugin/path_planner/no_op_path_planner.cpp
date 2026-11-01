#include "open_apero_core/plugin/path_planner/no_op_path_planner.h"

#include <utility>

namespace open_apero {
namespace path_planner {

PluginRegistry::Registration<NoOpPathPlanner> NoOpPathPlanner::registration("NoOpPathPlanner");

NoOpPathPlanner::NoOpPathPlanner(AgentInterface& agent) : PathPlanner(agent) {}

void NoOpPathPlanner::setupFromParamMap(Configurable::ParamMap* /*param_map*/) {}

std::pair<std::vector<Node*>, double> NoOpPathPlanner::planPath(Node* /*start_node*/, GraphType* /*graph*/,
                                                                Node* /*required_terminal_node*/) {
  return {{}, 0.0};
}

}  // namespace path_planner
}  // namespace open_apero
