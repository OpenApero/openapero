#pragma once

/**
 * @file no_op_path_planner.h
 * @brief PathPlanner implementation for layers that do their own selection.
 */

#include <utility>
#include <vector>

#include "open_apero_core/plugin/path_planner/path_planner.h"

namespace open_apero {
namespace path_planner {

/**
 * @brief PathPlanner placeholder for layers whose planPath implementation performs
 *        selection without a graph path planner. Always returns an empty path.
 */
class NoOpPathPlanner : public PathPlanner {
 public:
  explicit NoOpPathPlanner(AgentInterface& agent);  // NOLINT(google-runtime-references) non-const ref is intentional

  void setupFromParamMap(Configurable::ParamMap* param_map) override;

  std::pair<std::vector<Node*>, double> planPath(Node* start_node, GraphType* graph, Node* required_terminal_node) override;

 protected:
  static PluginRegistry::Registration<NoOpPathPlanner> registration;
};

}  // namespace path_planner
}  // namespace open_apero
