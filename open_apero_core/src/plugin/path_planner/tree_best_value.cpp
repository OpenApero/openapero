#include "open_apero_core/plugin/path_planner/tree_best_value.h"

#include <algorithm>
#include <stdexcept>
#include <vector>

#include "open_apero_core/graph/path.h"

namespace open_apero {
namespace path_planner {

PluginRegistry::Registration<TreeBestValue> TreeBestValue::registration("TreeBestValue");

TreeBestValue::TreeBestValue(AgentInterface& agent) : PathPlanner(agent) {}

void TreeBestValue::setupFromParamMap(Configurable::ParamMap* param_map) {
  PathPlanner::setupFromParamMap(param_map);

  rng_.seed(agent_.getRandomSeed(), (*param_map)["param_namespace"]);
}

std::pair<std::vector<Node*>, double> TreeBestValue::planPath(Node* start_node, GraphType* graph, Node* required_terminal_node) {
  if (graph != nullptr) {
    throw std::runtime_error("TreeBestValue: Graph not supported");
  }

  auto start_tree_node = start_node->as<TreeNode>();
  if (!start_tree_node) {
    throw std::runtime_error("TreeBestValue: Can only select next best for TreeNodes");
  }

  if (start_tree_node->parent) {
    throw std::runtime_error("TreeBestValue: start node must be the tree root.");
  }

  // compute value for all nodes
  std::vector<TreeNode*> all_tree_nodes;
  start_tree_node->getTree(&all_tree_nodes);
  suppressGains(std::vector<Node*>(all_tree_nodes.begin(), all_tree_nodes.end()), start_node);

  // A lone root has no step to offer.
  if (all_tree_nodes.size() == 1) {
    // A lone root reaches no destination; the start node is not one it can be asked for.
    if (required_terminal_node != nullptr) {
      return std::pair<std::vector<Node*>, double>({}, kDoubleMin);
    }
    return std::pair<std::vector<Node*>, double>({start_node}, 0.0);
  }

  // remove the start node from the list
  all_tree_nodes.erase(std::remove(all_tree_nodes.begin(), all_tree_nodes.end(), start_tree_node), all_tree_nodes.end());

  // Running best: each candidate is folded in via PathPlanner::updateBest, which scores it per the
  // configured objective and keeps the highest-scoring path. Invalid (no nodes) until one wins.
  PathSteps steps;
  PathLedger ledger;
  Path best(kDoubleMin, kDoubleMin, kDoubleMax);
  double best_score = kDoubleMin;

  for (auto& goal_node : all_tree_nodes) {
    // reconstruct path for each node
    PathNodes path_nodes;
    double path_cost = 0.0;
    for (auto current = goal_node; current != start_tree_node; current = current->parent) {
      path_nodes.push_back(current);
      path_cost += current->edge.cost;
    }

    if (!path_nodes.empty()) {
      path_nodes.push_back(start_tree_node);
      std::reverse(path_nodes.begin(), path_nodes.end());
    } else {
      continue;
    }

    if (path_cost > agent_.getCurrentCostBudget()) {
      continue;
    }

    // The path is now from the start node to the goal node
    if (path_nodes.front() != start_tree_node) {
      throw std::runtime_error("TreeBestValue: Path does not start with start node");
    }

    const double path_gain = ledger.pathGain(path_nodes, p_is_gain_correlated_);
    const double path_ratio = path_gain / path_cost;

    // Fold into the running best
    updateBest(Path(path_ratio, path_gain, path_cost, steps.chain(path_nodes)), best, best_score, required_terminal_node);
  }

  // The running best (folded in by updateBest) is the selection. When there is none an exploring
  // search still steps to some child, but a search asked for a destination reports nothing.
  std::vector<TreeNode*> children;
  start_tree_node->getChildren(&children);
  const std::vector<Node*> fallback_candidates(children.begin(), children.end());

  const std::vector<Node*> best_path_nodes =
      concludeSearch(best, start_tree_node, required_terminal_node, fallback_candidates, &rng_.engine, &best_score);

  return {best_path_nodes, best_score};
}

}  // namespace path_planner
}  // namespace open_apero
