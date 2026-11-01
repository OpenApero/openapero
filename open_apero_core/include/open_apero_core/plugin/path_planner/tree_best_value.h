#pragma once

/**
 * @file tree_best_value.h
 * @brief Path planner that returns the root-to-node path of a tree that scores best under the objective.
 */

#include <utility>
#include <vector>

#include "open_apero_core/common/seeding.h"
#include "open_apero_core/graph/tree_node.h"
#include "open_apero_core/plugin/path_planner/path_planner.h"

namespace open_apero {
namespace path_planner {

/**
 * @brief Selects the whole root-to-node path that scores best under the objective.
 *
 * Scores the path from the root to every node of the tree rooted at the current node by the
 * configured Objective (PathPlanner::updateBest) and returns the best one.
 */
class TreeBestValue : public PathPlanner {
 public:
  /**
   * @brief Construct a TreeBestValue path planner.
   * @param agent Reference to the parent agent interface.
   */
  explicit TreeBestValue(AgentInterface& agent);  // NOLINT(google-runtime-references) non-const ref to agent is intentional

  /**
   * @brief Select the next best path by finding the highest-value subtree.
   * @param start_node The node to start the search from.
   * @param graph Unused for tree-based search (may be nullptr).
   * @param required_terminal_node Tree node the selected path must end at, or nullptr to search freely.
   * @return A pair of the best path (as a vector of nodes) and its value.
   */
  std::pair<std::vector<Node*>, double> planPath(Node* start_node, GraphType* graph, Node* required_terminal_node) override;

  /**
   * @brief Initialize parameters from the parameter map.
   * @param param_map Pointer to the plugin parameter map.
   */
  void setupFromParamMap(Configurable::ParamMap* param_map) override;

  /** @brief Reseed the random-child RNG for a new episode. */
  void resetEpisode() override {
    PathPlanner::resetEpisode();
    rng_.reseed(agent_.getRandomSeed());
  }

 protected:
  static PluginRegistry::Registration<TreeBestValue> registration;

  // Reproducible RNG for the random-child fallback pick.
  SeededRng rng_;
};

}  // namespace path_planner
}  // namespace open_apero
