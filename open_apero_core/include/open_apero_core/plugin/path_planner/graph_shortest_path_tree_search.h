#pragma once

/**
 * @file graph_shortest_path_tree_search.h
 * @brief Path planner that builds a shortest path tree and selects the best value path.
 */

#include <limits>
#include <utility>
#include <vector>

#include "open_apero_core/common/seeding.h"
#include "open_apero_core/plugin/path_planner/helpers.h"
#include "open_apero_core/plugin/path_planner/path_planner.h"

namespace open_apero {
namespace path_planner {

/**
 * @brief Selects a path from a shortest-path tree (SPT).
 *
 * Computes shortest paths from the start node (Dijkstra) and scores the path to each goal
 * candidate: nodes whose gain lies within top_node_percentage of the gain range below the maximum,
 * plus frontier nodes.
 */
class GraphShortestPathTreeSearch : public PathPlanner {
 public:
  /**
   * @brief Construct a GraphShortestPathTreeSearch path planner.
   * @param agent Reference to the parent agent interface.
   */
  explicit GraphShortestPathTreeSearch(AgentInterface& agent);  // NOLINT(google-runtime-references) non-const ref to agent is intentional

  /**
   * @brief Select the next best path using shortest path tree search.
   * @param start_node The node to start the search from.
   * @param graph The graph to search over; a null graph has no path to any destination.
   * @param required_terminal_node Node the selected path must end at, or nullptr to search freely.
   * @return A pair of the best path (as a vector of nodes) and its value.
   */
  std::pair<std::vector<Node*>, double> planPath(Node* start_node, GraphType* graph, Node* required_terminal_node) override;

  /**
   * @brief Initialize parameters from the parameter map.
   * @param param_map Pointer to the plugin parameter map.
   */
  void setupFromParamMap(Configurable::ParamMap* param_map) override;

  /** @brief Reseed the tie-break RNG for a new episode. */
  void resetEpisode() override {
    PathPlanner::resetEpisode();
    rng_.reseed(agent_.getRandomSeed());
  }

  /**
   * @brief Directly set search parameters for offline benchmarking.
   * @param obj The optimization objective.
   * @param top_node_percentage Fraction of the gain range, below the maximum gain, whose nodes are candidates.
   * @param is_gain_correlated Whether nodes share gain anchors, so a path counts each anchor once.
   */
  void setupRaw(Objective obj, double top_node_percentage, bool is_gain_correlated = false) {
    p_objective_ = obj;
    p_top_node_percentage_ = top_node_percentage;
    p_is_gain_correlated_ = is_gain_correlated;
    rng_.seed(agent_.getRandomSeed(), "graph_shortest_path_tree_search_raw");
  }

 protected:
  static PluginRegistry::Registration<GraphShortestPathTreeSearch> registration;

  double p_top_node_percentage_ = 1.0;

  // Reproducible RNG for the tie-break random neighbor pick.
  SeededRng rng_;
};

}  // namespace path_planner
}  // namespace open_apero
