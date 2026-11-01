#pragma once

/**
 * @file graph_depth_wise_beam_search.h
 * @brief Path planner that uses depth-wise beam search on a graph to find the best path.
 */

#include "open_apero_core/plugin/path_planner/graph_beam_search.h"

namespace open_apero {
namespace path_planner {

/**
 * @brief Selects the next path using depth-wise beam search (DBS) over a graph.
 *
 * Expands paths depth by depth up to beam_depth, keeping the top beam_width candidates per
 * level; with check_previous_path the previously selected path competes as a candidate.
 */
class GraphDepthWiseBeamSearch : public GraphBeamSearch {
 public:
  /**
   * @brief Construct a GraphDepthWiseBeamSearch path planner.
   * @param agent Reference to the parent agent interface.
   */
  explicit GraphDepthWiseBeamSearch(AgentInterface& agent);  // NOLINT(google-runtime-references) non-const ref to agent is intentional

 protected:
  void searchBeams(const Path& start_path, GraphType* graph, Node* required_terminal_node, PathSteps& steps, PathLedger& ledger, Path& best,
                   double& best_score) override;

  static PluginRegistry::Registration<GraphDepthWiseBeamSearch> registration;
};

}  // namespace path_planner
}  // namespace open_apero
