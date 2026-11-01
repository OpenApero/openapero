#pragma once

/**
 * @file graph_node_wise_beam_search.h
 * @brief Path planner that uses node-wise beam search on a graph to find the best path.
 */

#include "open_apero_core/plugin/path_planner/graph_beam_search.h"

namespace open_apero {
namespace path_planner {

/**
 * @brief Selects the next path using node-wise beam search (NBS) over a graph.
 *
 * Advances depth by depth like depth-wise beam search, but keeps up to beam_width partial
 * paths per graph node (a min-heap per node) instead of one global beam.
 */
class GraphNodeWiseBeamSearch : public GraphBeamSearch {
 public:
  /**
   * @brief Construct a GraphNodeWiseBeamSearch path planner.
   * @param agent Reference to the parent agent interface.
   */
  explicit GraphNodeWiseBeamSearch(AgentInterface& agent);  // NOLINT(google-runtime-references) non-const ref to agent is intentional

 protected:
  void searchBeams(const Path& start_path, GraphType* graph, Node* required_terminal_node, PathSteps& steps, PathLedger& ledger, Path& best,
                   double& best_score) override;

  static PluginRegistry::Registration<GraphNodeWiseBeamSearch> registration;
};

}  // namespace path_planner
}  // namespace open_apero
