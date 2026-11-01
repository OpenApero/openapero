// DBS retains one global beam per depth. check_previous_path enables the
// receding-horizon extension beyond Qu et al.'s Algorithm 1.
// Paper mapping: the paper's beam width B is beam_width, its depth D is beam_depth and its
// remaining budget C is the agent's cost budget; beams are ranked by the path preference
// relation (Path::operator>: ratio, then gain, then lower cost).
// gain_suppression_radius is a code-only extension of the paper's search.

#include "open_apero_core/plugin/path_planner/graph_depth_wise_beam_search.h"

#include <cstddef>
#include <vector>

namespace open_apero {
namespace path_planner {

PluginRegistry::Registration<GraphDepthWiseBeamSearch> GraphDepthWiseBeamSearch::registration("GraphDepthWiseBeamSearch");

GraphDepthWiseBeamSearch::GraphDepthWiseBeamSearch(AgentInterface& agent) : GraphBeamSearch(agent, "graph_depth_wise_beam_search_raw") {}

void GraphDepthWiseBeamSearch::searchBeams(const Path& start_path, GraphType* graph, Node* required_terminal_node, PathSteps& steps,
                                           PathLedger& ledger, Path& best, double& best_score) {
  std::vector<Path> open_list;
  open_list.push_back(start_path);

  for (int depth = 0; depth < p_beam_depth_; ++depth) {
    // A min-heap exposes the worst path for global top-k pruning.
    PathMinHeap next_open_list;

    for (const Path& path : open_list) {
      auto node_id = getNodeId(path.last->node, graph);

      ledger.load(path);
      auto neighbors = boost::adjacent_vertices(node_id, *graph);

      for (auto neighbor_it = neighbors.first; neighbor_it != neighbors.second; ++neighbor_it) {
        auto new_node_id = *neighbor_it;
        auto new_node = (*graph)[new_node_id].get();

        if (ledger.revisitsEdge(new_node)) {
          continue;
        }

        EdgeId edge_id = boost::edge(node_id, new_node_id, *graph).first;
        auto edge = (*graph)[edge_id];
        double new_cost = path.cost + edge->cost;

        if (new_cost > agent_.getCurrentCostBudget()) {
          continue;
        }

        edge->is_considered = true;

        double new_gain = path.gain + ledger.marginalGain(new_node, p_is_gain_correlated_);
        double new_ratio = new_gain / new_cost;
        const Path new_path(new_ratio, new_gain, new_cost, steps.extend(path.last, new_node));

        bool kept = true;
        if (next_open_list.size() < static_cast<std::size_t>(p_beam_width_)) {
          next_open_list.push(new_path);
        } else if (new_path > next_open_list.top()) {
          next_open_list.pop();
          next_open_list.push(new_path);
        } else {
          kept = false;
        }

        if (!updateBest(new_path, best, best_score, required_terminal_node) && !kept) {
          steps.discard();
        }
      }
    }

    open_list.clear();
    while (!next_open_list.empty()) {
      open_list.push_back(next_open_list.top());
      next_open_list.pop();
    }
  }
}

}  // namespace path_planner
}  // namespace open_apero
