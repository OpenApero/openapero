// NBS retains a separate beam per destination node. check_previous_path enables
// the receding-horizon extension beyond Qu et al.'s Algorithm 2.
// Paper mapping: the paper's beam width B is beam_width, its depth D is beam_depth and its
// remaining budget C is the agent's cost budget; beams are ranked by the path preference
// relation (Path::operator>: ratio, then gain, then lower cost).
// gain_suppression_radius is a code-only extension of the paper's search.

#include "open_apero_core/plugin/path_planner/graph_node_wise_beam_search.h"

#include <cstddef>
#include <unordered_map>
#include <vector>

namespace open_apero {
namespace path_planner {

PluginRegistry::Registration<GraphNodeWiseBeamSearch> GraphNodeWiseBeamSearch::registration("GraphNodeWiseBeamSearch");

GraphNodeWiseBeamSearch::GraphNodeWiseBeamSearch(AgentInterface& agent) : GraphBeamSearch(agent, "graph_node_wise_beam_search_raw") {}

void GraphNodeWiseBeamSearch::searchBeams(const Path& start_path, GraphType* graph, Node* required_terminal_node, PathSteps& steps,
                                          PathLedger& ledger, Path& best, double& best_score) {
  // Per-node search state as arrays in vertex order, reused across depths.
  std::vector<NodeId> vertex_ids;
  std::unordered_map<const Node*, std::size_t> vertex_index;
  const auto vertices = boost::vertices(*graph);
  for (auto node_it = vertices.first; node_it != vertices.second; ++node_it) {
    vertex_index.emplace((*graph)[*node_it].get(), vertex_ids.size());
    vertex_ids.push_back(*node_it);
  }
  std::vector<std::vector<Path>> open_list(vertex_ids.size());
  // Per-node min-heaps expose each destination's worst path for top-k pruning.
  std::vector<PathMinHeap> next_open_list(vertex_ids.size());
  open_list[vertex_index.at(start_path.last->node)].push_back(start_path);

  for (int depth = 0; depth < p_beam_depth_; ++depth) {
    for (std::size_t index = 0; index < vertex_ids.size(); ++index) {
      auto node_id = vertex_ids[index];

      for (const Path& path : open_list[index]) {
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
          PathMinHeap& new_v_next_open_list = next_open_list[vertex_index.at(new_node)];
          if (new_v_next_open_list.size() < static_cast<std::size_t>(p_beam_width_)) {
            new_v_next_open_list.push(new_path);
          } else if (new_path > new_v_next_open_list.top()) {
            new_v_next_open_list.pop();
            new_v_next_open_list.push(new_path);
          } else {
            kept = false;
          }

          if (!updateBest(new_path, best, best_score, required_terminal_node) && !kept) {
            steps.discard();
          }
        }
      }
    }

    for (std::size_t index = 0; index < vertex_ids.size(); ++index) {
      open_list[index].clear();
      PathMinHeap& paths = next_open_list[index];
      while (!paths.empty()) {
        open_list[index].push_back(paths.top());
        paths.pop();
      }
    }
  }
}

}  // namespace path_planner
}  // namespace open_apero
