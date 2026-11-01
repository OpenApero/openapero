#include "open_apero_core/plugin/path_planner/helpers.h"

#include <cstddef>
#include <limits>
#include <tuple>
#include <vector>

namespace open_apero {

std::vector<Node*> getAdjacentNodes(NodeId node_id, const GraphType* graph) {
  std::vector<Node*> adjacent;
  boost::graph_traits<GraphType>::adjacency_iterator it, end;
  for (std::tie(it, end) = boost::adjacent_vertices(node_id, *graph); it != end; ++it) {
    adjacent.push_back((*graph)[*it].get());
  }
  return adjacent;
}

void findShortestPaths(const NodeId start_node, std::unordered_map<NodeId, double>& distances,
                       std::unordered_map<NodeId, NodeId>& predecessors, GraphType* graph) {
  std::priority_queue<PqElement, std::vector<PqElement>, std::greater<PqElement>> pq;

  distances.clear();
  predecessors.clear();

  for (auto vp = boost::vertices(*graph); vp.first != vp.second; ++vp.first) {
    distances[*vp.first] = std::numeric_limits<double>::infinity();
    predecessors[*vp.first] = *vp.first;
  }

  distances[start_node] = 0.0;
  pq.push({start_node, 0.0});

  while (!pq.empty()) {
    NodeId current = pq.top().node_id;
    double current_distance = pq.top().distance;
    pq.pop();

    if (current_distance > distances[current]) {
      continue;
    }

    auto neighbors = boost::adjacent_vertices(current, *graph);
    for (auto it = neighbors.first; it != neighbors.second; ++it) {
      NodeId neighbor = *it;
      EdgeId edge = boost::edge(current, *it, *graph).first;

      double edge_cost = (*graph)[edge]->cost;
      double new_distance = distances[current] + edge_cost;

      if (new_distance < distances[neighbor]) {
        distances[neighbor] = new_distance;
        predecessors[neighbor] = current;
        pq.push({neighbor, new_distance});
      }
    }
  }
}

}  // namespace open_apero
