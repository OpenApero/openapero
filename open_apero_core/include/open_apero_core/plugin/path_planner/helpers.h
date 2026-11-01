#pragma once

/**
 * @file helpers.h
 * @brief Helper utilities for graph-based path-planner search algorithms.
 */

#include <algorithm>
#include <functional>
#include <queue>
#include <unordered_map>
#include <vector>

#include "open_apero_core/gain/gain_anchor.h"
#include "open_apero_core/graph/graph_node.h"
#include "open_apero_core/graph/path.h"

namespace open_apero {

/**
 * @brief Element for a priority queue used in shortest path computations.
 */
struct PqElement {
  NodeId node_id;
  double distance;
  bool operator>(const PqElement& other) const { return distance > other.distance; }
};

/**
 * @brief The nodes one edge out from @p node_id, in the graph's own adjacency order (unsorted,
 *        unfiltered, so a seeded fallback pick reproduces).
 * @param node_id Vertex to take the out-neighbors of.
 * @param graph The graph to read.
 * @return The adjacent nodes, empty when the vertex has no outgoing edges.
 */
std::vector<Node*> getAdjacentNodes(NodeId node_id, const GraphType* graph);

/**
 * @brief Compute single-source shortest paths from a start node using Dijkstra's algorithm.
 * @param start_node The starting node ID.
 * @param distances Output map of node IDs to shortest distances.
 * @param predecessors Output map of node IDs to predecessor node IDs.
 * @param graph The graph to search.
 */
void findShortestPaths(const NodeId start_node, std::unordered_map<NodeId, double>& distances,
                       std::unordered_map<NodeId, NodeId>& predecessors, GraphType* graph);

}  // namespace open_apero
