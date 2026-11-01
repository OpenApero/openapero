#pragma once

/**
 * @file graph_node.h
 * @brief Graph node type and Boost graph typedefs for the exploration planner.
 *
 * Extends Node with graph-specific state (visited, reachability) and provides
 * the Boost adjacency_list graph type used throughout the planner.
 */

#include <memory>
#include <vector>

#include <boost/graph/adjacency_list.hpp>

#include "open_apero_core/graph/edge.h"
#include "open_apero_core/graph/node.h"

namespace open_apero {

/**
 * @brief A node in the exploration graph, extending Node with a reachability flag.
 */
struct GraphNode : public Node {
 public:
  /**
   * @brief Create a shallow copy of this graph node.
   * @return A unique pointer to the copied node.
   */
  std::unique_ptr<Node> shallowCopy() override;

 public:
  /// @brief Whether this node is reachable from the start node in the current graph.
  bool connect_with_start_node = false;
};

/// @brief Boost bidirectional adjacency list graph with GraphNode vertices and Edge edges.
using GraphType =
    boost::adjacency_list<boost::listS, boost::listS, boost::bidirectionalS, std::shared_ptr<GraphNode>, std::shared_ptr<Edge>>;

/// @brief Vertex descriptor type for GraphType.
using NodeId = GraphType::vertex_descriptor;

/// @brief Edge descriptor type for GraphType.
using EdgeId = GraphType::edge_descriptor;

/// @brief Sentinel value representing an invalid node ID.
const NodeId kInvalidNodeId = boost::graph_traits<GraphType>::null_vertex();

/**
 * @brief Look up the vertex descriptor for a given node in the graph (linear scan).
 * @param node Pointer to the node to look up.
 * @param graph Pointer to the graph to search.
 * @return The vertex descriptor, or kInvalidNodeId if either pointer is null or the node is not in the graph.
 */
NodeId getNodeId(const Node* node, const GraphType* graph);

/**
 * @brief Get the edge between two nodes in the graph.
 *
 * With a null @p graph both nodes must be TreeNodes, @p source the parent of @p target (throws
 * otherwise), and the child's own edge is returned.
 * @param source Pointer to the source node.
 * @param target Pointer to the target node.
 * @param graph Pointer to the graph (optional).
 * @return Pointer to the edge, or nullptr if not found.
 */
Edge* getEdge(Node* source, Node* target, const GraphType* graph = nullptr);

/**
 * @brief Run breadth-first search from a start node and return all reachable nodes.
 * @param start_node Pointer to the BFS starting node.
 * @param graph Pointer to the graph to traverse.
 * @return Vector of pointers to all nodes reachable from start_node.
 */
std::vector<Node*> runBfs(Node* start_node, const GraphType* graph);

}  // namespace open_apero
