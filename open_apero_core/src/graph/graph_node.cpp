#include "open_apero_core/graph/graph_node.h"

#include <queue>
#include <stdexcept>
#include <unordered_set>

#include "open_apero_core/graph/tree_node.h"

namespace open_apero {

// Preserve node data but not graph connectivity.
std::unique_ptr<Node> GraphNode::shallowCopy() {
  auto copy = std::make_unique<GraphNode>();
  copy->copyFieldsFrom(*this);
  copy->connect_with_start_node = connect_with_start_node;
  return copy;
}

NodeId getNodeId(const Node* node, const GraphType* graph) {
  if (node == nullptr || graph == nullptr) {
    return kInvalidNodeId;
  }

  auto vertices = boost::vertices(*graph);
  for (auto it = vertices.first; it != vertices.second; ++it) {
    if ((*graph)[*it].get() == node) {
      return *it;
    }
  }
  return kInvalidNodeId;
}

Edge* getEdge(Node* source, Node* target, const GraphType* graph) {
  // If the graph is nullptr, then source and target must be tree node
  if (graph == nullptr) {
    auto source_tree_node = source->as<TreeNode>();
    if (source_tree_node == nullptr) {
      throw std::runtime_error("getEdge called with nullptr graph and non-TreeNode source");
    }

    auto target_tree_node = target->as<TreeNode>();
    if (target_tree_node == nullptr) {
      throw std::runtime_error("getEdge called with nullptr graph and non-TreeNode target");
    }

    if (source_tree_node != target_tree_node->parent) {
      throw std::runtime_error("getEdge called with non-parent-child nodes");
    }

    return &(target_tree_node->edge);
  }

  if (source == nullptr || target == nullptr) {
    return nullptr;
  }

  auto vertices = boost::vertices(*graph);
  NodeId source_id = kInvalidNodeId;
  NodeId target_id = kInvalidNodeId;
  for (auto it = vertices.first; it != vertices.second; ++it) {
    auto node = (*graph)[*it].get();
    if (node == source) {
      source_id = *it;
    } else if (node == target) {
      target_id = *it;
    }
  }

  if (source_id == kInvalidNodeId || target_id == kInvalidNodeId) {
    return nullptr;
  }

  auto edge_id_existance = boost::edge(source_id, target_id, *graph);
  if (edge_id_existance.second) {
    return (*graph)[edge_id_existance.first].get();
  } else {
    return nullptr;
  }
}

std::vector<Node*> runBfs(Node* start_node, const GraphType* graph) {
  if (!graph) {
    throw std::runtime_error("Graph is nullptr");
  }

  auto current_id = getNodeId(start_node, graph);
  if (current_id == kInvalidNodeId) {
    throw std::runtime_error("Current node not found in the graph");
  }

  std::queue<NodeId> bfs_queue;
  bfs_queue.push(current_id);
  std::unordered_set<NodeId> visited;
  visited.insert(current_id);

  while (!bfs_queue.empty()) {
    NodeId current = bfs_queue.front();
    bfs_queue.pop();

    // Get neighbors of the current node
    auto neighbors = boost::adjacent_vertices(current, *graph);
    for (auto it = neighbors.first; it != neighbors.second; ++it) {
      NodeId neighbor = *it;

      // If the neighbor has not been visited, mark it as connected and add to queue
      if (visited.find(neighbor) == visited.end()) {
        visited.insert(neighbor);
        bfs_queue.push(neighbor);
      }
    }
  }

  // Map the visited nodes to the actual nodes
  std::vector<Node*> bfs_visited_nodes;
  for (auto it = visited.begin(); it != visited.end(); ++it) {
    bfs_visited_nodes.push_back((*graph)[*it].get());
  }
  return bfs_visited_nodes;
}

}  // namespace open_apero
