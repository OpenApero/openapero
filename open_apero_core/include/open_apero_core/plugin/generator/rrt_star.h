#pragma once

/**
 * @file rrt_star.h
 * @brief RRT* node/edge generator with tree rewiring.
 */

#include <memory>
#include <string>
#include <vector>

#include "open_apero_core/plugin/evaluator/evaluator.h"
#include "open_apero_core/plugin/generator/rrt.h"

namespace open_apero {

namespace generator {

/**
 * @brief Extension of RRT (RRAT* in the paper) that rewires tree segments for cheaper paths.
 */
class RRTStar : public RRT {
 public:
  /** @brief Constructs the RRT* generator and binds it to the agent. */
  explicit RRTStar(AgentInterface& agent);  // NOLINT(google-runtime-references) non-const ref to agent is intentional

  /**
   * @brief Generates edges for a node cluster and rewires nearby nodes to cheaper parents.
   * @param node_cluster Cluster of newly generated nodes to connect into the tree.
   * @param new_edges    Output list populated with the newly created edges.
   * @return true on success.
   */
  bool generateNewEdges(NodeClusterPtrs& node_cluster, std::vector<Edge*>& new_edges) override;

  /**
   * @brief Updates the tree root when the robot moves to a new start node.
   * @param previous_start The old root node being replaced.
   * @param new_start      The new root node after the robot has moved.
   */
  void setNewStartNode(Node* previous_start, Node* new_start) override;

  /**
   * @brief Rewires the entire tree rooted at @p root to ensure optimal parent assignments.
   * @param root The root of the tree to rewire.
   */
  void rewireTree(TreeNode* root);

 protected:
  static PluginRegistry::Registration<RRTStar> registration;

  /**
   * @brief Attempts to reconnect @p node to the cheapest reachable parent in @p candidates.
   *
   * The candidates list may include the node's current parent. If @p force_rewire is true,
   * the method returns true as long as it successfully connects to any candidate (including
   * the original parent).
   *
   * @param node         The node whose parent is being reconsidered.
   * @param candidates   Set of potential new parent nodes.
   * @param force_rewire If true, accept the original parent as a valid rewire target.
   * @return true if the node was successfully rewired to a new parent.
   */
  bool rewireToBestParent(Node* node, const std::vector<Node*>& candidates, bool force_rewire = false);

 private:
  /** @brief Removes all intermediate nodes from the subtree rooted at @p node. */
  void removeIntermediateNodes(TreeNode* node);
};

}  // namespace generator

}  // namespace open_apero
