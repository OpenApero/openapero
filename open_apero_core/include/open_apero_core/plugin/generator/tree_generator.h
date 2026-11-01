#pragma once

/**
 * @file tree_generator.h
 * @brief Base class for tree-based node/edge generators.
 */

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "open_apero_core/agent/agent_interface.h"
#include "open_apero_core/common/typedef.h"
#include "open_apero_core/common/utils.h"
#include "open_apero_core/geometry/bounding_volume.h"
#include "open_apero_core/graph/tree_node.h"
#include "open_apero_core/plugin/generator/graph_generator.h"
#include "open_apero_core/plugin/plugin_registry.h"
#include "open_apero_core/plugin/robot_model/robot_model.h"

namespace open_apero {

/**
 * @brief Base tree-structured node/edge generator managing a rooted exploration tree.
 */
class TreeGenerator : public GraphGenerator {
 public:
  /** @brief Constructs the TreeGenerator and binds it to the agent. */
  explicit TreeGenerator(AgentInterface& agent);  // NOLINT(google-runtime-references) non-const ref to agent is intentional

  virtual ~TreeGenerator() = default;

  /** @brief Resets tree state for a new episode. */
  void resetEpisode() override {
    GraphGenerator::resetEpisode();
    root_node_.reset();
  }

  /**
   * @brief Returns every node of the tree, from its root.
   * @param start_node Unused.
   * @return Flat list of all node pointers in the tree.
   */
  std::vector<Node*> getAllNodes(Node* start_node) const override;

  /**
   * @brief Returns every edge of the tree, from its root.
   * @param start_node Unused.
   * @return Flat list of all edge pointers in the tree.
   */
  std::vector<Edge*> getAllEdges(Node* start_node) const override;

  /**
   * @brief Creates and initializes the root node of the tree at the robot's initial state.
   * @param init_state The robot's starting pose.
   * @return Pair of (graph pointer, root node pointer).
   */
  std::pair<GraphType*, Node*> generateInitNode(const RobotState& init_state) override;

  /**
   * @brief Connects a newly generated node cluster into the tree by creating edges.
   * @param node_cluster Cluster of candidate nodes to attach.
   * @param new_edges    Output list populated with the newly created edges.
   * @return true on success.
   */
  bool generateNewEdges(NodeClusterPtrs& node_cluster, std::vector<Edge*>& new_edges) override;

  /**
   * @brief Inserts an intermediate waypoint node along an existing edge.
   * @param start_node The edge's parent node.
   * @param goal_node  The edge's child node.
   * @param edge       The edge to split.
   * @param node_index Index of the trajectory waypoint at which to insert.
   * @return Pair of (new intermediate node, the edge from start_node to it).
   */
  std::pair<Node*, Edge*> insertIntermediateNode(Node* start_node, Node* goal_node, Edge* edge, int node_index) override;

  /**
   * @brief Insert a new intermediate node at the endpoint of @p executed_edge as the new tree root.
   *
   * Never returns nullptr: when the edge back to @p previous cannot be planned, the tree is
   * dropped for a fresh single-node root at the reached state.
   *
   * @param previous Existing tree root node; becomes a child of the new root after rerooting.
   * @param executed_edge Edge whose endpoint gives the new root's robot state.
   */
  Node* insertIntermediateNodeAtCurrentState(Node* previous, const Edge& executed_edge) override;

  bool containsNode(const Node* node) const override;

  /**
   * @brief Revalidates the edges a map update can affect and drops the subtrees behind untraversable ones.
   * @param start_node Root of the tree to update.
   */
  void updateNodeEdge(Node* start_node) override;

  /**
   * @brief Changes the sensor/heading orientation of a node and updates connected edges.
   * @param node        The node whose orientation is being changed.
   * @param orientation The new sensor orientation as a quaternion.
   * @return true if the orientation change was accepted (collision-free).
   */
  bool setNodeOrientation(Node* node, const Eigen::Quaterniond& orientation) override;

  /** @brief Tree edges are unidirectional (directed from parent to child). */
  bool isEdgeBidirectional() const override { return false; }

 protected:
  /**
   * @brief Rebuilds the kd-tree index from the current set of tree nodes.
   * @return true on success.
   */
  bool rebuildKdTree() override;

  /** @brief Recursive helper for updateNodeEdge (prunes without rebuilding KD-tree). */
  void updateNodeEdgeRecursive(Node* node);

  std::unique_ptr<TreeNode> root_node_;  // root node of the tree
};

}  // namespace open_apero
