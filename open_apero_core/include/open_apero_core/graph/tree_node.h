#pragma once

/**
 * @file tree_node.h
 * @brief Tree-structured planning node with parent-child relationships and edge data.
 *
 * Extends Node with tree hierarchy (parent/children pointers), an edge to the
 * parent, and recursive tree traversal utilities.
 */

#include <memory>
#include <vector>

#include "open_apero_core/graph/edge.h"
#include "open_apero_core/graph/node.h"

namespace open_apero {

/**
 * @brief A node in the exploration tree, with parent/child links and an associated edge.
 *
 * Provides recursive tree traversal methods (children, leaves, subtree).
 */
struct TreeNode : public Node {
 public:
  /**
   * @brief Create a new child node owned by this node and return a pointer to it.
   * @return Pointer to the newly created child node.
   */
  TreeNode* spawnChild();

  /**
   * @brief Collect pointers to all immediate children of this node.
   * @param result Vector to which child pointers are appended.
   */
  void getChildren(std::vector<TreeNode*>* result);

  /**
   * @brief Recursively collect pointers to all leaf nodes (nodes with no children).
   * @param result Vector to which leaf node pointers are appended.
   */
  void getLeaves(std::vector<TreeNode*>* result);

  /**
   * @brief Recursively collect pointers to all nodes in the subtree rooted at this node.
   * @param result Vector to which node pointers are appended.
   */
  void getTree(std::vector<TreeNode*>* result);

  /**
   * @brief Recursively collect subtree node pointers up to a maximum depth.
   * @param result Vector to which node pointers are appended.
   * @param maxdepth Maximum recursion depth.
   */
  void getTree(std::vector<TreeNode*>* result, int maxdepth);

  /**
   * @brief Create a shallow copy of this tree node (excludes children and info).
   *
   * The copy keeps the edge and the parent pointer, so it names a parent that does not own it.
   * @return A unique pointer to the shallow-copied node.
   */
  std::unique_ptr<Node> shallowCopy() override;

 public:
  /// @brief Edge connecting this node to its parent.
  Edge edge;

  /// @brief Pointer to the parent node. Nullptr for the root node.
  TreeNode* parent = nullptr;

  /// @brief Owned child nodes.
  std::vector<std::unique_ptr<TreeNode>> children;
};

/**
 * @brief Move a tree node to a new parent, updating both old and new parent's children.
 * @param node Pointer to the node to move.
 * @param new_parent Pointer to the new parent node.
 * @return True if the move was successful.
 */
bool moveNodeToNewParent(TreeNode* node, TreeNode* new_parent);

/**
 * @brief Remove a tree node from its parent's children list, destroying it and its subtree.
 * @param node Pointer to the node to remove; dangling after a successful call.
 * @return True if the removal was successful.
 */
bool removeNodeFromParent(TreeNode* node);

}  // namespace open_apero
