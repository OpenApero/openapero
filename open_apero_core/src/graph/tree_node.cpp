#include "open_apero_core/graph/tree_node.h"

#include <memory>
#include <stdexcept>
#include <vector>

namespace open_apero {

TreeNode* TreeNode::spawnChild() {
  children.push_back(std::make_unique<TreeNode>());
  children.back()->parent = this;
  return children.back().get();
}

void TreeNode::getChildren(std::vector<TreeNode*>* result) {
  for (int i = 0; i < static_cast<int>(children.size()); ++i) {
    result->push_back(children[i].get());
  }
}

void TreeNode::getLeaves(std::vector<TreeNode*>* result) {
  if (children.empty()) {
    result->push_back(this);
    return;
  }
  for (int i = 0; i < static_cast<int>(children.size()); ++i) {
    children[i]->getLeaves(result);
  }
}

void TreeNode::getTree(std::vector<TreeNode*>* result) {
  result->push_back(this);
  for (int i = 0; i < static_cast<int>(children.size()); ++i) {
    children[i]->getTree(result);
  }
}

void TreeNode::getTree(std::vector<TreeNode*>* result, int maxdepth) {
  result->push_back(this);
  if (maxdepth > 0) {
    for (int i = 0; i < static_cast<int>(children.size()); ++i) {
      children[i]->getTree(result, maxdepth - 1);
    }
  }
}

std::unique_ptr<Node> TreeNode::shallowCopy() {
  auto copy = std::make_unique<TreeNode>();
  copy->copyFieldsFrom(*this);
  copy->edge = edge;
  // The parent does not own the copy.
  copy->parent = parent;
  return copy;
}

bool moveNodeToNewParent(TreeNode* node, TreeNode* new_parent) {
  if (node->parent == new_parent) {
    return false;
  }

  // Sanity check
  bool is_node_found = false;

  auto old_parent = node->parent;
  for (int i = 0; i < static_cast<int>(old_parent->children.size()); ++i) {
    if (old_parent->children[i].get() == node) {
      new_parent->children.push_back(std::move(old_parent->children[i]));
      old_parent->children.erase(old_parent->children.begin() + i);
      node->parent = new_parent;

      is_node_found = true;
      break;
    }
  }

  // Sanity check
  if (!is_node_found) {
    throw std::runtime_error("moveNodeToNewParent: node is not a child of the old parent which its parent pointer points to.");
  }

  return true;
}

bool removeNodeFromParent(TreeNode* node) {
  if (!node->parent) {
    return false;
  }

  // Sanity check
  bool is_node_found = false;

  auto parent = node->parent;
  for (int i = 0; i < static_cast<int>(parent->children.size()); ++i) {
    if (parent->children[i].get() == node) {
      parent->children.erase(parent->children.begin() + i);

      is_node_found = true;
      break;
    }
  }

  // Sanity check
  if (!is_node_found) {
    throw std::runtime_error("removeNodeFromParent: node is not a child of the parent which its parent pointer points to.");
  }

  return true;
}
}  // namespace open_apero
