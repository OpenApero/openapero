/**
 * @file test_node_copy.cpp
 * @brief Node copy semantics for payloads that own sub-nodes, and for the per-node flags.
 *
 * Copying a Node deep-copies its info payload; YawPlanningInfo must clone its orientation nodes
 * without slicing their dynamic type or sharing mutable state. sampling_role, is_observed and
 * is_gain_suppressed survive every copy path, and each has its default.
 */

#include <memory>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "open_apero_core/graph/graph_node.h"
#include "open_apero_core/graph/node.h"
#include "open_apero_core/graph/tree_node.h"

namespace open_apero {
namespace {

/// @brief Payload carrying a marker, to check nested info is copied not shared.
struct MarkerInfo : public NodeInfo {
  int marker = 0;
  std::unique_ptr<NodeInfo> deepCopy() const override { return std::make_unique<MarkerInfo>(*this); }
};

/// @brief A YawPlanningInfo with one TreeNode and one GraphNode orientation,
/// a null slot, a visited mask and a selected index.
std::unique_ptr<YawPlanningInfo> makeYawPlanningInfo() {
  auto info = std::make_unique<YawPlanningInfo>();

  auto tree_orientation = std::make_unique<TreeNode>();
  tree_orientation->robot_state.setSensorPosition(Eigen::Vector3d(1.0, 2.0, 3.0));
  tree_orientation->is_traversable = true;
  auto marker = std::make_unique<MarkerInfo>();
  marker->marker = 42;
  tree_orientation->info = std::move(marker);

  auto graph_orientation = std::make_unique<GraphNode>();
  graph_orientation->is_at_frontier = true;
  graph_orientation->sampling_role = SamplingRole::kFrontier;

  info->orientations.push_back(std::move(tree_orientation));
  info->orientations.push_back(std::move(graph_orientation));
  info->orientations.push_back(nullptr);
  info->visited_orientation = {true, false, true};
  info->active_orientation = 1;
  return info;
}

TEST(YawPlanningInfoDeepCopy, DefaultActiveOrientationIsNotAnIndex) {
  // An indeterminate value here would index orientations out of bounds.
  YawPlanningInfo info;
  EXPECT_LT(info.active_orientation, 0);
}

TEST(YawPlanningInfoDeepCopy, KeepsThePayloadType) {
  Node original;
  original.info = makeYawPlanningInfo();

  const Node copy(original);
  ASSERT_NE(copy.info, nullptr);
  EXPECT_NE(copy.info->as<YawPlanningInfo>(), nullptr) << "the payload was sliced to a plain NodeInfo";
}

TEST(YawPlanningInfoDeepCopy, PreservesOrientationsMaskAndIndex) {
  Node original;
  original.info = makeYawPlanningInfo();

  const Node copy(original);
  const auto* copied = copy.info->as<YawPlanningInfo>();
  ASSERT_NE(copied, nullptr);

  ASSERT_EQ(copied->orientations.size(), 3u);
  EXPECT_EQ(copied->visited_orientation, std::vector<bool>({true, false, true}));
  EXPECT_EQ(copied->active_orientation, 1);
  EXPECT_EQ(copied->orientations[2], nullptr) << "a null slot must stay null so indices still line up";
}

TEST(YawPlanningInfoDeepCopy, PreservesEachOrientationsDynamicType) {
  Node original;
  original.info = makeYawPlanningInfo();

  const Node copy(original);
  const auto* copied = copy.info->as<YawPlanningInfo>();
  ASSERT_NE(copied, nullptr);

  // make_unique<Node>(*orientation) would slice both of these to plain Nodes.
  EXPECT_NE(copied->orientations[0]->as<TreeNode>(), nullptr);
  EXPECT_NE(copied->orientations[1]->as<GraphNode>(), nullptr);
  EXPECT_TRUE(copied->orientations[0]->is_traversable);
  EXPECT_TRUE(copied->orientations[1]->is_at_frontier);
}

// sampling_role is provenance rather than derived state: an evaluator keyed on it
// misroutes every copied node if a copy path forgets to carry it over.
TEST(NodeSamplingRole, SurvivesEveryCopyPath) {
  Node node;
  node.sampling_role = SamplingRole::kFrontier;
  GraphNode graph_node;
  graph_node.sampling_role = SamplingRole::kGain;
  TreeNode tree_node;
  tree_node.sampling_role = SamplingRole::kFrontier;

  const Node constructed(node);
  EXPECT_EQ(constructed.sampling_role, SamplingRole::kFrontier);

  Node assigned;
  assigned = node;
  EXPECT_EQ(assigned.sampling_role, SamplingRole::kFrontier);

  EXPECT_EQ(node.shallowCopy()->sampling_role, SamplingRole::kFrontier);
  EXPECT_EQ(graph_node.shallowCopy()->sampling_role, SamplingRole::kGain);
  EXPECT_EQ(tree_node.shallowCopy()->sampling_role, SamplingRole::kFrontier);
}

// is_observed permanently exempts a node from re-evaluation, so a copy path that
// dropped it would resurrect ray casts already ruled out -- and one that invented it
// would silently freeze a node whose viewpoint nobody has seen.
TEST(NodeObservedViewpoint, SurvivesEveryCopyPath) {
  Node node;
  node.is_observed = true;
  GraphNode graph_node;
  graph_node.is_observed = true;
  TreeNode tree_node;
  tree_node.is_observed = true;

  const Node constructed(node);
  EXPECT_TRUE(constructed.is_observed);

  Node assigned;
  assigned = node;
  EXPECT_TRUE(assigned.is_observed);

  EXPECT_TRUE(node.shallowCopy()->is_observed);
  EXPECT_TRUE(graph_node.shallowCopy()->is_observed);
  EXPECT_TRUE(tree_node.shallowCopy()->is_observed);
}

// A node inserted into a graph must start out re-evaluatable, or it would carry the
// evaluator's permanent skip from birth.
TEST(NodeObservedViewpoint, DefaultsToUnobserved) {
  EXPECT_FALSE(Node().is_observed);
  EXPECT_FALSE(GraphNode().is_observed);
  EXPECT_FALSE(TreeNode().is_observed);
}

// is_gain_suppressed is rewritten before every search; a copy carries the verdict it was taken with.
TEST(NodeGainSuppression, SurvivesEveryCopyPath) {
  Node node;
  node.is_gain_suppressed = true;
  GraphNode graph_node;
  graph_node.is_gain_suppressed = true;
  TreeNode tree_node;
  tree_node.is_gain_suppressed = true;

  const Node constructed(node);
  EXPECT_TRUE(constructed.is_gain_suppressed);

  Node assigned;
  assigned = node;
  EXPECT_TRUE(assigned.is_gain_suppressed);

  EXPECT_TRUE(node.shallowCopy()->is_gain_suppressed);
  EXPECT_TRUE(graph_node.shallowCopy()->is_gain_suppressed);
  EXPECT_TRUE(tree_node.shallowCopy()->is_gain_suppressed);
}

TEST(NodeGainSuppression, DefaultsToNotSuppressed) {
  EXPECT_FALSE(Node().is_gain_suppressed);
  EXPECT_FALSE(GraphNode().is_gain_suppressed);
  EXPECT_FALSE(TreeNode().is_gain_suppressed);
}

TEST(NodeSamplingRole, DefaultsToTheDefaultRole) {
  // Single-population generators leave the role untouched, so kDefault must be the default.
  EXPECT_EQ(Node().sampling_role, SamplingRole::kDefault);
  EXPECT_EQ(GraphNode().sampling_role, SamplingRole::kDefault);
  EXPECT_EQ(TreeNode().sampling_role, SamplingRole::kDefault);
}

TEST(YawPlanningInfoDeepCopy, OrientationsAreIndependentOfTheOriginal) {
  Node original;
  original.info = makeYawPlanningInfo();

  Node copy(original);
  auto* copied = copy.info->as<YawPlanningInfo>();
  ASSERT_NE(copied, nullptr);

  // Distinct objects, and a nested payload that is cloned rather than shared.
  auto* source = original.info->as<YawPlanningInfo>();
  ASSERT_NE(source, nullptr);
  EXPECT_NE(copied->orientations[0].get(), source->orientations[0].get());

  const auto* copied_marker = copied->orientations[0]->info->as<MarkerInfo>();
  ASSERT_NE(copied_marker, nullptr);
  EXPECT_EQ(copied_marker->marker, 42);

  copied->orientations[0]->robot_state.setSensorPosition(Eigen::Vector3d(9.0, 9.0, 9.0));
  copied->orientations[0]->info->as<MarkerInfo>()->marker = 7;
  EXPECT_NEAR(source->orientations[0]->robot_state.getSensorPosition().x(), 1.0, 1e-12);
  EXPECT_EQ(source->orientations[0]->info->as<MarkerInfo>()->marker, 42);
}

// Assignment shares Node's deep-copy path with construction; both must be covered.
TEST(YawPlanningInfoDeepCopy, CopyAssignmentDeepCopiesToo) {
  Node original;
  original.info = makeYawPlanningInfo();

  Node assigned;
  assigned = original;

  const auto* copied = assigned.info->as<YawPlanningInfo>();
  ASSERT_NE(copied, nullptr);
  ASSERT_EQ(copied->orientations.size(), 3u);
  EXPECT_NE(copied->orientations[0]->as<TreeNode>(), nullptr);
  EXPECT_NE(copied->orientations[0].get(), original.info->as<YawPlanningInfo>()->orientations[0].get());
}

// The shallow-copy contract excludes owned children; cloning must not resurrect
// them, or a copy would own a subtree the original still believes it owns.
TEST(YawPlanningInfoDeepCopy, DoesNotCloneOwnedTreeChildren) {
  auto info = std::make_unique<YawPlanningInfo>();
  auto tree_orientation = std::make_unique<TreeNode>();
  tree_orientation->spawnChild();
  tree_orientation->spawnChild();
  std::vector<TreeNode*> original_children;
  tree_orientation->getChildren(&original_children);
  ASSERT_EQ(original_children.size(), 2u);
  info->orientations.push_back(std::move(tree_orientation));

  Node original;
  original.info = std::move(info);
  const Node copy(original);

  auto* copied_tree = copy.info->as<YawPlanningInfo>()->orientations[0]->as<TreeNode>();
  ASSERT_NE(copied_tree, nullptr);
  std::vector<TreeNode*> copied_children;
  copied_tree->getChildren(&copied_children);
  EXPECT_TRUE(copied_children.empty());
}

}  // namespace
}  // namespace open_apero
