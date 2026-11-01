/**
 * @file test_gain_suppression.cpp
 * @brief Tests of suppressNonMaximumClusters(): greedy non-maximum suppression over node clusters.
 *
 * Clusters are the nodes sharing one sensor position, scored by their best traversable node.
 */

#include <memory>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "open_apero_core/gain/gain_anchor.h"
#include "open_apero_core/gain/gain_suppression.h"
#include "open_apero_core/graph/graph_node.h"

namespace open_apero {
namespace {

constexpr double kRadius = 1.5;

class GainSuppressionTest : public ::testing::Test {
 protected:
  /// @brief A node at (@p x, 0, 0) carrying one anchor of @p gain; gain 0 carries none.
  Node* addNode(double x, double gain, bool is_traversable = true) {
    auto node = std::make_shared<GraphNode>();
    node->robot_state.setSensorPosition(Eigen::Vector3d(x, 0.0, 0.0));
    node->is_traversable = is_traversable;
    if (gain > 0.0) {
      node->gain_anchors = {std::make_shared<GainAnchor>(gain)};
    }
    owned_.push_back(node);
    nodes_.push_back(node.get());
    return node.get();
  }

  void suppress(const Node* start_node = nullptr, double radius = kRadius) { suppressNonMaximumClusters(nodes_, start_node, radius); }

  std::vector<std::shared_ptr<GraphNode>> owned_;
  std::vector<Node*> nodes_;
};

TEST_F(GainSuppressionTest, RadiusZeroSuppressesNothingAndClearsEveryFlag) {
  Node* best = addNode(0.0, 10.0);
  Node* near = addNode(1.0, 5.0);
  near->is_gain_suppressed = true;

  suppress(nullptr, 0.0);

  EXPECT_FALSE(best->is_gain_suppressed);
  EXPECT_FALSE(near->is_gain_suppressed);
}

TEST_F(GainSuppressionTest, TheBestClusterSuppressesTheClustersWithinTheRadius) {
  Node* weaker = addNode(1.0, 5.0);
  Node* best = addNode(0.0, 10.0);
  Node* beyond = addNode(-2.0, 4.0);

  suppress();

  EXPECT_FALSE(best->is_gain_suppressed);
  EXPECT_TRUE(weaker->is_gain_suppressed);
  EXPECT_FALSE(beyond->is_gain_suppressed);
}

TEST_F(GainSuppressionTest, AClusterExactlyAtTheRadiusIsKept) {
  Node* best = addNode(0.0, 10.0);
  Node* at_radius = addNode(kRadius, 5.0);

  suppress();

  EXPECT_FALSE(best->is_gain_suppressed);
  EXPECT_FALSE(at_radius->is_gain_suppressed);
}

// B lies between A and C, within the radius of both: a suppressed cluster suppresses nothing.
TEST_F(GainSuppressionTest, ASuppressedClusterSuppressesNothing) {
  Node* a = addNode(0.0, 5.0);
  Node* b = addNode(1.4, 6.0);
  Node* c = addNode(2.8, 7.0);

  suppress();

  EXPECT_FALSE(c->is_gain_suppressed);
  EXPECT_TRUE(b->is_gain_suppressed);
  EXPECT_FALSE(a->is_gain_suppressed) << "only the suppressed B lies within the radius of A";
}

TEST_F(GainSuppressionTest, AClusterIsScoredByItsBestTraversableNode) {
  Node* sharp_view = addNode(0.0, 9.0);
  Node* sharp_rest = addNode(0.0, 2.0);
  Node* broad_first = addNode(1.0, 8.0);
  Node* broad_second = addNode(1.0, 8.0);
  Node* broad_blocked = addNode(1.0, 20.0, /*is_traversable=*/false);

  suppress();

  EXPECT_FALSE(sharp_view->is_gain_suppressed);
  EXPECT_FALSE(sharp_rest->is_gain_suppressed);
  EXPECT_TRUE(broad_first->is_gain_suppressed) << "8 + 8 outweighs 9, but the best view decides";
  EXPECT_TRUE(broad_second->is_gain_suppressed);
  EXPECT_TRUE(broad_blocked->is_gain_suppressed) << "an untraversable node neither scores nor escapes its cluster's verdict";
}

TEST_F(GainSuppressionTest, NodesOfOneClusterNeverSuppressEachOther) {
  Node* first = addNode(0.0, 10.0);
  Node* second = addNode(0.0, 3.0);
  Node* third = addNode(0.0, 1.0);

  suppress();

  EXPECT_FALSE(first->is_gain_suppressed);
  EXPECT_FALSE(second->is_gain_suppressed);
  EXPECT_FALSE(third->is_gain_suppressed);
}

TEST_F(GainSuppressionTest, AZeroGainClusterNeitherSuppressesNorIsSuppressed) {
  Node* waypoint = addNode(0.0, 0.0);
  Node* frontier = addNode(1.2, 10.0);

  suppress();

  EXPECT_FALSE(frontier->is_gain_suppressed);
  EXPECT_FALSE(waypoint->is_gain_suppressed);
}

TEST_F(GainSuppressionTest, TheStartNodeIsNeitherScoredNorSuppressed) {
  Node* start = addNode(0.0, 100.0);
  Node* start_cluster_mate = addNode(0.0, 1.0);
  Node* neighbor = addNode(1.0, 5.0);

  suppress(start);

  EXPECT_FALSE(start->is_gain_suppressed);
  EXPECT_TRUE(start_cluster_mate->is_gain_suppressed) << "without the start the cluster scores 1";
  EXPECT_FALSE(neighbor->is_gain_suppressed);
}

// Intermediate nodes are inserted at the robot's state or a replan horizon and carry no gain anchors.
TEST_F(GainSuppressionTest, AnIntermediateNodeTakesNoPart) {
  Node* kept = addNode(0.0, 10.0);
  Node* intermediate = addNode(1.0, 0.0);
  intermediate->is_intermediate = true;
  Node* beyond = addNode(2.0, 4.0);

  suppress();

  EXPECT_FALSE(intermediate->is_gain_suppressed);
  EXPECT_FALSE(beyond->is_gain_suppressed) << "the intermediate node 1 m from it suppresses nothing";
  EXPECT_FALSE(kept->is_gain_suppressed);
}

// The robot stands on one node of a cluster: that node is the start, its cluster mates still compete.
TEST_F(GainSuppressionTest, TheRobotsClusterCompetesThroughItsOtherNodes) {
  Node* start = addNode(0.0, 0.0);
  Node* turn_in_place = addNode(0.0, 10.0);
  Node* near = addNode(0.7, 5.0);

  suppress(start);

  EXPECT_FALSE(turn_in_place->is_gain_suppressed);
  EXPECT_TRUE(near->is_gain_suppressed);
}

TEST_F(GainSuppressionTest, TheRobotsClusterWithNothingLeftSuppressesNothing) {
  Node* start = addNode(0.0, 0.0);
  Node* spent_mate = addNode(0.0, 0.0);
  Node* near = addNode(0.7, 5.0);

  suppress(start);

  EXPECT_FALSE(near->is_gain_suppressed) << "the first node may lie 0.7 m from the robot";
  EXPECT_FALSE(spent_mate->is_gain_suppressed);
}

TEST_F(GainSuppressionTest, ATieKeepsTheClusterMetFirst) {
  Node* first = addNode(0.0, 5.0);
  Node* second = addNode(1.0, 5.0);

  suppress();

  EXPECT_FALSE(first->is_gain_suppressed);
  EXPECT_TRUE(second->is_gain_suppressed);
}

TEST_F(GainSuppressionTest, RunningAgainDecidesFromTheGainsHeldNow) {
  Node* left = addNode(0.0, 10.0);
  Node* right = addNode(1.0, 5.0);
  suppress();
  ASSERT_TRUE(right->is_gain_suppressed);

  left->gain_anchors.front()->gain = 1.0;
  suppress();

  EXPECT_TRUE(left->is_gain_suppressed);
  EXPECT_FALSE(right->is_gain_suppressed);
}

}  // namespace
}  // namespace open_apero
