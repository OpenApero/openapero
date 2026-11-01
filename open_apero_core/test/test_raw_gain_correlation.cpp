/**
 * @file test_raw_gain_correlation.cpp
 * @brief Tests that setupRaw() configures correlated gain for every planner that reads it.
 *
 * Checked by behavior rather than a getter: the flag must reach the gain computation.
 */

#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "open_apero_core/agent/agent_interface.h"
#include "open_apero_core/graph/graph_node.h"
#include "open_apero_core/plugin/path_planner/graph_depth_wise_beam_search.h"
#include "open_apero_core/plugin/path_planner/graph_node_wise_beam_search.h"
#include "open_apero_core/plugin/path_planner/graph_shortest_path_tree_search.h"
#include "plugin_test_stubs.h"

namespace open_apero {
namespace {

/**
 * @brief A chain start -> near -> far whose two gainful nodes see one big thing in common.
 *
 * The shared anchor is worth 10 and each node has a private anchor worth 1, so both nodes report a single-node
 * gain of 11. A path over both is worth 22 if the shared thing is counted twice and 12 if it is counted once
 * -- far enough apart that no tie-break or rounding can be mistaken for the flag doing its job.
 */
class RawGainCorrelationTest : public ::testing::Test {
 protected:
  static constexpr double kUncorrelatedPathGain = 22.0;
  static constexpr double kCorrelatedPathGain = 12.0;

  void SetUp() override {
    const GainAnchorPtr shared = std::make_shared<GainAnchor>(10.0);

    start_ = addNode({});
    near_ = addNode({shared, std::make_shared<GainAnchor>(1.0)});
    far_ = addNode({shared, std::make_shared<GainAnchor>(1.0)});

    addEdge(0, 1);
    addEdge(1, 2);
  }

  Node* addNode(const GainAnchorPtrVector& anchors) {
    auto node = std::make_shared<GraphNode>();
    node->is_traversable = true;
    node->gain_anchors = anchors;
    ids_.push_back(boost::add_vertex(node, graph_));
    return node.get();
  }

  void addEdge(std::size_t from, std::size_t to) {
    auto edge = std::make_shared<Edge>();
    edge->cost = 1.0;
    edge->is_traversable = true;
    boost::add_edge(ids_[from], ids_[to], edge, graph_);
  }

  /// @brief The whole chain, so the score being compared really is the two-node path's.
  void expectWalkedTheWholeChain(const std::vector<Node*>& path) {
    ASSERT_EQ(path.size(), 3u);
    EXPECT_EQ(path.front(), start_);
    EXPECT_EQ(path.back(), far_);
  }

  SearchOnlyAgent agent_;
  GraphType graph_;
  std::vector<NodeId> ids_;
  Node* start_ = nullptr;
  Node* near_ = nullptr;
  Node* far_ = nullptr;
};

TEST_F(RawGainCorrelationTest, NodeWiseBeamSearchHonorsTheFlagFromSetupRaw) {
  path_planner::GraphNodeWiseBeamSearch uncorrelated(agent_);
  uncorrelated.setupRaw(Objective::kPathGain, 4, 4, /*check_previous_path=*/false);
  const std::pair<std::vector<Node*>, double> loose = uncorrelated.planPath(start_, &graph_, nullptr);
  expectWalkedTheWholeChain(loose.first);
  EXPECT_DOUBLE_EQ(loose.second, kUncorrelatedPathGain);

  path_planner::GraphNodeWiseBeamSearch correlated(agent_);
  correlated.setupRaw(Objective::kPathGain, 4, 4, /*check_previous_path=*/false, /*is_gain_correlated=*/true);
  const std::pair<std::vector<Node*>, double> tight = correlated.planPath(start_, &graph_, nullptr);
  expectWalkedTheWholeChain(tight.first);
  EXPECT_DOUBLE_EQ(tight.second, kCorrelatedPathGain);
}

TEST_F(RawGainCorrelationTest, DepthWiseBeamSearchHonorsTheFlagFromSetupRaw) {
  path_planner::GraphDepthWiseBeamSearch uncorrelated(agent_);
  uncorrelated.setupRaw(Objective::kPathGain, 4, 4, /*check_previous_path=*/false);
  const std::pair<std::vector<Node*>, double> loose = uncorrelated.planPath(start_, &graph_, nullptr);
  expectWalkedTheWholeChain(loose.first);
  EXPECT_DOUBLE_EQ(loose.second, kUncorrelatedPathGain);

  path_planner::GraphDepthWiseBeamSearch correlated(agent_);
  correlated.setupRaw(Objective::kPathGain, 4, 4, /*check_previous_path=*/false, /*is_gain_correlated=*/true);
  const std::pair<std::vector<Node*>, double> tight = correlated.planPath(start_, &graph_, nullptr);
  expectWalkedTheWholeChain(tight.first);
  EXPECT_DOUBLE_EQ(tight.second, kCorrelatedPathGain);
}

TEST_F(RawGainCorrelationTest, ShortestPathTreeSearchHonorsTheFlagFromSetupRaw) {
  // top_node_percentage 1.0 nominates every node as a goal, so the far end of the chain is reachable and the
  // comparison is about the flag rather than about which goals were proposed.
  path_planner::GraphShortestPathTreeSearch uncorrelated(agent_);
  uncorrelated.setupRaw(Objective::kPathGain, /*top_node_percentage=*/1.0);
  const std::pair<std::vector<Node*>, double> loose = uncorrelated.planPath(start_, &graph_, nullptr);
  expectWalkedTheWholeChain(loose.first);
  EXPECT_DOUBLE_EQ(loose.second, kUncorrelatedPathGain);

  path_planner::GraphShortestPathTreeSearch correlated(agent_);
  correlated.setupRaw(Objective::kPathGain, /*top_node_percentage=*/1.0, /*is_gain_correlated=*/true);
  const std::pair<std::vector<Node*>, double> tight = correlated.planPath(start_, &graph_, nullptr);
  expectWalkedTheWholeChain(tight.first);
  EXPECT_DOUBLE_EQ(tight.second, kCorrelatedPathGain);
}

// The default stays uncorrelated: a setupRaw() call that omits the flag must keep its numbers.
TEST_F(RawGainCorrelationTest, OmittingTheFlagLeavesGainUncorrelated) {
  path_planner::GraphNodeWiseBeamSearch search(agent_);
  search.setupRaw(Objective::kPathGain, 4, 4, /*check_previous_path=*/false);

  EXPECT_DOUBLE_EQ(search.planPath(start_, &graph_, nullptr).second, kUncorrelatedPathGain);
}

}  // namespace
}  // namespace open_apero
