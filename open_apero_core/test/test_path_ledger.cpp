/**
 * @file test_path_ledger.cpp
 * @brief PathLedger: what a node is worth to a path, and which edges the path has traversed.
 */

#include <cmath>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "open_apero_core/gain/gain_anchor.h"
#include "open_apero_core/graph/graph_node.h"
#include "open_apero_core/graph/path.h"

namespace open_apero {
namespace {

class PathLedgerTest : public ::testing::Test {
 protected:
  Node* addNode(const GainAnchorPtrVector& anchors) {
    auto node = std::make_shared<GraphNode>();
    node->gain_anchors = anchors;
    nodes_.push_back(node);
    return node.get();
  }

  static GainAnchorPtr anchor(double gain) { return std::make_shared<GainAnchor>(gain); }

  Path pathOf(const PathNodes& nodes) { return Path(0.0, 0.0, 0.0, steps_.chain(nodes)); }

  /// @brief What @p candidate is worth after @p nodes, correlated and uncorrelated.
  std::pair<double, double> worth(const PathNodes& nodes, Node* candidate) {
    ledger_.load(pathOf(nodes));
    return {ledger_.marginalGain(candidate, true), ledger_.marginalGain(candidate, false)};
  }

  std::vector<std::shared_ptr<GraphNode>> nodes_;
  PathSteps steps_;
  PathLedger ledger_;
};

TEST_F(PathLedgerTest, ANodeOffThePathIsWorthItsOwnGain) {
  Node* on_path = addNode({anchor(3.0)});
  Node* candidate = addNode({anchor(4.0), anchor(0.5)});

  const auto [correlated, uncorrelated] = worth({on_path}, candidate);
  EXPECT_DOUBLE_EQ(correlated, 4.5);
  EXPECT_DOUBLE_EQ(uncorrelated, 4.5);
}

TEST_F(PathLedgerTest, ANodeAlreadyOnThePathIsWorthNothing) {
  Node* candidate = addNode({anchor(4.0)});

  const auto [correlated, uncorrelated] = worth({addNode({}), candidate}, candidate);
  EXPECT_DOUBLE_EQ(correlated, 0.0);
  EXPECT_DOUBLE_EQ(uncorrelated, 0.0);
}

TEST_F(PathLedgerTest, ASharedAnchorIsPaidOnceWhenCorrelated) {
  const GainAnchorPtr shared = anchor(10.0);
  Node* first = addNode({shared, anchor(1.0)});
  Node* second = addNode({shared});
  Node* candidate = addNode({shared, anchor(2.0)});

  const auto [correlated, uncorrelated] = worth({first, second}, candidate);
  EXPECT_DOUBLE_EQ(correlated, 2.0);
  EXPECT_DOUBLE_EQ(uncorrelated, 12.0);
}

TEST_F(PathLedgerTest, ANonFiniteGainReachesTheCaller) {
  Node* candidate = addNode({anchor(std::numeric_limits<double>::quiet_NaN())});

  EXPECT_TRUE(std::isnan(worth({addNode({anchor(1.0)})}, candidate).first));
}

TEST_F(PathLedgerTest, ASuppressedNodeIsWorthNothingAndHoldsNoAnchor) {
  const GainAnchorPtr shared = anchor(5.0);
  Node* waypoint = addNode({shared});
  waypoint->is_gain_suppressed = true;
  Node* candidate = addNode({shared});

  EXPECT_DOUBLE_EQ(worth({addNode({})}, waypoint).first, 0.0);
  EXPECT_DOUBLE_EQ(worth({addNode({}), waypoint}, candidate).first, 5.0);
}

TEST_F(PathLedgerTest, LoadingAPathForgetsThePreviousOne) {
  Node* on_path = addNode({anchor(3.0)});
  Node* candidate = addNode({anchor(4.0)});

  EXPECT_DOUBLE_EQ(worth({on_path, candidate}, candidate).first, 0.0);
  EXPECT_DOUBLE_EQ(worth({on_path}, candidate).first, 4.0);
}

TEST_F(PathLedgerTest, OnlyTheEdgesFromTheLastNodeOnThePathAreRevisits) {
  Node* a = addNode({});
  Node* b = addNode({});
  Node* c = addNode({});
  ledger_.load(pathOf({a, b, c, a}));

  EXPECT_TRUE(ledger_.revisitsEdge(b)) << "a -> b is on the path";
  EXPECT_FALSE(ledger_.revisitsEdge(c)) << "b -> c is, a -> c is not";
  EXPECT_FALSE(ledger_.revisitsEdge(a));

  ledger_.load(pathOf({a, c, a}));
  EXPECT_TRUE(ledger_.revisitsEdge(c)) << "a -> c is on the new path";
  EXPECT_FALSE(ledger_.revisitsEdge(b)) << "the entry from the earlier path is stale";
}

TEST_F(PathLedgerTest, PathGainPaysEveryNodeButTheFirstForWhatItAdds) {
  const GainAnchorPtr shared = anchor(5.0);
  Node* start = addNode({shared, anchor(1.0)});
  Node* a = addNode({shared, anchor(2.0)});
  Node* b = addNode({shared, anchor(4.0)});

  EXPECT_DOUBLE_EQ(ledger_.pathGain({start, a, b}, true), 6.0) << "the start's anchors are visited, not paid";
  EXPECT_DOUBLE_EQ(ledger_.pathGain({start, b, a}, true), 6.0);
  EXPECT_DOUBLE_EQ(ledger_.pathGain({start, a, b, a}, false), 16.0) << "a revisited node adds nothing";
  EXPECT_DOUBLE_EQ(ledger_.pathGain({start}, true), 0.0);
}

TEST_F(PathLedgerTest, ToNodesListsThePathInOrder) {
  Node* a = addNode({});
  Node* b = addNode({});
  EXPECT_EQ(pathOf({a, b, a}).toNodes(), (PathNodes{a, b, a}));
  EXPECT_FALSE(Path(0.0, 0.0, 0.0).isValid());
}

}  // namespace
}  // namespace open_apero
