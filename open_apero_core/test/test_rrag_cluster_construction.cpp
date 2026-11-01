/**
 * @file test_rrag_cluster_construction.cpp
 * @brief RRAG::buildClusterAt: one node per orientation candidate, the robot's shared gain anchors
 *        attached once per cluster, and nothing added to the graph when no candidate is traversable.
 */

#include <cstddef>
#include <memory>
#include <tuple>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "open_apero_core/gain/gain_anchor.h"
#include "open_apero_core/graph/graph_node.h"
#include "open_apero_core/graph/node.h"
#include "open_apero_core/plugin/robot_model/robot_model.h"
#include "open_apero_core/state/robot_state.h"
#include "rrag_test_stubs.h"

namespace open_apero {
namespace {

const Eigen::Vector3d kPosition(2.0, 0.0, 0.0);

class RRAGClusterConstructionTest : public ::testing::Test {
 protected:
  void SetUp() override {
    agent_.provide(&map_, &robot_);
    generator_ = std::make_unique<RefreshProbe>(agent_);
    std::tie(std::ignore, std::ignore) = generator_->generateInitNode(makeState(robot_, Eigen::Vector3d::Zero()));
  }

  std::vector<Eigen::Quaterniond> candidateOrientations() const {
    std::vector<Eigen::Quaterniond> orientations;
    robot_.sampleOrientations(kPosition, &orientations);
    return orientations;
  }

  StubAgent agent_;
  StubMap map_{agent_};
  StationingRobot robot_{agent_};
  std::unique_ptr<RefreshProbe> generator_;
};

// Untraversable candidates are members too, marked as such, as long as one candidate is traversable.
TEST_F(RRAGClusterConstructionTest, EveryCandidateBecomesANodeWithItsOwnVerdictAndState) {
  const std::vector<Eigen::Quaterniond> orientations = candidateOrientations();
  ASSERT_GE(orientations.size(), 2u);
  robot_.blocked_orientations.push_back(orientations[1]);

  std::vector<RobotState> expected_states;
  std::vector<bool> expected_traversable;
  robot_.checkOrientations(kPosition, orientations, &expected_states, &expected_traversable, StationingCheck::kEnforce);
  ASSERT_FALSE(expected_traversable[1]) << "fixture must block one candidate";
  ASSERT_TRUE(expected_traversable[0]) << "fixture must leave another traversable";

  const std::size_t vertices_before = generator_->numVertices();
  const NodeClusterPtrs cluster = generator_->buildClusterAt(kPosition, StationingCheck::kEnforce);

  ASSERT_EQ(cluster.size(), orientations.size());
  EXPECT_EQ(generator_->numVertices(), vertices_before + orientations.size());
  for (std::size_t i = 0; i < cluster.size(); ++i) {
    const Node* node = cluster[i];
    EXPECT_NE(getNodeId(node, generator_->graph()), kInvalidNodeId) << "candidate " << i;
    EXPECT_EQ(node->is_traversable, expected_traversable[i]) << "candidate " << i;
    EXPECT_TRUE(node->robot_state.isSameSensorPose(expected_states[i])) << "candidate " << i;
    EXPECT_TRUE(node->robot_state.getBasePosition().isApprox(expected_states[i].getBasePosition())) << "candidate " << i;
    EXPECT_TRUE(node->candidate_orientations.empty()) << "candidate " << i;
    EXPECT_TRUE(node->is_candidate_traversable.empty()) << "candidate " << i;
    EXPECT_TRUE(node->gain_anchors.empty()) << "a robot attaching no shared anchors gives no anchors";
  }
}

TEST_F(RRAGClusterConstructionTest, TheRobotAttachesSharedAnchorsOnceToTheFinishedCluster) {
  robot_.shares_gain_anchor = true;
  robot_.shared_anchor_calls.clear();

  const std::size_t vertices_before = generator_->numVertices();
  const NodeClusterPtrs cluster = generator_->buildClusterAt(kPosition, StationingCheck::kEnforce);

  ASSERT_EQ(cluster.size(), 4u);
  EXPECT_EQ(generator_->numVertices(), vertices_before + 4u) << "anchors stay out of the graph";
  ASSERT_EQ(robot_.shared_anchor_calls.size(), 1u);
  EXPECT_TRUE(robot_.shared_anchor_calls[0].position.isApprox(kPosition));
  EXPECT_EQ(robot_.shared_anchor_calls[0].cluster, cluster) << "every member, in candidate order";

  const GainAnchor* shared = cluster[0]->gain_anchors.empty() ? nullptr : cluster[0]->gain_anchors[0].get();
  ASSERT_NE(shared, nullptr);
  for (std::size_t i = 0; i < cluster.size(); ++i) {
    ASSERT_EQ(cluster[i]->gain_anchors.size(), 1u) << "candidate " << i;
    EXPECT_EQ(cluster[i]->gain_anchors[0].get(), shared) << "candidate " << i;
  }
}

TEST_F(RRAGClusterConstructionTest, NoTraversableCandidateYieldsAnEmptyClusterAndAnUnchangedGraph) {
  robot_.blocked_positions.push_back(kPosition);
  robot_.shared_anchor_calls.clear();

  const std::size_t vertices_before = generator_->numVertices();
  const NodeClusterPtrs cluster = generator_->buildClusterAt(kPosition, StationingCheck::kEnforce);

  EXPECT_TRUE(cluster.empty());
  EXPECT_EQ(generator_->numVertices(), vertices_before);
  EXPECT_TRUE(robot_.shared_anchor_calls.empty()) << "no cluster, no anchors to share";
}

TEST_F(RRAGClusterConstructionTest, EnforcedStationingAsksTheRuleAndRefusesAForbiddenPosition) {
  robot_.forbidden_positions.push_back(kPosition);

  const std::size_t vertices_before = generator_->numVertices();
  const NodeClusterPtrs cluster = generator_->buildClusterAt(kPosition, StationingCheck::kEnforce);

  ASSERT_EQ(robot_.stationing_queries.size(), 1u);
  EXPECT_TRUE(robot_.stationing_queries[0].isApprox(kPosition));
  EXPECT_TRUE(cluster.empty());
  EXPECT_EQ(generator_->numVertices(), vertices_before);
}

TEST_F(RRAGClusterConstructionTest, BootstrapExemptionSkipsTheRuleAndBuildsAtAForbiddenPosition) {
  robot_.forbidden_positions.push_back(kPosition);

  const NodeClusterPtrs cluster = generator_->buildClusterAt(kPosition, StationingCheck::kExemptBootstrap);

  EXPECT_TRUE(robot_.stationing_queries.empty());
  ASSERT_EQ(cluster.size(), candidateOrientations().size());
  for (const Node* node : cluster) {
    EXPECT_TRUE(node->is_traversable);
  }
}

}  // namespace
}  // namespace open_apero
