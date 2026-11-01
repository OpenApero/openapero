/**
 * @file test_generator_sampling.cpp
 * @brief How many positions RRAG and RRT draw per generateNewNode() call, and which sampling and
 *        edge-length settings their validation accepts.
 *
 * sample_max_tries: N draws at most N positions (0 = unlimited); a robot without sampling bounds
 * ends the call with no node.
 */

#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "open_apero_core/graph/edge.h"
#include "open_apero_core/graph/node.h"
#include "open_apero_core/plugin/generator/rrt.h"
#include "open_apero_core/plugin/plugin.h"
#include "rrag_test_stubs.h"

namespace open_apero {
namespace {

/// Scripted position samples, counted; without bounds it samples nothing.
class SamplingRobot : public StationingRobot {
 public:
  explicit SamplingRobot(AgentInterface& agent) : StationingRobot(agent) {}

  bool samplePosition(Eigen::Vector3d* position, double /*z_default*/) const override {
    ++num_samples;
    if (!has_bounds) {
      return false;
    }
    *position = sample;
    return true;
  }

  bool has_bounds = true;
  Eigen::Vector3d sample = Eigen::Vector3d::Zero();
  mutable int num_samples = 0;
};

/// Prices edges by their length in states; there is no owning layer to ask.
class RrtProbe : public generator::RRT {
 public:
  RrtProbe(AgentInterface& agent, const Configurable::ParamMap& extra_params) : RRT(agent) {
    Configurable::ParamMap params{{"param_namespace", "/test_rrt"}};
    for (const auto& entry : extra_params) {
      params[entry.first] = entry.second;
    }
    setupFromParamMap(&params);
  }

  bool computeCost(Edge* edge) override {
    edge->cost = static_cast<double>(edge->trajectory.size());
    return true;
  }
};

class GeneratorSamplingTest : public ::testing::Test {
 protected:
  GeneratorSamplingTest() { agent_.provide(&map_, &robot_); }

  /// An RRAG with one indexed cluster at kAnchor, the sample landing on it.
  Node* rootRragAtAnAnchor(RefreshProbe* rrag) {
    Node* root = rrag->generateInitNode(makeState(robot_, Eigen::Vector3d::Zero())).second;
    rrag->placeCluster(kAnchor);
    robot_.sample = kAnchor;
    return root;
  }

  /// An RRT whose seed child is indexed, the sample landing on it.
  Node* rootRrtAtItsSeed(RrtProbe* rrt) {
    Node* root = rrt->generateInitNode(makeState(robot_, Eigen::Vector3d::Zero())).second;
    NodeClusterPtrs seed = rrt->generateNewNode(root);
    std::vector<Edge*> edges;
    EXPECT_FALSE(seed.empty());
    EXPECT_TRUE(rrt->generateNewEdges(seed, edges)) << "the seed child must be indexed";
    robot_.sample = Eigen::Vector3d::Zero();
    return root;
  }

  static std::string validationError(Configurable* configurable) {
    std::string error;
    return configurable->checkParamsValid(&error) ? std::string() : error;
  }

  const Eigen::Vector3d kAnchor{2.0, 0.0, 0.0};
  StubAgent agent_;
  StubMap map_{agent_};
  SamplingRobot robot_{agent_};
};

TEST_F(GeneratorSamplingTest, RragDrawsSampleMaxTriesPositions) {
  RefreshProbe rrag(agent_, {{"sample_max_tries", "3"}});
  Node* root = rootRragAtAnAnchor(&rrag);

  // Every sample lands on the anchor, closer than min_edge_length, and is rejected.
  EXPECT_TRUE(rrag.generateNewNode(root).empty());
  EXPECT_EQ(robot_.num_samples, 3);
}

TEST_F(GeneratorSamplingTest, RragAddsNoNodeWithoutSamplingBounds) {
  RefreshProbe rrag(agent_, {{"sample_max_tries", "0"}});
  Node* root = rootRragAtAnAnchor(&rrag);
  const std::size_t num_vertices = rrag.numVertices();
  robot_.has_bounds = false;

  EXPECT_TRUE(rrag.generateNewNode(root).empty());
  EXPECT_EQ(robot_.num_samples, 1) << "unlimited tries end at the first failed draw";
  EXPECT_EQ(rrag.numVertices(), num_vertices);
}

TEST_F(GeneratorSamplingTest, RrtDrawsSampleMaxTriesPositions) {
  RrtProbe rrt(agent_, {{"sample_max_tries", "3"}});
  Node* root = rootRrtAtItsSeed(&rrt);
  ASSERT_EQ(robot_.num_samples, 0) << "the seed child is not sampled";

  EXPECT_TRUE(rrt.generateNewNode(root).empty());
  EXPECT_EQ(robot_.num_samples, 3);
}

TEST_F(GeneratorSamplingTest, RrtAddsNoNodeWithoutSamplingBounds) {
  RrtProbe rrt(agent_, {{"sample_max_tries", "0"}});
  Node* root = rootRrtAtItsSeed(&rrt);
  const std::size_t num_nodes = rrt.getAllNodes(root).size();
  robot_.has_bounds = false;

  EXPECT_TRUE(rrt.generateNewNode(root).empty());
  EXPECT_EQ(robot_.num_samples, 1);
  EXPECT_EQ(rrt.getAllNodes(root).size(), num_nodes);
}

TEST_F(GeneratorSamplingTest, ValidationRejectsNegativeSampleMaxTries) {
  RefreshProbe rrag(agent_, {{"sample_max_tries", "-1"}});
  RrtProbe rrt(agent_, {{"sample_max_tries", "-1"}});
  EXPECT_EQ(validationError(&rrag), "sample_max_tries expected >= 0 (0 = unlimited)");
  EXPECT_EQ(validationError(&rrt), "sample_max_tries expected >= 0 (0 = unlimited)");

  RefreshProbe unlimited(agent_, {{"sample_max_tries", "0"}});
  EXPECT_EQ(validationError(&unlimited), "");
}

TEST_F(GeneratorSamplingTest, ValidationRequiresAPositiveMinimumEdgeLengthBelowTheMaximum) {
  RefreshProbe zero_min(agent_, {{"min_edge_length", "0.0"}});
  EXPECT_EQ(validationError(&zero_min), "min_edge_length expected > 0.0");

  RrtProbe max_below_min(agent_, {{"min_edge_length", "1.5"}, {"max_edge_length", "1.0"}});
  EXPECT_EQ(validationError(&max_below_min), "max_edge_length expected 0.0 (no limit) or >= min_edge_length");

  RefreshProbe no_limit(agent_, {{"min_edge_length", "1.5"}, {"max_edge_length", "0.0"}});
  EXPECT_EQ(validationError(&no_limit), "");
  RefreshProbe equal(agent_, {{"min_edge_length", "1.5"}, {"max_edge_length", "1.5"}});
  EXPECT_EQ(validationError(&equal), "");
}

}  // namespace
}  // namespace open_apero
