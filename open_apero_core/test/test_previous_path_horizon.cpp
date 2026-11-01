/**
 * @file test_previous_path_horizon.cpp
 * @brief Tests the receding-horizon reuse of the previously chosen path (check_previous_path).
 *
 * The live search is limited to one step (beam_depth 1) after priming a longer horizon, on a
 * unit-cost, unit-gain chain start -> near -> mid -> far, so a replayed prefix scores its own
 * length. Both beam searches run as one typed suite; reuse of a pruned node's address is untested.
 */

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <boost/graph/adjacency_list.hpp>
#include <gtest/gtest.h>

#include "open_apero_core/gain/gain_anchor.h"
#include "open_apero_core/graph/edge.h"
#include "open_apero_core/graph/graph_node.h"
#include "open_apero_core/graph/node.h"
#include "open_apero_core/plugin/path_planner/graph_depth_wise_beam_search.h"
#include "open_apero_core/plugin/path_planner/graph_node_wise_beam_search.h"
#include "open_apero_core/plugin/path_planner/path_planner.h"
#include "open_apero_core/plugin/plugin.h"

#include "plugin_test_stubs.h"

namespace open_apero {
namespace {

constexpr double kAmpleBudget = 1000.0;
constexpr double kEdgeCost = 1.0;
constexpr double kNodeGain = 1.0;

/// @brief Enough depth for the live search to walk the whole chain on its own, and to record it.
constexpr int kPrimeDepth = 3;
/// @brief One step: everything longer in a later answer came from the replayed horizon.
constexpr int kBlindDepth = 1;
constexpr int kBeamWidth = 4;

constexpr char kWholeChain[] = "start -> near -> mid -> far";
constexpr char kFirstStepOnly[] = "start -> near";

/// @brief ConfigAgent starting at kAmpleBudget with seed 0; setRemainingCostBudget() spends budget the way a run does.
class BudgetedSearchAgent : public ConfigAgent {
 public:
  BudgetedSearchAgent() {
    setCostBudget(kAmpleBudget);
    setRandomSeed(0);
  }
};

/// @brief The chain start -> near -> mid -> far, primed as a horizon and then interfered with.
template <typename PlannerType>
class PreviousPathHorizonTest : public ::testing::Test {
 protected:
  void SetUp() override {
    start_ = addNode("start", 0.0);
    near_ = addNode("near", kNodeGain);
    mid_ = addNode("mid", kNodeGain);
    far_ = addNode("far", kNodeGain);

    addEdge(start_, near_);
    addEdge(near_, mid_);
    addEdge(mid_, far_);
  }

  Node* addNode(const std::string& name, double gain) {
    auto node = std::make_shared<GraphNode>();
    node->is_traversable = true;
    if (gain > 0.0) {
      node->gain_anchors.push_back(std::make_shared<GainAnchor>(gain));
    }

    Node* raw = node.get();
    ids_[raw] = boost::add_vertex(node, graph_);
    names_[raw] = name;
    owners_[raw] = node;
    return raw;
  }

  void addEdge(Node* from, Node* to) {
    auto edge = std::make_shared<Edge>();
    edge->cost = kEdgeCost;
    edge->is_traversable = true;
    boost::add_edge(ids_[from], ids_[to], edge, graph_);
  }

  void removeEdge(Node* from, Node* to) { boost::remove_edge(ids_[from], ids_[to], graph_); }

  /// @brief Drops a node the way graph maintenance does. The graph held the only owning reference,
  /// so this frees it and leaves a horizon that still names it holding a dangling pointer -- which
  /// is the state the replay has to survive. Its name is kept so a regression that returns it is
  /// reported as a route rather than as a crash.
  void removeNode(Node* node) {
    const NodeId id = ids_[node];
    ids_.erase(node);
    boost::clear_vertex(id, graph_);
    boost::remove_vertex(id, graph_);
    // The premise, not a detail: were the node still owned somewhere, the horizon would hold a live
    // pointer and the test below would quietly stop covering the case it is named for.
    ASSERT_TRUE(owners_[node].expired()) << "removing " << names_[node] << " did not free it";
    owners_.erase(node);
  }

  /// @brief Records the whole chain as the horizon, then blinds the live search to one step.
  ///
  /// Primes with the reuse off on purpose: the recording at the end of planPath() is unconditional,
  /// so the horizon is captured either way and the primed answer stays a pure live-search result.
  /// @param planner The planner to prime; left configured with @p check_previous_path at kBlindDepth.
  /// @param check_previous_path The setting under test.
  void primeThenBlind(PlannerType* planner, bool check_previous_path) {
    planner->setupRaw(Objective::kPathGain, kPrimeDepth, kBeamWidth, /*check_previous_path=*/false);
    ASSERT_EQ(route(planner->planPath(start_, &graph_, nullptr).first), kWholeChain);
    planner->setupRaw(Objective::kPathGain, kBlindDepth, kBeamWidth, check_previous_path);
  }

  /// @brief The parameters a shipped config supplies, so the default is tested through real parsing.
  /// @param beam_depth Search depth to configure.
  /// @param check_previous_path The key's value, or nullopt to leave the key out entirely.
  Configurable::ParamMap params(int beam_depth, std::optional<bool> check_previous_path) const {
    Configurable::ParamMap map;
    map["param_namespace"] = "test_previous_path_horizon";
    map["objective"] = "path_gain";
    map["beam_depth"] = std::to_string(beam_depth);
    map["beam_width"] = std::to_string(kBeamWidth);
    if (check_previous_path.has_value()) {
      // Both param-map builders render a YAML bool as 1/0 because setParam parses without
      // std::boolalpha; "true"/"false" here would throw instead of testing anything.
      map["check_previous_path"] = check_previous_path.value() ? "1" : "0";
    }
    return map;
  }

  /// @brief A path as its route, so a wrong answer reads as nodes rather than as pointers. Pointers
  /// are looked up, never dereferenced, since a pruned node's may be dangling.
  std::string route(const std::vector<Node*>& path) const {
    std::string text;
    for (const Node* node : path) {
      const auto it = names_.find(node);
      if (!text.empty()) {
        text += " -> ";
      }
      text += (it == names_.end()) ? "<unnamed>" : it->second;
    }
    return text.empty() ? "<empty>" : text;
  }

  BudgetedSearchAgent agent_;
  GraphType graph_;
  std::unordered_map<const Node*, NodeId> ids_;
  std::unordered_map<const Node*, std::string> names_;
  /// @brief Non-owning handles used only to confirm that removeNode() really destroys a node.
  std::unordered_map<const Node*, std::weak_ptr<GraphNode>> owners_;
  Node* start_ = nullptr;
  Node* near_ = nullptr;
  Node* mid_ = nullptr;
  Node* far_ = nullptr;
};

using BeamSearches = ::testing::Types<path_planner::GraphNodeWiseBeamSearch, path_planner::GraphDepthWiseBeamSearch>;
TYPED_TEST_SUITE(PreviousPathHorizonTest, BeamSearches);

// The baseline every other test is read against: with the reuse off, the answer is whatever the live
// search can see, and the recorded horizon buys nothing.
TYPED_TEST(PreviousPathHorizonTest, WithoutTheReuseTheAnswerStopsAtTheLiveHorizon) {
  TypeParam planner(this->agent_);
  this->primeThenBlind(&planner, /*check_previous_path=*/false);

  EXPECT_EQ(this->route(planner.planPath(this->start_, &this->graph_, nullptr).first), kFirstStepOnly);
}

TYPED_TEST(PreviousPathHorizonTest, ReplaysTheHorizonPastTheLiveSearchDepth) {
  TypeParam planner(this->agent_);
  this->primeThenBlind(&planner, /*check_previous_path=*/true);

  const std::pair<std::vector<Node*>, double> replanned = planner.planPath(this->start_, &this->graph_, nullptr);
  EXPECT_EQ(this->route(replanned.first), kWholeChain);
  // Scored as the path it is: three gainful nodes, not the one step the live search could reach.
  EXPECT_DOUBLE_EQ(replanned.second, 3.0 * kNodeGain);
}

// The robot arrived at the first step, so that step is behind it: the replay has to resume at the
// one after it. Resuming at the wrong index leaves the horizon unusable (there is no self-edge) and
// the answer collapses to what the live search sees.
TYPED_TEST(PreviousPathHorizonTest, ResumesAfterTheStepAlreadyReached) {
  TypeParam planner(this->agent_);
  this->primeThenBlind(&planner, /*check_previous_path=*/true);

  EXPECT_EQ(this->route(planner.planPath(this->near_, &this->graph_, nullptr).first), "near -> mid -> far");
}

// The dominant case in a real run: the planning layer splits the first edge at the replanning
// horizon, so the next start node is an intermediate node that was not on the recorded path but is
// joined to its second node by the remainder edge.
TYPED_TEST(PreviousPathHorizonTest, ContinuesFromAnIntermediateNodeSplitOntoTheFirstEdge) {
  TypeParam planner(this->agent_);
  this->primeThenBlind(&planner, /*check_previous_path=*/true);

  Node* const split = this->addNode("split", 0.0);
  this->addEdge(split, this->near_);

  EXPECT_EQ(this->route(planner.planPath(split, &this->graph_, nullptr).first), "split -> near -> mid -> far");
}

TYPED_TEST(PreviousPathHorizonTest, StopsWhereTheHorizonLostAnEdge) {
  TypeParam planner(this->agent_);
  this->primeThenBlind(&planner, /*check_previous_path=*/true);

  this->removeEdge(this->near_, this->mid_);

  // Every hop is re-read from the graph, so the answer stops at the gap rather than asserting a
  // connection the graph no longer makes.
  EXPECT_EQ(this->route(planner.planPath(this->start_, &this->graph_, nullptr).first), kFirstStepOnly);
}

TYPED_TEST(PreviousPathHorizonTest, StopsWhereTheHorizonLostANodeToPruning) {
  TypeParam planner(this->agent_);
  this->primeThenBlind(&planner, /*check_previous_path=*/true);

  ASSERT_NO_FATAL_FAILURE(this->removeNode(this->mid_));

  // The horizon now names freed memory. Reaching it is a lookup among live nodes, never a
  // dereference, so this must answer normally -- and answer with live nodes only.
  EXPECT_EQ(this->route(planner.planPath(this->start_, &this->graph_, nullptr).first), kFirstStepOnly);
}

TYPED_TEST(PreviousPathHorizonTest, ReplaysOnlyAsFarAsTheBudgetReaches) {
  TypeParam planner(this->agent_);
  this->primeThenBlind(&planner, /*check_previous_path=*/true);

  // Room for one edge and no more, so the horizon is affordable only one step in.
  this->agent_.setRemainingCostBudget(1.5 * kEdgeCost);

  EXPECT_EQ(this->route(planner.planPath(this->start_, &this->graph_, nullptr).first), kFirstStepOnly);
}

// Reuse is an extra candidate, not a commitment: the replayed horizon is scored against the live
// search on this cycle's gains, and loses when the graph has moved on.
TYPED_TEST(PreviousPathHorizonTest, AbandonsTheHorizonWhenTheGraphMovedOn) {
  TypeParam planner(this->agent_);
  this->primeThenBlind(&planner, /*check_previous_path=*/true);

  this->mid_->gain_anchors.clear();
  this->far_->gain_anchors.clear();
  Node* const alternative = this->addNode("alternative", 10.0 * kNodeGain);
  this->addEdge(this->start_, alternative);

  EXPECT_EQ(this->route(planner.planPath(this->start_, &this->graph_, nullptr).first), "start -> alternative");
}

// A path chosen to reach one destination is not a horizon to extend: it is the answer to a different
// question, and recording it would throw away the exploration horizon it was never comparable to.
TYPED_TEST(PreviousPathHorizonTest, ADestinationSearchIsNotRecordedAsTheHorizon) {
  TypeParam planner(this->agent_);
  this->primeThenBlind(&planner, /*check_previous_path=*/true);

  ASSERT_EQ(this->route(planner.planPath(this->start_, &this->graph_, this->near_).first), kFirstStepOnly);

  EXPECT_EQ(this->route(planner.planPath(this->start_, &this->graph_, nullptr).first), kWholeChain);
}

TYPED_TEST(PreviousPathHorizonTest, RepeatedReplansKeepTheHorizon) {
  TypeParam planner(this->agent_);
  this->primeThenBlind(&planner, /*check_previous_path=*/true);

  // Each answer is recorded in turn, so a replay that consumed or truncated the horizon would show
  // up as a shorter second answer from an unchanged graph.
  EXPECT_EQ(this->route(planner.planPath(this->start_, &this->graph_, nullptr).first), kWholeChain);
  EXPECT_EQ(this->route(planner.planPath(this->start_, &this->graph_, nullptr).first), kWholeChain);
}

TYPED_TEST(PreviousPathHorizonTest, ResetEpisodeDropsTheHorizon) {
  TypeParam planner(this->agent_);
  this->primeThenBlind(&planner, /*check_previous_path=*/true);

  planner.resetEpisode();

  // A new episode starts in a new scene; last episode's route through it is not a horizon.
  EXPECT_EQ(this->route(planner.planPath(this->start_, &this->graph_, nullptr).first), kFirstStepOnly);
}

// The param map is how a run actually configures this, and the key's absence is a live setting: the
// default is on, while every shipped config turns it off. A silent flip either way moves published
// numbers without anyone editing a config.
TYPED_TEST(PreviousPathHorizonTest, OmittingTheParamReusesTheHorizon) {
  TypeParam planner(this->agent_);

  Configurable::ParamMap priming = this->params(kPrimeDepth, std::nullopt);
  planner.setupFromParamMap(&priming);
  ASSERT_EQ(this->route(planner.planPath(this->start_, &this->graph_, nullptr).first), kWholeChain);

  Configurable::ParamMap blinded = this->params(kBlindDepth, std::nullopt);
  planner.setupFromParamMap(&blinded);

  EXPECT_EQ(this->route(planner.planPath(this->start_, &this->graph_, nullptr).first), kWholeChain);
}

TYPED_TEST(PreviousPathHorizonTest, TheParamTurnsTheReuseOff) {
  TypeParam planner(this->agent_);

  Configurable::ParamMap priming = this->params(kPrimeDepth, /*check_previous_path=*/false);
  planner.setupFromParamMap(&priming);
  ASSERT_EQ(this->route(planner.planPath(this->start_, &this->graph_, nullptr).first), kWholeChain);

  Configurable::ParamMap blinded = this->params(kBlindDepth, /*check_previous_path=*/false);
  planner.setupFromParamMap(&blinded);

  EXPECT_EQ(this->route(planner.planPath(this->start_, &this->graph_, nullptr).first), kFirstStepOnly);
}

}  // namespace
}  // namespace open_apero
