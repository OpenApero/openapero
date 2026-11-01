/**
 * @file test_goal_constrained_search.cpp
 * @brief Tests for goal-constrained search: PathPlanner::planPath with a destination.
 *
 * Drives the real planner plugins, since beam pruning, the no-graph shortcut and the fallback all
 * sit outside updateBest().
 */

#include <cstddef>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "open_apero_core/agent/agent_interface.h"
#include "open_apero_core/graph/graph_node.h"
#include "open_apero_core/graph/path.h"
#include "open_apero_core/graph/tree_node.h"
#include "open_apero_core/plugin/path_planner/graph_depth_wise_beam_search.h"
#include "open_apero_core/plugin/path_planner/graph_node_wise_beam_search.h"
#include "open_apero_core/plugin/path_planner/graph_shortest_path_tree_search.h"
#include "open_apero_core/plugin/path_planner/graph_tsp_search.h"
#include "open_apero_core/plugin/path_planner/no_op_path_planner.h"
#include "open_apero_core/plugin/path_planner/path_planner.h"
#include "open_apero_core/plugin/path_planner/tree_best_value.h"
#include "plugin_test_stubs.h"

namespace open_apero {
namespace {

/// @brief Exposes the protected selection hook so a candidate stream can be fed in directly.
class SelectionProbe : public PathPlanner {
 public:
  explicit SelectionProbe(AgentInterface& agent) : PathPlanner(agent) {}

  void configure(Objective objective) { p_objective_ = objective; }

  /// @brief Offers one candidate, optionally requiring it to end at @p required_terminal_node.
  void offer(Path&& candidate, Node* required_terminal_node = nullptr) {
    updateBest(std::move(candidate), best_, best_score_, required_terminal_node);
  }

  /// @brief Terminal node of the currently selected path, or nullptr if nothing has been selected.
  const Node* winner() const { return best_.isValid() ? best_.last->node : nullptr; }

  /// @brief Runs the shared conclusion step against whatever has been offered so far.
  std::vector<Node*> conclude(Node* start_node, Node* required_terminal_node, const std::vector<Node*>& fallback_candidates) {
    std::mt19937 rng(0);
    return concludeSearch(best_, start_node, required_terminal_node, fallback_candidates, &rng, &best_score_);
  }

  std::pair<std::vector<Node*>, double> planPath(Node* /*start_node*/, GraphType* /*graph*/, Node* /*required_terminal_node*/) override {
    return {};
  }

 private:
  Path best_{kDoubleMin, kDoubleMin, kDoubleMax};  // matches how the real planners seed the running best
  double best_score_ = kDoubleMin;
};

PathSteps steps;

/// @brief A candidate path ending at @p end, carrying the metrics the search would have accumulated.
Path makeCandidate(Node* end, double gain, double cost) {
  return Path(gain / cost, gain, cost, steps.extend(nullptr, end));
}

/**
 * @brief A chain start -> middle -> terminal plus a rival branch off the start.
 *
 * The rival carries the highest single-node gain and sits one cheap hop away, so an unconstrained search
 * always prefers it and a constrained one can only win by walking the chain to the end. That makes the two
 * behaviors distinguishable by the returned path alone.
 */
class GoalConstrainedSearchTest : public ::testing::Test {
 protected:
  void SetUp() override {
    start_ = addNode(0.0);
    middle_ = addNode(1.0);
    terminal_ = addNode(1.0);
    rival_ = addNode(50.0);

    addEdge(0, 1, 1.0);
    addEdge(1, 2, 1.0);
    addEdge(0, 3, 1.0);
  }

  Node* addNode(double gain) {
    auto node = std::make_shared<GraphNode>();
    node->is_traversable = true;
    node->gain_anchors = {std::make_shared<GainAnchor>(gain)};
    ids_.push_back(boost::add_vertex(node, graph_));
    return node.get();
  }

  void addEdge(std::size_t from, std::size_t to, double cost) {
    auto edge = std::make_shared<Edge>();
    edge->cost = cost;
    edge->is_traversable = true;
    boost::add_edge(ids_[from], ids_[to], edge, graph_);
  }

  /// @brief A search configured the way a caller with a fixed destination would set it up.
  std::unique_ptr<path_planner::GraphNodeWiseBeamSearch> makeSearch() {
    auto search = std::make_unique<path_planner::GraphNodeWiseBeamSearch>(agent_);
    search->setupRaw(Objective::kPathGain, /*beam_depth=*/4, /*beam_width=*/4, /*check_previous_path=*/false);
    return search;
  }

  /// @brief The depth-wise twin of makeSearch(), configured identically.
  std::unique_ptr<path_planner::GraphDepthWiseBeamSearch> makeDepthWiseSearch() {
    auto search = std::make_unique<path_planner::GraphDepthWiseBeamSearch>(agent_);
    search->setupRaw(Objective::kPathGain, /*beam_depth=*/4, /*beam_width=*/4, /*check_previous_path=*/false);
    return search;
  }

  /// @brief Shortest-path-tree search that proposes only the single highest-gain node as a goal.
  std::unique_ptr<path_planner::GraphShortestPathTreeSearch> makeShortestPathSearch() {
    auto search = std::make_unique<path_planner::GraphShortestPathTreeSearch>(agent_);
    search->setupRaw(Objective::kPathGain, /*top_node_percentage=*/0.0);
    return search;
  }

  SearchOnlyAgent agent_;
  GraphType graph_;
  std::vector<NodeId> ids_;
  Node* start_ = nullptr;
  Node* middle_ = nullptr;
  Node* terminal_ = nullptr;
  Node* rival_ = nullptr;
};

// The constraint has to be applied while candidates are being scored, not afterwards: a prefix that outscores
// every complete path would otherwise be selected and there would be nothing left to fall back to.
TEST_F(GoalConstrainedSearchTest, SelectionSkipsCandidatesThatEndElsewhere) {
  SelectionProbe probe(agent_);
  probe.configure(Objective::kPathGain);

  probe.offer(makeCandidate(rival_, 50.0, 1.0), terminal_);
  EXPECT_EQ(probe.winner(), nullptr) << "a higher-scoring path ending elsewhere is not an answer";

  probe.offer(makeCandidate(terminal_, 2.0, 2.0), terminal_);
  EXPECT_EQ(probe.winner(), terminal_);
}

// The destination is an argument rather than planner state, so the very same candidate is an answer or not
// depending only on what this call asked for. Nothing has to be cleared afterwards.
TEST_F(GoalConstrainedSearchTest, TheSameCandidateIsAcceptedWhenNoDestinationIsRequired) {
  SelectionProbe constrained(agent_);
  constrained.configure(Objective::kPathGain);
  constrained.offer(makeCandidate(rival_, 50.0, 1.0), terminal_);
  EXPECT_EQ(constrained.winner(), nullptr);

  SelectionProbe unconstrained(agent_);
  unconstrained.configure(Objective::kPathGain);
  unconstrained.offer(makeCandidate(rival_, 50.0, 1.0));
  EXPECT_EQ(unconstrained.winner(), rival_);
}

// The behavior the feature exists for, end to end: the rival branch scores 50 against the chain's 2, so an
// unconstrained search takes it, and only the destination makes the search walk the chain instead.
TEST_F(GoalConstrainedSearchTest, ConstrainedSearchReturnsAPathToTheTerminalNode) {
  auto search = makeSearch();

  const std::pair<std::vector<Node*>, double> result = search->planPath(start_, &graph_, terminal_);

  ASSERT_EQ(result.first.size(), 3u);
  EXPECT_EQ(result.first.front(), start_);
  EXPECT_EQ(result.first[1], middle_);
  EXPECT_EQ(result.first.back(), terminal_);
  EXPECT_DOUBLE_EQ(result.second, 2.0);
}

// The same graph without a destination, to show the previous test measures the constraint and not the graph.
TEST_F(GoalConstrainedSearchTest, UnconstrainedSearchIsUnaffected) {
  auto search = makeSearch();

  const std::pair<std::vector<Node*>, double> result = search->planPath(start_, &graph_, nullptr);

  ASSERT_EQ(result.first.size(), 2u);
  EXPECT_EQ(result.first.back(), rival_) << "the highest-gain branch still wins when nothing is required";
  EXPECT_DOUBLE_EQ(result.second, 50.0);
}

// Omitting the destination (the two-argument PathPlanner::planPath, which derived planners hide) must mean the
// same as passing nullptr, so one call site can serve both kinds of query without branching.
TEST_F(GoalConstrainedSearchTest, ANullDestinationSearchesFreely) {
  auto search = makeSearch();
  PathPlanner& planner = *search;

  const std::pair<std::vector<Node*>, double> result = planner.planPath(start_, &graph_);

  ASSERT_EQ(result.first.size(), 2u);
  EXPECT_EQ(result.first.back(), rival_);
}

// An unreachable destination must come back empty. The random-neighbor fallback exists to keep an exploring
// robot moving when everything was pruned, but here it would answer a question that was not asked -- and the
// caller cannot tell a fallback path from a real one except by re-checking the destination itself.
TEST_F(GoalConstrainedSearchTest, AnUnreachableTerminalNodeYieldsNoPath) {
  auto unreachable = std::make_shared<GraphNode>();
  unreachable->is_traversable = true;
  unreachable->gain_anchors = {std::make_shared<GainAnchor>(1.0)};
  boost::add_vertex(unreachable, graph_);  // no edges lead to it

  auto search = makeSearch();

  const std::pair<std::vector<Node*>, double> result = search->planPath(start_, &graph_, unreachable.get());

  EXPECT_TRUE(result.first.empty());
}

// Edges are directed, so a node that only leads back to the start cannot be reached going forward. Membership
// in the graph is not reachability.
TEST_F(GoalConstrainedSearchTest, ReachabilityFollowsEdgeDirection) {
  auto search = makeSearch();

  const std::pair<std::vector<Node*>, double> result = search->planPath(start_, &graph_, rival_);
  ASSERT_EQ(result.first.size(), 2u) << "the rival is reachable from the start";

  // Nothing points back at the start, so the reverse trip does not exist.
  auto reverse_search = makeSearch();
  EXPECT_TRUE(reverse_search->planPath(rival_, &graph_, start_).first.empty());
}

// A selected path always traverses at least one edge, so the start node is never a destination the search can
// satisfy: the seed path is never offered for scoring. Returning it would hand back a plan to stay put, which
// every caller here discards anyway.
TEST_F(GoalConstrainedSearchTest, RequiringTheStartNodeYieldsNoPath) {
  auto search = makeSearch();

  const std::pair<std::vector<Node*>, double> result = search->planPath(start_, &graph_, start_);

  EXPECT_TRUE(result.first.empty());
}

// The no-graph shortcut answers "stay where you are", which is a fine answer to "where next" and no answer at
// all to "how do I get to X". It runs before the search, so it needs the destination check of its own.
TEST_F(GoalConstrainedSearchTest, ANullGraphReachesNoDestination) {
  auto search = makeSearch();

  EXPECT_TRUE(search->planPath(start_, nullptr, terminal_).first.empty());

  // Unconstrained, the shortcut reports the start node.
  const std::pair<std::vector<Node*>, double> unconstrained = search->planPath(start_, nullptr, nullptr);
  ASSERT_EQ(unconstrained.first.size(), 1u);
  EXPECT_EQ(unconstrained.first.front(), start_);
}

// Depth-wise beam search honors a destination as node-wise beam search does.
TEST_F(GoalConstrainedSearchTest, DepthWiseSearchAlsoReturnsAPathToTheTerminalNode) {
  auto search = makeDepthWiseSearch();

  const std::pair<std::vector<Node*>, double> result = search->planPath(start_, &graph_, terminal_);

  ASSERT_EQ(result.first.size(), 3u);
  EXPECT_EQ(result.first.back(), terminal_);

  // Unconstrained, it prefers the gain-50 branch.
  auto unconstrained = makeDepthWiseSearch();
  EXPECT_EQ(unconstrained->planPath(start_, &graph_, nullptr).first.back(), rival_);
}

TEST_F(GoalConstrainedSearchTest, DepthWiseSearchSuppressesItsFallback) {
  auto unreachable = std::make_shared<GraphNode>();
  unreachable->is_traversable = true;
  unreachable->gain_anchors = {std::make_shared<GainAnchor>(1.0)};
  boost::add_vertex(unreachable, graph_);

  auto search = makeDepthWiseSearch();

  EXPECT_TRUE(search->planPath(start_, &graph_, unreachable.get()).first.empty());
}

// Shortest-path-tree search only builds paths to nominated goals (here just the gain-50 rival), so a
// destination must replace the nominations rather than filter them.
TEST_F(GoalConstrainedSearchTest, ShortestPathSearchReachesADestinationItWouldNeverNominate) {
  auto nominating_search = makeShortestPathSearch();
  ASSERT_EQ(nominating_search->planPath(start_, &graph_, nullptr).first.back(), rival_)
      << "unconstrained, only the highest-gain node is proposed as a goal";

  auto search = makeShortestPathSearch();
  const std::pair<std::vector<Node*>, double> result = search->planPath(start_, &graph_, terminal_);

  ASSERT_EQ(result.first.size(), 3u);
  EXPECT_EQ(result.first.front(), start_);
  EXPECT_EQ(result.first.back(), terminal_);
}

// A start node that is not a vertex of the graph is a caller error, reported before any search.
TEST_F(GoalConstrainedSearchTest, ShortestPathSearchRejectsAStartNodeOutsideTheGraph) {
  GraphNode foreign;
  foreign.is_traversable = true;

  auto search = makeShortestPathSearch();

  EXPECT_THROW(search->planPath(&foreign, &graph_, nullptr), std::runtime_error);
  EXPECT_THROW(search->planPath(&foreign, &graph_, terminal_), std::runtime_error);
}

TEST_F(GoalConstrainedSearchTest, ShortestPathSearchSuppressesItsFallback) {
  auto unreachable = std::make_shared<GraphNode>();
  unreachable->is_traversable = true;
  unreachable->gain_anchors = {std::make_shared<GainAnchor>(1.0)};
  boost::add_vertex(unreachable, graph_);

  auto search = makeShortestPathSearch();

  EXPECT_TRUE(search->planPath(start_, &graph_, unreachable.get()).first.empty());
}

/// @brief A root with two children, one of which carries all the gain. TreeBestValue plans over trees, not graphs.
class GoalConstrainedTreeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    root_.gain_anchors = {std::make_shared<GainAnchor>(0.0)};
    dull_ = spawnChild(&root_, 1.0);
    rich_ = spawnChild(&root_, 50.0);
  }

  TreeNode* spawnChild(TreeNode* parent, double gain) {
    TreeNode* child = parent->spawnChild();
    child->is_traversable = true;
    child->gain_anchors = {std::make_shared<GainAnchor>(gain)};
    child->edge.cost = 1.0;
    child->edge.is_traversable = true;
    return child;
  }

  std::unique_ptr<path_planner::TreeBestValue> makeSearch() {
    auto search = std::make_unique<path_planner::TreeBestValue>(agent_);
    Configurable::ParamMap params{{"param_namespace", "/tree_best_value_test"}, {"objective", "path_gain"}};
    search->setupFromParamMap(&params);
    return search;
  }

  SearchOnlyAgent agent_;
  TreeNode root_;
  TreeNode* dull_ = nullptr;
  TreeNode* rich_ = nullptr;
};

TEST_F(GoalConstrainedTreeTest, TreeSearchReturnsAPathToTheRequiredNode) {
  ASSERT_EQ(makeSearch()->planPath(&root_, nullptr, nullptr).first.back(), rich_) << "unconstrained, the high-gain child wins";

  const std::pair<std::vector<Node*>, double> result = makeSearch()->planPath(&root_, nullptr, dull_);

  ASSERT_EQ(result.first.size(), 2u);
  EXPECT_EQ(result.first.back(), dull_);
}

// concludeSearch() reports a search with no candidate and no fallback instead of drawing from an empty range.
TEST_F(GoalConstrainedSearchTest, ConcludingWithNoCandidateAndNoFallbackReports) {
  SelectionProbe probe(agent_);
  probe.configure(Objective::kPathGain);

  EXPECT_THROW(probe.conclude(start_, nullptr, {}), std::runtime_error);
}

// ...but a search asked for a destination concludes with the empty path instead, before any fallback is
// considered.
TEST_F(GoalConstrainedSearchTest, ConcludingAConstrainedSearchNeverNeedsAFallback) {
  SelectionProbe probe(agent_);
  probe.configure(Objective::kPathGain);

  EXPECT_TRUE(probe.conclude(start_, terminal_, {}).empty());
}

// A node from another tree is not a destination this search can reach, and the backtrack that builds
// candidates only ever ends at nodes of THIS tree -- so the constraint is satisfied by construction rather
// than by a membership check.
TEST_F(GoalConstrainedTreeTest, TreeSearchRejectsANodeFromAnotherTree) {
  TreeNode foreign;
  foreign.gain_anchors = {std::make_shared<GainAnchor>(1.0)};

  EXPECT_TRUE(makeSearch()->planPath(&root_, nullptr, &foreign).first.empty());
}

// The two planners that bypass updateBest() cannot aim at a destination -- TSP's tour comes from the routing
// solver, NoOp plans nothing -- so both return an empty path when one is required.
TEST_F(GoalConstrainedSearchTest, PlannersThatCannotAimStillRefuseCleanly) {
  path_planner::GraphTspSearch tsp(agent_);
  tsp.setupRaw(Objective::kPathGain, /*top_node_percentage=*/0.0);
  EXPECT_TRUE(tsp.planPath(start_, &graph_, terminal_).first.empty());

  path_planner::NoOpPathPlanner no_op(agent_);
  Configurable::ParamMap params{{"param_namespace", "/no_op_test"}};
  no_op.setupFromParamMap(&params);
  EXPECT_TRUE(no_op.planPath(start_, &graph_, terminal_).first.empty());

  // Unconstrained, TSP routes a tour and NoOp plans nothing.
  EXPECT_FALSE(tsp.planPath(start_, &graph_, nullptr).first.empty());
  EXPECT_TRUE(no_op.planPath(start_, &graph_, nullptr).first.empty());
}

}  // namespace
}  // namespace open_apero
