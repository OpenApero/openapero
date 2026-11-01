/**
 * @file test_path_planner_selection.cpp
 * @brief Tests for PathPlanner::updateBest tie-breaking.
 *
 * On an exact score tie a frontier-terminated candidate wins under the objectives that read
 * is_at_frontier, otherwise Path::operator> decides. Driven through updateBest directly.
 */

#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "open_apero_core/agent/agent_interface.h"
#include "open_apero_core/graph/path.h"
#include "open_apero_core/plugin/path_planner/path_planner.h"

#include "plugin_test_stubs.h"

namespace open_apero {
namespace {

constexpr double kBudget = 50.0;

/// @brief ConfigAgent with kBudget, the only thing updateBest reads, and the default seed.
class BudgetOnlyAgent : public ConfigAgent {
 public:
  BudgetOnlyAgent() {
    setCostBudget(kBudget);
    setRandomSeed(-1);
  }
};

/// @brief Exposes the protected selection hooks so a candidate stream can be fed in directly.
class SelectionProbe : public PathPlanner {
 public:
  explicit SelectionProbe(AgentInterface& agent) : PathPlanner(agent) {}

  void configure(Objective objective) { p_objective_ = objective; }
  void offer(Path&& candidate) { updateBest(std::move(candidate), best_, best_score_, nullptr); }

  /// @brief Terminal node of the currently selected path, or nullptr if nothing has been selected.
  const Node* winner() const { return best_.isValid() ? best_.last->node : nullptr; }
  double score() const { return best_score_; }

  std::pair<std::vector<Node*>, double> planPath(Node* /*start_node*/, GraphType* /*graph*/, Node* /*required_terminal_node*/) override {
    return {};
  }

 private:
  Path best_{kDoubleMin, kDoubleMin, kDoubleMax};  // matches how the real planners seed the running best
  double best_score_ = kDoubleMin;
};

PathSteps steps;

/// @brief A one-hop candidate ending at @p end, with the metrics a zero-gain or gainful path would carry.
Path makeCandidate(Node* end, double gain, double cost) {
  return Path(gain / cost, gain, cost, steps.extend(nullptr, end));
}

class PathPlannerSelectionTest : public ::testing::Test {
 protected:
  Node* makeNode(bool is_at_frontier) {
    nodes_.push_back(std::make_unique<Node>());
    nodes_.back()->is_at_frontier = is_at_frontier;
    return nodes_.back().get();
  }

  BudgetOnlyAgent agent_;
  std::vector<std::unique_ptr<Node>> nodes_;
};

// The degenerate case the tie-break exists for: nothing observed carries gain, so every score is zero. The
// frontier candidate must win even though a non-frontier one was enumerated first and is cheaper -- reaching
// the frontier is the only move that can reveal gain.
TEST_F(PathPlannerSelectionTest, ZeroGainSelectionPrefersFrontierUnderExpectedGain) {
  SelectionProbe probe(agent_);
  probe.configure(Objective::kExpectedGain);

  Node* interior = makeNode(false);
  Node* frontier = makeNode(true);

  probe.offer(makeCandidate(interior, 0.0, 1.0));
  probe.offer(makeCandidate(frontier, 0.0, 4.0));

  EXPECT_EQ(probe.winner(), frontier);
  EXPECT_DOUBLE_EQ(probe.score(), 0.0);
}

// Among tied frontier candidates the fallback is Path::operator> -- ratio, then gain, then LOWER cost -- so
// the nearest frontier wins, and it wins from either enumeration order.
TEST_F(PathPlannerSelectionTest, ZeroGainSelectionPrefersTheNearestFrontier) {
  Node* near_frontier = makeNode(true);
  Node* far_frontier = makeNode(true);

  for (const bool near_first : {true, false}) {
    SelectionProbe probe(agent_);
    probe.configure(Objective::kExpectedGain);
    if (near_first) {
      probe.offer(makeCandidate(near_frontier, 0.0, 2.0));
      probe.offer(makeCandidate(far_frontier, 0.0, 9.0));
    } else {
      probe.offer(makeCandidate(far_frontier, 0.0, 9.0));
      probe.offer(makeCandidate(near_frontier, 0.0, 2.0));
    }
    EXPECT_EQ(probe.winner(), near_frontier) << "near_first=" << near_first;
  }
}

// The frontier tie-break applies to equal scores only: a strictly better score still wins.
TEST_F(PathPlannerSelectionTest, StrictlyBetterScoreBeatsAFrontierTieBreak) {
  SelectionProbe probe(agent_);
  probe.configure(Objective::kExpectedGain);

  Node* frontier = makeNode(true);
  Node* gainful_interior = makeNode(false);

  probe.offer(makeCandidate(frontier, 0.0, 4.0));           // score 0 * budget = 0
  probe.offer(makeCandidate(gainful_interior, 10.0, 4.0));  // score 10

  EXPECT_EQ(probe.winner(), gainful_interior);
  EXPECT_DOUBLE_EQ(probe.score(), 10.0);
}

// The tie-break fires on an EXACT tie only: a margin far below kEpsilon must still win outright, even
// against a cheaper frontier incumbent that a tolerant comparison would prefer.
TEST_F(PathPlannerSelectionTest, AMarginallyBetterScoreStillWinsOutright) {
  SelectionProbe probe(agent_);
  probe.configure(Objective::kExpectedGain);

  Node* cheap_frontier = makeNode(true);
  Node* barely_better_interior = makeNode(false);

  probe.offer(makeCandidate(cheap_frontier, 0.0, 1.0));           // score 0 * budget = 0
  probe.offer(makeCandidate(barely_better_interior, 1e-9, 4.0));  // score 1e-9 > 0

  EXPECT_EQ(probe.winner(), barely_better_interior);
  EXPECT_DOUBLE_EQ(probe.score(), 1e-9);
}

// kPathGain and kPathRatio do not model the frontier, so they must not be handed frontier information through
// the tie-break -- that would silently turn the two exploration-agnostic baselines into explorers. The frontier
// candidate here is the more expensive one, so preferring it could only come from a frontier rule.
TEST_F(PathPlannerSelectionTest, FrontierAgnosticObjectivesDoNotPreferTheFrontier) {
  for (const Objective objective : {Objective::kPathGain, Objective::kPathRatio}) {
    SelectionProbe probe(agent_);
    probe.configure(objective);

    Node* interior = makeNode(false);
    Node* frontier = makeNode(true);

    probe.offer(makeCandidate(interior, 0.0, 1.0));
    probe.offer(makeCandidate(frontier, 0.0, 4.0));

    EXPECT_EQ(probe.winner(), interior) << "objective=" << toString(objective);
  }
}

// Every objective -- the frontier-agnostic ones included -- still falls back to Path::operator> on a tie, so
// enumeration order never decides. Without this the cheaper candidate loses purely for being seen second.
TEST_F(PathPlannerSelectionTest, EveryObjectiveBreaksTiesByLowerCost) {
  for (const Objective objective : {Objective::kPathGain, Objective::kPathRatio, Objective::kExpectedGain, Objective::kTableGain}) {
    SelectionProbe probe(agent_);
    probe.configure(objective);

    Node* far_node = makeNode(false);
    Node* near_node = makeNode(false);

    probe.offer(makeCandidate(far_node, 0.0, 9.0));
    probe.offer(makeCandidate(near_node, 0.0, 2.0));

    EXPECT_EQ(probe.winner(), near_node) << "objective=" << toString(objective);
  }
}

// kTableGain's score reads is_at_frontier too, so it gets the same treatment. Its table starts empty, which is
// exactly the degenerate regime: valueAt() contributes nothing and every candidate scores zero.
TEST_F(PathPlannerSelectionTest, ZeroGainSelectionPrefersFrontierUnderTableGain) {
  SelectionProbe probe(agent_);
  probe.configure(Objective::kTableGain);

  Node* interior = makeNode(false);
  Node* frontier = makeNode(true);

  probe.offer(makeCandidate(interior, 0.0, 1.0));
  probe.offer(makeCandidate(frontier, 0.0, 4.0));

  EXPECT_EQ(probe.winner(), frontier);
}

}  // namespace
}  // namespace open_apero
