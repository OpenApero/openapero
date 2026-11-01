/**
 * @file test_translation_adjacency_modes.cpp
 * @brief Which endpoint orientations RRAG connects two clusters at, and what that costs.
 *
 * The path-heading modes score each side against the planned corridor's own end directions; the
 * cell mode fans one corridor out to every bracketing candidate. Connected pairs are skipped first.
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/graph/adjacency_list.hpp>
#include <gtest/gtest.h>

#include "open_apero_core/agent/agent_interface.h"
#include "open_apero_core/common/typedef.h"
#include "open_apero_core/common/utils.h"
#include "open_apero_core/graph/edge.h"
#include "open_apero_core/graph/graph_node.h"
#include "open_apero_core/plugin/generator/rrag.h"
#include "open_apero_core/plugin/map/map.h"
#include "open_apero_core/plugin/plugin_factory.h"
#include "open_apero_core/plugin/robot_model/robot_model.h"
#include "open_apero_core/plugin/robot_model/simple_robot.h"
#include "open_apero_core/state/robot_state.h"

#include "plugin_test_stubs.h"

namespace open_apero {
namespace {

/// Eight yaw candidates, so a cluster's orientations sit 45 degrees apart.
constexpr int kNumYawCandidates = 8;

/// The two cluster positions, and the corner every plan detours through. The chord runs
/// at 42.7 deg, the first leg leaves at 73.3 deg and the second arrives at 11.3 deg, so
/// no two of the three directions pick the same candidate.
const Eigen::Vector3d kFromPosition(0.0, 0.0, 0.0);
const Eigen::Vector3d kToPosition(2.6, 2.4, 0.0);
const Eigen::Vector3d kCornerOffset(0.6, 2.0, 0.0);

/// Candidate indices the three directions select: 45 deg for the chord, 90 deg for the
/// departure, 0 deg for the arrival, each bracketed by its 45-deg neighbor.
constexpr int kChordIndex = 1;
constexpr int kDepartureIndex = 2;
constexpr int kArrivalIndex = 0;

/// A corner bending the corridor by 128 deg: it leaves at 113.8 deg (bracketed by 135
/// and 90) and arrives at -13.7 deg (bracketed by 0 and -45), so neither end bracket
/// contains the 45 deg the chord picks.
const Eigen::Vector3d kSharpCornerOffset(-1.5, 3.4, 0.0);

/// The cell corners kSharpCornerOffset selects, as {departure, arrival} candidate pairs.
const EdgeList kSharpCornerPairs{{2, 0}, {2, 7}, {3, 0}, {3, 7}};

class StubMap : public Map {
 public:
  explicit StubMap(AgentInterface& agent) : Map(agent) {}
  void setupFromParamMap(ParamMap* /*param_map*/) override {}
};

// SimpleRobot::setupFromParamMap configures its bounding volume through the factory; ConfigAgent's empty
// in-memory factory yields the (unbounded) defaults.
class StubAgent : public ConfigAgent {
 public:
  StubAgent() {
    setCostBudget(0.0);
    setRandomSeed(5);
  }
};

/// A robot whose corridors bend: every plan detours through one corner, so its end
/// directions differ from the chord. Its enclosing cell is the two candidates bracketing
/// a direction.
class CorneringRobot : public robot_model::SimpleRobot {
 public:
  explicit CorneringRobot(AgentInterface& agent) : robot_model::SimpleRobot(agent) {
    Configurable::ParamMap params{{"param_namespace", "/test_translation_adjacency"},
                                  {"num_orientation_candidates", std::to_string(kNumYawCandidates)},
                                  {"max_linear_velocity", "0.5"},
                                  {"max_yaw_rate", "1.6"},
                                  {"time_step", "0.2"},
                                  {"safe_radius", "0.0"},
                                  {"collision_check_yaw", "0"}};
    setupFromParamMap(&params);
  }

  Traversability getStateTraversabilityRaw(const RobotState& /*state*/, Map* /*map*/) const override {
    return Traversability::kTraversable;
  }

  bool planTrajectory(const RobotState& start, const RobotState& goal, Map* map, RobotTrajectory* trajectory) override {
    ++plan_calls;
    if (!plan_succeeds) {
      return false;
    }

    // The corner is entered already facing the second leg, so the L is two walks with no
    // rotation between them and its legs are the departure and the arrival.
    const Eigen::Vector3d corner_position = start.getSensorPosition() + corner_offset;
    const Eigen::Vector3d second_leg = goal.getSensorPosition() - corner_position;
    RobotState corner;
    placeRobotAtSensorPose(&corner, {corner_position, quaternionFromYaw(std::atan2(second_leg.y(), second_leg.x()))});

    RobotTrajectory route = steer(start, corner, map);
    const RobotTrajectory second_half = steer(corner, goal, map);
    route.insert(route.end(), second_half.begin() + 1, second_half.end());
    setUniformTrajectoryTimes(route, getTimeStep());
    *trajectory = std::move(route);
    return true;
  }

  EdgeList getTranslationAdjacency(const std::vector<RobotState>& from_states, const std::vector<RobotState>& to_states,
                                   const Eigen::Vector3d& departure_direction, const Eigen::Vector3d& arrival_direction,
                                   bool connect_enclosing_cell) const override {
    if (!connect_enclosing_cell) {
      return RobotModel::getTranslationAdjacency(from_states, to_states, departure_direction, arrival_direction, false);
    }

    EdgeList edges;
    for (const int from_index : bracketingIndices(from_states, departure_direction)) {
      for (const int to_index : bracketingIndices(to_states, arrival_direction)) {
        edges.emplace_back(from_index, to_index);
      }
    }
    return edges;
  }

  int plan_calls = 0;
  bool plan_succeeds = true;
  /// How sharply the corridors bend, which is what decides the two end brackets.
  Eigen::Vector3d corner_offset = kCornerOffset;

 private:
  /// The two candidates closest to @p direction, which brackets it as long as the
  /// direction falls strictly between them.
  static std::vector<int> bracketingIndices(const std::vector<RobotState>& states, const Eigen::Vector3d& direction) {
    const double direction_yaw = std::atan2(direction.y(), direction.x());
    std::vector<int> indices(states.size());
    for (std::size_t i = 0; i < indices.size(); ++i) {
      indices[i] = static_cast<int>(i);
    }
    std::stable_sort(indices.begin(), indices.end(), [&states, direction_yaw](int lhs, int rhs) {
      return std::abs(angleDifference(states[lhs].getSensorYaw(), direction_yaw)) <
             std::abs(angleDifference(states[rhs].getSensorYaw(), direction_yaw));
    });
    indices.resize(std::min<std::size_t>(2, indices.size()));
    return indices;
  }
};

/// Drives RRAG's cluster-to-cluster edge generation directly, without a sampling cycle
/// or an owning layer.
class AdjacencyProbe : public generator::RRAG {
 public:
  AdjacencyProbe(AgentInterface& agent, const std::string& translation_adjacency) : generator::RRAG(agent) {
    Configurable::ParamMap params{{"param_namespace", "/test_translation_adjacency"}, {"translation_adjacency", translation_adjacency}};
    setupFromParamMap(&params);
  }

  using generator::RRAG::generateEdgesFromClusterToCluster;

  // Every edge builder costs its result through the owning layer; there is none here.
  bool computeCost(Edge* edge) override {
    edge->cost = static_cast<double>(edge->trajectory.size());
    return true;
  }

  GraphType* graph() { return graph_.get(); }

  /// A full candidate cluster at @p position, in the graph but left out of the spatial
  /// index: these tests call the edge generator themselves.
  NodeClusterPtrs placeCluster(const Eigen::Vector3d& position) {
    std::vector<Eigen::Quaterniond> orientations;
    agent_.getRobot().sampleOrientations(position, &orientations);

    NodeClusterPtrs cluster;
    for (const auto& orientation : orientations) {
      auto node = std::make_shared<GraphNode>();
      agent_.getRobot().placeRobotAtSensorPose(&node->robot_state, {position, orientation});
      node->is_traversable = true;
      boost::add_vertex(node, *graph_);
      cluster.push_back(node.get());
    }
    return cluster;
  }
};

/// The positions a trajectory visits, with each in-place run folded into one entry.
std::vector<Eigen::Vector3d> distinctPositions(const RobotTrajectory& trajectory) {
  std::vector<Eigen::Vector3d> positions;
  for (const auto& state : trajectory) {
    if (positions.empty() || !isSamePosition(positions.back(), state.getSensorPosition())) {
      positions.push_back(state.getSensorPosition());
    }
  }
  return positions;
}

void expectSamePositions(const std::vector<Eigen::Vector3d>& actual, const std::vector<Eigen::Vector3d>& expected) {
  ASSERT_EQ(actual.size(), expected.size());
  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_LT((actual[i] - expected[i]).norm(), 1e-9) << "position " << i;
  }
}

/// A state at a pose, placed the way the robot under test places one.
RobotState stateAt(const RobotModel& robot, const Eigen::Vector3d& position, double yaw) {
  RobotState state;
  robot.placeRobotAtSensorPose(&state, {position, quaternionFromYaw(yaw)});
  return state;
}

std::vector<RobotState> statesOf(const NodeClusterPtrs& cluster) {
  std::vector<RobotState> states;
  states.reserve(cluster.size());
  for (const Node* node : cluster) {
    states.push_back(node->robot_state);
  }
  return states;
}

int indexOf(const NodeClusterPtrs& cluster, const Node* node) {
  for (int i = 0; i < static_cast<int>(cluster.size()); ++i) {
    if (cluster[i] == node) {
      return i;
    }
  }
  return -1;
}

/// The vertices an inserted edge was added between, which is what says the edge really
/// runs from the node whose pose it starts at.
std::pair<Node*, Node*> endpointsOf(const Edge* edge, const GraphType* graph) {
  for (const auto& edge_id : boost::make_iterator_range(boost::edges(*graph))) {
    if ((*graph)[edge_id].get() == edge) {
      return {(*graph)[boost::source(edge_id, *graph)].get(), (*graph)[boost::target(edge_id, *graph)].get()};
    }
  }
  return {nullptr, nullptr};
}

class TranslationAdjacencyTest : public ::testing::Test {
 protected:
  TranslationAdjacencyTest() : map_(agent_), robot_(agent_) { agent_.provide(&map_, &robot_); }

  RobotState rootState() const {
    RobotState state;
    robot_.placeRobotAtSensorPose(&state, {Eigen::Vector3d(-5.0, -5.0, 0.0), quaternionFromYaw(0.0)});
    return state;
  }

  StubAgent agent_;
  StubMap map_;
  CorneringRobot robot_;
};

// The chord and the two path directions must actually disagree, or nothing below
// distinguishes the modes.
TEST_F(TranslationAdjacencyTest, TheFixtureDirectionsSelectDifferentCandidates) {
  AdjacencyProbe generator(agent_, "straight_line");
  std::tie(std::ignore, std::ignore) = generator.generateInitNode(rootState());
  NodeClusterPtrs from_cluster = generator.placeCluster(kFromPosition);
  NodeClusterPtrs to_cluster = generator.placeCluster(kToPosition);

  const std::vector<RobotState> from_states = statesOf(from_cluster);
  const std::vector<RobotState> to_states = statesOf(to_cluster);
  const Eigen::Vector3d chord_direction = to_states.front().getSensorPosition() - from_states.front().getSensorPosition();
  const EdgeList chord =
      robot_.getTranslationAdjacency(from_states, to_states, chord_direction, chord_direction, /*connect_enclosing_cell=*/false);
  ASSERT_EQ(chord.size(), 1u);
  EXPECT_EQ(chord.front().first, kChordIndex);
  EXPECT_EQ(chord.front().second, kChordIndex);
  EXPECT_NE(kChordIndex, kDepartureIndex);
  EXPECT_NE(kChordIndex, kArrivalIndex);
}

// One planned corridor serving the whole enclosing cell: four edges, all carrying the
// seed's corridor, each ending exactly on the pose of the node it was inserted between.
TEST_F(TranslationAdjacencyTest, PathHeadingCellFansOneCorridorOutToTheEnclosingCell) {
  AdjacencyProbe generator(agent_, "path_heading_cell");
  std::tie(std::ignore, std::ignore) = generator.generateInitNode(rootState());
  NodeClusterPtrs from_cluster = generator.placeCluster(kFromPosition);
  NodeClusterPtrs to_cluster = generator.placeCluster(kToPosition);

  // The corridor the generator will plan, taken beforehand so its position sequence is
  // known independently.
  RobotTrajectory seed_trajectory;
  ASSERT_TRUE(robot_.planTrajectory(from_cluster[kChordIndex]->robot_state, to_cluster[kChordIndex]->robot_state, &map_, &seed_trajectory));
  const std::vector<Eigen::Vector3d> seed_positions = distinctPositions(seed_trajectory);
  ASSERT_GT(seed_positions.size(), 2u) << "the fixture corridor must bend";
  robot_.plan_calls = 0;

  const std::vector<Edge*> edges = generator.generateEdgesFromClusterToCluster(from_cluster, to_cluster);

  EXPECT_EQ(edges.size(), 4u);
  EXPECT_EQ(robot_.plan_calls, 1) << "the corridor is planned once for the whole cell";

  EdgeList connected_pairs;
  for (Edge* edge : edges) {
    const std::pair<Node*, Node*> endpoints = endpointsOf(edge, generator.graph());
    ASSERT_NE(endpoints.first, nullptr) << "every returned edge must be in the graph";
    EXPECT_TRUE(edge->trajectory.front().isSameSensorPose(endpoints.first->robot_state));
    EXPECT_TRUE(edge->trajectory.back().isSameSensorPose(endpoints.second->robot_state));
    expectSamePositions(distinctPositions(edge->trajectory), seed_positions);
    connected_pairs.emplace_back(indexOf(from_cluster, endpoints.first), indexOf(to_cluster, endpoints.second));
  }

  const EdgeList expected_pairs{
      {kChordIndex, kArrivalIndex}, {kChordIndex, kChordIndex}, {kDepartureIndex, kArrivalIndex}, {kDepartureIndex, kChordIndex}};
  std::sort(connected_pairs.begin(), connected_pairs.end());
  EXPECT_EQ(connected_pairs, expected_pairs);
}

// The seed is a template, not a member of the cell: with the chord's candidate outside
// both end brackets the seed is dropped, and the cell is still filled from one plan.
TEST_F(TranslationAdjacencyTest, ASeedPairOutsideTheReselectedCellIsNotInserted) {
  robot_.corner_offset = kSharpCornerOffset;

  AdjacencyProbe generator(agent_, "path_heading_cell");
  std::tie(std::ignore, std::ignore) = generator.generateInitNode(rootState());
  NodeClusterPtrs from_cluster = generator.placeCluster(kFromPosition);
  NodeClusterPtrs to_cluster = generator.placeCluster(kToPosition);
  robot_.plan_calls = 0;

  const std::vector<Edge*> edges = generator.generateEdgesFromClusterToCluster(from_cluster, to_cluster);

  ASSERT_EQ(edges.size(), 4u) << "the whole 2 x 2 cross product, none of it the seed's own pair";
  EXPECT_EQ(robot_.plan_calls, 1) << "and still one corridor plan for all four";

  EdgeList connected_pairs;
  for (Edge* edge : edges) {
    const std::pair<Node*, Node*> endpoints = endpointsOf(edge, generator.graph());
    ASSERT_NE(endpoints.first, nullptr) << "every returned edge must be in the graph";
    EXPECT_TRUE(edge->trajectory.front().isSameSensorPose(endpoints.first->robot_state));
    EXPECT_TRUE(edge->trajectory.back().isSameSensorPose(endpoints.second->robot_state));
    connected_pairs.emplace_back(indexOf(from_cluster, endpoints.first), indexOf(to_cluster, endpoints.second));
  }
  std::sort(connected_pairs.begin(), connected_pairs.end());
  EXPECT_EQ(connected_pairs, kSharpCornerPairs);

  const std::pair<int, int> seed_pair{kChordIndex, kChordIndex};
  EXPECT_EQ(std::find(connected_pairs.begin(), connected_pairs.end(), seed_pair), connected_pairs.end())
      << "the pair the seed was planned for is in neither bracket, so nothing carries it";
}

// The early skip: without it the maintenance pass would replan the corridor every cycle,
// since the pairs to compare against the graph are not known until after the plan.
TEST_F(TranslationAdjacencyTest, AConnectedClusterPairIsSkippedWithoutPlanning) {
  AdjacencyProbe generator(agent_, "path_heading_cell");
  std::tie(std::ignore, std::ignore) = generator.generateInitNode(rootState());
  NodeClusterPtrs from_cluster = generator.placeCluster(kFromPosition);
  NodeClusterPtrs to_cluster = generator.placeCluster(kToPosition);

  ASSERT_EQ(generator.generateEdgesFromClusterToCluster(from_cluster, to_cluster).size(), 4u);
  const int plan_calls_after_first_pass = robot_.plan_calls;

  EXPECT_TRUE(generator.generateEdgesFromClusterToCluster(from_cluster, to_cluster).empty());
  EXPECT_EQ(robot_.plan_calls, plan_calls_after_first_pass);
}

// The skip walks out-edges only, by design: B to A carries the corridor B departs along,
// which is not A's corridor reversed, so a backwards-only pair counts as unconnected.
TEST_F(TranslationAdjacencyTest, EdgesInTheReverseDirectionDoNotSkipTheForwardPair) {
  AdjacencyProbe generator(agent_, "path_heading_cell");
  std::tie(std::ignore, std::ignore) = generator.generateInitNode(rootState());
  NodeClusterPtrs from_cluster = generator.placeCluster(kFromPosition);
  NodeClusterPtrs to_cluster = generator.placeCluster(kToPosition);

  ASSERT_FALSE(generator.generateEdgesFromClusterToCluster(to_cluster, from_cluster).empty())
      << "connect the two clusters the other way round first";
  const int plan_calls_after_reverse = robot_.plan_calls;

  const std::vector<Edge*> forward_edges = generator.generateEdgesFromClusterToCluster(from_cluster, to_cluster);

  EXPECT_EQ(forward_edges.size(), 4u);
  EXPECT_EQ(robot_.plan_calls, plan_calls_after_reverse + 1) << "the forward corridor is planned, not skipped";
  for (Edge* edge : forward_edges) {
    const std::pair<Node*, Node*> endpoints = endpointsOf(edge, generator.graph());
    ASSERT_NE(endpoints.first, nullptr);
    EXPECT_NE(indexOf(from_cluster, endpoints.first), -1) << "every new edge leaves the from cluster";
    EXPECT_NE(indexOf(to_cluster, endpoints.second), -1) << "and arrives in the to cluster";
  }
}

// The default mode, pinned: one edge, at the pair the chord picks.
TEST_F(TranslationAdjacencyTest, StraightLineConnectsOnlyTheChordPair) {
  AdjacencyProbe generator(agent_, "straight_line");
  std::tie(std::ignore, std::ignore) = generator.generateInitNode(rootState());
  NodeClusterPtrs from_cluster = generator.placeCluster(kFromPosition);
  NodeClusterPtrs to_cluster = generator.placeCluster(kToPosition);

  const std::vector<Edge*> edges = generator.generateEdgesFromClusterToCluster(from_cluster, to_cluster);

  ASSERT_EQ(edges.size(), 1u);
  const std::pair<Node*, Node*> endpoints = endpointsOf(edges.front(), generator.graph());
  EXPECT_EQ(indexOf(from_cluster, endpoints.first), kChordIndex);
  EXPECT_EQ(indexOf(to_cluster, endpoints.second), kChordIndex);
}

// One edge again, but at the pair the corridor's own end directions pick -- and the
// seed, planned for the chord pair, is not itself inserted.
TEST_F(TranslationAdjacencyTest, PathHeadingConnectsThePathDirectionsPairNotTheChordPair) {
  AdjacencyProbe generator(agent_, "path_heading");
  std::tie(std::ignore, std::ignore) = generator.generateInitNode(rootState());
  NodeClusterPtrs from_cluster = generator.placeCluster(kFromPosition);
  NodeClusterPtrs to_cluster = generator.placeCluster(kToPosition);

  const std::vector<Edge*> edges = generator.generateEdgesFromClusterToCluster(from_cluster, to_cluster);

  ASSERT_EQ(edges.size(), 1u);
  const std::pair<Node*, Node*> endpoints = endpointsOf(edges.front(), generator.graph());
  EXPECT_EQ(indexOf(from_cluster, endpoints.first), kDepartureIndex);
  EXPECT_EQ(indexOf(to_cluster, endpoints.second), kArrivalIndex);
  EXPECT_TRUE(edges.front()->trajectory.front().isSameSensorPose(from_cluster[kDepartureIndex]->robot_state));
  EXPECT_TRUE(edges.front()->trajectory.back().isSameSensorPose(to_cluster[kArrivalIndex]->robot_state));
}

// The seed is the only plan made, so a seed that fails leaves the pair unconnected.
TEST_F(TranslationAdjacencyTest, ASeedPlanFailureConnectsNothing) {
  AdjacencyProbe generator(agent_, "path_heading_cell");
  std::tie(std::ignore, std::ignore) = generator.generateInitNode(rootState());
  NodeClusterPtrs from_cluster = generator.placeCluster(kFromPosition);
  NodeClusterPtrs to_cluster = generator.placeCluster(kToPosition);
  robot_.plan_succeeds = false;

  EXPECT_TRUE(generator.generateEdgesFromClusterToCluster(from_cluster, to_cluster).empty());
  EXPECT_EQ(boost::num_edges(*generator.graph()), 0u);
}

TEST_F(TranslationAdjacencyTest, AnUnknownTranslationAdjacencyIsRejected) {
  EXPECT_THROW(AdjacencyProbe(agent_, "chord"), std::runtime_error);
}

// ---------------------------------------------------------------------------
// The two end directions the path-heading modes re-select against.
// ---------------------------------------------------------------------------

// A corridor opens with an in-place rotation whenever the start orientation is not the
// direction of travel; reporting that step would hand the re-selection a zero vector.
TEST_F(TranslationAdjacencyTest, TheDepartureDeltaSkipsALeadingInPlaceRun) {
  const Eigen::Vector3d start_position(1.0, -2.0, 0.5);
  const Eigen::Vector3d walked_position(1.0, -1.7, 0.5);
  const RobotTrajectory trajectory{stateAt(robot_, start_position, 0.0), stateAt(robot_, start_position, 0.4),
                                   stateAt(robot_, start_position, 0.8), stateAt(robot_, walked_position, 0.8)};

  const Eigen::Vector3d delta = trajectoryDepartureDelta(trajectory);
  EXPECT_LT((delta - (walked_position - start_position)).norm(), 1e-9) << "the rotation displaces the robot by nothing";
}

// The mirror case: a corridor ends with an in-place rotation whenever the goal
// orientation is not the direction it arrived along.
TEST_F(TranslationAdjacencyTest, TheArrivalDeltaSkipsATrailingInPlaceRun) {
  const Eigen::Vector3d approach_position(1.0, -2.0, 0.5);
  const Eigen::Vector3d final_position(1.0, -1.7, 0.5);
  const RobotTrajectory trajectory{stateAt(robot_, approach_position, 0.8), stateAt(robot_, final_position, 0.8),
                                   stateAt(robot_, final_position, 0.4), stateAt(robot_, final_position, 0.0)};

  const Eigen::Vector3d delta = trajectoryArrivalDelta(trajectory);
  EXPECT_LT((delta - (final_position - approach_position)).norm(), 1e-9);
}

// A trajectory that never leaves its position has no direction of travel at either end,
// so it throws rather than answering with a zero vector the re-selection would read as 0 rad.
TEST_F(TranslationAdjacencyTest, APureRotationHasNeitherEndDirection) {
  const Eigen::Vector3d position(0.4, 0.4, 0.0);
  const RobotTrajectory rotation{stateAt(robot_, position, 0.0), stateAt(robot_, position, 0.4), stateAt(robot_, position, 0.8)};

  EXPECT_THROW(trajectoryDepartureDelta(rotation), std::invalid_argument);
  EXPECT_THROW(trajectoryArrivalDelta(rotation), std::invalid_argument);
}

}  // namespace
}  // namespace open_apero
