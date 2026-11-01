/**
 * @file test_simple_robot_traversability.cpp
 * @brief What the sphere sweep concludes from partial observation, and the margin it measures.
 *
 * Unobserved spheres are skipped and min_observed_spheres of the rest must pass for kTraversable;
 * clearance uses observed geometry only, and stationing requires the full margin.
 */

#include <cstddef>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "open_apero_core/common/typedef.h"
#include "open_apero_core/geometry/geometric_sphere.h"
#include "open_apero_core/geometry/transform3d.h"
#include "open_apero_core/plugin/map/map.h"
#include "open_apero_core/plugin/map/tsdf_map.h"
#include "open_apero_core/plugin/plugin_factory.h"
#include "open_apero_core/plugin/robot_model/robot_model.h"
#include "open_apero_core/plugin/robot_model/simple_robot.h"
#include "open_apero_core/state/robot_state.h"
#include "open_apero_core/visualization/visualization_markers.h"
#include "plugin_test_stubs.h"

namespace open_apero {
namespace {

/// Sensor height the tests place the robot at, chosen so the body and contact
/// spheres land in distinguishable halves of the scripted distance field.
constexpr double kSensorZ = 1.0;
/// Contact sphere offset below the base, and the height that separates the two.
constexpr double kContactOffsetZ = -0.5;
constexpr double kSplitZ = kSensorZ + kContactOffsetZ / 2.0;

/// Body geometry the tests share: one sphere at the base, one foot below it.
constexpr double kBodyRadius = 0.25;
constexpr double kBodyMargin = 0.10;
constexpr double kContactRadius = 0.15;

/// A distance the body sphere reads as clear of everything.
constexpr double kFarFromObstacles = 5.0;
/// A distance that puts the body sphere inside an obstacle, or a foot on the ground.
constexpr double kTouching = 0.05;

/// ConfigAgent serving StubFactory, with no budget and the default seed.
class StubAgent : public ConfigAgent {
 public:
  StubAgent() {
    useFactory(&stub_factory_);
    setCostBudget(0.0);
    setRandomSeed(-1);
  }

 private:
  StubFactory stub_factory_;
};

/// A TSDF map whose distance field is whatever a test scripts it to be. Refusing
/// to answer is how a test says a position is unobserved, which is the state the
/// map starts in.
class ScriptedTsdfMap : public map::TsdfMap {
 public:
  explicit ScriptedTsdfMap(AgentInterface& agent) : TsdfMap(agent) {}

  void setupFromParamMap(ParamMap* /*param_map*/) override {}
  Occupancy getVoxelOccupancy(const Eigen::Vector3d& /*point*/) override { return Occupancy::kUnknown; }
  double getVoxelSize() override { return 0.1; }
  bool getVoxelCenter(Eigen::Vector3d* /*center*/, const Eigen::Vector3d& /*point*/) override { return false; }
  double getVoxelWeight(const Eigen::Vector3d& /*point*/) override { return 0.0; }
  double getMaximumWeight() override { return 0.0; }

  bool getDistanceAtPosition(const Eigen::Vector3d& position, double* distance) override { return distance_field(position, distance); }

  std::function<bool(const Eigen::Vector3d&, double*)> distance_field = [](const Eigen::Vector3d&, double*) { return false; };
};

/// Scripts what the body half and the foot half of the robot see. An unset value
/// leaves that half unobserved.
void scriptHalves(ScriptedTsdfMap* map, std::optional<double> body, std::optional<double> contact) {
  map->distance_field = [body, contact](const Eigen::Vector3d& position, double* distance) {
    const std::optional<double>& value = position.z() > kSplitZ ? body : contact;
    if (!value.has_value()) {
      return false;
    }
    *distance = *value;
    return true;
  };
}

/// Exposes the raw sweep and lets a test replace the sphere geometry outright,
/// which is the only way to give a contact sphere a margin (configuration always
/// zeroes theirs).
class RobotProbe : public robot_model::SimpleRobot {
 public:
  RobotProbe(AgentInterface& agent, Configurable::ParamMap params) : robot_model::SimpleRobot(agent) {
    params["param_namespace"] = "/test_traversability";
    setupFromParamMap(&params);
  }

  using robot_model::SimpleRobot::getStateTraversabilityRaw;

  void setSpheres(const GeometricSpheres& collision, const GeometricSpheres& contact) {
    p_collision_spheres_ = collision;
    p_contact_spheres_ = contact;
  }

  /// The robot state at the shared test height, facing along +x.
  RobotState stateAtTestHeight() const { return stateAt(Eigen::Vector3d(0.0, 0.0, kSensorZ)); }

  RobotState stateAt(const Eigen::Vector3d& position) const {
    RobotState state;
    placeRobotAtSensorPose(&state, {position, Eigen::Quaterniond::Identity()});
    return state;
  }
};

/// Markers a robot renders for a trajectory, filtered to one namespace.
std::vector<VisualizationMarker> markersInNamespace(const RobotProbe& robot, const RobotTrajectory& trajectory, const std::string& ns) {
  VisualizationMarkers markers;
  robot.getVisualization(trajectory, &markers);
  std::vector<VisualizationMarker> filtered;
  for (const auto& marker : markers.getMarkers()) {
    if (marker.ns == ns) {
      filtered.push_back(marker);
    }
  }
  return filtered;
}

/// Body sphere plus one foot, with the margin the clearance tests measure against.
Configurable::ParamMap defaultGeometry() {
  return Configurable::ParamMap{
      {"collision_sphere_x", "0.0"},
      {"collision_sphere_y", "0.0"},
      {"collision_sphere_z", "0.0"},
      {"collision_sphere_r", std::to_string(kBodyRadius)},
      {"collision_sphere_safe_margins", std::to_string(kBodyMargin)},
      {"contact_sphere_x", "0.0"},
      {"contact_sphere_y", "0.0"},
      {"contact_sphere_z", std::to_string(kContactOffsetZ)},
      {"contact_sphere_r", std::to_string(kContactRadius)},
  };
}

// ---------------------------------------------------------------------------
// The verdict: what partial observation is enough to commit to.
// ---------------------------------------------------------------------------

TEST(SphereSweepVerdict, ABodyInsideAnObstacleIsUntraversable) {
  StubAgent agent;
  ScriptedTsdfMap map(agent);
  agent.provide(&map, nullptr);
  RobotProbe robot(agent, defaultGeometry());
  scriptHalves(&map, kTouching, kTouching);

  EXPECT_EQ(robot.getStateTraversabilityRaw(robot.stateAtTestHeight(), &map), Traversability::kUntraversable);
}

TEST(SphereSweepVerdict, AFootObservedOffTheGroundIsUntraversable) {
  StubAgent agent;
  ScriptedTsdfMap map(agent);
  agent.provide(&map, nullptr);
  RobotProbe robot(agent, defaultGeometry());
  scriptHalves(&map, kFarFromObstacles, kFarFromObstacles);

  EXPECT_EQ(robot.getStateTraversabilityRaw(robot.stateAtTestHeight(), &map), Traversability::kUntraversable);
}

TEST(SphereSweepVerdict, EverySphereObservedPassingIsTraversable) {
  StubAgent agent;
  ScriptedTsdfMap map(agent);
  agent.provide(&map, nullptr);
  RobotProbe robot(agent, defaultGeometry());
  scriptHalves(&map, kFarFromObstacles, kTouching);

  EXPECT_EQ(robot.getStateTraversabilityRaw(robot.stateAtTestHeight(), &map), Traversability::kTraversable);
}

TEST(SphereSweepVerdict, DefaultThresholdLeavesAnyUnobservedSphereUnproven) {
  StubAgent agent;
  ScriptedTsdfMap map(agent);
  agent.provide(&map, nullptr);
  RobotProbe robot(agent, defaultGeometry());
  scriptHalves(&map, std::nullopt, kTouching);

  EXPECT_EQ(robot.getStateTraversabilityRaw(robot.stateAtTestHeight(), &map), Traversability::kPotentiallyTraversable);
}

TEST(SphereSweepVerdict, LoweredThresholdCommitsOnTheObservedFootAlone) {
  StubAgent agent;
  ScriptedTsdfMap map(agent);
  agent.provide(&map, nullptr);
  Configurable::ParamMap params = defaultGeometry();
  params["min_observed_spheres"] = "1";
  RobotProbe robot(agent, params);
  scriptHalves(&map, std::nullopt, kTouching);

  EXPECT_EQ(robot.getStateTraversabilityRaw(robot.stateAtTestHeight(), &map), Traversability::kTraversable);
}

TEST(SphereSweepVerdict, NothingObservedStaysUnprovenAtAnyPositiveThreshold) {
  StubAgent agent;
  ScriptedTsdfMap map(agent);
  agent.provide(&map, nullptr);
  Configurable::ParamMap params = defaultGeometry();
  params["min_observed_spheres"] = "1";
  RobotProbe robot(agent, params);
  scriptHalves(&map, std::nullopt, std::nullopt);

  EXPECT_EQ(robot.getStateTraversabilityRaw(robot.stateAtTestHeight(), &map), Traversability::kPotentiallyTraversable);
}

TEST(SphereSweepVerdict, ARobotWithNoSpheresIsTraversableAtTheDefaultThreshold) {
  StubAgent agent;
  ScriptedTsdfMap map(agent);
  agent.provide(&map, nullptr);
  // No geometry at all: the sentinel resolves to "all zero of them observed", which is
  // vacuously met.
  RobotProbe robot(agent, Configurable::ParamMap{});

  EXPECT_EQ(robot.getStateTraversabilityRaw(robot.stateAtTestHeight(), &map), Traversability::kTraversable);
}

TEST(SphereSweepVerdict, RejectsAThresholdTheGeometryCanNeverMeet) {
  StubAgent agent;
  ScriptedTsdfMap map(agent);
  agent.provide(&map, nullptr);
  Configurable::ParamMap params = defaultGeometry();
  params["min_observed_spheres"] = "3";  // only two spheres exist
  RobotProbe robot(agent, params);

  std::string error_message;
  EXPECT_FALSE(robot.checkParamsValid(&error_message));
  EXPECT_NE(error_message.find("min_observed_spheres expected <= 2"), std::string::npos);
}

// ---------------------------------------------------------------------------
// The margin: how much room an observed body has left.
// ---------------------------------------------------------------------------

TEST(SphereClearance, MarginFullyClearReportsFullClearance) {
  StubAgent agent;
  ScriptedTsdfMap map(agent);
  agent.provide(&map, nullptr);
  RobotProbe robot(agent, defaultGeometry());
  scriptHalves(&map, kBodyRadius + kBodyMargin, kTouching);

  EXPECT_DOUBLE_EQ(robot.getStateClearance(robot.stateAtTestHeight(), &map), 1.0);
}

TEST(SphereClearance, MarginBeyondFullIsNotRewarded) {
  StubAgent agent;
  ScriptedTsdfMap map(agent);
  agent.provide(&map, nullptr);
  RobotProbe robot(agent, defaultGeometry());
  scriptHalves(&map, kFarFromObstacles, kTouching);

  EXPECT_DOUBLE_EQ(robot.getStateClearance(robot.stateAtTestHeight(), &map), 1.0);
}

TEST(SphereClearance, HalfTheMarginReportsHalfClearance) {
  StubAgent agent;
  ScriptedTsdfMap map(agent);
  agent.provide(&map, nullptr);
  RobotProbe robot(agent, defaultGeometry());
  scriptHalves(&map, kBodyRadius + kBodyMargin / 2.0, kTouching);

  EXPECT_NEAR(robot.getStateClearance(robot.stateAtTestHeight(), &map), 0.5, 1e-9);
}

TEST(SphereClearance, MarginUsedUpClampsToTheConfiguredFloor) {
  StubAgent agent;
  ScriptedTsdfMap map(agent);
  agent.provide(&map, nullptr);
  Configurable::ParamMap params = defaultGeometry();
  params["clearance_margin_eps"] = "0.001";
  RobotProbe robot(agent, params);
  // Touching the obstacle, so the relative margin is negative before clamping.
  scriptHalves(&map, kBodyRadius / 2.0, kTouching);

  EXPECT_DOUBLE_EQ(robot.getStateClearance(robot.stateAtTestHeight(), &map), 0.001);
}

TEST(SphereClearance, AnUnobservedSphereDoesNotSpendMargin) {
  StubAgent agent;
  ScriptedTsdfMap map(agent);
  agent.provide(&map, nullptr);
  RobotProbe robot(agent, defaultGeometry());
  scriptHalves(&map, std::nullopt, kTouching);

  EXPECT_DOUBLE_EQ(robot.getStateClearance(robot.stateAtTestHeight(), &map), 1.0);
}

TEST(SphereClearance, ASphereDeclaringNoMarginNeverConstrainsClearance) {
  StubAgent agent;
  ScriptedTsdfMap map(agent);
  agent.provide(&map, nullptr);
  Configurable::ParamMap params = defaultGeometry();
  params["collision_sphere_safe_margins"] = "0.0";
  RobotProbe robot(agent, params);
  scriptHalves(&map, kBodyRadius + 1e-6, kTouching);

  EXPECT_DOUBLE_EQ(robot.getStateClearance(robot.stateAtTestHeight(), &map), 1.0);
}

TEST(SphereClearance, TheTightestSphereSpeaksForTheWholeBody) {
  StubAgent agent;
  ScriptedTsdfMap map(agent);
  agent.provide(&map, nullptr);
  // Two body spheres, stacked so the field can tell them apart: the upper one is
  // clear, the lower one has spent half its margin.
  Configurable::ParamMap params = defaultGeometry();
  params["collision_sphere_x"] = "0.0,0.0";
  params["collision_sphere_y"] = "0.0,0.0";
  params["collision_sphere_z"] = "0.0,1.0";
  params["collision_sphere_r"] = std::to_string(kBodyRadius) + "," + std::to_string(kBodyRadius);
  params["collision_sphere_safe_margins"] = std::to_string(kBodyMargin) + "," + std::to_string(kBodyMargin);
  RobotProbe robot(agent, params);

  map.distance_field = [](const Eigen::Vector3d& position, double* distance) {
    *distance = position.z() > kSensorZ + 0.5 ? kFarFromObstacles : kBodyRadius + kBodyMargin / 2.0;
    return true;
  };

  EXPECT_NEAR(robot.getStateClearance(robot.stateAtTestHeight(), &map), 0.5, 1e-9);
}

TEST(SphereClearance, ContactSpheresAreExcludedEvenWhenTheyCarryAMargin) {
  StubAgent agent;
  ScriptedTsdfMap map(agent);
  agent.provide(&map, nullptr);
  RobotProbe robot(agent, defaultGeometry());

  // A foot is meant to be touching the ground, so its distance-to-surface would
  // read as no margin left. Configuration cannot express this; inject it.
  GeometricSphere foot;
  foot.center = Eigen::Vector3d(0.0, 0.0, kContactOffsetZ);
  foot.radius = kContactRadius;
  foot.safe_margin = kBodyMargin;
  robot.setSpheres({}, {foot});
  scriptHalves(&map, kFarFromObstacles, kTouching);

  EXPECT_DOUBLE_EQ(robot.getStateClearance(robot.stateAtTestHeight(), &map), 1.0);
}

// ---------------------------------------------------------------------------
// Safety: traversability plus the full margin, and where it is enforced.
// ---------------------------------------------------------------------------

TEST(StateSafety, AFullMarginStateIsSafe) {
  StubAgent agent;
  ScriptedTsdfMap map(agent);
  agent.provide(&map, nullptr);
  RobotProbe robot(agent, defaultGeometry());
  scriptHalves(&map, kFarFromObstacles, kTouching);

  EXPECT_TRUE(robot.isStateSafe(robot.stateAtTestHeight(), &map));
}

TEST(StateSafety, ATraversableStateThatHasSpentItsMarginIsNotSafe) {
  StubAgent agent;
  ScriptedTsdfMap map(agent);
  agent.provide(&map, nullptr);
  RobotProbe robot(agent, defaultGeometry());
  scriptHalves(&map, kBodyRadius + kBodyMargin / 2.0, kTouching);

  EXPECT_TRUE(robot.isStateTraversable(robot.stateAtTestHeight(), &map));
  EXPECT_FALSE(robot.isStateSafe(robot.stateAtTestHeight(), &map));
}

TEST(StateSafety, AnUntraversableStateIsNeverSafe) {
  StubAgent agent;
  ScriptedTsdfMap map(agent);
  agent.provide(&map, nullptr);
  RobotProbe robot(agent, defaultGeometry());
  scriptHalves(&map, kTouching, kTouching);

  EXPECT_FALSE(robot.isStateSafe(robot.stateAtTestHeight(), &map));
}

TEST(StationingSafety, CheckOrientationsRejectsATraversableButTightPosition) {
  StubAgent agent;
  ScriptedTsdfMap map(agent);
  agent.provide(&map, nullptr);
  Configurable::ParamMap params = defaultGeometry();
  params["num_orientation_candidates"] = "2";
  RobotProbe robot(agent, params);
  scriptHalves(&map, kBodyRadius + kBodyMargin / 2.0, kTouching);

  std::vector<Eigen::Quaterniond> candidates;
  robot.sampleOrientations(Eigen::Vector3d(0.0, 0.0, kSensorZ), &candidates);
  std::vector<RobotState> states;
  std::vector<bool> is_traversable;
  robot.checkOrientations(Eigen::Vector3d(0.0, 0.0, kSensorZ), candidates, &states, &is_traversable);

  ASSERT_EQ(is_traversable.size(), candidates.size());
  for (std::size_t i = 0; i < is_traversable.size(); ++i) {
    EXPECT_FALSE(is_traversable[i]) << "candidate " << i;
  }
}

TEST(StationingSafety, TheBootstrapSeedIsExemptFromTheMargin) {
  StubAgent agent;
  ScriptedTsdfMap map(agent);
  agent.provide(&map, nullptr);
  Configurable::ParamMap params = defaultGeometry();
  params["num_orientation_candidates"] = "2";
  RobotProbe robot(agent, params);
  scriptHalves(&map, kBodyRadius + kBodyMargin / 2.0, kTouching);

  std::vector<Eigen::Quaterniond> candidates;
  robot.sampleOrientations(Eigen::Vector3d(0.0, 0.0, kSensorZ), &candidates);
  std::vector<RobotState> states;
  std::vector<bool> is_traversable;
  robot.checkOrientations(Eigen::Vector3d(0.0, 0.0, kSensorZ), candidates, &states, &is_traversable, StationingCheck::kExemptBootstrap);

  ASSERT_EQ(is_traversable.size(), candidates.size());
  for (std::size_t i = 0; i < is_traversable.size(); ++i) {
    EXPECT_TRUE(is_traversable[i]) << "candidate " << i;
  }
}

TEST(StationingSafety, AMarginlessRobotIsGatedOnTraversabilityAlone) {
  StubAgent agent;
  ScriptedTsdfMap map(agent);
  agent.provide(&map, nullptr);
  Configurable::ParamMap params = defaultGeometry();
  params["collision_sphere_safe_margins"] = "0.0";
  params["num_orientation_candidates"] = "2";
  RobotProbe robot(agent, params);
  // Barely clear of the obstacle: traversable, and with no margin declared there
  // is nothing further to ask of it.
  scriptHalves(&map, kBodyRadius + 1e-6, kTouching);

  std::vector<Eigen::Quaterniond> candidates;
  robot.sampleOrientations(Eigen::Vector3d(0.0, 0.0, kSensorZ), &candidates);
  std::vector<RobotState> states;
  std::vector<bool> is_traversable;
  robot.checkOrientations(Eigen::Vector3d(0.0, 0.0, kSensorZ), candidates, &states, &is_traversable);

  ASSERT_EQ(is_traversable.size(), candidates.size());
  for (std::size_t i = 0; i < is_traversable.size(); ++i) {
    EXPECT_TRUE(is_traversable[i]) << "candidate " << i;
  }
}

// ---------------------------------------------------------------------------
// Making the margin visible: the shell is the only place the rule shows up.
// ---------------------------------------------------------------------------

TEST(MarginVisualization, TheShellWarmsAsTheMarginIsSpent) {
  StubAgent agent;
  ScriptedTsdfMap map(agent);
  agent.provide(&map, nullptr);
  RobotProbe robot(agent, defaultGeometry());
  const RobotTrajectory trajectory{robot.stateAtTestHeight()};

  scriptHalves(&map, kFarFromObstacles, kTouching);
  const std::vector<VisualizationMarker> intact = markersInNamespace(robot, trajectory, "collision_sphere_safe_margin");
  scriptHalves(&map, kBodyRadius, kTouching);
  const std::vector<VisualizationMarker> spent = markersInNamespace(robot, trajectory, "collision_sphere_safe_margin");

  ASSERT_EQ(intact.size(), 1u);
  ASSERT_EQ(spent.size(), 1u);
  // Pin the direction of the ramp, not the colormap: a shell grazing an obstacle
  // must read hotter than an untouched one, whichever palette supplies the hue.
  EXPECT_GT(spent.front().color.r, intact.front().color.r);
  EXPECT_GT(intact.front().color.b, spent.front().color.b);
  // Still a shell you can see the body through.
  EXPECT_LT(spent.front().color.a, 1.0);
}

TEST(MarginVisualization, AnUnobservedSphereRendersAsAnIntactShell) {
  StubAgent agent;
  ScriptedTsdfMap map(agent);
  agent.provide(&map, nullptr);
  RobotProbe robot(agent, defaultGeometry());
  const RobotTrajectory trajectory{robot.stateAtTestHeight()};

  scriptHalves(&map, kFarFromObstacles, kTouching);
  const std::vector<VisualizationMarker> observed = markersInNamespace(robot, trajectory, "collision_sphere_safe_margin");
  scriptHalves(&map, std::nullopt, kTouching);
  const std::vector<VisualizationMarker> unobserved = markersInNamespace(robot, trajectory, "collision_sphere_safe_margin");

  // The metric already treats unobserved as full margin, so the shell agrees with
  // it rather than inventing a third appearance. The consequence worth knowing: a
  // shell reading intact means "nothing says otherwise", not "measured clear".
  ASSERT_EQ(observed.size(), 1u);
  ASSERT_EQ(unobserved.size(), 1u);
  EXPECT_DOUBLE_EQ(unobserved.front().color.r, observed.front().color.r);
  EXPECT_DOUBLE_EQ(unobserved.front().color.b, observed.front().color.b);
}

TEST(MarginVisualization, WithoutADistanceFieldMapTheShellStaysNeutral) {
  StubAgent agent;
  PlainMap map(agent);  // a Map with no distance field at all
  agent.provide(&map, nullptr);
  RobotProbe robot(agent, defaultGeometry());

  const std::vector<VisualizationMarker> shells = markersInNamespace(robot, {robot.stateAtTestHeight()}, "collision_sphere_safe_margin");

  ASSERT_EQ(shells.size(), 1u);
  EXPECT_DOUBLE_EQ(shells.front().color.r, shells.front().color.g);
  EXPECT_DOUBLE_EQ(shells.front().color.g, shells.front().color.b);
}

TEST(MarginVisualization, TheStrideThinsTheBodiesButKeepsBothEndpoints) {
  StubAgent agent;
  ScriptedTsdfMap map(agent);
  agent.provide(&map, nullptr);
  Configurable::ParamMap params = defaultGeometry();
  params["collision_sphere_visualization_stride"] = "10";
  RobotProbe robot(agent, params);
  scriptHalves(&map, kFarFromObstacles, kTouching);

  RobotTrajectory trajectory;
  for (int i = 0; i < 5; ++i) {
    trajectory.push_back(robot.stateAt(Eigen::Vector3d(0.2 * i, 0.0, kSensorZ)));
  }

  const std::vector<VisualizationMarker> bodies = markersInNamespace(robot, trajectory, "collision_sphere");

  // A stride past the end of the trajectory leaves exactly the two poses the
  // stationing rule judges, rather than one or none.
  ASSERT_EQ(bodies.size(), 2u);
  EXPECT_TRUE(bodies.front().position.isApprox(trajectory.front().getBasePosition()));
  EXPECT_TRUE(bodies.back().position.isApprox(trajectory.back().getBasePosition()));
}

TEST(MarginVisualization, TheDefaultStrideDrawsEveryState) {
  StubAgent agent;
  ScriptedTsdfMap map(agent);
  agent.provide(&map, nullptr);
  RobotProbe robot(agent, defaultGeometry());
  scriptHalves(&map, kFarFromObstacles, kTouching);

  RobotTrajectory trajectory;
  for (int i = 0; i < 4; ++i) {
    trajectory.push_back(robot.stateAt(Eigen::Vector3d(0.2 * i, 0.0, kSensorZ)));
  }

  EXPECT_EQ(markersInNamespace(robot, trajectory, "collision_sphere").size(), trajectory.size());
}

}  // namespace
}  // namespace open_apero
