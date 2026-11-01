/**
 * @file test_simple_robot_sampling.cpp
 * @brief Narrowing the position draw to the mapped region, and the cases that must not narrow it.
 *
 * Where the mapped region is missing, disjoint or in the wrong frame, the draw sequence must equal
 * an unrestricted robot's, draw for draw (same seed, same number of draws).
 */

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "open_apero_core/plugin/map/map.h"
#include "open_apero_core/plugin/plugin_factory_config.h"
#include "open_apero_core/plugin/robot_model/simple_robot.h"

#include "plugin_test_stubs.h"

namespace open_apero {
namespace {

/// @brief Shared by every robot here, so they all draw the same RNG stream.
constexpr const char* kNamespace = "/sampling_robot";

/// @brief The volume every test plans in, wider than any mapped region below.
constexpr double kVolumeXyHalfWidth = 10.0;
constexpr double kVolumeZMin = -1.0;
constexpr double kVolumeZMax = 5.0;

constexpr int kNumDraws = 200;

/// ConfigAgent with no budget; its fixed seed 7 makes the sampling draws reproduce across runs.
class SamplingAgent : public ConfigAgent {
 public:
  SamplingAgent() { setCostBudget(0.0); }
};

/// @brief A map that reports whatever region a test gives it, and counts who asks.
class RegionMap : public Map {
 public:
  explicit RegionMap(AgentInterface& agent) : Map(agent) {}

  void setupFromParamMap(ParamMap* /*param_map*/) override {}

  bool getTotalMapRegion(Eigen::Vector3d* total_map_min, Eigen::Vector3d* total_map_max) override {
    ++num_region_queries;
    if (!has_region) {
      return false;
    }
    *total_map_min = region_min;
    *total_map_max = region_max;
    return true;
  }

  void setRegion(const Eigen::Vector3d& min, const Eigen::Vector3d& max) {
    region_min = min;
    region_max = max;
    has_region = true;
  }

  bool has_region = false;
  Eigen::Vector3d region_min = Eigen::Vector3d::Zero();
  Eigen::Vector3d region_max = Eigen::Vector3d::Zero();
  int num_region_queries = 0;
};

void installVolume(SamplingAgent* agent, const std::string& ns, double rotation_degrees = 0.0) {
  agent->factory().setParams(ns, {{"x_min", std::to_string(-kVolumeXyHalfWidth)},
                                  {"x_max", std::to_string(kVolumeXyHalfWidth)},
                                  {"y_min", std::to_string(-kVolumeXyHalfWidth)},
                                  {"y_max", std::to_string(kVolumeXyHalfWidth)},
                                  {"z_min", std::to_string(kVolumeZMin)},
                                  {"z_max", std::to_string(kVolumeZMax)},
                                  {"rotation", std::to_string(rotation_degrees)}});
}

Configurable::ParamMap robotParams(bool restrict_to_mapped_region, bool sample_z = false) {
  return {{"param_namespace", kNamespace},
          {"restrict_sampling_to_mapped_region", restrict_to_mapped_region ? "1" : "0"},
          {"sample_z", sample_z ? "1" : "0"}};
}

std::vector<Eigen::Vector3d> draw(robot_model::SimpleRobot* robot, int count, double z_default = 0.0) {
  std::vector<Eigen::Vector3d> positions;
  positions.reserve(count);
  for (int i = 0; i < count; ++i) {
    Eigen::Vector3d position;
    EXPECT_TRUE(robot->samplePosition(&position, z_default));
    positions.push_back(position);
  }
  return positions;
}

const VisualizationMarker* findMarker(const VisualizationMarkers& markers, const std::string& marker_namespace, int type) {
  const auto& all = markers.getMarkers();
  const auto it = std::find_if(all.begin(), all.end(), [&marker_namespace, type](const VisualizationMarker& marker) {
    return marker.ns == marker_namespace && marker.type == type;
  });
  return it == all.end() ? nullptr : &*it;
}

/// @brief What an unrestricted robot draws: the reference every fallback must match.
///
/// sample_z has to match the robot under test: a sampled height consumes a third
/// RNG draw per position, which would shift the whole stream against it.
std::vector<Eigen::Vector3d> volumeOnlyDraws(int count, bool sample_z = false) {
  SamplingAgent agent;
  RegionMap map(agent);
  robot_model::SimpleRobot robot(agent);
  agent.provide(&map, &robot);
  installVolume(&agent, std::string(kNamespace) + "/bounding_volume");
  Configurable::ParamMap params = robotParams(false, sample_z);
  robot.setupFromParamMap(&params);
  return draw(&robot, count);
}

TEST(SimpleRobotSampling, LeavesTheDrawAloneAndDoesNotQueryTheMapWhenDisabled) {
  SamplingAgent agent;
  RegionMap map(agent);
  robot_model::SimpleRobot robot(agent);
  agent.provide(&map, &robot);
  map.setRegion(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(2.0, 3.0, 1.0));
  installVolume(&agent, std::string(kNamespace) + "/bounding_volume");
  Configurable::ParamMap params = robotParams(false);
  robot.setupFromParamMap(&params);

  robot.update();

  // The per-cycle query walks the whole map, so a robot that has not opted in
  // must not pay for it -- nor even reach for the map, which lighter agents in
  // other tests do not provide.
  EXPECT_EQ(map.num_region_queries, 0);
  EXPECT_EQ(draw(&robot, kNumDraws), volumeOnlyDraws(kNumDraws));
}

TEST(SimpleRobotSampling, NarrowsTheHorizontalDrawToTheMappedRegion) {
  SamplingAgent agent;
  RegionMap map(agent);
  robot_model::SimpleRobot robot(agent);
  agent.provide(&map, &robot);
  map.setRegion(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(2.0, 3.0, 1.0));
  installVolume(&agent, std::string(kNamespace) + "/bounding_volume");
  Configurable::ParamMap params = robotParams(true);
  robot.setupFromParamMap(&params);
  robot.update();

  const std::vector<Eigen::Vector3d> positions = draw(&robot, kNumDraws);

  double x_lowest = kVolumeXyHalfWidth;
  double x_highest = -kVolumeXyHalfWidth;
  for (const auto& position : positions) {
    EXPECT_GE(position.x(), 0.0);
    EXPECT_LE(position.x(), 2.0);
    EXPECT_GE(position.y(), 0.0);
    EXPECT_LE(position.y(), 3.0);
    x_lowest = std::min(x_lowest, position.x());
    x_highest = std::max(x_highest, position.x());
  }
  // The draw fills the narrowed box rather than collapsing inside it.
  EXPECT_LT(x_lowest, 0.2);
  EXPECT_GT(x_highest, 1.8);
}

TEST(SimpleRobotSampling, TheBoundingVolumeStillCapsTheNarrowedDraw) {
  SamplingAgent agent;
  RegionMap map(agent);
  robot_model::SimpleRobot robot(agent);
  agent.provide(&map, &robot);
  // A region reaching past the volume is still capped by the volume.
  map.setRegion(Eigen::Vector3d(8.0, -40.0, 0.0), Eigen::Vector3d(40.0, 40.0, 1.0));
  installVolume(&agent, std::string(kNamespace) + "/bounding_volume");
  Configurable::ParamMap params = robotParams(true);
  robot.setupFromParamMap(&params);
  robot.update();

  for (const auto& position : draw(&robot, kNumDraws)) {
    EXPECT_GE(position.x(), 8.0);
    EXPECT_LE(position.x(), kVolumeXyHalfWidth);
    EXPECT_GE(position.y(), -kVolumeXyHalfWidth);
    EXPECT_LE(position.y(), kVolumeXyHalfWidth);
  }
}

TEST(SimpleRobotSampling, FallsBackToTheVolumeWhileTheMapHasNoRegion) {
  SamplingAgent agent;
  RegionMap map(agent);
  robot_model::SimpleRobot robot(agent);
  agent.provide(&map, &robot);
  installVolume(&agent, std::string(kNamespace) + "/bounding_volume");
  Configurable::ParamMap params = robotParams(true);
  robot.setupFromParamMap(&params);

  // An empty map at episode start reports no region at all.
  robot.update();

  EXPECT_EQ(map.num_region_queries, 1);
  EXPECT_EQ(draw(&robot, kNumDraws), volumeOnlyDraws(kNumDraws));
}

TEST(SimpleRobotSampling, FallsBackToTheVolumeWhenTheRegionDoesNotMeetIt) {
  SamplingAgent agent;
  RegionMap map(agent);
  robot_model::SimpleRobot robot(agent);
  agent.provide(&map, &robot);
  map.setRegion(Eigen::Vector3d(100.0, 100.0, 0.0), Eigen::Vector3d(101.0, 101.0, 1.0));
  installVolume(&agent, std::string(kNamespace) + "/bounding_volume");
  Configurable::ParamMap params = robotParams(true);
  robot.setupFromParamMap(&params);
  robot.update();

  EXPECT_EQ(draw(&robot, kNumDraws), volumeOnlyDraws(kNumDraws));
}

TEST(SimpleRobotSampling, NarrowsHeightToTheMappedRegionWhenHeightIsSampled) {
  SamplingAgent agent;
  RegionMap map(agent);
  robot_model::SimpleRobot robot(agent);
  agent.provide(&map, &robot);
  map.setRegion(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(2.0, 3.0, 0.1));
  installVolume(&agent, std::string(kNamespace) + "/bounding_volume");
  Configurable::ParamMap params = robotParams(true, /*sample_z=*/true);
  robot.setupFromParamMap(&params);
  robot.update();

  // A height above everything the map has reached is as wasted a draw as a
  // position beside it, so the band narrows on the same terms.
  double z_highest = kVolumeZMin;
  for (const auto& position : draw(&robot, kNumDraws)) {
    EXPECT_GE(position.z(), 0.0);
    EXPECT_LE(position.z(), 0.1);
    z_highest = std::max(z_highest, position.z());
  }
  EXPECT_GT(z_highest, 0.08);
}

TEST(SimpleRobotSampling, VisualizesTheStaticAndEffectiveSamplingVolumesDistinctly) {
  SamplingAgent agent;
  RegionMap map(agent);
  robot_model::SimpleRobot robot(agent);
  agent.provide(&map, &robot);
  map.setRegion(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(2.0, 3.0, 0.1));
  installVolume(&agent, std::string(kNamespace) + "/bounding_volume");
  Configurable::ParamMap params = robotParams(true, /*sample_z=*/true);
  robot.setupFromParamMap(&params);
  robot.update();

  VisualizationMarkers markers;
  robot.getVisualization({}, &markers);

  const VisualizationMarker* static_outline = findMarker(markers, "bounding_volume", VisualizationMarker::LINE_LIST);
  ASSERT_NE(static_outline, nullptr);
  EXPECT_DOUBLE_EQ(static_outline->color.r, Color::orange().r);
  EXPECT_DOUBLE_EQ(static_outline->color.g, Color::orange().g);
  EXPECT_DOUBLE_EQ(static_outline->color.b, Color::orange().b);

  const VisualizationMarker* effective_fill = findMarker(markers, "sampling_volume", VisualizationMarker::CUBE);
  ASSERT_NE(effective_fill, nullptr);
  EXPECT_TRUE(effective_fill->position.isApprox(Eigen::Vector3d(1.0, 1.5, 0.05)));
  EXPECT_TRUE(effective_fill->scale.isApprox(Eigen::Vector3d(2.0, 3.0, 0.1)));
  EXPECT_DOUBLE_EQ(effective_fill->color.r, Color::teal().r);
  EXPECT_DOUBLE_EQ(effective_fill->color.g, Color::teal().g);
  EXPECT_DOUBLE_EQ(effective_fill->color.b, Color::teal().b);

  const VisualizationMarker* effective_outline = findMarker(markers, "sampling_volume", VisualizationMarker::LINE_LIST);
  ASSERT_NE(effective_outline, nullptr);
  EXPECT_EQ(effective_outline->points.size(), 24u);
}

TEST(SimpleRobotSampling, FallsBackToTheVolumeWhenTheHeightBandDoesNotMeetIt) {
  SamplingAgent agent;
  RegionMap map(agent);
  robot_model::SimpleRobot robot(agent);
  agent.provide(&map, &robot);
  // Overlapping horizontally, disjoint in height.
  map.setRegion(Eigen::Vector3d(0.0, 0.0, 100.0), Eigen::Vector3d(2.0, 3.0, 101.0));
  installVolume(&agent, std::string(kNamespace) + "/bounding_volume");
  Configurable::ParamMap params = robotParams(true, /*sample_z=*/true);
  robot.setupFromParamMap(&params);
  robot.update();

  EXPECT_EQ(draw(&robot, kNumDraws), volumeOnlyDraws(kNumDraws, /*sample_z=*/true));
}

TEST(SimpleRobotSampling, KeepsTheDefaultHeightWhenHeightIsNotSampled) {
  SamplingAgent agent;
  RegionMap map(agent);
  robot_model::SimpleRobot robot(agent);
  agent.provide(&map, &robot);
  map.setRegion(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(2.0, 3.0, 0.1));
  installVolume(&agent, std::string(kNamespace) + "/bounding_volume");
  Configurable::ParamMap params = robotParams(true, /*sample_z=*/false);
  robot.setupFromParamMap(&params);
  robot.update();

  // Well outside the region's height band: the default is passed through, not clamped.
  constexpr double kZDefault = 3.0;
  for (const auto& position : draw(&robot, kNumDraws, kZDefault)) {
    EXPECT_DOUBLE_EQ(position.z(), kZDefault);
  }
}

TEST(SimpleRobotSampling, AnUnreachableHeightBandCannotVetoTheHorizontalNarrowing) {
  SamplingAgent agent;
  RegionMap map(agent);
  robot_model::SimpleRobot robot(agent);
  agent.provide(&map, &robot);
  // Height disjoint from the volume, but the height is not drawn, so it has no
  // say: the horizontal narrowing must still happen.
  map.setRegion(Eigen::Vector3d(0.0, 0.0, 100.0), Eigen::Vector3d(2.0, 3.0, 101.0));
  installVolume(&agent, std::string(kNamespace) + "/bounding_volume");
  Configurable::ParamMap params = robotParams(true, /*sample_z=*/false);
  robot.setupFromParamMap(&params);
  robot.update();

  for (const auto& position : draw(&robot, kNumDraws)) {
    EXPECT_GE(position.x(), 0.0);
    EXPECT_LE(position.x(), 2.0);
    EXPECT_GE(position.y(), 0.0);
    EXPECT_LE(position.y(), 3.0);
  }
}

TEST(SimpleRobotSampling, ResetEpisodeDropsTheCachedRegion) {
  SamplingAgent agent;
  RegionMap map(agent);
  robot_model::SimpleRobot robot(agent);
  agent.provide(&map, &robot);
  map.setRegion(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(2.0, 3.0, 1.0));
  installVolume(&agent, std::string(kNamespace) + "/bounding_volume");
  Configurable::ParamMap params = robotParams(true);
  robot.setupFromParamMap(&params);
  robot.update();
  draw(&robot, 10);

  // The next episode starts on an empty map, and resetEpisode also rewinds the
  // RNG, so the draw returns to the unrestricted reference stream.
  robot.resetEpisode();

  EXPECT_EQ(draw(&robot, kNumDraws), volumeOnlyDraws(kNumDraws));
}

TEST(SimpleRobotSampling, DropsTheCachedRegionWhenTheMapStopsReportingOne) {
  SamplingAgent agent;
  RegionMap map(agent);
  robot_model::SimpleRobot robot(agent);
  agent.provide(&map, &robot);
  map.setRegion(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(2.0, 3.0, 1.0));
  installVolume(&agent, std::string(kNamespace) + "/bounding_volume");
  Configurable::ParamMap params = robotParams(true);
  robot.setupFromParamMap(&params);
  robot.update();

  // A map reporting no region leaves the extents untouched rather than zeroing
  // them, so the cached ones have to be dropped or they outlive their cycle.
  map.has_region = false;
  robot.update();

  EXPECT_EQ(draw(&robot, kNumDraws), volumeOnlyDraws(kNumDraws));
}

TEST(SimpleRobotSampling, DoesNotNarrowARotatedVolume) {
  SamplingAgent agent;
  RegionMap map(agent);
  robot_model::SimpleRobot robot(agent);
  agent.provide(&map, &robot);
  map.setRegion(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(2.0, 3.0, 1.0));
  // A rotated volume states its extents in its own frame while the mapped region
  // is world-axis-aligned, so the two cannot be intersected componentwise. Point
  // the robot at the rotated volume through bounding_volume_args rather than
  // moving its own namespace, which would change its RNG stream.
  installVolume(&agent, "/rotated_volume", /*rotation_degrees=*/45.0);
  Configurable::ParamMap params = robotParams(true);
  params["bounding_volume_args"] = "/rotated_volume";
  robot.setupFromParamMap(&params);
  robot.update();

  EXPECT_EQ(draw(&robot, kNumDraws), volumeOnlyDraws(kNumDraws));

  VisualizationMarkers markers;
  robot.getVisualization({}, &markers);
  const VisualizationMarker* sampling_fill = findMarker(markers, "sampling_volume", VisualizationMarker::CUBE);
  ASSERT_NE(sampling_fill, nullptr);
  EXPECT_TRUE(sampling_fill->position.isApprox(Eigen::Vector3d::Zero()));
  EXPECT_TRUE(sampling_fill->scale.isApprox(Eigen::Vector3d(20.0, 20.0, 0.04)));
  const VisualizationMarker* static_outline = findMarker(markers, "bounding_volume", VisualizationMarker::LINE_LIST);
  ASSERT_NE(static_outline, nullptr);
  ASSERT_GE(static_outline->points.size(), 2u);
  const Eigen::Quaterniond volume_to_world(Eigen::AngleAxisd(std::acos(-1.0) / 4.0, Eigen::Vector3d::UnitZ()));
  EXPECT_TRUE(static_outline->points[0].isApprox(volume_to_world * Eigen::Vector3d(-10.0, -10.0, -1.0)));
  EXPECT_TRUE(static_outline->points[1].isApprox(volume_to_world * Eigen::Vector3d(10.0, -10.0, -1.0)));
}

}  // namespace
}  // namespace open_apero
