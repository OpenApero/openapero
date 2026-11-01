/**
 * @file test_plugin_behavior.cpp
 * @brief Direct contracts of plugins no other test drives: RRT growth, yaw selection, sensor grids
 *        and view boxes, the occupied voxel a ray stops at, clearing voxels seen from ancestors, and
 *        the evaluator never being handed an observed node.
 *
 * The sensor view checks pin the frustum outline the camera and LiDAR models draw, point for point,
 * against the pinhole and spherical ray formulas.
 */

#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "open_apero_core/common/typedef.h"
#include "open_apero_core/common/utils.h"
#include "open_apero_core/gain/gain_anchor.h"
#include "open_apero_core/graph/edge.h"
#include "open_apero_core/graph/graph_node.h"
#include "open_apero_core/graph/node.h"
#include "open_apero_core/graph/tree_node.h"
#include "open_apero_core/plugin/evaluator/evaluator.h"
#include "open_apero_core/plugin/evaluator/simple_yaw_planning_evaluator.h"
#include "open_apero_core/plugin/evaluator/simulated_sensor_evaluator.h"
#include "open_apero_core/plugin/generator/rrt.h"
#include "open_apero_core/plugin/map/occupancy_map.h"
#include "open_apero_core/plugin/plugin.h"
#include "open_apero_core/plugin/plugin_registry.h"
#include "open_apero_core/plugin/robot_model/simple_robot.h"
#include "open_apero_core/plugin/sensor_model/iterative_ray_caster.h"
#include "open_apero_core/plugin/sensor_model/iterative_ray_caster_lidar.h"
#include "open_apero_core/plugin/sensor_model/simple_ray_caster.h"
#include "open_apero_core/state/robot_state.h"
#include "open_apero_core/visualization/visualization_markers.h"
#include "plugin_test_stubs.h"

namespace open_apero {
namespace {

/// Yaw the stub evaluator rewards most [rad].
constexpr double kPreferredYaw = 1.0;

/// Scores an orientation by how closely its yaw faces kPreferredYaw; casts no rays.
class TestYawGainEvaluator : public evaluator::SimulatedSensorEvaluator {
 public:
  explicit TestYawGainEvaluator(AgentInterface& agent) : SimulatedSensorEvaluator(agent) {}

  void setupFromParamMap(ParamMap* /*param_map*/) override {}

  double getRealAccumulatedGain() const override { return 0.0; }

  static PluginRegistry::Registration<TestYawGainEvaluator> registration;

 protected:
  bool evaluateUnobserved(const NodeClusterPtrs& cluster) override {
    for (Node* node : cluster) {
      node->gain_anchors = {std::make_shared<GainAnchor>(std::cos(node->robot_state.getSensorYaw() - kPreferredYaw))};
    }
    return true;
  }

  bool evaluateFromVisibleVoxels(Node* /*node*/) override { return true; }
};
PluginRegistry::Registration<TestYawGainEvaluator> TestYawGainEvaluator::registration("TestYawGainEvaluator");

/// Applies orientations directly, standing in for the owning layer's generator.
class YawEvaluatorProbe : public evaluator::SimpleYawPlanningEvaluator {
 public:
  explicit YawEvaluatorProbe(AgentInterface& agent) : SimpleYawPlanningEvaluator(agent) {}

 protected:
  bool setNodeOrientation(Node* node, const Eigen::Quaterniond& orientation) override {
    node->robot_state.setSensorOrientation(orientation);
    return true;
  }
};

/// Prices edges by their length in states; there is no owning layer to ask.
class RrtProbe : public generator::RRT {
 public:
  explicit RrtProbe(AgentInterface& agent) : RRT(agent) {}

  bool computeCost(Edge* edge) override {
    edge->cost = static_cast<double>(edge->trajectory.size());
    return true;
  }
};

class CameraProbe : public sensor_model::IterativeRayCaster {
 public:
  explicit CameraProbe(AgentInterface& agent) : IterativeRayCaster(agent) {}
  int rayColumns() const { return c_res_x_; }
  int rayRows() const { return c_res_y_; }
  void direction(Eigen::Vector3d* result, double relative_x, double relative_y) { getDirectionVector(result, relative_x, relative_y); }
};

class LidarProbe : public sensor_model::IterativeRayCasterLidar {
 public:
  explicit LidarProbe(AgentInterface& agent) : IterativeRayCasterLidar(agent) {}
  int rayColumns() const { return c_res_x_; }
  int rayRows() const { return c_res_y_; }
};

/// Records the clusters the Evaluator base hands to its hooks, and refuses each one.
class ClusterRecordingEvaluator : public Evaluator {
 public:
  explicit ClusterRecordingEvaluator(AgentInterface& agent) : Evaluator(agent) {}

  void setupFromParamMap(ParamMap* /*param_map*/) override {}

  double getRealAccumulatedGain() const override { return 0.0; }

  std::vector<NodeClusterPtrs> evaluated;
  std::vector<NodeClusterPtrs> reevaluated;

 protected:
  bool evaluateUnobserved(const NodeClusterPtrs& cluster) override {
    evaluated.push_back(cluster);
    return false;
  }

  bool reevaluateUnobserved(const NodeClusterPtrs& cluster, const Node* /*current_node*/) override {
    reevaluated.push_back(cluster);
    return false;
  }
};

class PluginBehaviorTest : public ::testing::Test {
 protected:
  PluginBehaviorTest() {
    agent_.provide(&map_, &robot_);
    agent_.factory().setParams(
        "/robot/bounding_volume",
        {{"x_min", "-3.0"}, {"x_max", "3.0"}, {"y_min", "-3.0"}, {"y_max", "3.0"}, {"z_min", "-1.0"}, {"z_max", "1.0"}});
    Configurable::ParamMap robot_params{{"param_namespace", "/robot"}};
    robot_.setupFromParamMap(&robot_params);
  }

  RobotState stateAt(const Eigen::Vector3d& position, double yaw = 0.0) const {
    RobotState state;
    robot_.placeRobotAtSensorPose(&state, {position, quaternionFromYaw(yaw)});
    return state;
  }

  ConfigAgent agent_;
  FreeTsdfMap map_{agent_};
  robot_model::SimpleRobot robot_{agent_};
};

TEST_F(PluginBehaviorTest, RrtGrowsEdgesFromItsStartNode) {
  RrtProbe rrt(agent_);
  Configurable::ParamMap params{{"param_namespace", "/rrt"}, {"min_edge_length", "0.5"}, {"max_edge_length", "2.0"}};
  rrt.setupFromParamMap(&params);

  const auto [graph, root] = rrt.generateInitNode(stateAt(Eigen::Vector3d::Zero()));
  EXPECT_EQ(graph, nullptr) << "a tree keeps no boost graph";

  std::vector<Edge*> edges;
  for (int round = 0; round < 10; ++round) {
    NodeClusterPtrs cluster = rrt.generateNewNode(root);
    std::vector<Edge*> new_edges;
    if (!cluster.empty() && rrt.generateNewEdges(cluster, new_edges)) {
      edges.insert(edges.end(), new_edges.begin(), new_edges.end());
    }
  }

  ASSERT_FALSE(edges.empty());
  // The first child is seeded at the start, so the first edge leaves the start node.
  EXPECT_TRUE(edges.front()->trajectory.front().isSameSensorPose(root->robot_state));
  for (const Edge* edge : edges) {
    EXPECT_GE(edge->trajectory.size(), 2u);
    EXPECT_TRUE(edge->is_traversable);
  }
  EXPECT_EQ(rrt.getAllNodes(root).size(), edges.size() + 1) << "every tree node but the root owns one edge";
}

TEST_F(PluginBehaviorTest, SimpleYawPlanningEvaluatorKeepsTheBestScoringYaw) {
  agent_.factory().setParams("/yaw/following_evaluator", {{"type", "TestYawGainEvaluator"}});
  YawEvaluatorProbe evaluator(agent_);
  Configurable::ParamMap params{{"param_namespace", "/yaw"}};
  evaluator.setupFromParamMap(&params);

  TreeNode node;
  node.robot_state = stateAt(Eigen::Vector3d::Zero());
  robot_.sampleOrientations(Eigen::Vector3d::Zero(), &node.candidate_orientations);
  node.is_candidate_traversable.assign(node.candidate_orientations.size(), true);
  ASSERT_EQ(node.candidate_orientations.size(), 12u);

  ASSERT_TRUE(evaluator.evaluate({&node}));

  // Of twelve yaws 30 degrees apart, 60 degrees faces 1 rad most closely.
  EXPECT_NEAR(node.robot_state.getSensorYaw(), M_PI / 3.0, 1e-9);
  EXPECT_NEAR(node.getSingleNodeGain(), std::cos(M_PI / 3.0 - kPreferredYaw), 1e-12);
}

TEST_F(PluginBehaviorTest, CameraRayGridFollowsRangeFieldOfViewAndVoxelSize) {
  CameraProbe camera(agent_);
  Configurable::ParamMap params{{"ray_length", "5.0"}};
  camera.setupFromParamMap(&params);
  camera.assureParamsValid();

  // Defaults: 640 x 480 px at a 320 px focal length; one ray per 0.2 m voxel at 5 m range.
  const double fov_x = 2.0 * std::atan2(640.0, 640.0);
  const double fov_y = 2.0 * std::atan2(480.0, 640.0);
  EXPECT_EQ(camera.rayColumns(), static_cast<int>(std::ceil(5.0 * fov_x / kStubVoxelSize)));
  EXPECT_EQ(camera.rayRows(), static_cast<int>(std::ceil(5.0 * fov_y / kStubVoxelSize)));
  EXPECT_EQ(camera.rayColumns(), 40);
  EXPECT_EQ(camera.rayRows(), 33);
}

TEST_F(PluginBehaviorTest, LidarRayGridFollowsRangeFieldOfViewAndVoxelSize) {
  LidarProbe lidar(agent_);
  Configurable::ParamMap params{{"ray_length", "5.0"}};
  lidar.setupFromParamMap(&params);
  lidar.assureParamsValid();

  // Defaults: 360 x 30 degrees; the vertical count stays below the 128 beams.
  EXPECT_EQ(lidar.rayColumns(), static_cast<int>(std::ceil(5.0 * 2.0 * M_PI / kStubVoxelSize)));
  EXPECT_EQ(lidar.rayRows(), static_cast<int>(std::ceil(5.0 * (30.0 * M_PI / 180.0) / kStubVoxelSize)));
  EXPECT_EQ(lidar.rayColumns(), 158);
  EXPECT_EQ(lidar.rayRows(), 14);
}

/// Pinhole ray through normalized image coordinates (0, 0) top-left to (1, 1) bottom-right.
Eigen::Vector3d cameraRay(double relative_x, double relative_y) {
  return Eigen::Vector3d(320.0, (0.5 - relative_x) * 640.0, (0.5 - relative_y) * 480.0).normalized();
}

/// LiDAR ray at normalized angular coordinates across a 360 x 30 degree field of view.
Eigen::Vector3d lidarRay(double relative_x, double relative_y) {
  const double phi = (0.5 - relative_x) * 2.0 * M_PI;
  const double theta = M_PI / 2.0 + (relative_y - 0.5) * (30.0 * M_PI / 180.0);
  return Eigen::Vector3d(std::sin(theta) * std::cos(phi), std::sin(theta) * std::sin(phi), std::cos(theta));
}

/// The outline both models draw: four rays from the origin, then the far-plane edges segment by segment.
std::vector<Eigen::Vector3d> expectedOutline(Eigen::Vector3d (*ray)(double, double), int res_x, int res_y, double range) {
  auto far = [&](double x, double y) { return Eigen::Vector3d(range * ray(x, y)); };
  const Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  std::vector<Eigen::Vector3d> points{origin, far(0.0, 0.0), origin, far(0.0, 1.0), origin, far(1.0, 0.0), origin, far(1.0, 1.0)};
  for (int i = 1; i <= res_x; ++i) {
    const double previous = static_cast<double>(i - 1) / res_x;
    const double current = static_cast<double>(i) / res_x;
    points.insert(points.end(), {far(previous, 0.0), far(current, 0.0), far(previous, 1.0), far(current, 1.0)});
  }
  for (int i = 1; i <= res_y; ++i) {
    const double previous = static_cast<double>(i - 1) / res_y;
    const double current = static_cast<double>(i) / res_y;
    points.insert(points.end(), {far(0.0, previous), far(0.0, current), far(1.0, previous), far(1.0, current)});
  }
  return points;
}

void expectOutline(const VisualizationMarkers& markers, const std::vector<Eigen::Vector3d>& expected) {
  ASSERT_EQ(markers.getMarkers().size(), 1u);
  const std::vector<Eigen::Vector3d>& points = markers.getMarkers().front().points;
  ASSERT_EQ(points.size(), expected.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    EXPECT_LT((points[i] - expected[i]).norm(), 1e-9) << "outline point " << i;
  }
}

TEST_F(PluginBehaviorTest, CameraSensorViewTracesTheFrustumOutline) {
  CameraProbe camera(agent_);
  Configurable::ParamMap params{{"ray_length", "5.0"}};
  camera.setupFromParamMap(&params);

  GraphNode node;
  node.robot_state = stateAt(Eigen::Vector3d::Zero());
  VisualizationMarkers markers;
  camera.visualizeSensorView(&markers, &node);

  // One segment per 0.25 m of far-plane arc.
  const int res_x = static_cast<int>(std::ceil(5.0 * 2.0 * std::atan2(640.0, 640.0) / 0.25));
  const int res_y = static_cast<int>(std::ceil(5.0 * 2.0 * std::atan2(480.0, 640.0) / 0.25));
  expectOutline(markers, expectedOutline(&cameraRay, res_x, res_y, 5.0));
}

TEST_F(PluginBehaviorTest, LidarSensorViewTracesTheScanOutline) {
  LidarProbe lidar(agent_);
  Configurable::ParamMap params{{"ray_length", "5.0"}};
  lidar.setupFromParamMap(&params);

  GraphNode node;
  node.robot_state = stateAt(Eigen::Vector3d::Zero());
  VisualizationMarkers markers;
  lidar.visualizeSensorView(&markers, &node);

  const int res_x = static_cast<int>(std::ceil(5.0 * 2.0 * M_PI / 0.25));
  const int res_y = static_cast<int>(std::ceil(5.0 * (30.0 * M_PI / 180.0) / 0.25));
  expectOutline(markers, expectedOutline(&lidarRay, res_x, res_y, 5.0));
}

/// Occupied from x = kWallX on, free before it.
class WallOccupancyMap : public map::OccupancyMap {
 public:
  static constexpr double kWallX = 1.0;

  explicit WallOccupancyMap(AgentInterface& agent) : OccupancyMap(agent) {}
  void setupFromParamMap(ParamMap* /*param_map*/) override {}
  Occupancy getVoxelOccupancy(const Eigen::Vector3d& point) override {
    Eigen::Vector3d center;
    getVoxelCenter(&center, point);
    return center.x() >= kWallX ? Occupancy::kOccupied : Occupancy::kFree;
  }
  double getVoxelSize() override { return kStubVoxelSize; }
  bool getVoxelCenter(Eigen::Vector3d* center, const Eigen::Vector3d& point) override {
    *center = ((point / kStubVoxelSize).array().floor() + 0.5).matrix() * kStubVoxelSize;
    return true;
  }
};

TEST(SimpleRayCasterHits, TheOccupiedVoxelARayStopsAtIsVisible) {
  ConfigAgent agent;
  WallOccupancyMap map(agent);
  agent.provide(&map, nullptr);
  sensor_model::SimpleRayCaster caster(agent);
  Configurable::ParamMap params{{"ray_length", "3.0"}};
  caster.setupFromParamMap(&params);
  caster.assureParamsValid();

  VoxelOccupancyMap visible;
  caster.getVisibleVoxels(&visible, {Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()});

  int num_occupied = 0;
  for (const auto& [center, occupancy] : visible) {
    EXPECT_LT(center.x(), WallOccupancyMap::kWallX + kStubVoxelSize) << "no ray passes the first occupied voxel";
    num_occupied += occupancy == Occupancy::kOccupied ? 1 : 0;
  }
  EXPECT_GT(num_occupied, 0);
}

/// Validation message of a sensor model set up with @p params, or "" when it is valid.
template <class Caster>
std::string casterValidationError(ConfigAgent* agent, const Configurable::ParamMap& params) {
  Caster caster(*agent);
  Configurable::ParamMap map = params;
  caster.setupFromParamMap(&map);
  std::string error;
  return caster.checkParamsValid(&error) ? std::string() : error;
}

TEST_F(PluginBehaviorTest, RayCastersRejectANonPositiveRayStepAndADegenerateGrid) {
  for (const std::string& ray_step : {"0.0", "-0.1"}) {
    SCOPED_TRACE(ray_step);
    EXPECT_EQ(casterValidationError<sensor_model::SimpleRayCaster>(&agent_, {{"ray_step", ray_step}}), "ray_step expected > 0.0");
    EXPECT_EQ(casterValidationError<sensor_model::IterativeRayCaster>(&agent_, {{"ray_step", ray_step}}), "ray_step expected > 0.0");
    EXPECT_EQ(casterValidationError<sensor_model::IterativeRayCasterLidar>(&agent_, {{"ray_step", ray_step}}), "ray_step expected > 0.0");
  }

  EXPECT_EQ(casterValidationError<sensor_model::SimpleRayCaster>(&agent_, {{"downsampling_factor", "0.0"}}),
            "downsampling_factor expected > 0.0");
  EXPECT_EQ(casterValidationError<sensor_model::IterativeRayCasterLidar>(&agent_, {{"downsampling_factor", "-1.0"}}),
            "downsampling_factor expected > 0.0");

  // A 0.1 m range at 0.2 m voxels is one ray per axis.
  EXPECT_NE(casterValidationError<sensor_model::SimpleRayCaster>(&agent_, {{"ray_length", "0.1"}}).find("degenerate ray grid"),
            std::string::npos);
  EXPECT_NE(casterValidationError<sensor_model::IterativeRayCasterLidar>(&agent_, {{"ray_length", "0.1"}}).find("degenerate ray grid"),
            std::string::npos);

  EXPECT_EQ(casterValidationError<sensor_model::SimpleRayCaster>(&agent_, {}), "");
  EXPECT_EQ(casterValidationError<sensor_model::IterativeRayCasterLidar>(&agent_, {}), "");
}

TEST_F(PluginBehaviorTest, TheCameraViewBoxReachesTheFullRangeStraightAhead) {
  // The shipped camera: 480 x 640 px at f = 625.58 px, 3 m range.
  CameraProbe camera(agent_);
  Configurable::ParamMap params{{"ray_length", "3.0"}, {"focal_length", "625.58"}, {"resolution_x", "480"}, {"resolution_y", "640"}};
  camera.setupFromParamMap(&params);

  const Eigen::Vector3d position(1.0, 2.0, 0.5);
  Eigen::Vector3d min;
  Eigen::Vector3d max;
  camera.getSensorViewBoundary(min, max, {position, Eigen::Quaterniond::Identity()});
  EXPECT_NEAR(max.x() - position.x(), 3.0, 1e-12);
  EXPECT_NEAR(min.x(), position.x(), 1e-12);
  // Half the 42 degree horizontal and 54.2 degree vertical view, at the far edge midpoints.
  EXPECT_NEAR(max.y() - position.y(), 3.0 * std::sin(std::atan2(240.0, 625.58)), 1e-9);
  EXPECT_NEAR(max.z() - position.z(), 3.0 * std::sin(std::atan2(320.0, 625.58)), 1e-9);

  // Turned by less than half the field of view, the forward direction is still inside it.
  camera.getSensorViewBoundary(min, max, {position, quaternionFromYaw(0.2)});
  EXPECT_NEAR(max.x() - position.x(), 3.0, 1e-12);
}

TEST_F(PluginBehaviorTest, TheCameraViewBoxIsTightForAPitchedAndYawedCamera) {
  CameraProbe camera(agent_);
  Configurable::ParamMap params{{"ray_length", "3.0"}, {"focal_length", "625.58"}, {"resolution_x", "480"}, {"resolution_y", "640"}};
  camera.setupFromParamMap(&params);

  const Eigen::Vector3d position(0.3, -0.2, 1.0);
  for (const double pitch : {0.3, -0.5}) {
    const Transform3d transform{position, quaternionFromEulerAnglesZyx(Eigen::Vector3d(0.7, pitch, 0.0))};
    Eigen::Vector3d min;
    Eigen::Vector3d max;
    camera.getSensorViewBoundary(min, max, transform);

    // The view is the cone of rays out to ray_length: the box holds every far point and the origin, and each
    // face touches the view. Sampled on a 401 x 401 grid of rays; an extreme on an edge arc or inside the
    // cap is a stationary point there, so the samples miss it by a second-order amount only.
    Eigen::Vector3d sampled_min = position;
    Eigen::Vector3d sampled_max = position;
    constexpr int kSamples = 400;
    for (int i = 0; i <= kSamples; ++i) {
      for (int j = 0; j <= kSamples; ++j) {
        Eigen::Vector3d direction;
        camera.direction(&direction, static_cast<double>(i) / kSamples, static_cast<double>(j) / kSamples);
        const Eigen::Vector3d far_point = position + 3.0 * (transform.orientation * direction).normalized();
        ASSERT_TRUE((far_point.array() >= min.array() - 1e-9).all() && (far_point.array() <= max.array() + 1e-9).all())
            << "pitch " << pitch << ": " << far_point.transpose() << " outside [" << min.transpose() << ", " << max.transpose() << "]";
        sampled_min = sampled_min.cwiseMin(far_point);
        sampled_max = sampled_max.cwiseMax(far_point);
      }
    }
    EXPECT_LT((min - sampled_min).norm(), 1e-4) << "pitch " << pitch << ": " << min.transpose() << " vs " << sampled_min.transpose();
    EXPECT_LT((max - sampled_max).norm(), 1e-4) << "pitch " << pitch << ": " << max.transpose() << " vs " << sampled_max.transpose();
  }
}

TEST_F(PluginBehaviorTest, TheLidarViewBoxSurroundsTheSensor) {
  // Defaults: 360 x 30 degrees.
  LidarProbe lidar(agent_);
  Configurable::ParamMap params{{"ray_length", "5.0"}};
  lidar.setupFromParamMap(&params);

  const Eigen::Vector3d position(1.0, 2.0, 0.5);
  Eigen::Vector3d min;
  Eigen::Vector3d max;
  lidar.getSensorViewBoundary(min, max, {position, quaternionFromYaw(0.3)});
  EXPECT_TRUE((max - position).head<2>().isApprox(Eigen::Vector2d(5.0, 5.0)));
  EXPECT_TRUE((position - min).head<2>().isApprox(Eigen::Vector2d(5.0, 5.0)));
  EXPECT_NEAR(max.z() - position.z(), 5.0 * std::sin(15.0 * M_PI / 180.0), 1e-12);
  EXPECT_NEAR(position.z() - min.z(), 5.0 * std::sin(15.0 * M_PI / 180.0), 1e-12);
}

TEST_F(PluginBehaviorTest, ClearFromParentsReadsTheActiveOrientationOfAYawPlannedParent) {
  agent_.factory().setParams("/yaw/following_evaluator", {{"type", "NaiveEvaluator"}, {"clear_from_parents", "1"}});
  agent_.factory().setParams("/yaw/following_evaluator/sensor_model", {{"type", "IterativeRayCaster"}, {"ray_length", "1.0"}});
  YawEvaluatorProbe evaluator(agent_);
  Configurable::ParamMap params{{"param_namespace", "/yaw"}};
  evaluator.setupFromParamMap(&params);

  // A root never evaluated (no info), a child and a grandchild at the same pose.
  TreeNode root;
  root.robot_state = stateAt(Eigen::Vector3d::Zero());
  const auto place = [this](TreeNode* node) {
    node->robot_state = stateAt(Eigen::Vector3d(0.5, 0.0, 0.0));
    robot_.sampleOrientations(node->robot_state.getSensorPosition(), &node->candidate_orientations);
    node->is_candidate_traversable.assign(node->candidate_orientations.size(), true);
  };
  TreeNode* child = root.spawnChild();
  place(child);
  TreeNode* grandchild = child->spawnChild();
  place(grandchild);

  ASSERT_TRUE(evaluator.evaluate({child}));
  ASSERT_TRUE(evaluator.evaluate({grandchild}));

  const auto* child_info = child->info->as<YawPlanningInfo>();
  const auto* grandchild_info = grandchild->info->as<YawPlanningInfo>();
  ASSERT_NE(child_info, nullptr);
  ASSERT_NE(grandchild_info, nullptr);
  const int active = child_info->active_orientation;
  ASSERT_GE(active, 0);

  const auto visibleVoxels = [](const YawPlanningInfo& info, int index) {
    const auto* sensor_info = info.orientations.at(index)->info->as<SimulatedSensorInfo>();
    return sensor_info == nullptr ? std::size_t{0} : sensor_info->visible_voxels.size();
  };
  ASSERT_GT(visibleVoxels(*child_info, active), 0u);
  EXPECT_EQ(visibleVoxels(*grandchild_info, active), 0u) << "the parent's active orientation saw every voxel of the same pose";
  const int num_orientations = static_cast<int>(grandchild_info->orientations.size());
  const int opposite = (active + num_orientations / 2) % num_orientations;
  EXPECT_GT(visibleVoxels(*grandchild_info, opposite), 0u) << "voxels only other orientations of the parent could see stay";
}

// An observed node's gain is spent for the rest of the episode, so no evaluator is handed one, and
// a cluster with nothing left to evaluate never reaches the evaluator at all.
TEST(EvaluatorObservedNodes, OnlyUnobservedNodesReachTheEvaluator) {
  ConfigAgent agent;
  ClusterRecordingEvaluator evaluator(agent);
  Node observed;
  observed.markObserved();
  Node unobserved;

  EXPECT_FALSE(evaluator.evaluate({&observed, &unobserved})) << "the evaluator's own verdict";
  EXPECT_FALSE(evaluator.updateEvaluation({&observed, &unobserved}));
  ASSERT_EQ(evaluator.evaluated.size(), 1u);
  EXPECT_EQ(evaluator.evaluated.front(), NodeClusterPtrs{&unobserved});
  ASSERT_EQ(evaluator.reevaluated.size(), 1u);
  EXPECT_EQ(evaluator.reevaluated.front(), NodeClusterPtrs{&unobserved});

  EXPECT_TRUE(evaluator.evaluate({&observed})) << "nothing left to evaluate";
  EXPECT_TRUE(evaluator.updateEvaluation({&observed}));
  EXPECT_EQ(evaluator.evaluated.size(), 1u);
  EXPECT_EQ(evaluator.reevaluated.size(), 1u);
}

}  // namespace
}  // namespace open_apero
