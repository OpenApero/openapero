/**
 * @file test_camera_observation_boundary.cpp
 * @brief A camera's view ends on the sphere of radius ray_length or, with observation_boundary planar, on the plane at
 *        camera depth ray_length, as a depth camera's does.
 *
 * Every check runs the shipped camera, 480 x 640 px at f = 625.58 px over 3 m, whose planar view reaches 3.56 m from
 * the sensor at its corners.
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "open_apero_core/agent/agent_interface.h"
#include "open_apero_core/common/utils.h"
#include "open_apero_core/geometry/transform3d.h"
#include "open_apero_core/graph/node.h"
#include "open_apero_core/plugin/plugin.h"
#include "open_apero_core/plugin/sensor_model/iterative_ray_caster.h"
#include "open_apero_core/plugin/sensor_model/simple_ray_caster.h"
#include "open_apero_core/state/robot_state.h"
#include "open_apero_core/visualization/visualization_markers.h"
#include "plugin_test_stubs.h"

namespace open_apero {
namespace {

constexpr double kRayLength = 3.0;
constexpr double kFocalLength = 625.58;
constexpr double kResolutionX = 480.0;
constexpr double kResolutionY = 640.0;

/// The shipped camera; an empty @p observation_boundary leaves the key out.
Configurable::ParamMap cameraParams(const std::string& observation_boundary) {
  Configurable::ParamMap params{{"ray_length", "3.0"}, {"focal_length", "625.58"}, {"resolution_x", "480"}, {"resolution_y", "640"}};
  if (!observation_boundary.empty()) {
    params["observation_boundary"] = observation_boundary;
  }
  return params;
}

/// Camera-frame point at @p depth through normalized image coordinates, (0, 0) top-left to (1, 1) bottom-right.
Eigen::Vector3d atDepth(double depth, double relative_x, double relative_y) {
  return depth * Eigen::Vector3d(1.0, (0.5 - relative_x) * kResolutionX / kFocalLength, (0.5 - relative_y) * kResolutionY / kFocalLength);
}

Eigen::Vector3d toWorld(const Transform3d& transform, const Eigen::Vector3d& point_camera) {
  return transform.position + transform.orientation * point_camera;
}

/// Sets @p camera up with cameraParams(@p observation_boundary) and validates it.
template <class Camera>
void configure(Camera* camera, const std::string& observation_boundary) {
  Configurable::ParamMap params = cameraParams(observation_boundary);
  camera->setupFromParamMap(&params);
  camera->assureParamsValid();
}

class CameraObservationBoundaryTest : public ::testing::Test {
 protected:
  CameraObservationBoundaryTest() { agent_.provide(&map_, nullptr); }

  ConfigAgent agent_;
  FreeOccupancyMap map_{agent_};
};

TEST_F(CameraObservationBoundaryTest, APlanarViewHoldsTheCornersOutToDepthRayLength) {
  sensor_model::SimpleRayCaster planar(agent_);
  configure(&planar, "planar");
  sensor_model::SimpleRayCaster spherical(agent_);
  configure(&spherical, "spherical");
  sensor_model::SimpleRayCaster unset(agent_);
  configure(&unset, "");

  const Transform3d transform{Eigen::Vector3d(0.3, -0.2, 1.0), quaternionFromEulerAnglesZyx(Eigen::Vector3d(0.7, 0.3, 0.0))};
  const Eigen::Vector3d corner = atDepth(kRayLength - 0.001, 0.02, 0.98);
  ASSERT_GT(corner.norm(), kRayLength + 0.5);
  EXPECT_TRUE(planar.pointInSensorView(toWorld(transform, corner), transform));
  EXPECT_FALSE(spherical.pointInSensorView(toWorld(transform, corner), transform));
  EXPECT_FALSE(unset.pointInSensorView(toWorld(transform, corner), transform)) << "spherical is the default";

  EXPECT_FALSE(planar.pointInSensorView(toWorld(transform, atDepth(kRayLength + 0.001, 0.02, 0.98)), transform));
  EXPECT_FALSE(planar.pointInSensorView(toWorld(transform, atDepth(kRayLength + 0.001, 0.5, 0.5)), transform));

  // The margin moves the far plane out by itself.
  EXPECT_TRUE(planar.pointInSensorView(toWorld(transform, atDepth(kRayLength + 0.04, 0.5, 0.5)), transform, 0.05));
  EXPECT_FALSE(planar.pointInSensorView(toWorld(transform, atDepth(kRayLength + 0.06, 0.5, 0.5)), transform, 0.05));
}

TEST_F(CameraObservationBoundaryTest, ThePlanarViewBoxIsTheBoxOfTheOriginAndTheFarCorners) {
  sensor_model::SimpleRayCaster planar(agent_);
  configure(&planar, "planar");
  sensor_model::SimpleRayCaster spherical(agent_);
  configure(&spherical, "spherical");
  sensor_model::SimpleRayCaster unset(agent_);
  configure(&unset, "");

  const Eigen::Vector3d position(0.3, -0.2, 1.0);
  for (const double pitch : {0.0, 0.3, -0.5}) {
    SCOPED_TRACE(pitch);
    const Transform3d transform{position, quaternionFromEulerAnglesZyx(Eigen::Vector3d(0.7, pitch, 0.0))};
    Eigen::Vector3d expected_min = position;
    Eigen::Vector3d expected_max = position;
    for (const double relative_x : {0.0, 1.0}) {
      for (const double relative_y : {0.0, 1.0}) {
        const Eigen::Vector3d corner = toWorld(transform, atDepth(kRayLength, relative_x, relative_y));
        expected_min = expected_min.cwiseMin(corner);
        expected_max = expected_max.cwiseMax(corner);
      }
    }

    Eigen::Vector3d min;
    Eigen::Vector3d max;
    planar.getSensorViewBoundary(min, max, transform);
    EXPECT_LT((min - expected_min).norm(), 1e-9) << min.transpose() << " vs " << expected_min.transpose();
    EXPECT_LT((max - expected_max).norm(), 1e-9) << max.transpose() << " vs " << expected_max.transpose();

    // The spherical view lies inside the planar one, and spherical is the default.
    Eigen::Vector3d spherical_min;
    Eigen::Vector3d spherical_max;
    spherical.getSensorViewBoundary(spherical_min, spherical_max, transform);
    EXPECT_TRUE((min.array() <= spherical_min.array() + 1e-9).all() && (spherical_max.array() <= max.array() + 1e-9).all());
    Eigen::Vector3d unset_min;
    Eigen::Vector3d unset_max;
    unset.getSensorViewBoundary(unset_min, unset_max, transform);
    EXPECT_EQ(unset_min, spherical_min);
    EXPECT_EQ(unset_max, spherical_max);
  }
}

TEST_F(CameraObservationBoundaryTest, APlanarOutlineIsTheFourEdgesAndTheFarRectangle) {
  sensor_model::SimpleRayCaster planar(agent_);
  configure(&planar, "planar");

  const Transform3d transform{Eigen::Vector3d(0.3, -0.2, 1.0), quaternionFromEulerAnglesZyx(Eigen::Vector3d(0.7, 0.3, 0.0))};
  Node node;
  node.robot_state.setSensorTransform(transform);
  VisualizationMarkers markers;
  planar.visualizeSensorView(&markers, &node);

  const auto far = [&](double relative_x, double relative_y) { return toWorld(transform, atDepth(kRayLength, relative_x, relative_y)); };
  const Eigen::Vector3d& origin = transform.position;
  const std::vector<Eigen::Vector3d> expected{origin,    far(0, 0), origin,    far(0, 1), origin,    far(1, 0), origin,    far(1, 1),
                                              far(0, 0), far(1, 0), far(0, 1), far(1, 1), far(0, 0), far(0, 1), far(1, 0), far(1, 1)};
  ASSERT_EQ(markers.getMarkers().size(), 1u);
  const std::vector<Eigen::Vector3d>& points = markers.getMarkers().front().points;
  ASSERT_EQ(points.size(), expected.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    EXPECT_LT((points[i] - expected[i]).norm(), 1e-9) << "outline point " << i;
  }
}

/// Whether @p voxels holds a voxel centered at @p center.
bool holdsVoxel(const VoxelOccupancyMap& voxels, const Eigen::Vector3d& center) {
  for (const auto& [voxel_center, occupancy] : voxels) {
    if ((voxel_center - center).norm() < 1e-6) {
      return true;
    }
  }
  return false;
}

// The voxel the top-left corner ray crosses just before depth 3 m, camera-frame x in [2.8, 3.0), y in [1.0, 1.2) and
// z in [1.4, 1.6): every point of it is more than 3.28 m from the sensor.
template <class Caster>
void expectAPlanarCasterReachesTheFarCorner(ConfigAgent* agent) {
  Caster planar(*agent);
  configure(&planar, "planar");
  Caster spherical(*agent);
  configure(&spherical, "spherical");

  // Turned a quarter, so the camera's depth is world y.
  const Transform3d transform{Eigen::Vector3d::Zero(), quaternionFromYaw(M_PI / 2.0)};
  const Eigen::Vector3d corner_voxel = toWorld(transform, Eigen::Vector3d(2.9, 1.1, 1.5));
  ASSERT_GT((Eigen::Vector3d(2.8, 1.0, 1.4)).norm(), kRayLength + 0.25);

  VoxelOccupancyMap planar_voxels;
  VoxelOccupancyMap spherical_voxels;
  planar.getVisibleVoxels(&planar_voxels, transform);
  spherical.getVisibleVoxels(&spherical_voxels, transform);

  EXPECT_TRUE(holdsVoxel(planar_voxels, corner_voxel));
  EXPECT_FALSE(holdsVoxel(spherical_voxels, corner_voxel));
  EXPECT_GT(planar_voxels.size(), spherical_voxels.size());

  // The planar caster stops at the plane, whose depth is a voxel boundary; the spherical one within half a voxel
  // diagonal of the sphere.
  for (const auto& [center, occupancy] : planar_voxels) {
    EXPECT_LT((transform.orientation.inverse() * center).x(), kRayLength) << center.transpose();
  }
  for (const auto& [center, occupancy] : spherical_voxels) {
    EXPECT_LT(center.norm(), kRayLength + 0.5 * std::sqrt(3.0) * kStubVoxelSize) << center.transpose();
  }
}

TEST_F(CameraObservationBoundaryTest, APlanarSimpleRayCasterReachesTheFarCorner) {
  expectAPlanarCasterReachesTheFarCorner<sensor_model::SimpleRayCaster>(&agent_);
}

TEST_F(CameraObservationBoundaryTest, APlanarIterativeRayCasterReachesTheFarCorner) {
  expectAPlanarCasterReachesTheFarCorner<sensor_model::IterativeRayCaster>(&agent_);
}

/// Exposes the ray grid a SimpleRayCaster derives.
class SimpleRayCasterProbe : public sensor_model::SimpleRayCaster {
 public:
  explicit SimpleRayCasterProbe(AgentInterface& agent) : SimpleRayCaster(agent) {}
  int rayColumns() const { return c_res_x_; }
  int rayRows() const { return c_res_y_; }
};

/// Exposes the ray grid an IterativeRayCaster derives.
class IterativeRayCasterProbe : public sensor_model::IterativeRayCaster {
 public:
  explicit IterativeRayCasterProbe(AgentInterface& agent) : IterativeRayCaster(agent) {}
  int rayColumns() const { return c_res_x_; }
  int rayRows() const { return c_res_y_; }
};

// Rays lie downsampling_factor voxels apart across the observation boundary: the sphere's arc ray_length * fov, or the
// planar far face's ray_length * resolution / focal_length, wider by 2 tan(fov / 2) / fov, 1.05 across and 1.08 down.
template <class Probe>
void expectThePlanarGridSpansThePlanarFarFace(ConfigAgent* agent) {
  const auto configureQuarterVoxelSpacing = [](Probe* camera, const std::string& observation_boundary) {
    Configurable::ParamMap params = cameraParams(observation_boundary);
    params["downsampling_factor"] = "0.25";
    camera->setupFromParamMap(&params);
    camera->assureParamsValid();
  };
  Probe planar(*agent);
  configureQuarterVoxelSpacing(&planar, "planar");
  Probe spherical(*agent);
  configureQuarterVoxelSpacing(&spherical, "spherical");

  const double spacing = 0.25 * kStubVoxelSize;
  const double fov_x = 2.0 * std::atan2(kResolutionX, 2.0 * kFocalLength);
  const double fov_y = 2.0 * std::atan2(kResolutionY, 2.0 * kFocalLength);
  EXPECT_EQ(spherical.rayColumns(), static_cast<int>(std::ceil(kRayLength * fov_x / spacing)));
  EXPECT_EQ(spherical.rayRows(), static_cast<int>(std::ceil(kRayLength * fov_y / spacing)));
  EXPECT_EQ(planar.rayColumns(), static_cast<int>(std::ceil(kRayLength * kResolutionX / kFocalLength / spacing)));
  EXPECT_EQ(planar.rayRows(), static_cast<int>(std::ceil(kRayLength * kResolutionY / kFocalLength / spacing)));
  EXPECT_EQ(spherical.rayColumns(), 44);
  EXPECT_EQ(spherical.rayRows(), 57);
  EXPECT_EQ(planar.rayColumns(), 47);
  EXPECT_EQ(planar.rayRows(), 62);
}

TEST_F(CameraObservationBoundaryTest, APlanarSimpleRayCasterGridSpansThePlanarFarFace) {
  expectThePlanarGridSpansThePlanarFarFace<SimpleRayCasterProbe>(&agent_);
}

TEST_F(CameraObservationBoundaryTest, APlanarIterativeRayCasterGridSpansThePlanarFarFace) {
  expectThePlanarGridSpansThePlanarFarFace<IterativeRayCasterProbe>(&agent_);
}

TEST_F(CameraObservationBoundaryTest, AnUnknownObservationBoundaryThrows) {
  for (const std::string& name : {"conical", "Planar"}) {
    SCOPED_TRACE(name);
    sensor_model::IterativeRayCaster camera(agent_);
    Configurable::ParamMap params = cameraParams(name);
    try {
      camera.setupFromParamMap(&params);
      ADD_FAILURE() << "setup accepted observation_boundary " << name;
    } catch (const std::runtime_error& error) {
      EXPECT_NE(std::string(error.what()).find("unsupported observation_boundary \"" + name + "\""), std::string::npos) << error.what();
    }
  }
}

// The geometry tests below cast a 48 x 64 px camera with the shipped field of view at a downsampling_factor small
// enough that either boundary casts one ray per pixel, so both cast the same rays and differ only where the view ends.

/// Voxel size of the geometry tests' world [m].
constexpr double kGeometryVoxelSize = 0.1;

/// Rings of the image from its center (0) to its corners (kRings - 1), by normalized image radius.
constexpr int kRings = 6;

/// An observed world, free except every voxel at camera depth (x) of at least wall_depth.
class WallOccupancyMap : public map::OccupancyMap {
 public:
  explicit WallOccupancyMap(AgentInterface& agent) : OccupancyMap(agent) {}
  void setupFromParamMap(ParamMap* /*param_map*/) override {}
  Occupancy getVoxelOccupancy(const Eigen::Vector3d& point) override {
    Eigen::Vector3d center;
    getVoxelCenter(&center, point);
    return center.x() >= wall_depth ? Occupancy::kOccupied : Occupancy::kFree;
  }
  double getVoxelSize() override { return kGeometryVoxelSize; }
  bool getVoxelCenter(Eigen::Vector3d* center, const Eigen::Vector3d& point) override {
    *center = ((point / kGeometryVoxelSize).array().floor() + 0.5).matrix() * kGeometryVoxelSize;
    return true;
  }

  double wall_depth = std::numeric_limits<double>::infinity();
};

/// The ring of a camera-frame voxel center in front of the camera.
int ring(const Eigen::Vector3d& center) {
  const double u = std::abs(center.y() / center.x()) / (0.5 * kResolutionX / kFocalLength);
  const double v = std::abs(center.z() / center.x()) / (0.5 * kResolutionY / kFocalLength);
  return std::min(kRings - 1, static_cast<int>(std::sqrt(0.5 * (u * u + v * v)) * kRings));
}

/// Voxels per ring of @p voxels; with @p excluded, only those absent from it.
std::array<int, kRings> ringCounts(const VoxelOccupancyMap& voxels, const VoxelOccupancyMap* excluded = nullptr) {
  std::array<int, kRings> counts{};
  for (const auto& [center, occupancy] : voxels) {
    if (excluded == nullptr || excluded->count(center) == 0) {
      ++counts[ring(center)];
    }
  }
  return counts;
}

class ObservationBoundaryGeometryTest : public ::testing::Test {
 protected:
  ObservationBoundaryGeometryTest() { agent_.provide(&map_, nullptr); }

  /// What a @p Camera with @p observation_boundary sees from the origin, looking along +x.
  template <class Camera>
  VoxelOccupancyMap cast(const std::string& observation_boundary) {
    Camera camera(agent_);
    Configurable::ParamMap params{{"ray_length", "3.0"},  {"focal_length", "62.558"},      {"resolution_x", "48"},
                                  {"resolution_y", "64"}, {"downsampling_factor", "0.05"}, {"observation_boundary", observation_boundary}};
    camera.setupFromParamMap(&params);
    camera.assureParamsValid();
    VoxelOccupancyMap voxels;
    EXPECT_TRUE(camera.getVisibleVoxels(&voxels, Transform3d{Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()}));
    return voxels;
  }

  ConfigAgent agent_;
  WallOccupancyMap map_{agent_};
};

/// Planar over spherical voxels per ring.
std::array<double, kRings> ringRatios(const VoxelOccupancyMap& planar, const VoxelOccupancyMap& spherical) {
  const std::array<int, kRings> planar_counts = ringCounts(planar);
  const std::array<int, kRings> spherical_counts = ringCounts(spherical);
  std::array<double, kRings> ratios{};
  for (int k = 0; k < kRings; ++k) {
    EXPECT_GT(spherical_counts[k], 0) << "ring " << k;
    ratios[k] = static_cast<double>(planar_counts[k]) / static_cast<double>(spherical_counts[k]);
  }
  return ratios;
}

void expectNonDecreasing(const std::array<double, kRings>& ratios) {
  for (int k = 1; k < kRings; ++k) {
    EXPECT_GE(ratios[k], ratios[k - 1] - 0.01) << "ring " << k;
  }
}

TEST_F(ObservationBoundaryGeometryTest, ASimpleRayCastersPlanarViewAddsOnlyTheShellBeyondTheSphere) {
  const VoxelOccupancyMap spherical = cast<sensor_model::SimpleRayCaster>("spherical");
  const VoxelOccupancyMap planar = cast<sensor_model::SimpleRayCaster>("planar");

  for (const auto& [center, occupancy] : spherical) {
    EXPECT_EQ(planar.count(center), 1U) << "spherical-only voxel at " << center.transpose();
  }
  for (const auto& [center, occupancy] : planar) {
    if (spherical.count(center) == 0) {
      EXPECT_GT(center.norm(), kRayLength - kGeometryVoxelSize) << "planar-only voxel inside the sphere at " << center.transpose();
    }
  }

  // The shell is thinnest on the optical axis and thickest at the corners (3.56 m against 3 m).
  const std::array<double, kRings> ratios = ringRatios(planar, spherical);
  expectNonDecreasing(ratios);
  EXPECT_LT(ratios.front(), 1.15);
  EXPECT_GT(ratios.back(), 1.35);
}

TEST_F(ObservationBoundaryGeometryTest, AnIterativeRayCastersPlanarViewMatchesAtTheCenterAndGrowsTowardTheCorners) {
  const VoxelOccupancyMap spherical = cast<sensor_model::IterativeRayCaster>("spherical");
  const VoxelOccupancyMap planar = cast<sensor_model::IterativeRayCaster>("planar");

  const std::array<int, kRings> spherical_counts = ringCounts(spherical);
  const std::array<int, kRings> planar_only = ringCounts(planar, &spherical);
  const std::array<int, kRings> spherical_only = ringCounts(spherical, &planar);
  EXPECT_LE(planar_only.front() + spherical_only.front(), 0.02 * spherical_counts.front());

  const std::array<double, kRings> ratios = ringRatios(planar, spherical);
  expectNonDecreasing(ratios);
  EXPECT_NEAR(ratios.front(), 1.0, 0.02);
  EXPECT_GT(ratios.back(), 1.3);

  // Sections scale with each ray's length, which shifts samples along off-axis rays; the views still overlap.
  int spherical_only_total = 0;
  for (const int count : spherical_only) {
    spherical_only_total += count;
  }
  EXPECT_LE(spherical_only_total, 0.02 * static_cast<double>(spherical.size()));
}

TEST_F(ObservationBoundaryGeometryTest, AWallInsideBothBoundariesLooksTheSame) {
  // At depth 2 m every ray meets the wall (at most 2.37 m away) before either boundary.
  map_.wall_depth = 2.0;

  const VoxelOccupancyMap simple_spherical = cast<sensor_model::SimpleRayCaster>("spherical");
  const VoxelOccupancyMap simple_planar = cast<sensor_model::SimpleRayCaster>("planar");
  EXPECT_EQ(simple_planar.size(), simple_spherical.size());
  for (const auto& [center, occupancy] : simple_spherical) {
    EXPECT_EQ(simple_planar.count(center), 1U) << "spherical-only voxel at " << center.transpose();
  }

  const VoxelOccupancyMap iterative_spherical = cast<sensor_model::IterativeRayCaster>("spherical");
  const VoxelOccupancyMap iterative_planar = cast<sensor_model::IterativeRayCaster>("planar");
  EXPECT_NEAR(static_cast<double>(iterative_planar.size()), static_cast<double>(iterative_spherical.size()),
              0.02 * static_cast<double>(iterative_spherical.size()));
  const std::array<double, kRings> ratios = ringRatios(iterative_planar, iterative_spherical);
  for (int k = 0; k < kRings; ++k) {
    EXPECT_NEAR(ratios[k], 1.0, k < kRings / 2 ? 0.01 : 0.08) << "ring " << k;
  }
}

}  // namespace
}  // namespace open_apero
