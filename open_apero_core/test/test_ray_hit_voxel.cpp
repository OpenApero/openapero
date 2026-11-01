/**
 * @file test_ray_hit_voxel.cpp
 * @brief A ray's hit is recorded on the voxel that holds the surface, not on the voxel in front of it.
 *
 * Every sensor model stops a ray at the first sample reading kOccupied, which is any sample within one
 * voxel of a surface. The hit recorded is the voxel the map locates the surface in, and it overrides
 * a free reading another sample left on the same voxel.
 */

#include <cmath>
#include <limits>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "open_apero_core/agent/agent_interface.h"
#include "open_apero_core/common/typedef.h"
#include "open_apero_core/geometry/transform3d.h"
#include "open_apero_core/graph/node.h"
#include "open_apero_core/plugin/map/tsdf_map.h"
#include "open_apero_core/plugin/plugin.h"
#include "open_apero_core/plugin/sensor_model/iterative_ray_caster.h"
#include "open_apero_core/plugin/sensor_model/iterative_ray_caster_lidar.h"
#include "open_apero_core/plugin/sensor_model/simple_ray_caster.h"
#include "plugin_test_stubs.h"

namespace open_apero {
namespace {

/// Voxel size of the wall map, the planner's mapping resolution [m].
constexpr double kVoxelSize = 0.05;

/// Depth behind the wall that the map has observed [m].
constexpr double kObservedDepth = 0.2;

/// A wall facing -x at wall_x. The distance field reads distance_scale times the true distance, as
/// a projective TSDF overestimates it, and the occupancy rule is the planner maps' own: occupied
/// within one voxel of the surface. Only points within observed_half_height of z = 0 have a value.
class WallTsdfMap : public map::TsdfMap {
 public:
  WallTsdfMap(AgentInterface& agent, double wall_x, double distance_scale,
              double observed_half_height = std::numeric_limits<double>::infinity())
      : TsdfMap(agent), wall_x_(wall_x), distance_scale_(distance_scale), observed_half_height_(observed_half_height) {}
  void setupFromParamMap(ParamMap* /*param_map*/) override {}
  Occupancy getVoxelOccupancy(const Eigen::Vector3d& point) override {
    double distance = 0.0;
    if (!getDistanceAtPosition(point, &distance)) {
      return Occupancy::kUnknown;
    }
    return distance < kVoxelSize ? Occupancy::kOccupied : Occupancy::kFree;
  }
  double getVoxelSize() override { return kVoxelSize; }
  bool getVoxelCenter(Eigen::Vector3d* center, const Eigen::Vector3d& point) override {
    *center = ((point / kVoxelSize).array().floor() + 0.5).matrix() * kVoxelSize;
    return true;
  }
  bool getDistanceAtPosition(const Eigen::Vector3d& position, double* distance) override {
    const double true_distance = wall_x_ - position.x();
    if (true_distance < -kObservedDepth || std::abs(position.z()) > observed_half_height_) {
      return false;
    }
    *distance = distance_scale_ * true_distance;
    return true;
  }
  double getVoxelWeight(const Eigen::Vector3d& /*point*/) override { return 0.0; }
  double getMaximumWeight() override { return 0.0; }

 private:
  double wall_x_;
  double distance_scale_;
  double observed_half_height_;
};

/// ConfigAgent serving StubFactory and its own WallTsdfMap.
class WallAgent : public ConfigAgent {
 public:
  WallAgent(double wall_x, double distance_scale) : map_(*this, wall_x, distance_scale) {
    useFactory(&stub_factory_);
    provide(&map_, nullptr);
    setCostBudget(0.0);
    setRandomSeed(-1);
  }

 private:
  StubFactory stub_factory_;
  WallTsdfMap map_;
};

/// A pinhole camera looking along +x with a 10 deg field of view over 2 m.
Configurable::ParamMap cameraParams() {
  return Configurable::ParamMap{{"ray_length", "2.0"}, {"focal_length", "366.0"}, {"resolution_x", "64"}, {"resolution_y", "64"}};
}

/// A lidar looking along +x with a 10 deg field of view per axis over 2 m.
Configurable::ParamMap lidarParams() {
  return Configurable::ParamMap{
      {"ray_length", "2.0"}, {"field_of_view_x", "10.0"}, {"field_of_view_y", "10.0"}, {"resolution_x", "64"}, {"resolution_y", "64"}};
}

template <typename Caster>
VoxelOccupancyMap castAtWall(double wall_x, double distance_scale, const Eigen::Vector3d& sensor_position, Configurable::ParamMap params) {
  WallAgent agent(wall_x, distance_scale);
  Caster caster(agent);
  caster.setupFromParamMap(&params);
  caster.assureParamsValid();
  VoxelOccupancyMap voxels;
  caster.getVisibleVoxels(&voxels, Transform3d{sensor_position, Eigen::Quaterniond::Identity()});
  return voxels;
}

/// Every occupied voxel lies in the wall's voxel layer, and no voxel of that layer reads free.
void expectHitsOnWallLayer(const VoxelOccupancyMap& voxels, double wall_x) {
  const double wall_layer_x = (std::floor(wall_x / kVoxelSize) + 0.5) * kVoxelSize;
  int hits = 0;
  for (const auto& [center, occupancy] : voxels) {
    const bool in_wall_layer = std::abs(center.x() - wall_layer_x) < 1e-6;
    if (occupancy == Occupancy::kOccupied) {
      ++hits;
      EXPECT_TRUE(in_wall_layer) << "occupied voxel centered at x " << center.x() << ", the wall's layer is at x " << wall_layer_x;
    } else if (in_wall_layer) {
      EXPECT_NE(occupancy, Occupancy::kFree) << "the wall's voxel at y " << center.y() << ", z " << center.z() << " reads free";
    }
  }
  EXPECT_GT(hits, 0);
}

// The wall lies 13 mm into its voxel and the sensor sits so that the first sample within one voxel
// of the wall falls in the voxel in front of it.
TEST(RayHitVoxel, RecordsTheVoxelHoldingTheWall) {
  constexpr double kWallX = 1.013;
  const Eigen::Vector3d sensor(0.02, 0.013, 0.007);
  {
    SCOPED_TRACE("SimpleRayCaster");
    expectHitsOnWallLayer(castAtWall<sensor_model::SimpleRayCaster>(kWallX, 1.0, sensor, cameraParams()), kWallX);
  }
  {
    SCOPED_TRACE("IterativeRayCaster");
    expectHitsOnWallLayer(castAtWall<sensor_model::IterativeRayCaster>(kWallX, 1.0, sensor, cameraParams()), kWallX);
  }
  {
    SCOPED_TRACE("IterativeRayCasterLidar");
    expectHitsOnWallLayer(castAtWall<sensor_model::IterativeRayCasterLidar>(kWallX, 1.0, sensor, lidarParams()), kWallX);
  }
}

// A distance field reading 1.5 times high lets a sample 4 cm in front of the wall read free, so the
// ray first reads occupied behind the wall. The wall's voxel, already recorded free by that ray, is the hit.
TEST(RayHitVoxel, AHitOverridesAFreeReadingOfTheSameVoxel) {
  constexpr double kWallX = 1.045;
  const Eigen::Vector3d sensor(0.005, 0.013, 0.007);
  {
    SCOPED_TRACE("SimpleRayCaster");
    expectHitsOnWallLayer(castAtWall<sensor_model::SimpleRayCaster>(kWallX, 1.5, sensor, cameraParams()), kWallX);
  }
  {
    SCOPED_TRACE("IterativeRayCaster");
    expectHitsOnWallLayer(castAtWall<sensor_model::IterativeRayCaster>(kWallX, 1.5, sensor, cameraParams()), kWallX);
  }
  {
    SCOPED_TRACE("IterativeRayCasterLidar");
    expectHitsOnWallLayer(castAtWall<sensor_model::IterativeRayCasterLidar>(kWallX, 1.5, sensor, lidarParams()), kWallX);
  }
}

// Every wall offset within a voxel and every sensor offset along the ray: the first sample reading
// occupied lies in the wall's voxel or the one in front, and the surface voxel is the wall's.
TEST(RayHitVoxel, TsdfMapLocatesTheWallsVoxelHeadOn) {
  ConfigAgent agent;
  int samples_in_front_of_the_wall = 0;
  for (int i = 0; i < 50; ++i) {
    const double wall_x = 1.0005 + 0.001 * i;
    WallTsdfMap map(agent, wall_x, 1.0);
    const double wall_layer_x = (std::floor(wall_x / kVoxelSize) + 0.5) * kVoxelSize;
    const Eigen::Vector3d direction = Eigen::Vector3d::UnitX();
    for (int j = 0; j < 16; ++j) {
      Eigen::Vector3d sample(0.0031 * j, 0.013, 0.007);
      while (map.getVoxelOccupancy(sample) != Occupancy::kOccupied) {
        sample += kVoxelSize * direction;
      }
      Eigen::Vector3d sample_voxel;
      map.getVoxelCenter(&sample_voxel, sample);
      if (std::abs(sample_voxel.x() - wall_layer_x) > 1e-6) {
        ++samples_in_front_of_the_wall;
      }
      Eigen::Vector3d surface_voxel;
      ASSERT_TRUE(map.getSurfaceVoxelCenter(&surface_voxel, sample, direction));
      EXPECT_NEAR(surface_voxel.x(), wall_layer_x, 1e-6) << "wall at x " << wall_x << ", sample at x " << sample.x();
    }
  }
  EXPECT_GT(samples_in_front_of_the_wall, 0);
}

// The map is observed only in a strip around the ray: the sample reads occupied but no candidate
// voxel center has a distance value, so the sample's voxel stays the hit.
TEST(RayHitVoxel, TsdfMapKeepsTheSamplesVoxelWhenNoCandidateHasAValue) {
  ConfigAgent agent;
  constexpr double kWallX = 1.013;
  WallTsdfMap map(agent, kWallX, 1.0, 0.01);
  const Eigen::Vector3d sample(kWallX - 0.04, 0.013, 0.007);
  ASSERT_EQ(map.getVoxelOccupancy(sample), Occupancy::kOccupied);
  Eigen::Vector3d sample_voxel;
  Eigen::Vector3d surface_voxel;
  map.getVoxelCenter(&sample_voxel, sample);
  ASSERT_TRUE(map.getSurfaceVoxelCenter(&surface_voxel, sample, Eigen::Vector3d::UnitX()));
  EXPECT_TRUE(surface_voxel.isApprox(sample_voxel));
}

TEST(RayHitVoxel, OccupancyMapDefaultsToTheSamplesVoxel) {
  ConfigAgent agent;
  FreeOccupancyMap map(agent);
  const Eigen::Vector3d sample(0.31, -0.05, 0.47);
  Eigen::Vector3d sample_voxel;
  Eigen::Vector3d surface_voxel;
  map.getVoxelCenter(&sample_voxel, sample);
  ASSERT_TRUE(map.getSurfaceVoxelCenter(&surface_voxel, sample, Eigen::Vector3d::UnitX()));
  EXPECT_TRUE(surface_voxel.isApprox(sample_voxel));
}

}  // namespace
}  // namespace open_apero
