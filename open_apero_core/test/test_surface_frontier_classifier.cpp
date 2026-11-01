/**
 * @file test_surface_frontier_classifier.cpp
 * @brief Pins the shared surface-frontier predicate.
 *
 * The predicate returns on the first observed neighbor, so its verdict depends on the offset order;
 * the order-dependence cases pin that behavior, which the published results rely on.
 */

#include <array>
#include <cmath>
#include <cstddef>
#include <string>
#include <unordered_map>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "open_apero_core/agent/agent_interface.h"
#include "open_apero_core/common/typedef.h"
#include "open_apero_core/plugin/map/occupancy_map.h"
#include "open_apero_core/plugin/map/surface_frontier_classifier.h"

#include "plugin_test_stubs.h"

namespace open_apero {
namespace {

/// @brief Voxels are one meter wide, so voxel centers land on whole coordinates.
constexpr double kVoxelSize = 1.0;

/// @brief ConfigAgent with no budget and the default seed; constructing a map is all it serves here.
class FrontierAgent : public ConfigAgent {
 public:
  FrontierAgent() {
    setCostBudget(0.0);
    setRandomSeed(-1);
  }
};

/// @brief An occupancy grid a test writes voxel by voxel; everything unset is unknown.
class ScriptedOccupancyMap : public map::OccupancyMap {
 public:
  explicit ScriptedOccupancyMap(AgentInterface& agent) : map::OccupancyMap(agent) {}

  void setupFromParamMap(ParamMap* /*param_map*/) override {}

  Occupancy getVoxelOccupancy(const Eigen::Vector3d& point) override {
    const auto entry = voxels_.find(key(point));
    return entry == voxels_.end() ? Occupancy::kUnknown : entry->second;
  }

  double getVoxelSize() override { return kVoxelSize; }

  bool getVoxelCenter(Eigen::Vector3d* center, const Eigen::Vector3d& point) override {
    const std::array<int, 3> index = key(point);
    *center = Eigen::Vector3d(static_cast<double>(index[0]), static_cast<double>(index[1]), static_cast<double>(index[2])) * kVoxelSize;
    return true;
  }

  /** @brief Marks the voxel containing @p point with @p occupancy. */
  void set(const Eigen::Vector3d& point, Occupancy occupancy) { voxels_[key(point)] = occupancy; }

 private:
  struct KeyHash {
    std::size_t operator()(const std::array<int, 3>& index) const {
      return (static_cast<std::size_t>(index[0]) * 73856093u) ^ (static_cast<std::size_t>(index[1]) * 19349663u) ^
             (static_cast<std::size_t>(index[2]) * 83492791u);
    }
  };

  static std::array<int, 3> key(const Eigen::Vector3d& point) {
    return {static_cast<int>(std::lround(point.x() / kVoxelSize)), static_cast<int>(std::lround(point.y() / kVoxelSize)),
            static_cast<int>(std::lround(point.z() / kVoxelSize))};
  }

  std::unordered_map<std::array<int, 3>, Occupancy, KeyHash> voxels_;
};

/// @brief Holds the agent and map alive for one predicate under test.
struct Fixture {
  FrontierAgent agent;
  ScriptedOccupancyMap map{agent};

  map::SurfaceFrontierClassifier make(bool accurate_frontiers, bool surface_frontiers, double checking_distance) {
    return map::SurfaceFrontierClassifier(&map, accurate_frontiers, surface_frontiers, checking_distance);
  }
};

const Eigen::Vector3d kOrigin{0.0, 0.0, 0.0};

TEST(SurfaceFrontierClassifier, UnknownSurroundingsAreNotAFrontier) {
  Fixture fixture;
  EXPECT_FALSE(fixture.make(false, true, 1.0).isSurfaceFrontier(kOrigin));
}

TEST(SurfaceFrontierClassifier, OccupiedFaceNeighborMakesAFrontier) {
  Fixture fixture;
  fixture.map.set({1.0, 0.0, 0.0}, Occupancy::kOccupied);
  EXPECT_TRUE(fixture.make(false, true, 1.0).isSurfaceFrontier(kOrigin));
}

TEST(SurfaceFrontierClassifier, FreeFaceNeighborAloneIsNotASurfaceFrontier) {
  Fixture fixture;
  fixture.map.set({1.0, 0.0, 0.0}, Occupancy::kFree);
  EXPECT_FALSE(fixture.make(false, true, 1.0).isSurfaceFrontier(kOrigin));
}

TEST(SurfaceFrontierClassifier, NonSurfaceModeAcceptsAnyObservedNeighbor) {
  Fixture fixture;
  fixture.map.set({1.0, 0.0, 0.0}, Occupancy::kFree);
  EXPECT_TRUE(fixture.make(false, false, 1.0).isSurfaceFrontier(kOrigin));
}

TEST(SurfaceFrontierClassifier, FaceScanMissesADiagonalNeighborThatTheFullScanFinds) {
  Fixture fixture;
  fixture.map.set({1.0, 1.0, 0.0}, Occupancy::kOccupied);
  EXPECT_FALSE(fixture.make(false, true, 1.0).isSurfaceFrontier(kOrigin));
  EXPECT_TRUE(fixture.make(true, true, 1.0).isSurfaceFrontier(kOrigin));
}

TEST(SurfaceFrontierClassifier, CheckingDistanceScalesTheNeighborOffsets) {
  Fixture fixture;
  fixture.map.set({2.0, 0.0, 0.0}, Occupancy::kOccupied);
  EXPECT_FALSE(fixture.make(false, true, 1.0).isSurfaceFrontier(kOrigin));
  EXPECT_TRUE(fixture.make(false, true, 2.0).isSurfaceFrontier(kOrigin));
}

// The first offset is +x. A free voxel there ends the scan before the occupied
// voxel behind the query is ever probed, so the answer is order-dependent.
TEST(SurfaceFrontierClassifier, FreeNeighborScannedFirstHidesAnOccupiedNeighborBehindIt) {
  Fixture fixture;
  fixture.map.set({1.0, 0.0, 0.0}, Occupancy::kFree);
  fixture.map.set({-1.0, 0.0, 0.0}, Occupancy::kOccupied);
  EXPECT_FALSE(fixture.make(false, true, 1.0).isSurfaceFrontier(kOrigin));
}

TEST(SurfaceFrontierClassifier, TheSameTwoNeighborsSwappedYieldTheOppositeVerdict) {
  Fixture fixture;
  fixture.map.set({1.0, 0.0, 0.0}, Occupancy::kOccupied);
  fixture.map.set({-1.0, 0.0, 0.0}, Occupancy::kFree);
  EXPECT_TRUE(fixture.make(false, true, 1.0).isSurfaceFrontier(kOrigin));
}

}  // namespace
}  // namespace open_apero
