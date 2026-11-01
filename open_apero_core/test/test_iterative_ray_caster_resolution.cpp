/**
 * @file test_iterative_ray_caster_resolution.cpp
 * @brief The ray grid the caster derives must hold at least two rays per axis.
 *
 * A ray_length, field of view and voxel size that yield fewer fail at construction.
 */

#include <stdexcept>
#include <string>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "open_apero_core/agent/agent_interface.h"
#include "open_apero_core/common/typedef.h"
#include "open_apero_core/plugin/map/occupancy_map.h"
#include "open_apero_core/plugin/plugin.h"
#include "open_apero_core/plugin/plugin_factory.h"
#include "open_apero_core/plugin/sensor_model/iterative_ray_caster.h"
#include "plugin_test_stubs.h"

namespace open_apero {
namespace {

/// Voxel size the grid arithmetic below is written against.
constexpr double kVoxelSize = 0.5;

/// An occupancy map that answers nothing but its voxel size, which is the only
/// thing the ray grid is derived from.
class StubOccupancyMap : public map::OccupancyMap {
 public:
  explicit StubOccupancyMap(AgentInterface& agent) : OccupancyMap(agent) {}
  void setupFromParamMap(ParamMap* /*param_map*/) override {}
  Occupancy getVoxelOccupancy(const Eigen::Vector3d& /*point*/) override { return Occupancy::kUnknown; }
  double getVoxelSize() override { return kVoxelSize; }
  bool getVoxelCenter(Eigen::Vector3d* /*center*/, const Eigen::Vector3d& /*point*/) override { return false; }
};

/// ConfigAgent serving StubFactory and its own StubOccupancyMap, with no budget and the default seed.
class StubAgent : public ConfigAgent {
 public:
  StubAgent() {
    useFactory(&stub_factory_);
    provide(&map_, nullptr);
    setCostBudget(0.0);
    setRandomSeed(-1);
  }

 private:
  StubFactory stub_factory_;
  StubOccupancyMap map_{*this};
};

/// The caster's parameters at a 320 px focal length and 3 m range.
Configurable::ParamMap casterParams(const std::string& resolution, const std::string& downsampling_factor) {
  return Configurable::ParamMap{{"ray_length", "3.0"},
                                {"focal_length", "320.0"},
                                {"resolution_x", resolution},
                                {"resolution_y", resolution},
                                {"downsampling_factor", downsampling_factor}};
}

TEST(IterativeRayCasterGrid, RejectsAConfigurationYieldingASingleRayPerAxis) {
  StubAgent agent;
  sensor_model::IterativeRayCaster caster(agent);
  // 3.0 m of range over a 0.37 rad field of view, thinned threefold across 0.5 m
  // voxels, asks for one ray per axis.
  Configurable::ParamMap params = casterParams("120", "3.0");
  caster.setupFromParamMap(&params);

  std::string error_message;
  EXPECT_FALSE(caster.checkParamsValid(&error_message));
  EXPECT_NE(error_message.find("at least 2 rays per axis"), std::string::npos);
  EXPECT_NE(error_message.find("downsampling_factor"), std::string::npos);
  EXPECT_THROW(caster.assureParamsValid(), std::invalid_argument);
}

TEST(IterativeRayCasterGrid, AcceptsAConfigurationYieldingAUsableGrid) {
  StubAgent agent;
  sensor_model::IterativeRayCaster caster(agent);
  // A 90 deg field of view over the same range and voxel size gives ten rays.
  Configurable::ParamMap params = casterParams("640", "1.0");
  caster.setupFromParamMap(&params);

  std::string error_message;
  EXPECT_TRUE(caster.checkParamsValid(&error_message)) << error_message;
  EXPECT_NO_THROW(caster.assureParamsValid());
}

TEST(IterativeRayCasterGrid, RejectsANonPositiveDownsamplingFactor) {
  StubAgent agent;
  sensor_model::IterativeRayCaster caster(agent);
  Configurable::ParamMap params = casterParams("640", "0.0");
  caster.setupFromParamMap(&params);

  std::string error_message;
  EXPECT_FALSE(caster.checkParamsValid(&error_message));
  EXPECT_NE(error_message.find("downsampling_factor expected > 0.0"), std::string::npos);
}

}  // namespace
}  // namespace open_apero
