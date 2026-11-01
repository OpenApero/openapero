#include "open_apero_core/plugin/sensor_model/sensor_model.h"

#include <chrono>
#include <cmath>
#include <mutex>
#include <sstream>
#include <stdexcept>

#include "open_apero_core/agent/agent_interface.h"

namespace open_apero {

SensorModel::SensorModel(AgentInterface& agent) : Plugin(agent) {}

void SensorModel::setupFromParamMap(Configurable::ParamMap* param_map) {
  map_ = dynamic_cast<map::OccupancyMap*>(&(agent_.getMap()));
  if (!map_) {
    throw std::runtime_error("'SensorModel' requires a map of type 'OccupancyMap'!");
  }

  setParam<double>(param_map, "ray_length", &p_ray_length_, 5.0);

  setParam<bool>(param_map, "log_cast_timing", &p_log_cast_timing_, false);
}

bool SensorModel::pointInSensorView(const Eigen::Vector3d& /*point*/, const Transform3d& /*sensor_transform*/, double /*margin*/) const {
  return true;
}

bool SensorModel::getVisibleVoxelsFromNode(VoxelOccupancyMap* result, const Node* node) {
  if (!p_log_cast_timing_) {
    getVisibleVoxels(result, {node->robot_state.getSensorTransform()});
  } else {
    const auto start_time = std::chrono::steady_clock::now();
    getVisibleVoxels(result, {node->robot_state.getSensorTransform()});
    const std::chrono::duration<double, std::milli> elapsed = std::chrono::steady_clock::now() - start_time;

    // Callers may cast from several OpenMP threads.
    std::lock_guard<std::mutex> lock(time_count_mutex_);
    time_count_.push_back(elapsed.count());
    if (time_count_.size() % 20 == 0) {
      double mean = 0.0;
      double stddev = 0.0;
      for (int i = 0; i < static_cast<int>(time_count_.size()); ++i) {
        mean += time_count_[i];
      }
      mean /= time_count_.size();
      for (int i = 0; i < static_cast<int>(time_count_.size()); ++i) {
        stddev += std::pow(time_count_[i] - mean, 2.0);
      }
      stddev = std::sqrt(stddev / time_count_.size());
      std::ostringstream ss;
      ss << "Raycasting: Mean: " << mean << " ms, Stddev: " << stddev << " ms";
      agent_.printInfo(ss.str());
    }
  }

  return true;
}

void SensorModel::recordVoxel(VoxelOccupancyMap* result, const Eigen::Vector3d& center, Occupancy occupancy) {
  const auto [voxel, inserted] = result->emplace(center, occupancy);
  if (!inserted && occupancy == Occupancy::kOccupied) {
    voxel->second = Occupancy::kOccupied;
  }
}

}  // namespace open_apero
