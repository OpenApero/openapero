#include "open_apero_core/plugin/evaluator/simulated_sensor_evaluator.h"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "open_apero_core/agent/agent_interface.h"
#include "open_apero_core/common/utils.h"
#include "open_apero_core/geometry/transform3d.h"
#include "open_apero_core/graph/node.h"
#include "open_apero_core/visualization/colormap.h"

namespace open_apero {
namespace evaluator {

namespace {

// The voxels @p ancestor saw: its own SimulatedSensorInfo or, when a yaw planning evaluator
// wraps this one, that of its active orientation; nullptr when it holds neither.
const SimulatedSensorInfo* ancestorVisibleVoxels(const Node& ancestor) {
  if (!ancestor.info) {
    return nullptr;
  }
  if (const auto* info = ancestor.info->as<SimulatedSensorInfo>()) {
    return info;
  }
  const auto* yaw_info = ancestor.info->as<YawPlanningInfo>();
  if (yaw_info == nullptr || yaw_info->active_orientation < 0 ||
      static_cast<std::size_t>(yaw_info->active_orientation) >= yaw_info->orientations.size()) {
    return nullptr;
  }
  const Node* active = yaw_info->orientations[yaw_info->active_orientation].get();
  return (active != nullptr && active->info) ? active->info->as<SimulatedSensorInfo>() : nullptr;
}

}  // namespace

SimulatedSensorEvaluator::SimulatedSensorEvaluator(AgentInterface& agent) : Evaluator(agent) {}

void SimulatedSensorEvaluator::setupFromParamMap(Configurable::ParamMap* param_map) {
  Evaluator::setupFromParamMap(param_map);

  setParam<bool>(param_map, "clear_from_parents", &p_clear_from_parents_, false);
  setParam<bool>(param_map, "visualize_sensor_view", &p_visualize_sensor_view_, false);

  setParam<double>(param_map, "frontier_min_unknown_ratio", &p_frontier_min_unknown_ratio_, 0.0);
  setParam<double>(param_map, "frontier_min_visible_ratio", &p_frontier_min_visible_ratio_, 0.0);

  setParam<double>(param_map, "update_gain", &p_update_gain_, 0.0);

  setParam<bool>(param_map, "only_update_for_updated_map", &p_only_update_for_updated_map_, true);

  map_ = dynamic_cast<map::OccupancyMap*>(&(agent_.getMap()));
  if (!map_) {
    throw std::runtime_error("'SimulatedSensorEvaluator' requires a map of type 'OccupancyMap'!");
  }

  std::string args;
  std::string param_ns = (*param_map)["param_namespace"];
  setParam<std::string>(param_map, "sensor_model_args", &args, param_ns + "/sensor_model");
  sensor_model_ = agent_.getFactory().createPlugin<SensorModel>(args, agent_, verbose_plugins_);
}

bool SimulatedSensorEvaluator::evaluateUnobserved(const NodeClusterPtrs& cluster) {
  for (Node* node : cluster) {
    VoxelOccupancyMap new_voxels;
    sensor_model_->getVisibleVoxelsFromNode(&new_voxels, node);

    if (bounding_volume_->is_setup) {
      auto it = new_voxels.begin();
      while (it != new_voxels.end()) {
        if (!bounding_volume_->contains(it->first)) {
          it = new_voxels.erase(it);  // erase returns the next iterator
        } else {
          ++it;
        }
      }
    }

    // Optionally count only voxels not already visible from an ancestor state.
    if (p_clear_from_parents_) {
      auto tree_node = node->as<TreeNode>();
      if (!tree_node) {
        throw std::runtime_error("SimulatedSensorEvaluator: 'clear_from_parents' only works with TreeNode nodes!");
      }

      auto previous = tree_node->parent;
      while (previous) {
        const SimulatedSensorInfo* previous_info = ancestorVisibleVoxels(*previous);
        if (previous_info != nullptr) {
          const auto& old_voxels = previous_info->visible_voxels;
          auto it = new_voxels.begin();
          while (it != new_voxels.end()) {
            if (old_voxels.find(it->first) != old_voxels.end()) {
              it = new_voxels.erase(it);
            } else {
              ++it;
            }
          }
        }

        previous = previous->parent;
      }
    }

    storeVisibleVoxels(node, new_voxels);
    evaluateFromVisibleVoxels(node);
    computeFrontierStatus(node);
  }

  return true;
}

bool SimulatedSensorEvaluator::reevaluateUnobserved(const NodeClusterPtrs& cluster, const Node* /*current_node*/) {
  NodeClusterPtrs cluster_to_update;
  cluster_to_update.reserve(cluster.size());

  for (Node* node : cluster) {
    if (!node->is_traversable) {
      continue;
    }

    // Avoid ray casting where neither value nor the changed map region can matter.
    bool is_node_worth_updating = (node->getSingleNodeGain() > p_update_gain_ || node->is_at_frontier);
    bool is_sensor_view_overlapping = nodeSensorViewOverlapsUpdatedMap(node);
    if (is_node_worth_updating && is_sensor_view_overlapping) {
      cluster_to_update.push_back(node);
    }
  }

  return evaluate(cluster_to_update);
}

bool SimulatedSensorEvaluator::nodeSensorViewOverlapsUpdatedMap(const Node* node) {
  return sensorViewOverlapsUpdatedMap(node->robot_state);
}

bool SimulatedSensorEvaluator::storeVisibleVoxels(Node* node, const VoxelOccupancyMap& new_voxels) {
  auto new_info = std::make_unique<SimulatedSensorInfo>();
  new_info->visible_voxels.reserve(new_voxels.size());
  for (const auto& [position, occupancy] : new_voxels) {
    new_info->visible_voxels[position] = {occupancy, 0.0};
  }
  node->info = std::move(new_info);
  return true;
}

void SimulatedSensorEvaluator::computeFrontierStatus(Node* node) {
  if (!node->info) {
    node->is_at_frontier = false;
    return;
  }

  // Zero thresholds disable frontier detection.
  if (p_frontier_min_visible_ratio_ < 1e-4 && p_frontier_min_unknown_ratio_ < 1e-4) {
    node->is_at_frontier = false;
    return;
  }

  SimulatedSensorInfo* info = node->info->as<SimulatedSensorInfo>();
  if (!info) {
    throw std::runtime_error("Node info is not of type SimulatedSensorInfo!");
  }

  int count_unknown_voxels = 0;
  for (const auto& [voxel_center, voxel_info] : info->visible_voxels) {
    if (voxel_info.occupancy == Occupancy::kUnknown) {
      ++count_unknown_voxels;
    }
  }

  int count_visible_voxels = info->visible_voxels.size();

  max_num_visible_voxels_ = std::max(max_num_visible_voxels_, count_visible_voxels);

  // Require both sufficient visibility and a sufficient unknown fraction.
  node->is_at_frontier = (count_visible_voxels > p_frontier_min_visible_ratio_ * max_num_visible_voxels_ &&
                          count_unknown_voxels > p_frontier_min_unknown_ratio_ * count_visible_voxels);
}

void SimulatedSensorEvaluator::getVisualization(VisualizationMarkers* markers, const Node* node) {
  if (p_visualize_sensor_view_) {
    sensor_model_->visualizeSensorView(markers, node);
  }

  if (bounding_volume_->is_setup) {
    bounding_volume_->visualize(markers);
  }

  const SimulatedSensorInfo* info = node->info ? node->info->as<SimulatedSensorInfo>() : nullptr;
  if (info == nullptr) {
    return;
  }

  VisualizationMarker marker;
  marker.ns = "voxel_gains";
  marker.type = VisualizationMarker::CUBE_LIST;
  double voxel_size = map_->getVoxelSize();
  marker.scale.x() = voxel_size;
  marker.scale.y() = voxel_size;
  marker.scale.z() = voxel_size;

  double max_gain = std::numeric_limits<double>::lowest();
  double min_gain = std::numeric_limits<double>::max();
  for (const auto& [voxel_center, voxel_info] : info->visible_voxels) {
    if (voxel_info.gain < kEpsilon) {
      continue;
    }

    max_gain = std::max(max_gain, voxel_info.gain);
    min_gain = std::min(min_gain, voxel_info.gain);
  }

  for (const auto& [voxel_center, voxel_info] : info->visible_voxels) {
    if (voxel_info.gain < kEpsilon) {
      continue;
    }

    marker.points.push_back(voxel_center);

    if (max_gain - min_gain > kEpsilon) {
      marker.colors.push_back(getColorFromColormap(voxel_info.gain, min_gain, max_gain, 0.4));
    } else {
      Color c;
      c.r = 1.0;
      c.g = 1.0;
      c.b = 0.0;
      c.a = 0.4;
      marker.colors.push_back(c);
    }
  }

  markers->addMarker(marker);
}

bool SimulatedSensorEvaluator::sensorViewOverlapsUpdatedMap(const RobotState& state) {
  return sensorViewOverlapsUpdatedMap({state.getSensorTransform()});
}

bool SimulatedSensorEvaluator::sensorViewOverlapsUpdatedMap(const Transform3d& sensor_transform) {
  if (!p_only_update_for_updated_map_) {
    return true;
  }

  Eigen::Vector3d updated_map_min;
  Eigen::Vector3d updated_map_max;
  if (!map_->getUpdatedMapRegion(&updated_map_min, &updated_map_max)) {
    return false;
  }

  Eigen::Vector3d sensor_view_min;
  Eigen::Vector3d sensor_view_max;
  sensor_model_->getSensorViewBoundary(sensor_view_min, sensor_view_max, sensor_transform);

  return boxesOverlap(sensor_view_min, sensor_view_max, updated_map_min, updated_map_max);
}

}  // namespace evaluator
}  // namespace open_apero
