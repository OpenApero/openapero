#include "open_apero_core/plugin/evaluator/surface_frontier_evaluator.h"

#include <algorithm>
#include <memory>
#include <stdexcept>

#include "open_apero_core/plugin/map/occupancy_map.h"

namespace open_apero {
namespace evaluator {

PluginRegistry::Registration<SurfaceFrontierEvaluator> SurfaceFrontierEvaluator::registration("SurfaceFrontierEvaluator");

SurfaceFrontierEvaluator::SurfaceFrontierEvaluator(AgentInterface& agent) : SimulatedSensorEvaluator(agent) {}

void SurfaceFrontierEvaluator::setupFromParamMap(Configurable::ParamMap* param_map) {
  if (dynamic_cast<map::OccupancyMap*>(&agent_.getMap()) == nullptr) {
    throw std::runtime_error("'SurfaceFrontierEvaluator' requires a map of type 'OccupancyMap'!");
  }
  SimulatedSensorEvaluator::setupFromParamMap(param_map);
  setParam<bool>(param_map, "accurate_frontiers", &p_accurate_frontiers_, false);
  setParam<bool>(param_map, "surface_frontiers", &p_surface_frontiers_, true);
  setParam<double>(param_map, "checking_distance", &p_checking_distance_, 1.0);

  setParam<double>(param_map, "frontier_voxel_weight", &p_frontier_voxel_weight_, 1.0);
  setParam<double>(param_map, "new_voxel_weight", &p_new_voxel_weight_, 0.01);

  // Fast mode uses face neighbors; accurate mode uses all 26 neighbors.
  frontier_classifier_.emplace(map_, p_accurate_frontiers_, p_surface_frontiers_, p_checking_distance_);
}

bool SurfaceFrontierEvaluator::isSurfaceFrontier(const Eigen::Vector3d& voxel) const {
  return frontier_classifier_->isSurfaceFrontier(voxel);
}

// Compute gain from visible voxels: unknown voxels at the surface frontier receive
// frontier_voxel_weight, other unknown voxels receive the smaller new_voxel_weight.
// This prioritizes viewpoints that extend the reconstructed surface boundary.
bool SurfaceFrontierEvaluator::evaluateFromVisibleVoxels(Node* node) {
  node->gain_anchors.clear();
  if (!node->info) {
    return false;
  }

  SimulatedSensorInfo* info = node->info->as<SimulatedSensorInfo>();
  if (info == nullptr) {
    return false;
  }

  double total_gain = 0.0;
  for (auto& [voxel_center, voxel_info] : info->visible_voxels) {
    if (voxel_info.occupancy == Occupancy::kUnknown) {
      if (isSurfaceFrontier(voxel_center)) {
        voxel_info.gain = p_frontier_voxel_weight_;
      } else {
        voxel_info.gain = p_new_voxel_weight_;
      }
      total_gain += voxel_info.gain;
    }
  }

  node->gain_anchors = {std::make_shared<GainAnchor>(total_gain)};
  return true;
}

double SurfaceFrontierEvaluator::getRealAccumulatedGain() const {
  return map_->getMappedSurfaceArea();
}

void SurfaceFrontierEvaluator::getVisualization(VisualizationMarkers* markers, const Node* node) {
  // Visualize sensor view
  if (p_visualize_sensor_view_) {
    sensor_model_->visualizeSensorView(markers, node);
  }

  // Visualize the bounding volume
  if (bounding_volume_->is_setup) {
    bounding_volume_->visualize(markers);
  }

  // Every visible voxel with gain.
  const SimulatedSensorInfo* info = node->info ? node->info->as<SimulatedSensorInfo>() : nullptr;
  if (info == nullptr) {
    return;
  }

  VisualizationMarker marker;
  marker.ns = "voxel_gains";
  marker.type = VisualizationMarker::CUBE_LIST;
  const double voxel_size = map_->getVoxelSize();
  marker.scale.x() = voxel_size;
  marker.scale.y() = voxel_size;
  marker.scale.z() = voxel_size;

  double max_gain = std::max(p_frontier_voxel_weight_, p_new_voxel_weight_);
  for (const auto& [voxel_center, voxel_info] : info->visible_voxels) {
    auto gain = voxel_info.gain;
    if (gain < 1e-4) {
      continue;
    }

    marker.points.push_back(voxel_center);

    Color c;
    c.r = 1.0;
    c.g = 0.8;
    c.b = 0.0;
    c.a = gain / max_gain;
    marker.colors.push_back(c);
  }

  markers->addMarker(marker);
}

}  // namespace evaluator
}  // namespace open_apero
