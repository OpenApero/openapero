#include "open_apero_core/plugin/evaluator/voxel_weight_evaluator.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

#include "open_apero_core/plugin/map/tsdf_map.h"
#include "open_apero_core/visualization/colormap.h"

namespace open_apero {
namespace evaluator {

PluginRegistry::Registration<VoxelWeightEvaluator> VoxelWeightEvaluator::registration("VoxelWeightEvaluator");

VoxelWeightEvaluator::VoxelWeightEvaluator(AgentInterface& agent) : SurfaceFrontierEvaluator(agent) {}

void VoxelWeightEvaluator::setupFromParamMap(Configurable::ParamMap* param_map) {
  tsdf_map_ = dynamic_cast<map::TsdfMap*>(&(agent_.getMap()));
  if (tsdf_map_ == nullptr) {
    throw std::runtime_error("'VoxelWeightEvaluator' requires a map of type 'TsdfMap'!");
  }

  // frontier_voxel_weight and new_voxel_weight are read by SurfaceFrontierEvaluator.
  SurfaceFrontierEvaluator::setupFromParamMap(param_map);
  setParam<double>(param_map, "min_impact_factor", &p_min_impact_factor_, 0.0);
  setParam<double>(param_map, "ray_angle_x", &p_ray_angle_x_, 0.0025);
  setParam<double>(param_map, "ray_angle_y", &p_ray_angle_y_, 0.0025);

  c_voxel_size_ = tsdf_map_->getVoxelSize();
}

bool VoxelWeightEvaluator::evaluateFromVisibleVoxels(Node* node) {
  node->gain_anchors.clear();
  if (!node->info) {
    return false;
  }

  SimulatedSensorInfo* info = node->info->as<SimulatedSensorInfo>();
  if (info == nullptr) {
    return false;
  }

  // Weights are taken as observed from the node's own sensor pose.
  double total_gain = 0.0;
  const auto& origin = node->robot_state.getSensorPosition();
  for (auto& [voxel_center, voxel_info] : info->visible_voxels) {
    voxel_info.gain = getVoxelValue(voxel_center, origin, voxel_info.occupancy);
    total_gain += voxel_info.gain;
  }

  node->gain_anchors = {std::make_shared<GainAnchor>(total_gain)};
  return true;
}

// Surface gain favors close, under-observed voxels. Unknown frontiers receive
// the larger exploration weight, while free voxels contribute nothing.
double VoxelWeightEvaluator::getVoxelValue(const Eigen::Vector3d& voxel, const Eigen::Vector3d& origin, Occupancy occupancy) {
  if (occupancy == Occupancy::kOccupied) {
    double z = (voxel - origin).norm();
    double spanned_angle = 2.0 * atan2(c_voxel_size_, z * 2.0);
    double new_weight = std::pow(spanned_angle, 2.0) / (p_ray_angle_x_ * p_ray_angle_y_) / std::pow(z, 2.0);
    double gain = new_weight / (new_weight + tsdf_map_->getVoxelWeight(voxel));
    if (gain > p_min_impact_factor_) {
      return gain;
    }
  } else if (occupancy == Occupancy::kUnknown) {
    if (p_frontier_voxel_weight_ > 0.0) {
      if (isSurfaceFrontier(voxel)) {
        return p_frontier_voxel_weight_;
      }
    }
    return p_new_voxel_weight_;
  }
  return 0;
}

void VoxelWeightEvaluator::getVisualization(VisualizationMarkers* markers, const Node* node) {
  if (p_visualize_sensor_view_) {
    sensor_model_->visualizeSensorView(markers, node);
  }

  if (bounding_volume_->is_setup) {
    bounding_volume_->visualize(markers);
  }

  // Color impact green-to-red, frontiers purple, and other unknowns teal.
  const SimulatedSensorInfo* info = node->info ? node->info->as<SimulatedSensorInfo>() : nullptr;
  if (info == nullptr) {
    return;
  }

  VisualizationMarker marker;
  marker.ns = "voxel_gains";
  marker.type = VisualizationMarker::CUBE_LIST;
  marker.scale.x() = c_voxel_size_;
  marker.scale.y() = c_voxel_size_;
  marker.scale.z() = c_voxel_size_;

  for (const auto& [voxel_center, voxel_info] : info->visible_voxels) {
    const double value = voxel_info.gain;
    if (value > 0.0) {
      marker.points.push_back(voxel_center);
      Color color;
      if (value == p_frontier_voxel_weight_) {
        color.r = 0.6;
        color.g = 0.4;
        color.b = 1.0;
        color.a = 0.5;
      } else if (value == p_new_voxel_weight_) {
        color.r = 0.0;
        color.g = 0.75;
        color.b = 1.0;
        color.a = 0.1;
      } else {
        color = getColorFromColormap(value, p_min_impact_factor_, 1.0, 0.5);
      }
      marker.colors.push_back(color);
    }
  }
  markers->addMarker(marker);
}

}  // namespace evaluator
}  // namespace open_apero
