#include "open_apero_core/plugin/sensor_model/simple_ray_caster.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "open_apero_core/geometry/transform3d.h"

namespace open_apero {
namespace sensor_model {

PluginRegistry::Registration<SimpleRayCaster> SimpleRayCaster::registration("SimpleRayCaster");

SimpleRayCaster::SimpleRayCaster(AgentInterface& agent) : CameraModel(agent) {}

void SimpleRayCaster::setupFromParamMap(Configurable::ParamMap* param_map) {
  CameraModel::setupFromParamMap(param_map);
  setParam<double>(param_map, "ray_step", &p_ray_step_, map_->getVoxelSize());
  setParam<double>(param_map, "downsampling_factor", &p_downsampling_factor_, 1.0);

  // Rays downsampling_factor voxels apart at the observation boundary, at most one per pixel.
  const double ray_spacing = map_->getVoxelSize() * p_downsampling_factor_;
  c_res_x_ =
      std::min(static_cast<int>(ceil(getObservationBoundaryWidth(c_field_of_view_x_, p_resolution_x_) / ray_spacing)), p_resolution_x_);
  c_res_y_ =
      std::min(static_cast<int>(ceil(getObservationBoundaryWidth(c_field_of_view_y_, p_resolution_y_) / ray_spacing)), p_resolution_y_);
}

// Precondition of getVisibleVoxels: a positive ray step, and at least two rays per axis so the
// normalization by (c_res - 1) is defined.
bool SimpleRayCaster::checkParamsValid(std::string* error_message) {
  if (!CameraModel::checkParamsValid(error_message)) {
    return false;
  }
  if (p_ray_step_ <= 0.0) {
    *error_message = "ray_step expected > 0.0";
    return false;
  }
  if (p_downsampling_factor_ <= 0.0) {
    *error_message = "downsampling_factor expected > 0.0";
    return false;
  }
  if (c_res_x_ < 2 || c_res_y_ < 2) {
    std::ostringstream ss;
    ss << "degenerate ray grid: computed ray resolution " << c_res_x_ << "x" << c_res_y_
       << " needs at least 2 rays per axis. Raise ray_length or the resolutions, or lower focal_length, downsampling_factor or the "
          "voxel size.";
    *error_message = ss.str();
    return false;
  }
  return true;
}

bool SimpleRayCaster::getVisibleVoxels(VoxelOccupancyMap* result, const Transform3d& sensor_transform) {
  // Naive ray-casting
  Eigen::Vector3d camera_direction;
  Eigen::Vector3d direction;
  Eigen::Vector3d current_position;
  Eigen::Vector3d voxel_center;
  for (int i = 0; i < c_res_x_; ++i) {
    for (int j = 0; j < c_res_y_; ++j) {
      CameraModel::getDirectionVector(&camera_direction, static_cast<double>(i) / (static_cast<double>(c_res_x_) - 1.0),
                                      static_cast<double>(j) / (static_cast<double>(c_res_y_) - 1.0));
      direction = sensor_transform.orientation * camera_direction;

      std::vector<Eigen::Vector3d> voxels;
      std::vector<Occupancy> voxel_occupancies;
      const double ray_length = getRayLength(camera_direction);
      double distance = 0.0;
      while (distance < ray_length) {
        current_position = sensor_transform.position + distance * direction;
        distance += p_ray_step_;

        // The first occupied reading is seen; the ray stops on the surface's voxel.
        Occupancy occ = map_->getVoxelOccupancy(current_position);
        if (occ == Occupancy::kOccupied) {
          map_->getSurfaceVoxelCenter(&voxel_center, current_position, direction);
          voxels.push_back(voxel_center);
          voxel_occupancies.push_back(occ);
          break;
        }

        // Add point with occupancy state
        map_->getVoxelCenter(&voxel_center, current_position);

        // Skip if this voxel is already in the list
        if ((voxels.size() > 0) && (voxels.back() - voxel_center).squaredNorm() < 1e-8) {
          continue;
        }

        voxels.push_back(voxel_center);
        voxel_occupancies.push_back(occ);
      }

      // Add to result with occupancy
      for (size_t k = 0; k < voxels.size(); ++k) {
        recordVoxel(result, voxels[k], voxel_occupancies[k]);
      }
    }
  }
  return true;
}

}  // namespace sensor_model
}  // namespace open_apero
