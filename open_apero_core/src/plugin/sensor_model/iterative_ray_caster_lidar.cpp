#include "open_apero_core/plugin/sensor_model/iterative_ray_caster_lidar.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "open_apero_core/geometry/transform3d.h"

namespace open_apero {
namespace sensor_model {

PluginRegistry::Registration<IterativeRayCasterLidar> IterativeRayCasterLidar::registration("IterativeRayCasterLidar");

IterativeRayCasterLidar::IterativeRayCasterLidar(AgentInterface& agent) : LidarModel(agent) {}

void IterativeRayCasterLidar::setupFromParamMap(Configurable::ParamMap* param_map) {
  LidarModel::setupFromParamMap(param_map);
  setParam<double>(param_map, "ray_step", &p_ray_step_, map_->getVoxelSize());
  setParam<double>(param_map, "downsampling_factor", &p_downsampling_factor_, 1.0);

  // Downsample to voxel size resolution at max range
  c_res_x_ = std::min(static_cast<int>(std::ceil(p_ray_length_ * p_field_of_view_x_ / (map_->getVoxelSize() * p_downsampling_factor_))),
                      p_resolution_x_);
  c_res_y_ = std::min(static_cast<int>(std::ceil(p_ray_length_ * p_field_of_view_y_ / (map_->getVoxelSize() * p_downsampling_factor_))),
                      p_resolution_y_);

  // Determine number of splits + split distances
  c_n_sections_ = std::floor(static_cast<double>(std::log2(std::min(c_res_x_, c_res_y_))));
  c_split_widths_.push_back(0);
  for (int i = 0; i < c_n_sections_; ++i) {
    c_split_widths_.push_back(std::pow(2, i));
    c_split_distances_.push_back(p_ray_length_ / std::pow(2.0, static_cast<double>(i)));
  }
  c_split_distances_.push_back(0.0);
  std::reverse(c_split_distances_.begin(), c_split_distances_.end());
  std::reverse(c_split_widths_.begin(), c_split_widths_.end());
}

// Precondition of getVisibleVoxels: a positive ray step, and at least two rays per axis so the
// normalization by (c_res - 1) is defined and c_n_sections_ >= 1.
bool IterativeRayCasterLidar::checkParamsValid(std::string* error_message) {
  if (!LidarModel::checkParamsValid(error_message)) {
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
       << " needs at least 2 rays per axis. Raise ray_length, the fields of view or the resolutions, or lower downsampling_factor or "
          "the voxel size.";
    *error_message = ss.str();
    return false;
  }
  return true;
}

bool IterativeRayCasterLidar::getVisibleVoxels(VoxelOccupancyMap* result, const Transform3d& sensor_transform) {
  // Setup ray table (contains at which segment to start, -1 if occluded
  Eigen::ArrayXXi ray_table = Eigen::ArrayXXi::Zero(c_res_x_, c_res_y_);

  // Ray-casting
  Eigen::Vector3d camera_direction;
  Eigen::Vector3d direction;
  Eigen::Vector3d current_position;
  Eigen::Vector3d voxel_center;
  double distance;
  bool cast_ray;
  for (int i = 0; i < c_res_x_; ++i) {
    for (int j = 0; j < c_res_y_; ++j) {
      int current_segment = ray_table(i, j);  // get ray starting segment
      if (current_segment < 0) {
        continue;  // already occluded ray
      }
      LidarModel::getDirectionVector(&camera_direction, static_cast<double>(i) / (static_cast<double>(c_res_x_) - 1.0),
                                     static_cast<double>(j) / (static_cast<double>(c_res_y_) - 1.0));
      direction = sensor_transform.orientation * camera_direction;
      distance = c_split_distances_[current_segment];
      cast_ray = true;
      while (cast_ray) {
        // iterate through all splits (segments)
        while (distance < c_split_distances_[current_segment + 1]) {
          current_position = sensor_transform.position + distance * direction;
          distance += p_ray_step_;

          Occupancy occ = map_->getVoxelOccupancy(current_position);
          if (occ == Occupancy::kOccupied) {
            map_->getSurfaceVoxelCenter(&voxel_center, current_position, direction);
          } else {
            map_->getVoxelCenter(&voxel_center, current_position);
          }
          recordVoxel(result, voxel_center, occ);

          // Check voxel occupied
          if (occ == Occupancy::kOccupied) {
            // Occlusion, mark neighboring rays as occluded
            markNeighboringRays(i, j, current_segment, -1, ray_table);
            cast_ray = false;
            break;
          }
        }
        if (cast_ray) {
          current_segment++;
          if (current_segment >= c_n_sections_) {
            cast_ray = false;  // done
          } else {
            // update ray starts of neighboring rays
            markNeighboringRays(i, j, current_segment - 1, current_segment, ray_table);
          }
        }
      }
    }
  }
  return true;
}

void IterativeRayCasterLidar::markNeighboringRays(int x, int y, int segment, int value, Eigen::ArrayXXi& ray_table) {
  // Set all nearby (towards bottom right) ray starts, depending on the segment
  // depth, to a value.
  for (int i = x; i < std::min(c_res_x_, x + c_split_widths_[segment]); ++i) {
    for (int j = y; j < std::min(c_res_y_, y + c_split_widths_[segment]); ++j) {
      ray_table(i, j) = value;
    }
  }
}

}  // namespace sensor_model
}  // namespace open_apero
