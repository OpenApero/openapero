// Ray density doubles with distance to maintain roughly one ray per voxel.
// Coarse near-field rays represent blocks of finer rays; an occupied hit marks
// the represented block as occluded so hidden fine rays are never cast.
// With a planar observation boundary the sections are slabs of camera depth:
// each ray's split distances scale with its own length.

#include "open_apero_core/plugin/sensor_model/iterative_ray_caster.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "open_apero_core/geometry/transform3d.h"

namespace open_apero {
namespace sensor_model {

PluginRegistry::Registration<IterativeRayCaster> IterativeRayCaster::registration("IterativeRayCaster");

IterativeRayCaster::IterativeRayCaster(AgentInterface& agent) : CameraModel(agent) {}

void IterativeRayCaster::setupFromParamMap(Configurable::ParamMap* param_map) {
  CameraModel::setupFromParamMap(param_map);
  setParam<double>(param_map, "ray_step", &p_ray_step_, map_->getVoxelSize());
  setParam<double>(param_map, "downsampling_factor", &p_downsampling_factor_, 1.0);

  // Rays downsampling_factor voxels apart at the observation boundary, at most one per pixel.
  const double ray_spacing = map_->getVoxelSize() * p_downsampling_factor_;
  c_res_x_ =
      std::min(static_cast<int>(ceil(getObservationBoundaryWidth(c_field_of_view_x_, p_resolution_x_) / ray_spacing)), p_resolution_x_);
  c_res_y_ =
      std::min(static_cast<int>(ceil(getObservationBoundaryWidth(c_field_of_view_y_, p_resolution_y_) / ray_spacing)), p_resolution_y_);

  // Each section doubles density, giving log2(min_resolution) sections.
  c_n_sections_ = std::floor(std::log2(std::min(static_cast<double>(c_res_x_), static_cast<double>(c_res_y_))));

  // Build far-to-near, then reverse so section zero is nearest and coarsest.
  c_split_widths_.push_back(0);
  for (int i = 0; i < c_n_sections_; ++i) {
    c_split_widths_.push_back(std::pow(2, i));
    c_split_distances_.push_back(p_ray_length_ / std::pow(2.0, static_cast<double>(i)));
  }
  c_split_distances_.push_back(0.0);
  std::reverse(c_split_distances_.begin(), c_split_distances_.end());
  std::reverse(c_split_widths_.begin(), c_split_widths_.end());
}

// Precondition of getVisibleVoxels: the ray grid spans at least two rays per
// axis, so the normalization by (c_res - 1) is defined and c_n_sections_ >= 1.
bool IterativeRayCaster::checkParamsValid(std::string* error_message) {
  if (!CameraModel::checkParamsValid(error_message)) {
    return false;
  }
  if (p_downsampling_factor_ <= 0.0) {
    *error_message = "downsampling_factor expected > 0.0";
    return false;
  }
  if (p_ray_step_ <= 0.0) {
    *error_message = "ray_step expected > 0.0";
    return false;
  }
  if (c_res_x_ < 2 || c_res_y_ < 2) {
    std::ostringstream ss;
    ss << "degenerate ray grid: computed ray resolution " << c_res_x_ << "x" << c_res_y_
       << " needs at least 2 rays per axis. It follows from ray_length: " << p_ray_length_ << ", focal_length: " << p_focal_length_
       << ", resolution_x: " << p_resolution_x_ << ", resolution_y: " << p_resolution_y_
       << ", downsampling_factor: " << p_downsampling_factor_ << ", voxel size: " << map_->getVoxelSize()
       << ". Raise ray_length or the resolutions, or lower focal_length, downsampling_factor or the voxel size.";
    *error_message = ss.str();
    return false;
  }
  return true;
}

bool IterativeRayCaster::getVisibleVoxels(VoxelOccupancyMap* result, const Transform3d& sensor_transform) {
  // Each cell stores its first section; -1 marks a ray as occluded.
  Eigen::ArrayXXi ray_table = Eigen::ArrayXXi::Zero(c_res_x_, c_res_y_);

  Eigen::Vector3d camera_direction;
  Eigen::Vector3d direction;
  Eigen::Vector3d current_position;
  Eigen::Vector3d voxel_center;
  double distance;
  bool cast_ray;
  for (int i = 0; i < c_res_x_; ++i) {
    for (int j = 0; j < c_res_y_; ++j) {
      int current_segment = ray_table(i, j);
      if (current_segment < 0) {
        continue;
      }

      CameraModel::getDirectionVector(&camera_direction, static_cast<double>(i) / (static_cast<double>(c_res_x_) - 1.0),
                                      static_cast<double>(j) / (static_cast<double>(c_res_y_) - 1.0));
      direction = sensor_transform.orientation * camera_direction;
      const double split_scale = getRayLength(camera_direction) / p_ray_length_;

      distance = split_scale * c_split_distances_[current_segment];
      cast_ray = true;
      while (cast_ray) {
        while (distance < split_scale * c_split_distances_[current_segment + 1]) {
          current_position = sensor_transform.position + distance * direction;
          distance += p_ray_step_;

          Occupancy occ = map_->getVoxelOccupancy(current_position);
          if (occ == Occupancy::kOccupied) {
            map_->getSurfaceVoxelCenter(&voxel_center, current_position, direction);
          } else {
            map_->getVoxelCenter(&voxel_center, current_position);
          }
          recordVoxel(result, voxel_center, occ);

          if (occ == Occupancy::kOccupied) {
            // One hit occludes the entire fine-ray block represented here.
            markNeighboringRays(i, j, current_segment, -1, ray_table);
            cast_ray = false;
            break;
          }
        }
        if (cast_ray) {
          current_segment++;
          if (current_segment >= c_n_sections_) {
            cast_ray = false;
          } else {
            // Fine rays start beyond the region their coarse parent covered.
            markNeighboringRays(i, j, current_segment - 1, current_segment, ray_table);
          }
        }
      }
    }
  }

  return true;
}

// Update the fine-ray block represented by (x, y) at this section. A value of
// -1 marks occlusion; a nonnegative value promotes the block to that section.
void IterativeRayCaster::markNeighboringRays(int x, int y, int segment, int value, Eigen::ArrayXXi& ray_table) {
  for (int i = x; i < std::min(c_res_x_, x + c_split_widths_[segment]); ++i) {
    for (int j = y; j < std::min(c_res_y_, y + c_split_widths_[segment]); ++j) {
      ray_table(i, j) = value;
    }
  }
}

}  // namespace sensor_model
}  // namespace open_apero
