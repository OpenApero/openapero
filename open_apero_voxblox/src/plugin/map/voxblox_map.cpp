// TSDF data supplies observation weights and surface estimates; ESDF data
// supplies clearance, collision, occupancy, and explored-volume queries.

#include "open_apero_voxblox/plugin/map/voxblox_map.h"

#include <cmath>
#include <string>
#include <vector>

#include <std_srvs/Trigger.h>
#include <voxblox_ros/ros_params.h>

namespace open_apero {
namespace map {

PluginRegistry::Registration<VoxbloxMap> VoxbloxMap::registration("VoxbloxMap");

VoxbloxMap::VoxbloxMap(AgentInterface& agent) : TsdfMap(agent) {}

voxblox::EsdfServer& VoxbloxMap::getEsdfServer() {
  return *esdf_server_;
}

const std::vector<std::string>& VoxbloxMap::nodeParamsReadByVoxblox() {
  // Every key the OpenApero fork of voxblox_ros reads through nh_private.
  static const std::vector<std::string> kKeys = {"T_B_C",
                                                 "T_B_D",
                                                 "T_V_G",
                                                 "accumulate_icp_corrections",
                                                 "add_robot_pose",
                                                 "allow_clear",
                                                 "anti_grazing",
                                                 "clear_checks_every_n_frames",
                                                 "clear_sphere_for_planning",
                                                 "clear_sphere_radius",
                                                 "color_mode",
                                                 "depth_camera_resolution_u",
                                                 "depth_camera_resolution_v",
                                                 "enable_icp",
                                                 "esdf_add_occupied_crust",
                                                 "esdf_default_distance_m",
                                                 "esdf_euclidean_distance",
                                                 "esdf_max_distance_m",
                                                 "esdf_min_diff_m",
                                                 "esdf_min_distance_m",
                                                 "fov_h_rad",
                                                 "frame_id",
                                                 "generate_mesh",
                                                 "generate_occupancy",
                                                 "gt_file_path",
                                                 "icp_corrected_frame",
                                                 "icp_inital_rotation_weighting",
                                                 "icp_inital_translation_weighting",
                                                 "icp_min_match_ratio",
                                                 "icp_mini_batch_size",
                                                 "icp_refine_roll_pitch",
                                                 "icp_subsample_keep_ratio",
                                                 "incremental",
                                                 "integration_order_mode",
                                                 "integrator_threads",
                                                 "intensity_colormap",
                                                 "intensity_focal_length",
                                                 "intensity_max_distance",
                                                 "intensity_max_value",
                                                 "intensity_min_value",
                                                 "invert_T_B_C",
                                                 "invert_T_B_D",
                                                 "invert_T_V_G",
                                                 "max_attempts_to_generate_viewpoint",
                                                 "max_block_distance_from_body",
                                                 "max_consecutive_ray_collisions",
                                                 "max_dist",
                                                 "max_integration_time_s",
                                                 "max_ray_length_m",
                                                 "max_weight",
                                                 "mesh_filename",
                                                 "mesh_integrator_threads",
                                                 "mesh_min_weight",
                                                 "mesh_use_color",
                                                 "method",
                                                 "min_dist",
                                                 "min_ray_length_m",
                                                 "min_time_between_msgs_sec",
                                                 "num_viewpoints",
                                                 "observation_boundary",
                                                 "occupied_sphere_radius",
                                                 "pointcloud_queue_size",
                                                 "pose_corrected_frame",
                                                 "publish_esdf_map",
                                                 "publish_map_every_n_sec",
                                                 "publish_pointclouds",
                                                 "publish_pointclouds_on_update",
                                                 "publish_slices",
                                                 "publish_traversable",
                                                 "publish_tsdf_map",
                                                 "recolor_by_error",
                                                 "sensor_frame",
                                                 "slice_level",
                                                 "sparsity_compensation_factor",
                                                 "start_voxel_subsampling_factor",
                                                 "subsample_factor",
                                                 "sync_queue_size",
                                                 "timestamp_tolerance_sec",
                                                 "traversability_radius",
                                                 "truncation_distance",
                                                 "tsdf_mesh_color_mode",
                                                 "tsdf_mesh_output_path",
                                                 "tsdf_proto_path",
                                                 "tsdf_surface_distance_threshold_factor",
                                                 "tsdf_voxel_ply_output_path",
                                                 "tsdf_voxel_size",
                                                 "tsdf_voxels_per_side",
                                                 "tsdf_world_frame",
                                                 "update_esdf_every_n_sec",
                                                 "update_mesh_every_n_sec",
                                                 "use_const_weight",
                                                 "use_freespace_pointcloud",
                                                 "use_sparsity_compensation_factor",
                                                 "use_tf_transforms",
                                                 "use_weight_dropoff",
                                                 "verbose",
                                                 "visualization_slice_level",
                                                 "visualize",
                                                 "voxblox_file_path",
                                                 "voxel_carving_enabled",
                                                 "world_frame"};
  return kKeys;
}

std::vector<std::string> VoxbloxMap::agentParamsReadByBackend() const {
  return nodeParamsReadByVoxblox();
}

void VoxbloxMap::setupFromParamMap(Configurable::ParamMap* /*param_map*/) {
  ::ros::NodeHandle nh_private("~");
  // The server subscribes its point-cloud and transform inputs on its first handle. Private, they stay off a robot's
  // global /pointcloud, so the map only mirrors the layers it receives on ~tsdf_map_in and ~esdf_map_in.
  esdf_server_.reset(new voxblox::EsdfServer(nh_private, nh_private));
  reset_mesh_display_client_ = nh_private.serviceClient<std_srvs::Trigger>("/voxblox_mesh_display/reset");

  c_voxel_size_ = esdf_server_->getEsdfMapPtr()->voxel_size();
  c_block_size_ = esdf_server_->getEsdfMapPtr()->block_size();
  c_maximum_weight_ = voxblox::getTsdfIntegratorConfigFromRosParam(nh_private).max_weight;  // direct access is not exposed
}

bool VoxbloxMap::getDistanceAtPosition(const Eigen::Vector3d& position, double* distance) {
  return esdf_server_->getEsdfMapPtr()->getDistanceAtPosition(position, distance);
}

Occupancy VoxbloxMap::getVoxelOccupancy(const Eigen::Vector3d& point) {
  double distance = 0.0;
  if (esdf_server_->getEsdfMapPtr()->getDistanceAtPosition(point, &distance)) {
    // The one-voxel threshold accounts for ESDF discretization near surfaces.
    if (distance < c_voxel_size_) {
      return Occupancy::kOccupied;
    } else {
      return Occupancy::kFree;
    }
  } else {
    return Occupancy::kUnknown;
  }
}

double VoxbloxMap::getVoxelSize() {
  return c_voxel_size_;
}

bool VoxbloxMap::getVoxelCenter(Eigen::Vector3d* center, const Eigen::Vector3d& point) {
  voxblox::BlockIndex block_id =
      esdf_server_->getEsdfMapPtr()->getEsdfLayerPtr()->computeBlockIndexFromCoordinates(point.cast<voxblox::FloatingPoint>());
  *center = voxblox::getOriginPointFromGridIndex(block_id, c_block_size_).cast<double>();
  voxblox::VoxelIndex voxel_id =
      voxblox::getGridIndexFromPoint<voxblox::VoxelIndex>((point - *center).cast<voxblox::FloatingPoint>(), 1.0 / c_voxel_size_);
  *center += voxblox::getCenterPointFromGridIndex(voxel_id, c_voxel_size_).cast<double>();

  constexpr double kPrecision = 1e-4;  // Stabilizes equality checks between voxel centers.
  center->x() = std::round(center->x() / kPrecision) * kPrecision;
  center->y() = std::round(center->y() / kPrecision) * kPrecision;
  center->z() = std::round(center->z() / kPrecision) * kPrecision;
  return true;
}

// Unallocated voxels have no observation weight.
double VoxbloxMap::getVoxelWeight(const Eigen::Vector3d& point) {
  voxblox::Point voxblox_point(point.x(), point.y(), point.z());
  voxblox::Block<voxblox::TsdfVoxel>::Ptr block = esdf_server_->getTsdfMapPtr()->getTsdfLayerPtr()->getBlockPtrByCoordinates(voxblox_point);
  if (block) {
    voxblox::TsdfVoxel* tsdf_voxel = block->getVoxelPtrByCoordinates(voxblox_point);
    if (tsdf_voxel) {
      return tsdf_voxel->weight;
    }
  }
  return 0.0;
}

double VoxbloxMap::getMaximumWeight() {
  return c_maximum_weight_;
}

double VoxbloxMap::getExploredVolume() const {
  auto esdf_layer = esdf_server_->getEsdfMapPtr()->getEsdfLayerPtr();
  voxblox::BlockIndexList blocks;
  esdf_layer->getAllAllocatedBlocks(&blocks);
  const size_t vps = esdf_layer->voxels_per_side();
  const size_t num_voxels_per_block = vps * vps * vps;
  const double dv = std::pow(esdf_layer->voxel_size(), 3);

  double volume = 0.0;
  for (voxblox::BlockIndex& index : blocks) {
    voxblox::Block<voxblox::EsdfVoxel>& block = esdf_layer->getBlockByIndex(index);
    for (size_t linear_index = 0; linear_index < num_voxels_per_block; ++linear_index) {
      voxblox::EsdfVoxel& voxel = block.getVoxelByLinearIndex(linear_index);
      if (!voxel.observed) {
        continue;
      }
      volume += dv;
    }
  }
  return volume;
}

double VoxbloxMap::getMappedSurfaceArea() const {
  auto esdf_layer = esdf_server_->getEsdfMapPtr()->getEsdfLayerPtr();
  voxblox::BlockIndexList blocks;
  esdf_layer->getAllAllocatedBlocks(&blocks);
  const size_t vps = esdf_layer->voxels_per_side();
  const size_t num_voxels_per_block = vps * vps * vps;
  const double da = std::pow(esdf_layer->voxel_size(), 2);

  double surface_area = 0.0;
  for (voxblox::BlockIndex& index : blocks) {
    voxblox::Block<voxblox::EsdfVoxel>& block = esdf_layer->getBlockByIndex(index);
    for (size_t linear_index = 0; linear_index < num_voxels_per_block; ++linear_index) {
      voxblox::EsdfVoxel& voxel = block.getVoxelByLinearIndex(linear_index);
      if (!voxel.observed) {
        continue;
      }
      if (std::abs(voxel.distance) < c_voxel_size_) {
        surface_area += da;
      }
    }
  }
  return surface_area;
}

bool VoxbloxMap::getUpdatedMapRegion(Eigen::Vector3d* updated_map_min, Eigen::Vector3d* updated_map_max) {
  return esdf_server_->getUpdatedMapRegion(updated_map_min, updated_map_max);
}

bool VoxbloxMap::getTotalMapRegion(Eigen::Vector3d* total_map_min, Eigen::Vector3d* total_map_max) {
  return esdf_server_->getTotalMapRegion(total_map_min, total_map_max);
}

bool VoxbloxMap::getMappedOpticalFrameTransforms(std::vector<Transform3d>* transforms) {
  std::vector<Eigen::Vector3d> positions;
  std::vector<Eigen::Quaterniond> orientations;
  if (!esdf_server_->getMappedOpticalFrameTransforms(&positions, &orientations)) {
    return false;
  }

  if (positions.size() != orientations.size()) {
    return false;
  }

  transforms->clear();
  for (size_t i = 0; i < positions.size(); ++i) {
    transforms->push_back({positions[i], orientations[i]});
  }
  return true;
}

void VoxbloxMap::resetEpisode() {
  esdf_server_->clear();
  resetVoxbloxMeshDisplay();
  esdf_server_->resetMapUpdateStatus();
}

void VoxbloxMap::resetVoxbloxMeshDisplay() {
  const std::string service_name = reset_mesh_display_client_.getService();

  if (!reset_mesh_display_client_.exists()) {
    ROS_DEBUG_STREAM("Voxblox mesh display reset service " << service_name << " is not available.");
    return;
  }

  std_srvs::Trigger reset_mesh_display;
  if (!reset_mesh_display_client_.call(reset_mesh_display)) {
    ROS_WARN_STREAM("Failed to call voxblox mesh display reset service " << service_name << ".");
  } else if (!reset_mesh_display.response.success) {
    ROS_WARN_STREAM("Voxblox mesh display reset service " << service_name << " returned failure: " << reset_mesh_display.response.message);
  }
}

bool VoxbloxMap::resetMapUpdateStatus() {
  return esdf_server_->resetMapUpdateStatus();
}

bool VoxbloxMap::saveMap(const std::string& filename) {
  // Voxblox requires the native extension.
  std::string filename_with_extension = filename;
  if (filename_with_extension.size() < 6 || filename_with_extension.substr(filename_with_extension.size() - 6) != ".vxblx") {
    filename_with_extension += ".vxblx";
  }
  return esdf_server_->saveMap(filename_with_extension);
}

}  // namespace map
}  // namespace open_apero
