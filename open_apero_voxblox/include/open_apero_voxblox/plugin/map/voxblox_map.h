#pragma once

/**
 * @file voxblox_map.h
 * @brief Voxblox-backed TSDF/ESDF map representation for collision checking,
 *        occupancy queries, and volumetric exploration metrics.
 */

#include <memory>
#include <string>
#include <vector>

#include <open_apero_core/common/typedef.h>
#include <open_apero_core/plugin/map/tsdf_map.h>
#include <open_apero_core/plugin/plugin_registry.h>
#include <ros/service_client.h>
#include <voxblox_ros/esdf_server.h>

namespace open_apero {
namespace map {

/**
 * @brief TSDF/ESDF map backed by voxblox, providing collision checking,
 *        occupancy queries, distance fields, and volumetric metrics.
 */
class VoxbloxMap : public TsdfMap {
 public:
  /**
   * @brief Construct the VoxbloxMap.
   * @param agent Reference to the parent agent interface.
   */
  explicit VoxbloxMap(AgentInterface& agent);  // NOLINT(google-runtime-references) non-const ref to agent is intentional

  /**
   * @brief Initialize the ESDF server and cache map constants from the param map.
   * @param param_map Pointer to the parameter map.
   */
  void setupFromParamMap(Configurable::ParamMap* param_map) override;

  /**
   * @brief Returns the ESDF distance at the given position.
   * @param position Query position in world coordinates.
   * @param distance Output ESDF distance value.
   * @return True if the distance was successfully retrieved.
   */
  bool getDistanceAtPosition(const Eigen::Vector3d& position, double* distance) override;

  /**
   * @brief Get the occupancy status of a voxel.
   * @param point The 3D query point.
   * @return kOccupied, kFree, or kUnknown.
   */
  Occupancy getVoxelOccupancy(const Eigen::Vector3d& point) override;

  /// @brief Return the voxel side length.
  double getVoxelSize() override;

  /**
   * @brief Compute the center coordinates of the voxel containing a point.
   * @param center Output voxel center position.
   * @param point The input query point.
   * @return True on success.
   */
  bool getVoxelCenter(Eigen::Vector3d* center, const Eigen::Vector3d& point) override;

  /**
   * @brief Get the stored TSDF integration weight at a point.
   * @param point The 3D query point.
   * @return The weight value, or 0.0 if not found.
   */
  double getVoxelWeight(const Eigen::Vector3d& point) override;

  /// @brief Return the maximum allowed TSDF weight (0 if uncapped).
  double getMaximumWeight() override;

  /// @brief Provide direct access to the underlying voxblox ESDF server.
  voxblox::EsdfServer& getEsdfServer();

  /** @brief Every parameter voxblox_ros reads from a node's private namespace, sorted (C locale): a superset
   *  of what the embedded servers read. They share the planner node's namespace, so the agent must not report them. */
  static const std::vector<std::string>& nodeParamsReadByVoxblox();

  /// @brief nodeParamsReadByVoxblox(): the embedded servers read the agent's (the planner node's) namespace.
  std::vector<std::string> agentParamsReadByBackend() const override;

  /// @brief Compute the total observed volume in cubic meters.
  double getExploredVolume() const override;

  /// @brief Compute the mapped surface area in square meters.
  double getMappedSurfaceArea() const override;

  /**
   * @brief Get the axis-aligned bounding box of recently updated map regions.
   * @param updated_map_min Output minimum corner.
   * @param updated_map_max Output maximum corner.
   * @return True if an updated region exists.
   */
  bool getUpdatedMapRegion(Eigen::Vector3d* updated_map_min, Eigen::Vector3d* updated_map_max) override;

  /**
   * @brief Get the axis-aligned bounding box of the entire map.
   * @param total_map_min Output minimum corner.
   * @param total_map_max Output maximum corner.
   * @return True if the map is non-empty.
   */
  bool getTotalMapRegion(Eigen::Vector3d* total_map_min, Eigen::Vector3d* total_map_max) override;

  /**
   * @brief Gets optical-frame transforms integrated since the last reset.
   * @param transforms Output optical frame transforms.
   * @return True if transforms are available.
   */
  bool getMappedOpticalFrameTransforms(std::vector<Transform3d>* transforms) override;

  /// @brief Resets all voxblox map data (TSDF + ESDF layers) for a new episode.
  void resetEpisode() override;

  /// @brief Clear the map-update tracking state.
  bool resetMapUpdateStatus() override;

  /**
   * @brief Persist the voxblox map to disk via the underlying ESDF server.
   * @param filename Path to the output map file.
   * @return True on success.
   */
  bool saveMap(const std::string& filename) override;

 protected:
  static PluginRegistry::Registration<VoxbloxMap> registration;

  /// @brief Reset the RViz voxblox mesh display if it is running.
  void resetVoxbloxMeshDisplay();

  // esdf server that contains the map, subscribe to external ESDF/TSDF updates
  std::unique_ptr<voxblox::EsdfServer> esdf_server_;
  ::ros::ServiceClient reset_mesh_display_client_;

  // cache constants
  double c_voxel_size_;
  double c_block_size_;
  double c_maximum_weight_;
};

}  // namespace map
}  // namespace open_apero
