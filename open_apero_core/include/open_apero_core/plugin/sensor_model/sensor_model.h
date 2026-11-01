#pragma once

/**
 * @file sensor_model.h
 * @brief Abstract base class for sensor models used in gain computation.
 *
 * SensorModel simulates what a sensor (camera or LiDAR) can observe from a
 * given viewpoint by performing ray casting into the volumetric map. Returns
 * the set of visible voxels for use by node evaluators.
 */

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "open_apero_core/geometry/transform3d.h"
#include "open_apero_core/graph/graph_node.h"
#include "open_apero_core/graph/node.h"
#include "open_apero_core/graph/tree_node.h"
#include "open_apero_core/plugin/map/occupancy_map.h"
#include "open_apero_core/plugin/plugin_factory.h"
#include "open_apero_core/visualization/visualization_markers.h"

namespace open_apero {

/**
 * @brief Abstract base class for sensor simulation via ray casting.
 *
 * Provides the interface for computing visible voxels from a given sensor
 * pose. Concrete implementations (SimpleRayCaster, IterativeRayCaster, etc.)
 * define the ray casting strategy and sensor geometry.
 */
class SensorModel : public Plugin {
 public:
  /** @brief Constructs a SensorModel and binds it to the agent. */
  explicit SensorModel(AgentInterface& agent);

  virtual ~SensorModel() = default;

  /**
   * @brief Returns voxels visible from a planning node's sensor pose.
   * @param result Output map of voxel positions to occupancy states.
   * @param node_in The planning node whose sensor pose is used.
   * @return true on success.
   */
  virtual bool getVisibleVoxelsFromNode(VoxelOccupancyMap* result, const Node* node_in);

  /**
   * @brief Returns visible voxel centers and their occupancy states for an explicit sensor pose.
   * @param result Output map of voxel positions to occupancy states.
   * @param sensor_transform Sensor transform in world frame.
   * @return true on success.
   */
  virtual bool getVisibleVoxels(VoxelOccupancyMap* result, const Transform3d& sensor_transform) = 0;

  /**
   * @brief Generates visualization markers showing the sensor's visible volume at a node.
   * @param markers Output visualization markers.
   * @param node The node whose sensor view to visualize.
   */
  virtual void visualizeSensorView(VisualizationMarkers* markers, const Node* node) = 0;

  /**
   * @brief Loads base sensor model parameters (ray length, log_cast_timing) from the config map.
   * @throws std::runtime_error if the agent's map is not an OccupancyMap.
   */
  void setupFromParamMap(Configurable::ParamMap* param_map) override;

  /** @brief Reset per-episode sensor model state. */
  void resetEpisode() override {
    std::lock_guard<std::mutex> lock(time_count_mutex_);
    time_count_.clear();
  }

  /**
   * @brief Returns the axis-aligned bounding box of the sensor's visible volume at a given pose.
   * @param min Output minimum corner of the bounding box.
   * @param max Output maximum corner of the bounding box.
   * @param sensor_transform Sensor transform in world frame.
   */
  virtual void getSensorViewBoundary(Eigen::Vector3d& min, Eigen::Vector3d& max, const Transform3d& sensor_transform) = 0;

  /**
   * @brief Whether a world point lies inside the sensor's visible volume at a pose.
   *
   * Range and field of view only, no occlusion. @p margin grows the volume (half a voxel
   * diagonal makes the test conservative for voxels). The default reports every point visible.
   *
   * @param point            Query point in world frame.
   * @param sensor_transform Sensor transform in world frame.
   * @param margin           Distance by which to expand the view volume.
   * @return true if @p point lies in the view volume.
   */
  virtual bool pointInSensorView(const Eigen::Vector3d& point, const Transform3d& sensor_transform, double margin = 0.0) const;

 protected:
  /**
   * @brief Records a voxel a ray reached; kOccupied overrides an earlier reading of the voxel.
   * @param result    Visible voxels of the cast.
   * @param center    Voxel center in world frame.
   * @param occupancy Occupancy the ray read.
   */
  static void recordVoxel(VoxelOccupancyMap* result, const Eigen::Vector3d& center, Occupancy occupancy);

  /// Occupancy map used for ray queries: the agent's map, which setup requires to be an OccupancyMap.
  map::OccupancyMap* map_ = nullptr;

  double p_ray_length_ = 5.0;

  /// Time every getVisibleVoxelsFromNode() cast and log the mean and spread every 20 casts.
  bool p_log_cast_timing_ = false;
  /// Cast durations [ms] this episode, guarded by time_count_mutex_.
  std::vector<double> time_count_;
  std::mutex time_count_mutex_;
};

}  // namespace open_apero
