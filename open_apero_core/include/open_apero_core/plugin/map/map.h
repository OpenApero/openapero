#pragma once

/**
 * @file map.h
 * @brief Base class of the map backends the planner queries.
 *
 * Map holds the queries every backend shares (frame bookkeeping for the map-update gate, hit regions, ground
 * height, statistics, saving), each with a default. Voxel occupancy queries are added by map::OccupancyMap and
 * distance queries by map::TsdfMap.
 */

#include <string>
#include <vector>

#include <Eigen/Core>

#include "open_apero_core/agent/agent_interface.h"
#include "open_apero_core/geometry/geometry_primitives.h"
#include "open_apero_core/geometry/transform3d.h"
#include "open_apero_core/plugin/plugin.h"
#include "open_apero_core/state/robot_state.h"
#include "open_apero_core/visualization/visualization_markers.h"

namespace open_apero {

/**
 * @brief Base class of every map backend used by the agent.
 *
 * Declares no pure virtual method: every method has a default (no-op, false or 0), so a backend overrides only what
 * it supports. Voxel occupancy queries, which SensorModel and the evaluators use, are declared by map::OccupancyMap;
 * distance queries, which SimpleRobot's traversability check uses, by map::TsdfMap.
 */
class Map : public Plugin {
 public:
  /** @brief Constructs a Map and binds it to the agent. */
  explicit Map(AgentInterface& agent);
  virtual ~Map() = default;

  /** @brief Resets per-episode map state (accumulated data, update tracking). Default: no-op. */
  void resetEpisode() override {}

  /**
   * @brief Returns the total volume that has been explored.
   * @return Explored volume in cubic meters.
   */
  virtual double getExploredVolume() const { return 0.0; }

  /** @brief Returns the total mapped surface area in square meters. */
  virtual double getMappedSurfaceArea() const { return 0.0; }

  /**
   * @brief Gets the axis-aligned bounding box of the map region updated since the last resetMapUpdateStatus().
   * @param updated_map_min Output minimum corner of the updated region.
   * @param updated_map_max Output maximum corner of the updated region.
   * @return true if an update region is available.
   */
  virtual bool getUpdatedMapRegion(Eigen::Vector3d* updated_map_min, Eigen::Vector3d* updated_map_max);

  /**
   * @brief Gets the axis-aligned bounding box encompassing the entire map.
   * @param total_map_min Output minimum corner.
   * @param total_map_max Output maximum corner.
   * @return true if the map region is available.
   */
  virtual bool getTotalMapRegion(Eigen::Vector3d* total_map_min, Eigen::Vector3d* total_map_max);

  /**
   * @brief Gets the optical-frame transforms the map integrated since the last resetMapUpdateStatus().
   * @param transforms Output list of optical frame transforms.
   * @return true if transforms are available.
   */
  virtual bool getMappedOpticalFrameTransforms(std::vector<Transform3d>* transforms);

  /**
   * @brief Reports whether an integrated frame lies within the tolerances of an optical frame.
   *
   * Default: scans getMappedOpticalFrameTransforms. Backends whose readiness needs
   * more than the geometric frame list override this.
   * @param optical_frame Optical-frame transform to match.
   * @param position_tolerance Position tolerance in meters.
   * @param angle_tolerance Orientation tolerance in radians.
   * @return true if a frame integrated since the last resetMapUpdateStatus() matches.
   */
  virtual bool hasMappedFrameNear(const Transform3d& optical_frame, double position_tolerance, double angle_tolerance);

  /**
   * @brief Replaces the stored obstacle hit region primitives.
   * @param hit_regions Hit region geometry primitives to store.
   */
  virtual void setHitRegions(const GeometryPrimitiveCollection& hit_regions);

  /**
   * @brief Retrieves all recorded obstacle hit region primitives.
   * @param hit_regions Output collection of hit region geometry primitives.
   */
  virtual void getHitRegions(GeometryPrimitiveCollection& hit_regions) const;

  /**
   * @brief Marks a failed motion (stopped -> target) as untraversable.
   *
   * Default: no-op. Backends that override it may also block the stopped state's own region, so
   * recovery planners must handle escaping from a blocked start; blocked regions accumulate until
   * resetEpisode().
   *
   * @param stopped The state where the robot actually stopped.
   * @param target  The target/planned state the robot failed to reach.
   */
  virtual void registerHit(const RobotState& stopped, const RobotState& target);

  /**
   * @brief Reports whether the exact cell of a state is currently reachable.
   *
   * Default: true (no information). Backends with a walkable-surface model override it, e.g. to
   * reject states inside a registered hit region.
   *
   * @param state Robot state to test.
   * @return true if the state's cell is reachable (or unknown).
   */
  virtual bool isStateReachable(const RobotState& state) const;

  /**
   * @brief Reports the height of the traversable ground under a state, if known.
   *
   * Default: false (no ground model).
   *
   * @param state Robot state whose ground to look up.
   * @param ground_z Output ground height in world coordinates; unchanged on failure.
   * @return true if a ground height was resolved for this state.
   */
  virtual bool getGroundHeight(const RobotState& state, double* ground_z) const;

  /**
   * @brief Starts a new map-update window.
   *
   * The agent calls it once per cycle, right before dispatching the next step (and when no layer offers one). A
   * backend clears the updated region getUpdatedMapRegion() reports and the frames
   * getMappedOpticalFrameTransforms() and hasMappedFrameNear() report, so with wait_for_map_updated they then
   * cover only frames integrated since that dispatch. The default does nothing.
   *
   * @return Whether the backend reset its status; the agent ignores it. The default returns false.
   */
  virtual bool resetMapUpdateStatus() { return false; }

  /**
   * @brief Persists the current map to a file.
   * @param filename Path to the output map file.
   * @return true if the map was saved successfully.
   */
  virtual bool saveMap(const std::string& filename);

  /**
   * @brief Generates visualization markers for the current map state.
   *
   * Produces hit-region triangles, mapped-optical-frame arrows, and
   * bounding-box cubes for the recently updated and total map extents.
   *
   * @param markers Output marker container.
   */
  virtual void getVisualization(VisualizationMarkers* markers);

  /**
   * @brief Keys of the agent's namespace that the backend reads itself, not through setParam(), such as the
   *        parameters of an embedded mapping server; the platform agent counts them as read. Default: none.
   */
  virtual std::vector<std::string> agentParamsReadByBackend() const { return {}; }
};

}  // namespace open_apero
