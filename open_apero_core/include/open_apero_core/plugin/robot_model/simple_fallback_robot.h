#pragma once

/**
 * @file simple_fallback_robot.h
 * @brief SimpleRobot variant that adds a local sampling-based fallback planner
 *        (implemented with OMPL) for use when the straight-line
 *        steer is untraversable.
 */

#include <string>
#include <utility>

#include <Eigen/Core>

#include "open_apero_core/agent/agent_interface.h"
#include "open_apero_core/plugin/plugin_registry.h"
#include "open_apero_core/plugin/robot_model/ompl_adapters.h"
#include "open_apero_core/plugin/robot_model/simple_robot.h"

namespace open_apero {
namespace robot_model {

/**
 * @brief SimpleRobot extended with a local sampling-based fallback planner.
 *
 * Inherits all geometry, kinematics, and collision logic from SimpleRobot. Its
 * planTrajectory() runs an OMPL planner over a bounded local state space when the
 * base class's straight-line steer is untraversable.
 */
class SimpleFallbackRobot : public SimpleRobot {
 public:
  explicit SimpleFallbackRobot(AgentInterface& agent);

  ~SimpleFallbackRobot() override = default;

  /**
   * @brief Loads SimpleRobot parameters plus OMPL planner configuration.
   *
   * With a planner configured and AgentInterface::getRandomSeed() >= 0, seeds OMPL's RNG before
   * building the planner. That RNG is process-global: the last robot set up decides the seed.
   *
   * @param param_map Pointer to the plugin parameter map.
   */
  void setupFromParamMap(Configurable::ParamMap* param_map) override;

  /**
   * @brief Plans a feasible trajectory from start to goal.
   *
   * Tries the straight-line steer first; if it is untraversable but both endpoints are traversable,
   * runs the OMPL planner in a local window around start/goal and validates its result.
   */
  bool planTrajectory(const RobotState& start, const RobotState& goal, Map* map, RobotTrajectory* trajectory) override;

 private:
  /** @brief Runs the OMPL planner. Called by planTrajectory() after the steer attempt fails. */
  bool planFallback(const RobotState& start, const RobotState& goal, Map* map, RobotTrajectory* trajectory);

  /** @brief Computes local OMPL bounds, clipped to the planning volume when it is active and unrotated. */
  std::pair<Eigen::Vector3d, Eigen::Vector3d> getLocalStateSpaceBounds(const Eigen::Vector3d& start, const Eigen::Vector3d& goal) const;

  /** @brief What the fallback planner optimizes. */
  enum class FallbackObjective {
    kTime,       ///< Minimize execution time of the steered path.
    kClearance,  ///< Maximize the robot's relative obstacle margin along the path.
  };

  ompl::base::StateSpacePtr ompl_state_space_;
  ompl::base::SpaceInformationPtr ompl_space_info_;
  ompl::base::PlannerPtr ompl_planner_;
  double p_ompl_planning_time_{0.01};
  double p_ompl_resolution_{0.01};
  double p_ompl_max_range_{2.0};
  std::string p_ompl_state_space_type_{"R3"};  ///< "R3" or "R3Yaw" once aliases are resolved.
  FallbackObjective ompl_objective_{FallbackObjective::kTime};

  static PluginRegistry::Registration<SimpleFallbackRobot> registration;
};

}  // namespace robot_model
}  // namespace open_apero
