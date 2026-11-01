#pragma once

/**
 * @file ompl_adapters.h
 * @brief OMPL state validation and optimization utilities.
 */

#include <string>
#include <utility>

#include <Eigen/Core>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include "open_apero_core/plugin/map/map.h"
#include "open_apero_core/plugin/robot_model/robot_model.h"
#include "open_apero_core/state/robot_state.h"

namespace open_apero {
namespace robot_model {

/**
 * @brief Map an `ompl_state_space_type` value to the name the OMPL helpers take.
 *
 * `R3` is the 3D position and `R3Yaw` the 3D position plus yaw; `R2` and `SE2` are accepted as
 * aliases of them. Any other value is returned unchanged.
 *
 * @param state_space_type Configured state space type.
 * @return "R3", "R3Yaw" or @p state_space_type.
 */
std::string resolveStateSpaceTypeAlias(const std::string& state_space_type);

/**
 * @brief Extract position and yaw from an OMPL state.
 */
void getPoseFromOmplState(const ompl::base::State* state, Eigen::Vector3d& pos, double& yaw, const std::string& state_space_type);

/**
 * @brief Set the bounds of an OMPL state space.
 */
void setStateSpaceBounds(const ompl::base::StateSpacePtr& space, const Eigen::Vector3d& lower, const Eigen::Vector3d& upper,
                         const std::string& state_space_type);

/**
 * @brief Configure start and goal states for an OMPL problem definition.
 */
void setStartAndGoalStates(const ompl::base::ProblemDefinitionPtr& pdef, const ompl::base::StateSpacePtr& space,
                           const RobotState& start_state, const RobotState& goal_state, const std::string& state_space_type);

/**
 * @brief OMPL state validity checker that delegates to the robot's traversability checks.
 *
 * Validity is RobotModel::isStateTraversable and clearance is RobotModel::getStateClearance,
 * both against robot->getMap(), so the checker never needs rebuilding.
 */
class OpenAperoStateValidityChecker : public ompl::base::StateValidityChecker {
 public:
  OpenAperoStateValidityChecker(const ompl::base::SpaceInformationPtr& si, const std::string& state_space_type, RobotModel* robot)
      : ompl::base::StateValidityChecker(si), robot_(robot), state_space_type_(state_space_type) {}

  bool isValid(const ompl::base::State* state) const override;
  double clearance(const ompl::base::State* state) const override;

 private:
  RobotModel* robot_;
  std::string state_space_type_;
};

/**
 * @brief OMPL motion validator that checks trajectory segments for traversability.
 */
class OpenAperoMotionValidator : public ompl::base::MotionValidator {
 public:
  OpenAperoMotionValidator(const ompl::base::SpaceInformationPtr& si, const std::string& state_space_type, RobotModel* robot)
      : ompl::base::MotionValidator(si), robot_(robot), state_space_type_(state_space_type) {}

  bool checkMotion(const ompl::base::State* s1, const ompl::base::State* s2) const override;
  bool checkMotion(const ompl::base::State* s1, const ompl::base::State* s2,
                   std::pair<ompl::base::State*, double>& lastValid) const override;

 private:
  RobotModel* robot_;
  std::string state_space_type_;
};

/**
 * @brief OMPL optimization objective that maximizes clearance from obstacles.
 *
 * Integrates 1/clearance along the path; finite because clearance is floored at the robot's
 * clearance_margin_eps.
 */
class ClearanceObjective : public ompl::base::StateCostIntegralObjective {
 public:
  ClearanceObjective(const ompl::base::SpaceInformationPtr& si) : ompl::base::StateCostIntegralObjective(si, true) {}

  ompl::base::Cost stateCost(const ompl::base::State* s) const {
    return ompl::base::Cost(1.0 / si_->getStateValidityChecker()->clearance(s));
  }
};

/**
 * @brief OMPL optimization objective that minimizes trajectory execution time.
 */
class TimeObjective : public ompl::base::OptimizationObjective {
 public:
  TimeObjective(const ompl::base::SpaceInformationPtr& si, const std::string& state_space_type, RobotModel* robot)
      : ompl::base::OptimizationObjective(si), robot_(robot), state_space_type_(state_space_type) {
    description_ = "Time";
  }

  ompl::base::Cost stateCost(const ompl::base::State* /*s*/) const override { return ompl::base::Cost(1.0); }
  ompl::base::Cost motionCost(const ompl::base::State* s1, const ompl::base::State* s2) const override;

 private:
  RobotModel* robot_;
  std::string state_space_type_;
};

}  // namespace robot_model
}  // namespace open_apero
