#include "open_apero_core/plugin/cost_computer/trajectory_time.h"

#include <algorithm>
#include <cmath>

#include "open_apero_core/common/utils.h"
#include "open_apero_core/plugin/robot_model/robot_model.h"

namespace open_apero {
namespace cost_computer {

PluginRegistry::Registration<TrajectoryTime> TrajectoryTime::registration("TrajectoryTime");

TrajectoryTime::TrajectoryTime(AgentInterface& agent) : CostComputer(agent) {}

void TrajectoryTime::setupFromParamMap(Configurable::ParamMap* /*param_map*/) {}

// Compute edge cost as the absolute time duration of the trajectory (end time - start time).
bool TrajectoryTime::computeCost(Edge* edge) {
  if (edge->trajectory.size() < 2) {
    edge->cost = 0.0;
    return false;
  }

  double cost = edge->trajectory.back().getTimeFromStart() - edge->trajectory.front().getTimeFromStart();
  edge->cost = (cost > 0 ? cost : -cost);  // In case a reverse direction
  return true;
}

double TrajectoryTime::getCostLowerBound(const RobotState& start, const RobotState& goal) {
  const RobotModel& robot = agent_.getRobot();
  const double distance = (goal.getSensorPosition() - start.getSensorPosition()).norm();
  const double yaw = std::abs(angleDifference(start.getSensorYaw(), goal.getSensorYaw()));
  return std::max(distance / robot.getMaxLinearVelocity(), yaw / robot.getMaxYawRate());
}

}  // namespace cost_computer
}  // namespace open_apero
