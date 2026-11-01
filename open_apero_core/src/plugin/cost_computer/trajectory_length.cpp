#include "open_apero_core/plugin/cost_computer/trajectory_length.h"

#include "open_apero_core/state/robot_state.h"

namespace open_apero {
namespace cost_computer {
PluginRegistry::Registration<TrajectoryLength> TrajectoryLength::registration("TrajectoryLength");

TrajectoryLength::TrajectoryLength(AgentInterface& agent) : CostComputer(agent) {}

void TrajectoryLength::setupFromParamMap(Configurable::ParamMap* /*param_map*/) {}

bool TrajectoryLength::computeCost(Edge* edge) {
  if (edge->trajectory.size() < 2) {
    edge->cost = 0.0;
    return false;
  }

  edge->cost = getTrajectoryLength(edge->trajectory);
  return true;
}

double TrajectoryLength::getCostLowerBound(const RobotState& start, const RobotState& goal) {
  return (goal.getSensorPosition() - start.getSensorPosition()).norm();
}

}  // namespace cost_computer
}  // namespace open_apero
