#include "open_apero_core/plugin/layer/backtracking_layer.h"

#include <cmath>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include <Eigen/Geometry>

#include "open_apero_core/logging/logging.h"
#include "open_apero_core/logging/scoped_timer.h"
#include "open_apero_core/plugin/layer/backtracking_layer_metrics.h"
#include "open_apero_core/plugin/plugin_factory.h"
#include "open_apero_core/plugin/robot_model/robot_model.h"

namespace open_apero {
namespace layer {

PluginRegistry::Registration<BacktrackingLayer> BacktrackingLayer::registration("BacktrackingLayer");

BacktrackingLayer::BacktrackingLayer(AgentInterface& agent) : Layer(agent) {
  metrics_ = std::make_unique<BacktrackingLayerMetrics>();
}

// The backtracker is the only child and is required; edge costs come from the agent's cost computer.
void BacktrackingLayer::setupFromParamMap(Configurable::ParamMap* param_map) {
  const std::string layer_ns = (*param_map)["param_namespace"];

  setParam<int>(param_map, "priority", &p_priority_, defaultPriority());
  setParam<int>(param_map, "promoted_priority", &p_promoted_priority_, 1000);

  std::string args;
  setParam<std::string>(param_map, "backtracker_args", &args, layer_ns + "/backtracker");
  backtracker_ = agent_.getFactory().createPlugin<Backtracker>(args, agent_, verbose_plugins_);
  getMetrics().backtracker = backtracker_.get();
}

void BacktrackingLayer::resetEpisode() {
  Layer::resetEpisode();
  if (backtracker_) {
    backtracker_->resetEpisode();
  }
  current_node_.reset();
  recovery_node_.reset();
  recovery_edge_.reset();
}

void BacktrackingLayer::onStateReached(const RobotState& state, const Edge* executed_edge, Node* /*active_node*/) {
  if (!current_node_) {
    current_node_ = std::make_shared<Node>();
  }

  // Anchor at the planned endpoint to preserve exact history continuity.
  if (executed_edge && !executed_edge->trajectory.empty()) {
    const RobotState& anchor = executed_edge->trajectory.back();
    const double pos_delta = (anchor.getSensorPosition() - state.getSensorPosition()).norm();
    const double angle_delta = std::abs(Eigen::AngleAxisd(anchor.getSensorOrientation().inverse() * state.getSensorOrientation()).angle());
    std::ostringstream os;
    os << "BacktrackingLayer: anchoring recovery start on executed_edge.back() (settle residual pos=" << pos_delta
       << " m, angle=" << angle_delta << " rad vs live state).";
    logging::debug(os.str());
    current_node_->robot_state = anchor;
  } else {
    logging::debug("BacktrackingLayer: no executed edge; anchoring recovery start on live state.");
    current_node_->robot_state = state;
  }

  if (backtracker_) {
    backtracker_->onStateReached(state);
  }

  if (executed_edge && !executed_edge->is_backtrack && backtracker_) {
    backtracker_->recordExecutedEdge(*executed_edge);
  }
}

bool BacktrackingLayer::planPath(const RobotState& /*current_state*/, Map* /*map*/, RobotModel* /*robot*/, std::vector<Node*>* planned_path,
                                 std::vector<Edge*>* planned_edges) {
  planned_path->clear();
  planned_edges->clear();

  if (!backtracker_ || !current_node_) {
    return false;
  }
  ScopedTimer plan_path_timer(getLayerMetrics().plan_path_s, agent_.isLoggingPerformance());

  // Use the anchored state rather than the threshold-offset live pose.
  RobotTrajectory trajectory;
  if (!backtracker_->buildTrajectory(current_node_->robot_state, &trajectory)) {
    return false;
  }

  if (trajectory.size() < 2) {
    return false;
  }

  recovery_edge_ = std::make_shared<Edge>();
  recovery_edge_->trajectory = std::move(trajectory);
  recovery_edge_->is_backtrack = true;
  recovery_edge_->is_traversable = true;
  computeCost(recovery_edge_.get());

  recovery_node_ = std::make_shared<Node>();
  recovery_node_->robot_state = recovery_edge_->trajectory.back();
  recovery_node_->is_intermediate = true;
  recovery_node_->is_traversable = true;

  *planned_path = {current_node_.get(), recovery_node_.get()};
  planned_edges->push_back(recovery_edge_.get());
  return true;
}

bool BacktrackingLayer::splitActiveEdge(Node* /*active_node*/, Edge* edge, int waypoint_idx, Node** intermediate_out,
                                        Edge** finished_edge_out) {
  if (!edge || waypoint_idx < 0 || waypoint_idx >= static_cast<int>(edge->trajectory.size()) || !intermediate_out || !finished_edge_out) {
    return false;
  }

  // Drop the unreached tail so the edge and intermediate share an endpoint.
  edge->trajectory.erase(edge->trajectory.begin() + waypoint_idx + 1, edge->trajectory.end());
  edge->is_traversable = true;
  edge->is_considered = true;
  computeCost(edge);

  recovery_node_ = std::make_shared<Node>();
  recovery_node_->robot_state = edge->trajectory.back();
  recovery_node_->is_intermediate = true;
  recovery_node_->is_traversable = true;

  *intermediate_out = recovery_node_.get();
  *finished_edge_out = edge;
  return true;
}

void BacktrackingLayer::getVisualization(Node* /*selected_node*/, VisualizationMarkers* markers) {
  if (backtracker_) {
    backtracker_->getVisualization(markers);
  }
}

int BacktrackingLayer::getPriority(const RobotState& state) const {
  // Prioritize an initial sweep or escape from an untraversable state.
  const bool init_done = !backtracker_ || backtracker_->isInitialExplorationDone();
  const bool traversable = agent_.getRobot().isStateTraversable(state, &agent_.getMap());
  return (!init_done || !traversable) ? p_promoted_priority_ : p_priority_;
}

}  // namespace layer
}  // namespace open_apero
