#include "open_apero_core/plugin/evaluator/evaluator.h"

#include <algorithm>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include "open_apero_core/agent/agent_interface.h"
#include "open_apero_core/geometry/bounding_volume.h"
#include "open_apero_core/plugin/plugin_factory.h"

namespace open_apero {

namespace {

NodeClusterPtrs unobservedNodes(const NodeClusterPtrs& cluster) {
  NodeClusterPtrs unobserved;
  unobserved.reserve(cluster.size());
  std::copy_if(cluster.begin(), cluster.end(), std::back_inserter(unobserved), [](const Node* node) { return !node->is_observed; });
  return unobserved;
}

}  // namespace

Evaluator::Evaluator(AgentInterface& agent) : Plugin(agent) {}

bool Evaluator::evaluate(const NodeClusterPtrs& cluster) {
  const NodeClusterPtrs unobserved = unobservedNodes(cluster);
  return unobserved.empty() || evaluateUnobserved(unobserved);
}

bool Evaluator::updateEvaluation(const NodeClusterPtrs& cluster, const Node* current_node) {
  const NodeClusterPtrs unobserved = unobservedNodes(cluster);
  return unobserved.empty() || reevaluateUnobserved(unobserved, current_node);
}

void Evaluator::setupFromParamMap(Configurable::ParamMap* param_map) {
  // The bounding volume defaults to the evaluator's own bounding_volume namespace.
  std::string ns = (*param_map)["param_namespace"];

  std::string bounding_volume_args;
  setParam<std::string>(param_map, "bounding_volume_args", &bounding_volume_args, ns + "/bounding_volume");
  bounding_volume_ = std::make_unique<BoundingVolume>();
  bounding_volume_->setInfoLogger([this](const std::string& msg) { agent_.printInfo(msg); });
  agent_.getFactory().configure(*bounding_volume_, bounding_volume_args, verbose_plugins_);
}

bool Evaluator::reevaluateUnobserved(const NodeClusterPtrs& /*cluster*/, const Node* /*current_node*/) {
  return false;
}

void Evaluator::getVisualization(VisualizationMarkers* /*markers*/, const Node* /*node*/) {}

void Evaluator::getPathVisualization(VisualizationMarkers* /*markers*/, const std::vector<Node*>& /*path*/) {}

void Evaluator::onNodeReached(Node* reached_node, const Edge* /*traversed_edge*/) {
  reached_node->markObserved();
}

}  // namespace open_apero
