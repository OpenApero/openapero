#include "open_apero_core/plugin/layer/layer.h"

#include <string>

#include "open_apero_core/plugin/evaluator/evaluator.h"
#include "open_apero_core/plugin/generator/generator.h"
#include "open_apero_core/plugin/path_planner/path_planner.h"
#include "open_apero_core/plugin/plugin_factory.h"

namespace open_apero {

// Every role is required: createPlugin either returns a configured plugin or
// throws naming the namespace, so no null checks are needed here.
void Layer::setupFromParamMap(Configurable::ParamMap* param_map) {
  const std::string layer_ns = (*param_map)["param_namespace"];

  setParam<int>(param_map, "priority", &p_priority_, defaultPriority());

  std::string args;
  setParam<std::string>(param_map, "generator_args", &args, layer_ns + "/generator");
  generator_ = agent_.getFactory().createPlugin<Generator>(args, agent_, verbose_plugins_);
  generator_->setLayer(this);

  setParam<std::string>(param_map, "evaluator_args", &args, layer_ns + "/evaluator");
  evaluator_ = agent_.getFactory().createPlugin<Evaluator>(args, agent_, verbose_plugins_);
  evaluator_->setLayer(this);

  setParam<std::string>(param_map, "path_planner_args", &args, layer_ns + "/path_planner");
  path_planner_ = agent_.getFactory().createPlugin<PathPlanner>(args, agent_, verbose_plugins_);
  path_planner_->setLayer(this);
}

}  // namespace open_apero
