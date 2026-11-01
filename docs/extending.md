# Extending Open Apero

How to add a plugin, such as a map, robot model, evaluator, path planner or layer: one worked example from the class
to the build and the test, then notes per role. The [module pages](modules/index.md) describe the shipped types, the
[Parameter Reference](parameters.md) their keys and the [API reference](api/plugin-interfaces.md) the base classes.

## Adding a Plugin

Derive from the role's base class ([Roles](#roles)) with a public constructor taking `open_apero::AgentInterface&`
that does no parameter-dependent work. In `setupFromParamMap()`, call the base class's where it defines one, then read
every key with `setParam()`; reject bad values in `checkParamsValid()`. Register the class under a `type:` name no
other library uses, and develop with the agent's `strict_params: true`, so a misspelt key stops the start. This
evaluator extends `evaluator::SimulatedSensorEvaluator` and scores only the unknown voxels within `max_range` of the
sensor:

```cpp
// src/near_unknown_evaluator.cpp
#include <memory>

#include <open_apero_core/plugin/evaluator/simulated_sensor_evaluator.h>
#include <open_apero_core/plugin/plugin_registry.h>

namespace my_project {

/// Gain: the unknown voxels of a node's simulated view within max_range of the sensor.
class NearUnknownEvaluator : public open_apero::evaluator::SimulatedSensorEvaluator {
 public:
  explicit NearUnknownEvaluator(open_apero::AgentInterface& agent) : SimulatedSensorEvaluator(agent) {}

  void setupFromParamMap(ParamMap* param_map) override {
    // The base reads bounding_volume, sensor_model and the frontier thresholds.
    SimulatedSensorEvaluator::setupFromParamMap(param_map);
    setParam<double>(param_map, "max_range", &p_max_range_, 2.0);
  }

  // What the episode has actually gained, logged as RealAccumulatedGain.
  double getRealAccumulatedGain() const override { return map_->getExploredVolume(); }

 protected:
  bool evaluateFromVisibleVoxels(open_apero::Node* node) override {
    node->gain_anchors.clear();
    auto* info = node->info ? node->info->as<open_apero::SimulatedSensorInfo>() : nullptr;
    if (info == nullptr) {
      return false;
    }
    const Eigen::Vector3d& sensor = node->robot_state.getSensorPosition();
    double gain = 0.0;
    for (auto& [center, voxel] : info->visible_voxels) {
      if (voxel.occupancy == open_apero::Occupancy::kUnknown && (center - sensor).norm() <= p_max_range_) {
        voxel.gain = 1.0;
        gain += 1.0;
      }
    }
    node->gain_anchors = {std::make_shared<open_apero::GainAnchor>(gain)};
    return true;
  }

 private:
  double p_max_range_ = 2.0;

  static open_apero::PluginRegistry::Registration<NearUnknownEvaluator> registration;
};

open_apero::PluginRegistry::Registration<NearUnknownEvaluator>
    NearUnknownEvaluator::registration("NearUnknownEvaluator");

}  // namespace my_project
```

Inside this repository, a core type goes in `plugin/<role>/` of `open_apero_core` (a ROS-dependent one in
`open_apero_ros`) and needs an entry of `recipes()` in `open_apero_core/test/test_plugin_conformance.cpp`, such as
`{"NaiveEvaluator", {Role::kEvaluator, {{"/sensor_model", caster}}}}`; [Contributing](contributing.md) lists every step.
The shipped planners read their evaluator from `/task_evaluator`, which `run_experiment_sim.launch` fills from
`open_apero_ros/config/tasks/_common.yaml` and `<task>.yaml`: add `config/tasks/near_unknown.yaml` holding
`type: "NearUnknownEvaluator"` and `max_range: 2.0`, list it in `TASK_EVALUATORS` of
`open_apero_ros/test/test_launch_task_args.py`, and run with `task:=near_unknown`. Only a
`SimulatedSensorEvaluator` subclass belongs there, as no other evaluator reads every key of `_common.yaml`.

### In Your Own Package

Add `open_apero_core`, `open_apero_ros` and `open_apero_voxblox` as `<depend>` entries of `package.xml`, and build the
plugins into a shared library:

```cmake
cmake_minimum_required(VERSION 3.10)
project(my_project)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(DEPS open_apero_core open_apero_ros open_apero_voxblox)
find_package(catkin REQUIRED COMPONENTS ${DEPS})
catkin_package(LIBRARIES my_plugins CATKIN_DEPENDS ${DEPS})
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(my_plugins SHARED src/near_unknown_evaluator.cpp src/return_home_layer.cpp)
target_link_libraries(my_plugins ${catkin_LIBRARIES})
```

`open_apero_node` links `open_apero_core` and `open_apero_ros` and loads the other plugin libraries named in its
`plugin_libraries` parameter (default `[libopen_apero_voxblox.so]`), searching a bare file name on the
`LD_LIBRARY_PATH` that `devel/setup.bash` sets; a node of your own calls `open_apero::loadPluginLibrary()`. A library
that cannot be loaded, or that registers a taken type name, stops the node. Select your types in a copy of
`open_apero_ros/config/planners/nbs.yaml` without its `evaluator_args` line, which would silently win over the
inline block, even with `strict_params`:

```yaml
# my_project/config/my_planner.yaml
plugin_libraries: [libopen_apero_voxblox.so, libmy_plugins.so]
layers:
  planning:
    # ... nbs.yaml's planning layer, without its evaluator_args line
    evaluator:
      type: "NearUnknownEvaluator"
      max_range: 2.0
      bounding_volume_args: "/target_bounding_volume"
      visualize_sensor_view: true
      frontier_min_unknown_ratio: 0.8  # frontier nodes, without which nbs's expected_gain is plain gain
      frontier_min_visible_ratio: 0.6
      sensor_model:  # the Habitat camera, as in config/tasks/_common.yaml; the launch arguments do not reach it
        type: "IterativeRayCaster"
        ray_length: 3.0
        observation_boundary: "planar"
        focal_length: 625.58
        resolution_x: 480
        resolution_y: 640
        downsampling_factor: 3.0
```

```bash
roslaunch open_apero_ros run_experiment_sim.launch planner:=my_planner \
  planner_config:=$(rospack find my_project)/config/my_planner.yaml
```

### Testing Without ROS

`PluginFactoryConfig` serves parameters from memory as strings (a `bool` is `"1"` or `"0"`), so a gtest builds plugins
without ROS. `open_apero_core/test/plugin_test_stubs.h` has a `ConfigAgent` and stub maps; it is not installed, so copy
it into your package's `test/`. `open_apero_ros/test/test_points_evaluator.cpp` has a `StubAgent` for an
`open_apero_ros` type.

```cpp
// test/test_near_unknown_evaluator.cpp
#include <gtest/gtest.h>
#include <open_apero_core/graph/graph_node.h>
#include <open_apero_core/plugin/evaluator/evaluator.h>
#include <open_apero_core/plugin/plugin_library.h>

#include "plugin_test_stubs.h"

namespace open_apero {

TEST(NearUnknownEvaluatorTest, BuildsInStrictModeAndEvaluates) {
  loadPluginLibrary("libmy_plugins.so");  // registers its types; the test does not link the library
  ConfigAgent agent;
  FreeOccupancyMap map(agent);  // SimulatedSensorEvaluator needs a map::OccupancyMap
  agent.provide(&map, nullptr);
  agent.factory().setStrictParams(true);  // an unread key throws, as with strict_params
  agent.factory().setParams("/evaluator", {{"type", "NearUnknownEvaluator"}, {"max_range", "2.0"}});
  agent.factory().setParams("/evaluator/sensor_model", {{"type", "IterativeRayCaster"}});

  auto evaluator = agent.getFactory().createPlugin<Evaluator>("/evaluator", agent, false);
  GraphNode node;
  ASSERT_TRUE(evaluator->evaluate({&node}));
  EXPECT_EQ(node.getSingleNodeGain(), 0.0);  // a free world holds nothing unknown
}

}  // namespace open_apero
```

```cmake
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_my_plugins test/test_near_unknown_evaluator.cpp)
  target_link_libraries(test_my_plugins ${catkin_LIBRARIES} gtest_main)
  add_dependencies(test_my_plugins my_plugins)
endif()
```

## Roles

| Role | Built by, at | Base class | Implement |
|------|--------------|------------|-----------|
| `map` | `Agent`, `map` | `Map`; `map::OccupancyMap`, `map::TsdfMap` | `setupFromParamMap`; for an occupancy map `getVoxelOccupancy`, `getVoxelSize`, `getVoxelCenter`; for a TSDF map also `getDistanceAtPosition`, `getVoxelWeight`, `getMaximumWeight` ([Maps](modules/maps.md#writing-a-map-backend)) |
| `robot` | `Agent`, `robot` | `RobotModel` | `planTrajectory`, `steer`, `placeRobotAtSensorPose`, `getStateTraversabilityRaw`, `isTraversabilityOrientationRelated`, and `samplePosition`, which every generator needs; optionally `getBodyBounds`, without which the generators re-check every node and edge each cycle |
| `cost_computer` | `Agent`, `cost_computer` | `CostComputer` | `setupFromParamMap`, `computeCost`; optionally `getCostLowerBound`, a cost no planned edge undercuts (default `0`) |
| a layer | `Agent`, `layers.<name>` | `Layer` | `layerName`, `onStateReached`, `expand`, `planPath`; `setupFromParamMap` unless it composes a generator, evaluator and path planner ([Adding a Layer](#adding-a-layer)) |
| `generator` | `PlanningLayer`, `layers.planning.generator` | `GraphGenerator`, which `PlanningLayer` requires; `TreeGenerator` for a tree | `generateNewNode`, `setNewStartNode`; from `GraphGenerator`, also `rebuildKdTree` and its other pure virtual methods |
| `evaluator` | `PlanningLayer`, `layers.planning.evaluator` | `Evaluator` | `evaluateUnobserved` (fill `Node::gain_anchors`), `getRealAccumulatedGain` |
| `evaluator` | as above | `evaluator::SimulatedSensorEvaluator` | `evaluateFromVisibleVoxels`, `getRealAccumulatedGain` |
| `path_planner` | `PlanningLayer`, `layers.planning.path_planner` | `PathPlanner` | `planPath` |
| `backtracker` | `BacktrackingLayer`, `layers.backtracking.backtracker` | `Backtracker` | `setupFromParamMap`, `buildTrajectory` |
| `sensor_model` | `SimulatedSensorEvaluator`, `<evaluator>.sensor_model` | `SensorModel` | `getVisibleVoxels`, `getSensorViewBoundary`, `visualizeSensorView` |

Defaults worth overriding: `Evaluator::reevaluateUnobserved()` never re-evaluates and `Evaluator::onNodeReached()`
calls `Node::markObserved()`. [Extension Hooks](modules/extension-hooks.md) lists the hooks no shipped plugin uses.

## From `type:` to Object

`PluginFactory::createPlugin()` reads `type:` and the keys of a namespace `ns`, constructs the class registered under
that name (a static `Registration` adds it to the `PluginRegistry` when its library loads), runs
`setupFromParamMap()` and `checkParamsValid()`, and reports the keys no setup read. A missing or unregistered
`type:`, a type of another role, a missing or unparseable key and, with `strict_params`, an unread key throw
`std::runtime_error`; a `false` from `checkParamsValid()` throws `std::invalid_argument`. Setup and validation errors
get the prefix "`<type>` at '`<namespace>`': ", so the message names every plugin on its way down. A plugin builds
its children the same way, with `agent_.getFactory().createPlugin<SensorModel>(args, agent_, verbose_plugins_)`,
`args` being the child's `<role>_args` key, by default `<ns>/sensor_model`.

`Plugin` adds the owning agent (`agent_`), `resetEpisode()` and registry construction to `Configurable`, the parameter
parsing ([How Parameters Are Read](parameters.md#how-parameters-are-read)), which records a key read without
`setParam()` through `markParamConsumed()`. A plain `Configurable` such as `BoundingVolume` is built by its owner and
set up with `PluginFactory::configure(object, ns, verbose)`.

## Lifecycle, Ownership and Threads

- Setup: the agent builds `map`, `robot` and `cost_computer`, then the layers in alphabetical key order; each owner
  builds its children in its own `setupFromParamMap()`, and a layer's children see `getLayer() == nullptr` there.
  With `build_plugins_on_init: true` (set in `config/planners/_common.yaml`) every layer then runs `onEpisodeStart()`,
  `onStateReached()`, `update()` and `warmUp()` once at a default state.
- Episode start: `resetEpisode()` on the map, robot, cost computer and layers (a layer resets its children), then
  `Layer::onEpisodeStart()`. Plugins are reused: clear their state in `resetEpisode()` and call the base class's.
- Every cycle: `Layer::onStateReached()`, `RobotModel::update()`, `Layer::update()`, `Layer::expand()`, then the
  layers' `getPriority()` and `planPath()` ([One Planning Cycle](concepts.md#one-planning-cycle)).

Each owner keeps its plugins in a `std::unique_ptr` until the agent is destroyed. Every call comes from the planning
loop's thread, where `RosAgent` also runs ROS callbacks between cycles. An evaluator may cast from threads of its own,
so a sensor model must cast without shared mutable state.

## Adding a Layer

Every cycle the agent asks the layers for a step in descending `getPriority()` order; the first whose `planPath()`
returns `true` wins ([Layers](modules/layers.md)). A step is `planned_path[0]` (the current state), `planned_path[1]`
(the destination) and `planned_edges[0]`, an edge with a cost from `computeCost()` whose trajectory has at least two
states and starts at time 0. The layer owns them until the step has run; a destination with `is_terminating` set ends
the episode once its edge has run.

A `ReturnHomeLayer` drives the robot back to its start pose. Its `getPriority()` returns `promoted_priority` once
`agent_.getCurrentCostBudget()` is at most `reserve_budget`, else `priority`; `possiblePriorities()` returns both;
`layerName()` returns `"return_home"`; `onEpisodeStart()` stores the start state in `home_`; `onStateReached()` and
`expand()` are empty; and `setupFromParamMap()` reads the three keys without calling `Layer::setupFromParamMap()`. Its
`planPath()`:

```cpp
bool planPath(const open_apero::RobotState& current_state, open_apero::Map* map, open_apero::RobotModel* robot,
              std::vector<open_apero::Node*>* planned_path, std::vector<open_apero::Edge*>* planned_edges) override {
  planned_path->clear();
  planned_edges->clear();
  if (getPriority(current_state) != p_promoted_priority_ ||
      open_apero::isSamePosition(current_state.getSensorPosition(), home_.getSensorPosition())) {
    return false;
  }
  auto edge = std::make_unique<open_apero::Edge>();
  if (!robot->planTrajectory(current_state, home_, map, &edge->trajectory)) {
    return false;
  }
  edge->is_traversable = true;
  computeCost(edge.get());  // the agent charges edge->cost to the budget

  // Members, since the agent keeps pointers to the step until it is executed.
  edge_ = std::move(edge);  // std::unique_ptr<open_apero::Edge>
  start_.robot_state = current_state;
  goal_.robot_state = edge_->trajectory.back();
  goal_.is_terminating = true;
  *planned_path = {&start_, &goal_};
  planned_edges->push_back(edge_.get());
  return true;
}
```

Its `layers` key becomes `instanceName()`, which names its CSV columns, visualization topics and log lines. No value
in one layer's `possiblePriorities()` may appear in another's; the agent throws at startup otherwise.

```yaml
layers:
  # planning (priority 0) and backtracking (-1000, promoted 1000) as shipped
  home:
    type: "ReturnHomeLayer"
    priority: -500
    promoted_priority: 2000
    reserve_budget: 30.0
```
