# Layers

How the agent chooses between planning behaviors, what the shipped layers do and how a layer type joins the stack.
Keys: [`PlanningLayer`](../parameters.md#planninglayer), [`BacktrackingLayer`](../parameters.md#backtrackinglayer);
classes: [Layers API](../api/layers.md).

## Priorities and Arbitration

Each cycle the agent reads every layer's `getPriority(state)` and asks the layers for a step in descending order;
the first whose `planPath()` returns `true` wins. Priorities must be distinct: a layer lists every value it can
return in `possiblePriorities()`, and the agent throws `std::runtime_error` when two layers share one.

| Layer | Default priority | Offers |
|-------|------------------|--------|
| `PlanningLayer` | `0` | The path planner's best path over its graph |
| `BacktrackingLayer` | `-1000`; `promoted_priority` (`1000`) while its backtracker's initial sweep runs or the robot's state is untraversable | A recovery trajectory from its `Backtracker`, unless the backtracker has none (`Reverse` with no edge remembered) |
| a registered layer | `0` unless the class overrides `defaultPriority()`, so set `priority:` | Whatever it implements |

## `PlanningLayer`

The graph planner: it composes a generator, which must be a `GraphGenerator`, an evaluator (`evaluator_args`,
shipped `/task_evaluator`, which `rrt` and `rrtstar` wrap in a `SimpleYawPlanningEvaluator`) and a path planner, and
keeps its own graph. In `onStateReached()` it moves the start node to the reached node and hands that node to
`Evaluator::onNodeReached()`. Each cycle it re-checks the nodes and edges the map update reaches
([Generators](generators.md)),
re-evaluates the nodes whose view the map updated ([re-evaluation](evaluators.md#re-evaluation-on-map-updates)) and
runs `max_expansions` (shipped `50`) rounds that each add, evaluate and price one node cluster. The robot model may
refine the planned first edge (`RobotModel::refineBeforeExecute()`), which must stay traversable, and the generator
cuts it after `replan_num_steps` (shipped `5`) time steps; without a traversable path the layer declines.

## `BacktrackingLayer`

The recovery layer: one [`Backtracker`](robot-models.md#backtrackers) and no graph. It passes every executed edge
that a backtracker did not produce to `Backtracker::recordExecutedEdge()`. When asked, it wraps the backtracker's
trajectory in one edge flagged `is_backtrack`, priced by the agent's cost computer, so recovery counts against the
same budget. With the shipped `RotateReverse` backtracker, every episode starts with one full turn on the spot.

## How a Layer Type Joins the Stack

Each key of the planner config's `layers` dictionary is one layer, built from its block's `type:`, so any registered
layer type joins from YAML alone ([Extending Open Apero](../extending.md#adding-a-layer)). The key becomes the
layer's `instanceName()`, which prefixes its [`performance_log.csv` columns](run-output.md#performance_logcsv) and
names its topics `layer_visualization/<name>` and `path_visualization/<name>`. The shipped stack:

```yaml
layers:
  planning:
    type: "PlanningLayer"  # with generator, evaluator, path_planner
  backtracking:
    type: "BacktrackingLayer"
    priority: -1000
    promoted_priority: 1000
    backtracker:
      type: "RotateReverse"
```
