# Built-in Plugins

The registered implementations shipped with `open_apero_core`, by role, and `GraphBeamSearch`, the abstract base of the
two beam searches; the abstract bases a new plugin derives from are on [Plugin Interfaces](plugin-interfaces.md). Their
YAML keys are listed in the [parameter reference](../parameters.md), and the [module pages](../modules/index.md) explain
each.

## Generators

```{doxygenclass} open_apero::generator::RRAG
:members:
:show: header-file
```

```{doxygenclass} open_apero::generator::RRT
:members:
:show: header-file
```

```{doxygenclass} open_apero::generator::RRTStar
:members:
:show: header-file
```

## Evaluators

```{doxygenclass} open_apero::evaluator::NaiveEvaluator
:members:
:show: header-file
```

```{doxygenclass} open_apero::evaluator::SurfaceFrontierEvaluator
:members:
:show: header-file
```

```{doxygenclass} open_apero::evaluator::VoxelWeightEvaluator
:members:
:show: header-file
```

```{doxygenclass} open_apero::evaluator::SimpleYawPlanningEvaluator
:members:
:show: header-file
```

## Path Planners

```{doxygenclass} open_apero::path_planner::GraphBeamSearch
:members:
:protected-members:
:show: header-file
```

```{doxygenclass} open_apero::path_planner::GraphNodeWiseBeamSearch
:members:
:show: header-file
```

```{doxygenclass} open_apero::path_planner::GraphDepthWiseBeamSearch
:members:
:show: header-file
```

```{doxygenclass} open_apero::path_planner::GraphShortestPathTreeSearch
:members:
:show: header-file
```

```{doxygenclass} open_apero::path_planner::GraphTspSearch
:members:
:show: header-file
```

```{doxygenclass} open_apero::path_planner::TreeBestValue
:members:
:show: header-file
```

```{doxygenclass} open_apero::path_planner::NoOpPathPlanner
:members:
:show: header-file
```

## Cost Computers

```{doxygenclass} open_apero::cost_computer::IdentityCost
:members:
:show: header-file
```

```{doxygenclass} open_apero::cost_computer::TrajectoryLength
:members:
:show: header-file
```

```{doxygenclass} open_apero::cost_computer::TrajectoryTime
:members:
:show: header-file
```

## Backtrackers

```{doxygenclass} open_apero::backtracker::Reverse
:members:
:show: header-file
```

```{doxygenclass} open_apero::backtracker::RotateInPlace
:members:
:show: header-file
```

```{doxygenclass} open_apero::backtracker::RotateReverse
:members:
:show: header-file
```

```{doxygenclass} open_apero::backtracker::ExecutedTrajectoryQueue
:members:
:show: header-file
```

## Robot Models

```{doxygenclass} open_apero::robot_model::SimpleRobot
:members:
:show: header-file
```

```{doxygenclass} open_apero::robot_model::SimpleFallbackRobot
:members:
:show: header-file
```

## Sensor Models

```{doxygenclass} open_apero::sensor_model::IterativeRayCaster
:members:
:show: header-file
```

```{doxygenclass} open_apero::sensor_model::IterativeRayCasterLidar
:members:
:show: header-file
```

```{doxygenclass} open_apero::sensor_model::SimpleRayCaster
:members:
:show: header-file
```
