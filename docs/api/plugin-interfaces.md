# Plugin Interfaces

The base class of every plugin role but `Map` and `Layer` ([Map](maps.md), [Layers](layers.md)), and the abstract
bases and capabilities a new plugin can build on. A new plugin derives from one of these, or from a built-in
implementation, and registers itself; see [Extending Open Apero](../extending.md). The hooks no shipped plugin
overrides are listed in [Extension Hooks](../modules/extension-hooks.md).

## Robot Model

```{doxygenclass} open_apero::RobotModel
:members:
:protected-members:
:show: header-file
```

```{doxygenenum} open_apero::StationingCheck
```

## Cost Computer

```{doxygenclass} open_apero::CostComputer
:members:
:show: header-file
```

## Generators

```{doxygenclass} open_apero::Generator
:members:
:protected-members:
:show: header-file
```

```{doxygenclass} open_apero::GraphGenerator
:members:
:protected-members:
:show: header-file
```

```{doxygenclass} open_apero::TreeGenerator
:members:
:protected-members:
:show: header-file
```

## Evaluators

```{doxygenclass} open_apero::Evaluator
:members:
:protected-members:
:show: header-file
```

```{doxygenclass} open_apero::evaluator::SimulatedSensorEvaluator
:members:
:protected-members:
:show: header-file
```

```{doxygenclass} open_apero::evaluator::YawPlanningEvaluator
:members:
:protected-members:
:show: header-file
```

## Sensor Model

```{doxygenclass} open_apero::SensorModel
:members:
:protected-members:
:show: header-file
```

```{doxygenclass} open_apero::sensor_model::CameraModel
:members:
:protected-members:
:show: header-file
```

```{doxygenclass} open_apero::sensor_model::LidarModel
:members:
:protected-members:
:show: header-file
```

## Path Planner

```{doxygenclass} open_apero::PathPlanner
:members:
:protected-members:
:show: header-file
```

```{doxygenenum} open_apero::Objective
```

```{doxygenfunction} open_apero::toString(Objective type)
```

```{doxygenfunction} open_apero::objectiveFromString
```

## Backtracker

```{doxygenclass} open_apero::Backtracker
:members:
:show: header-file
```

```{doxygenstruct} open_apero::ProbeSample
:members:
:show: header-file
```
