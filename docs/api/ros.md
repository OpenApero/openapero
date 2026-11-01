# ROS Integration

The ROS 1 agent, its plugin factory and the message conversions (`open_apero_ros`,
`open_apero_ros_bridge`); [ROS Node, Launch Files and Messages](../modules/ros.md) explains them. The ROS-free
`BenchmarkAgent` is on [Agent](agent.md).

## ROS Agent and Conversions

```{doxygennamespace} open_apero::ros
:members:
```

```{doxygenfunction} open_apero::installRosconsoleSink
```

```{doxygenfunction} open_apero::setLogLevelFromParam
```

## ROS Plugins

```{doxygenclass} open_apero::evaluator::PointsEvaluator
:members:
:show: header-file
```
