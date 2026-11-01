# Data Types

The values that flow between the agent, its layers and its plugins: states, nodes, edges, paths and their gain
bookkeeping, and the episode's history.

## Robot State

```{doxygenclass} open_apero::RobotState
:members:
:show: header-file
```

```{doxygentypedef} open_apero::RobotTrajectory
```

```{doxygenstruct} open_apero::Transform3d
:members:
:show: header-file
```

## Nodes

```{doxygenstruct} open_apero::Node
:members:
:show: header-file
```

```{doxygenstruct} open_apero::NodeInfo
:members:
:show: header-file
```

```{doxygenstruct} open_apero::SimulatedSensorInfo
:members:
:show: header-file
```

```{doxygenstruct} open_apero::VoxelInfo
:members:
:show: header-file
```

```{doxygenstruct} open_apero::YawPlanningInfo
:members:
:show: header-file
```

```{doxygenenum} open_apero::SamplingRole
```

```{doxygentypedef} open_apero::NodeClusterPtrs
```

```{doxygentypedef} open_apero::VoxelOccupancyMap
```

```{doxygenstruct} open_apero::GraphNode
:members:
:show: header-file
```

```{doxygenstruct} open_apero::TreeNode
:members:
:show: header-file
```

## Edges and Graphs

```{doxygenstruct} open_apero::Edge
:members:
:show: header-file
```

```{doxygentypedef} open_apero::GraphType
```

```{doxygenfunction} open_apero::getEdge
```

```{doxygenfunction} open_apero::getNodeId
```

```{doxygenfunction} open_apero::runBfs
```

## Paths and Gain Bookkeeping

```{doxygenstruct} open_apero::PathStep
:members:
:show: header-file
```

```{doxygenclass} open_apero::PathSteps
:members:
:show: header-file
```

```{doxygenstruct} open_apero::Path
:members:
:show: header-file
```

```{doxygenclass} open_apero::PathLedger
:members:
:show: header-file
```

```{doxygenstruct} open_apero::GainAnchor
:members:
:show: header-file
```

```{doxygenclass} open_apero::GainCostTable
:members:
:show: header-file
```

## History

```{doxygenclass} open_apero::HistoryData
:members:
:show: header-file
```

```{doxygenclass} open_apero::ObservedViewpoints
:members:
:show: header-file
```

## Enumerations

```{doxygenenum} open_apero::Traversability
```

```{doxygenenum} open_apero::SensorMountType
```

## Visualization

```{doxygenstruct} open_apero::VisualizationMarkers
:members:
:show: header-file
```
