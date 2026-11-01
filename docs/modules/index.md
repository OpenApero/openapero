# Modules

One page per part of Open Apero: how it behaves, which types ship and which keys matter. The first seven pages cover
the ROS-free core in `open_apero_core`, the rest the packages around it; [How Open Apero Works](../concepts.md) shows
how the parts fit together.

| Page | What it covers |
|------|----------------|
| [Agent and Execution](agent.md) | The loop, the settle gate, command phases, episode ends, seeding, history |
| [Layers](layers.md) | Priorities, `PlanningLayer`, `BacktrackingLayer`, adding a layer type |
| [Generators](generators.md) | `RRAG` and the trees `RRT` and `RRTStar` |
| [Evaluators and Sensor Models](evaluators.md) | Gain, frontier nodes, re-evaluation, the evaluator of each task, the ray casters |
| [Path Planners and Objectives](path-planners.md) | The objectives, NBS, DBS, SPT, TSP and `TreeBestValue` |
| [Robot Models, Cost Computers and Backtrackers](robot-models.md) | Robot models, bounding volumes, edge costs, recovery motions |
| [Maps](maps.md) | The map contracts and the `VoxbloxMap` backend |
| [ROS Node, Launch Files and Messages](ros.md) | `open_apero_node`, the robot-facing contract, launch and configuration files |
| [Simulation](simulation.md) | The Habitat renderer, the stand-in controllers and the HSSD scenes |
| [Run Output and Analysis](run-output.md) | The run directory, `performance_log.csv`, the Python loader |
| [Graph Benchmark](graph-benchmark.md) | The offline benchmark of the graph path planners |
| [Extension Hooks](extension-hooks.md) | Interfaces no shipped plugin uses, for custom robots and plugins |

```{toctree}
:hidden:

agent
layers
generators
evaluators
path-planners
robot-models
maps
ros
simulation
run-output
graph-benchmark
extension-hooks
```
