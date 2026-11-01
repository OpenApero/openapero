# ROS Node, Launch Files and Messages

How Open Apero runs under ROS Noetic: the simulated pipeline, the planner node, the launch and configuration files,
and the messages. This package's nodes' topics, services and parameters, the launch arguments and the scenes are
tables in the
[`open_apero_ros` README](https://github.com/OpenApero/openapero/blob/main/open_apero_ros/README.md).

## The Simulated Pipeline

`run_experiment_sim.launch` wires one simulated run:

```{include} ../../open_apero_ros/README.md
:start-after: "<!-- pipeline-start: docs/modules/ros.md shows the diagram up to pipeline-end -->"
:end-before: "<!-- pipeline-end -->"
```

A real robot replaces the renderer and the controller and keeps the rest
([Integrating a Robot](../integrating-a-robot.md)); [Simulation](simulation.md) covers the two simulated nodes.

## `open_apero_node`

The planner node runs the [`Agent`](agent.md) as a `RosAgent`, built from the parameter server, with a `VoxbloxMap`
that mirrors `voxblox_node` ([Maps](maps.md)). Each command is one trajectory on `/command/trajectory`, tagged with a
`command_sequence`. With `use_tf: false` (shipped) the robot answers in lockstep on `/robot_state`; with
`use_tf: true` the node polls the pose from TF at 20 Hz, and a `/command/result` with the same `command_sequence`
completes the command. The [settle gate](agent.md#the-settle-gate) then decides when the next cycle starts.
`~toggle_running` starts, stops and resumes an episode, `~reset` starts a new one, and `~get_wall_time` returns the
wall-clock seconds since its previous call.

## Launch Files

`run_experiment_sim.launch` starts the pipeline above, plus RViz with `rviz:=true` (default: `visualize`) and a
rosbag recorder with
`record_rosbag:=true`; `replay_rosbag.launch` plays such a bag back. `planner:=`, `task:=` and `scene_id:=` select
the configuration files; the
[README](https://github.com/OpenApero/openapero/blob/main/open_apero_ros/README.md#launch-files) lists every argument.

## Configuration Files

Each file-selecting launch argument has a folder under `open_apero_ros/config/`. The launch file loads the folder's
`_common.yaml`, then the selected `<value>.yaml` over it; a leading underscore marks a file no argument selects.

| Argument | Files | Loaded into |
|----------|-------|-------------|
| `planner:=nbs` | `planners/_common.yaml`, `planners/nbs.yaml` | `/open_apero_node` |
| `task:=recon` | `tasks/_common.yaml`, `tasks/recon.yaml` | `/task_evaluator` |
| `scene_id:=102816036` | `scenes/HSSD/_common.yaml`, `scenes/HSSD/102816036.yaml` | global parameters |

The planner finds the task through `evaluator_args` (`rrt`, `rrtstar`: `following_evaluator_args`); the robot and the
task find their bounding volumes in the scene's global parameters through `bounding_volume_args`. `task:=points` also
loads `tasks/_points_planning.yaml` into the planning layer. The files are not complete alone and use `$(env …)` and
`$(find …)`: load them through a launch file ([How Parameters Are Read](../parameters.md#how-parameters-are-read)).

The HSSD scenes take `/map_bounding_volume` (where the robot plans) and `/target_bounding_volume` (where gain counts)
from the stage meshes; a new environment or a real robot gives boxes in the map frame
([Bounding Volumes](robot-models.md#bounding-volumes)):

```yaml
map_bounding_volume:    {x_min: -5.0, x_max: 5.0, y_min: -5.0, y_max: 5.0, z_min: 0.0, z_max: 2.0}
target_bounding_volume: {x_min: -5.0, x_max: 5.0, y_min: -5.0, y_max: 5.0, z_min: 0.0, z_max: 2.0}
```

## Messages and Conversions

[`open_apero_msgs`](https://github.com/OpenApero/openapero/blob/main/open_apero_msgs/README.md) defines the
robot-facing messages `RobotState`, `RobotTrajectory`, `CommandResult` and `CollectiblePoint` /
`CollectiblePointArray`. The header-only
[`open_apero_ros_bridge`](https://github.com/OpenApero/openapero/blob/main/open_apero_ros_bridge/README.md) converts
`RobotState` to and from the core's type and the core's markers to `visualization_msgs`, builds the RViz status
overlay, and sends the core's log messages to `/rosout`.
