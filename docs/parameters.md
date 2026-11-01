# Parameter Reference

Every key that each plugin type, the `Agent` and `BoundingVolume` read, with its code default and its
constraints, for anyone writing or changing a planner configuration. Sections follow the YAML tree from the agent
down; a role's base-class table (such as [`RobotModel` Keys](#robotmodel-keys)) applies to every type of that role.
The [module pages](modules/index.md) explain what the keys do.

**Shipped** is the value the shipped simulation runs with (`run_experiment_sim.launch` with its default `planner:=nbs`
and no `task:=`), listed only where it differs from the code default; a parenthesis limits it to a file, planner or
`task:=`; **default** means the shipped setup runs on the code default. The values come from `open_apero_ros/config/`
(`planners/_common.yaml`, `planners/<planner>.yaml`, `tasks/_common.yaml`, `tasks/<task>.yaml`,
`tasks/_points_planning.yaml`, `scenes/HSSD/`) and from the launch file's `<param>` tags. A table without a Shipped
column runs on code defaults in every shipped setup.

## How Parameters Are Read

Every plugin is built from one namespace, a YAML block: its `type:` names the registered class, its other
scalar keys are the plugin's parameters. A nested block is a child plugin, named after its role (`robot`,
`generator`, `sensor_model`, …); a `<role>_args` key in the parent points that child at another namespace
instead (`->` below). The shipped NBS setup resolves to these namespaces; `dbs`, `spt` and `tsp` differ only
in the `path_planner` block, `rrt` and `rrtstar` where noted:

```text
/open_apero_node                 Agent (RosAgent): the Agent keys
├── map/                         type: VoxbloxMap
├── robot/                       type: SimpleFallbackRobot     bounding_volume_args -> /map_bounding_volume
├── cost_computer/               type: TrajectoryTime
└── layers/                      one sub-namespace per layer; the key is the layer's instanceName()
    ├── planning/                type: PlanningLayer           evaluator_args -> /task_evaluator
    │   ├── generator/           type: RRAG (rrt, rrtstar: RRT, RRTStar)
    │   ├── evaluator/           rrt, rrtstar only (no evaluator_args): type: SimpleYawPlanningEvaluator
    │   │                        following_evaluator_args -> /task_evaluator
    │   └── path_planner/        type: GraphNodeWiseBeamSearch (rrt, rrtstar: TreeBestValue)
    └── backtracking/            type: BacktrackingLayer
        └── backtracker/         type: RotateReverse
/task_evaluator                  type: NaiveEvaluator          bounding_volume_args -> /target_bounding_volume
└── sensor_model/                type: IterativeRayCaster
/map_bounding_volume             BoundingVolume, from config/scenes/HSSD/_common.yaml and <scene_id>.yaml
/target_bounding_volume          BoundingVolume, from the same files
```

The same layout in YAML, abridged from the shipped files; the launch file loads the first block into the
node's private namespace and the second into `/task_evaluator`:

```yaml
# open_apero_node's private namespace: config/planners/_common.yaml, then config/planners/nbs.yaml
cost_budget: 600                                # an Agent key
robot:                                          # child "robot" of the Agent
  type: "SimpleFallbackRobot"                   # selects the class; its keys sit beside it
  collision_sphere_r: 0.3                       # a list key holding one sphere
  bounding_volume_args: "/map_bounding_volume"  # read the volume from a root namespace
layers:
  planning:                                     # the layer's instanceName()
    type: "PlanningLayer"
    replan_num_steps: 5
    generator:
      type: "RRAG"
      min_edge_length: 1.5
    evaluator_args: "/task_evaluator"           # no evaluator: block; the evaluator lives there
    path_planner:
      type: "GraphNodeWiseBeamSearch"
      objective: "expected_gain"                # required
---
# /task_evaluator: config/tasks/_common.yaml, then config/tasks/<task>.yaml over it
type: "NaiveEvaluator"
bounding_volume_args: "/target_bounding_volume"
sensor_model:
  type: "IterativeRayCaster"
  ray_length: 3.0
```

- A key with a default is optional. A key marked **required** has none: leaving it out throws
  `std::runtime_error` at startup ("Parameter '…' not found in param map!"; for `layers`, "Agent:
  'layers' must be a non-empty dictionary …").
- An error raised inside a plugin's setup or validation carries the prefix "`<type>` at '`<namespace>`': " of every
  plugin on its way down, such as "PlanningLayer at '/open_apero_node/layers/planning': GraphNodeWiseBeamSearch at
  '/open_apero_node/layers/planning/path_planner': Invalid parameters: beam_width expected >= 1" (see
  [From `type:` to Object](extending.md#from-type-to-object)).
- Every plugin namespace needs a `type:` naming a registered class of its role, and every child plugin is required: the
  agent's `map`, `robot` and `cost_computer`, a `PlanningLayer`'s `generator`, `evaluator` and `path_planner`, a
  `BacktrackingLayer`'s `backtracker`, an evaluator's `sensor_model` (`SimpleYawPlanningEvaluator`: its
  `following_evaluator`), … A missing namespace or `type:` throws `PluginFactory: no plugin configured at '<ns>'
  (missing 'type'?)`; an unregistered name, or a class of another role, throws from `PluginRegistry`.
- A `<role>_args` key (`map_args`, `evaluator_args`, `bounding_volume_args`, …) moves that child to
  another namespace; its default is `<ns>/<role>`, `<ns>` being the plugin's own namespace.
- `/task_evaluator`, `/map_bounding_volume` and `/target_bounding_volume` are root namespaces, shared by
  every planner node on the master.
  [`run_experiment_sim.launch`](https://github.com/OpenApero/openapero/blob/main/open_apero_ros/README.md#run_experiment_simlaunch)
  fills `/task_evaluator` according to `task:=` and the two volumes from
  `config/scenes/<scene_dataset>/`.
- Each value is parsed with `operator>>` and must be consumed whole; otherwise setup throws `std::runtime_error`
  ("Failed to parse parameter '…'"; for a list, "Failed to parse vector parameter '…'"). A string value can therefore be
  neither empty nor contain whitespace (a path with a space fails): omit a key to keep an empty default. `inf` does not
  parse, and an `int` key does not take `1.5`.
- A value that breaks a stated constraint ("must be > 0", "in (0, 1]", …) throws `std::invalid_argument`
  ("Invalid parameters: …") once that plugin's setup has run. Some checks run inside the setup and throw
  `std::runtime_error` instead, such as an unknown name for a key that takes a fixed set of names.
- After a plugin's setup, the keys of its namespace that no setup read are reported once, as the warning
  "`<type>` ignores parameters: a, b (namespace '`<ns>`')" (`open_apero::BoundingVolume` for a bounding
  volume). The agent's own keys are checked after every plugin is built and reported as `Agent`; that check
  skips `log_level`, `run_dir`, `plugin_libraries` and the keys of the embedded Voxblox server (see
  [`VoxbloxMap`](#voxbloxmap)). With the agent's `strict_params: true` an unread key throws
  `std::runtime_error` instead. Keys in a child's sub-namespace belong to the child; a sub-namespace no child
  is created from (e.g. an inline `evaluator:` block next to `evaluator_args`) is never checked, not even
  with `strict_params`. `type` is exempt.
- `PluginFactoryRos` gives a plugin at namespace `<ns>` the leaf parameters directly under
  `<ns>/`; a sibling that only shares the prefix (`map_frame` next to `map`) is not among them.
- List keys (`double list`) take one comma-separated scalar, such as `collision_sphere_x: 0.0, 0.1`, with no
  space before a comma. A YAML list (`[0.0, 0.1]`) does not parse.
- Write YAML booleans unquoted (`true`); a quoted `"true"` is a string and does not parse. With
  `PluginFactoryConfig` (no ROS) every value is a string and a `bool` is `"1"` or `"0"`; `"true"` does not
  parse.
- Angles are in radians unless a row says degrees; lengths are in meters, durations in seconds,
  costs in the cost computer's unit (seconds for `TrajectoryTime`).

## Agent

### `Agent` Keys

Top-level keys, in the node's private namespace.

| Key | Type | Default | Meaning | Shipped |
|-----|------|---------|---------|---------|
| `layers` | dictionary | **required** | One sub-namespace per layer, each a block with a `type:`; must be non-empty. | `planning`, `backtracking` |
| `map_args` | string | `<ns>/map` | Namespace of the map. | default |
| `robot_args` | string | `<ns>/robot` | Namespace of the robot model. | default |
| `cost_computer_args` | string | `<ns>/cost_computer` | Namespace of the cost computer. | default |
| `cost_budget` | double | unbounded | Episode cost budget; below `1e-4`, or omitted, = unbounded. | `600`; `cost_budget:=` overrides |
| `random_seed` | int | `-1` | Seed of every stochastic plugin; negative = nondeterministic. See [random seeding](modules/agent.md#random-seeding). | default |
| `strict_params` | bool | `false` | Unread keys throw `std::runtime_error` instead of warning: a plugin's or bounding volume's right after its setup, the agent's own once every plugin is built (`RosAgent` checks them; another platform must call `Agent::reportUnreadParams()`). See [How Parameters Are Read](#how-parameters-are-read). | default |
| `target_position_threshold` | double | `0.1` | Sensor position tolerance of the settle gate [m]; must be > `1e-6`, the pose-equality epsilon. | default |
| `target_angle_threshold` | double | `0.1` | Sensor orientation tolerance of the settle gate [rad]; must be ≥ 2·acos(1 − `1e-6`) ≈ `2.83e-3`. | `0.20` |
| `observed_viewpoint_position_threshold` | double | `0.0` | Position threshold of `HistoryData::getObservedViewpoints()` [m]: a pose counts as already observed from exactly when an observed pose lies closer than it to its position and closer than `observed_viewpoint_angle_threshold` to its orientation. `0.0` together with `observed_viewpoint_angle_threshold` `0.0` disables the test (the poses are stored regardless); otherwise must be > `0.01`. No shipped plugin reads the index; it is exposed to custom plugins ([Extension Hooks](modules/extension-hooks.md)). | default |
| `observed_viewpoint_angle_threshold` | double | `0.0` | Geodesic orientation threshold paired with `observed_viewpoint_position_threshold` [rad]; must be > `0` unless both are `0.0`. No shipped plugin reads the index. | default |
| `wait_for_map_updated` | bool | `false` | The settle gate also waits for a mapped frame at the target pose. | `true` |
| `build_plugins_on_init` | bool | `false` | Run one warm-up pass through every layer at startup. | `true` |
| `log_performance` | bool | `false` | Time every phase and write `performance_log.csv`. | `true` (launch `log_data`) |
| `num_maps_to_save` | int | `0` | N map snapshots saved to `maps/`: map k at the first replan past k/N of the budget, never the final map ([Run Directory](modules/run-output.md#run-directory)); with an unbounded `cost_budget`, one snapshot when the episode ends or is stopped. | default |
| `visualize` | bool | `true` | Publish layer, map, robot and trajectory markers every cycle, with `RosAgent`'s `status_overlay` and `PointsEvaluator`'s `collectible_points`; `RosAgent` also takes its end-of-run RViz screenshot only when set. | default |
| `color_past_trajectory_by_budget` | bool | `false` | Color the executed trajectory by budget used. | `true` |
| `verbose` | bool | `true` | Log the layer stack, replan timing and per-layer cycle summaries. | default |
| `verbose_plugins` | bool | `true` | Log each plugin's resolved parameters when it is created. | default |
| `debug_diagnostics` | bool | `false` | Add nearest-frame detail to settle-gate warnings, and report a frame-ledger shortfall (fewer frames mapped than commands dispatched) even for a map that reports no frame list. | default |

### `RosAgent`

In `open_apero_ros`; reads the `Agent` keys plus:

| Key | Type | Default | Meaning | Shipped |
|-----|------|---------|---------|---------|
| `use_tf` | bool | `true` | Track the pose from TF and resolve commands by `command/result`; `false` follows `robot_state`. | `false` |
| `map_frame` | string | `map` | World frame of commands, markers and TF lookups; a `command/result` in another frame is dropped. | `world` (launch) |
| `base_frame` | string | `base` | Robot base frame (TF mode). | default |
| `camera_frame` | string | `camera_frame` | Sensor frame (TF mode). | default |
| `exit_on_episode_end` | bool | `true` | Shut the node down when an episode ends with the budget spent, a goal reached or no step; a `toggle_running(false)` stop leaves it running. `false` keeps it alive for `~reset`. | default |
| `run_dir` | string | empty | Run output directory, read at the first `toggle_running(true)`; required when `log_performance` is `true` or `num_maps_to_save` > 0. | set by `mission_coordinator_node` |
| `log_level` | string | `info` | Console level: `debug`, `info`, `warn`, `error` or `fatal` (read by `open_apero_node`); an unknown value keeps `info` with a warning. | default |
| `plugin_libraries` | string list | `[libopen_apero_voxblox.so]` | Plugin libraries `open_apero_node` loads before it builds the agent ([In Your Own Package](extending.md#in-your-own-package)); a YAML list, unlike the `double list` keys. | default |

## Layers

### `PlanningLayer`

Its `generator`, `evaluator` and `path_planner` children are listed under [Generators](#generators),
[Evaluators](#evaluators) and [Path Planners](#path-planners); the generator must be a `GraphGenerator`, or setup
throws.

| Key | Type | Default | Meaning | Shipped |
|-----|------|---------|---------|---------|
| `priority` | int | `0` | Arbitration priority; higher is asked first. Every priority a layer can report (`BacktrackingLayer`: `priority` and `promoted_priority`) must differ from every other layer's, or setup throws. | default |
| `max_expansions` | int | `50` | Expansion rounds (sample, connect, evaluate, price) per cycle; `0` or less never grows the graph. | default |
| `replan_num_steps` | int | unlimited | Cut the selected edge after this many time steps (`replan_num_steps` + 1 states) so the robot replans early; ≤ `0` = never. | `5` |
| `generator_args` | string | `<ns>/generator` | Namespace of the generator. | default |
| `evaluator_args` | string | `<ns>/evaluator` | Namespace of the evaluator. | `/task_evaluator` (`nbs.yaml`, `dbs.yaml`, `spt.yaml`, `tsp.yaml`) |
| `path_planner_args` | string | `<ns>/path_planner` | Namespace of the path planner. | default |

### `BacktrackingLayer`

Its `backtracker` child is listed under [Backtrackers](#backtrackers).

| Key | Type | Default | Meaning | Shipped |
|-----|------|---------|---------|---------|
| `priority` | int | `-1000` | Baseline priority. It and `promoted_priority` must differ from every priority another layer can report, or setup throws. | default |
| `promoted_priority` | int | `1000` | Priority while the backtracker's initial sweep runs or the robot's state is untraversable. | default |
| `backtracker_args` | string | `<ns>/backtracker` | Namespace of the backtracker. | default |

## Map

### `VoxbloxMap`

In `open_apero_voxblox`. Its own namespace holds only `type:`. The embedded Voxblox `EsdfServer` reads its
keys (`tsdf_voxel_size`, …; see the
[`open_apero_voxblox` parameters](https://github.com/OpenApero/openapero/blob/main/open_apero_voxblox/README.md#parameters))
from the node's private namespace, beside the `Agent` keys, and the agent's unread-key check skips them
(`VoxbloxMap::nodeParamsReadByVoxblox()`); that list also holds `verbose` and `visualize`, which the `Agent`
reads too. `VoxbloxMap` is a `TsdfMap`, hence an `OccupancyMap`: `SimpleRobot` and `VoxelWeightEvaluator`
require the former, the sensor models and the `SimulatedSensorEvaluator`s the latter, and each refuses another map
type at setup (`SimpleRobot` in `checkParamsValid()`).

## Robot Models

### `RobotModel` Keys

Read by every robot model.

| Key | Type | Default | Meaning | Shipped |
|-----|------|---------|---------|---------|
| `max_linear_velocity` | double | `1.0` | Maximum linear speed [m/s]; must be > 0. | `0.5` |
| `max_yaw_rate` | double | π/2 | Maximum yaw rate [rad/s]; must be > 0. | `1.6` |
| `time_step` | double | `0.05` | Time between consecutive trajectory states [s]; must be > 0. | `0.2` |
| `visualize` | bool | `true` | Publish the robot body along the dispatched trajectory (`SimpleRobot`: also its bounding volume and sampling region). | `false` |
| `safe_radius` | double | `0.0` | Unobserved space within this distance of the initial sensor position counts as traversable [m]. | default |
| `clearance_margin_eps` | double | `1e-4` | Floor of the relative clearance scale; in (0, 1]. | default |
| `num_orientation_candidates` | int | `12` | Evenly spaced yaws sampled per position; must be > 0. | `8` |
| `sensor_mount_type` | string | `fixed` | `fixed`, `free_pitch` or `unconstrained` (case-insensitive); exposed to robot-model subclasses, no in-tree plugin reads it. | default |
| `gaze_pitch_min` | double | −π/2 | Lowest plannable gaze pitch [rad], positive looks down; finite and ≥ −π/2. Exposed to subclasses; no in-tree plugin plans pitch. | default |
| `gaze_pitch_max` | double | π/2 | Highest plannable gaze pitch [rad]; finite, ≥ `gaze_pitch_min` and ≤ π/2. Exposed to subclasses; no in-tree plugin plans pitch. | default |
| `base_in_sensor_position_{x,y,z}` | double | `0.0` | Base position in the sensor frame [m]. `RosAgent` with `use_tf` measures it instead. | default |
| `base_in_sensor_orientation_{x,y,z,w}` | double | `0, 0, 0, 1` | Base orientation in the sensor frame (quaternion). `RosAgent` with `use_tf` measures it instead. | default |

### `SimpleRobot`

Sphere body checked against a `TsdfMap`. Reads the `RobotModel` keys plus:

| Key | Type | Default | Meaning | Shipped (as `SimpleFallbackRobot`) |
|-----|------|---------|---------|---------|
| `collision_sphere_{x,y,z,r}` | double list | empty | Collision spheres in the base frame [m]: centers and radii, one entry per sphere; the four lists must have equal length. | one sphere, `r` `0.3` at the origin |
| `collision_sphere_safe_margins` | double list | all `0.0` | Preferred obstacle margin per collision sphere [m]; nodes must keep it in full. If given, one entry per collision sphere. | default |
| `contact_sphere_{x,y,z,r}` | double list | empty | Spheres that must touch an obstacle (e.g. the ground) for a state to be traversable; the four lists must have equal length. | default |
| `collision_check_yaw` | bool | deduced | Whether traversability depends on yaw; deduced `true` when any sphere is off the vertical axis. | default |
| `min_observed_spheres` | int | `-1` | Collision and contact spheres that must be observed for `kTraversable` rather than `kPotentiallyTraversable`; `-1` = all; must not exceed their number. | default |
| `collision_sphere_visualization_stride` | int | `1` | Draw the body at every Nth trajectory state (endpoints always). | default |
| `sample_z` | bool | `true` | Sample the z coordinate; `false` uses the initial sensor height. | default |
| `restrict_sampling_to_mapped_region` | bool | `false` | Sample only inside the mapped part of the bounding volume; the whole volume while that part is empty. Skipped for a rotated volume. | default |
| `bounding_volume_args` | string | `<ns>/bounding_volume` | Namespace of the [bounding volume](#bounding-volumes) that bounds sampling and every planned state. It must be active: sampling draws from its x/y/z extents, so an unset volume samples only x = y = 0. | `/map_bounding_volume` |

### `SimpleFallbackRobot`

`SimpleRobot` that falls back to an OMPL planner in a local window when the straight-line steer is
blocked. Reads the `SimpleRobot` keys plus:

| Key | Type | Default | Meaning | Shipped |
|-----|------|---------|---------|---------|
| `ompl_planner` | string | empty | `RRT`, `RRTconnect`, `RRTstar`, `RRTsharp` or `InformedRRTstar`; omit the key to disable the fallback (an empty string does not parse). | `RRTstar` |
| `ompl_planning_time` | double | `0.01` | Time limit per fallback query [s]. | `0.002` |
| `ompl_resolution` | double | `0.01` | State-space segment resolution [m]; only the `clearance` objective's cost integral interpolates at it. Validity checks steer each motion and test every state (spacing `max_linear_velocity`·`time_step`). | `0.10` |
| `ompl_max_range` | double | `2.0` | OMPL planner range, and the margin of the local window around start and goal, which is clipped to the bounding volume when that is active and unrotated [m]. | `3.0` |
| `ompl_state_space_type` | string | `R3` | `R3`: 3D position, alias `R2`; `R3Yaw`: 3D position and yaw, alias `SE2`. Checked only with `ompl_planner` set. | default |
| `ompl_objective` | string | `time` | `time` (execution time) or `clearance` (obstacle margin). | default |
| `ompl_log_level` | string | `info` | `info`, `warn`, `error` or `none`; sets OMPL's process-global log level. Checked only with `ompl_planner` set. | `warn` |

With a planner set and `random_seed` ≥ 0, setup seeds OMPL's process-global generator from the seed and the
robot's namespace; see [random seeding](modules/agent.md#random-seeding).

## Bounding Volumes

The keys of the namespace a `bounding_volume_args` names;
[Bounding Volumes](modules/robot-models.md#bounding-volumes) explains what a volume bounds and when it is active.

| Key | Type | Default | Meaning | Shipped |
|-----|------|---------|---------|---------|
| `mesh_folder` | string | empty | Folder of the stage meshes; with `scene_id`, the box is the mesh's bounds. | HSSD stages folder |
| `scene_id` | string | empty | Stage mesh `<mesh_folder>/<scene_id>.glb` (HSSD), or `<mesh_folder>/<scene_id>/<code>.glb` (HM3D) if it contains `-`, `<code>` being the part after the first `-`. | per-scene file |
| `margin` | double | `0.0` | Padding added to the mesh bounds [m]; unused for an `x_min` … `z_max` box. | `0.2` |
| `x_min` … `z_max` | double | `0.0` | Box corners [m]; read unless both `mesh_folder` and `scene_id` are set. | default |
| `rotation` | double | `0.0` | Counter-clockwise yaw of the box about the world z axis through the origin [degrees]. `SimpleRobot` samples from the unrotated extents, and `SimpleFallbackRobot` clips its OMPL window only to an unrotated volume. | default |

The shipped `/map_bounding_volume` and `/target_bounding_volume` come from
`open_apero_ros/config/scenes/HSSD/`.

## Cost Computers

No keys: `type` alone selects `IdentityCost`, `TrajectoryLength` or `TrajectoryTime` (shipped);
[Cost Computers](modules/robot-models.md#cost-computers) compares them.

## Generators

### `Generator` and `GraphGenerator` Keys

Read by every generator.

| Key | Type | Default | Meaning | Shipped |
|-----|------|---------|---------|---------|
| `min_edge_length` | double | `0.8` | A sample closer than this to its nearest node is rejected [m]; must be > 0. | `1.5`; `1.0` (`task:=points`) |
| `max_edge_length` | double | `100.0` | A sample farther than this is moved to `min_edge_length` from its nearest node; also the edge connection radius of `RRAG` and `RRTStar` [m]. `0` = no limit; otherwise must be ≥ `min_edge_length`. | `3.0`; `2.0` (`task:=points`) |
| `visualize_edges` | bool | `false` | Draw every graph edge. | default |

### `RRAG`

Rapidly-exploring random annulus graph: node clusters (one node per orientation) joined by rotation
and translation edges. Keys, besides the shared ones:

| Key | Type | Default | Meaning |
|-----|------|---------|---------|
| `sample_max_tries` | int | `1000` | At most this many sampling attempts per expansion round; `0` = unlimited; must be ≥ 0. A robot model whose `samplePosition()` fails ends the round without a node. |
| `translation_adjacency` | string | `straight_line` | Which orientation pairs a translation edge joins: `straight_line` (best aligned with the chord), `path_heading` (with the planned path's end directions) or `path_heading_cell` (every pair bracketing them, for robot models that override `getTranslationAdjacency`; the in-tree robots connect one pair, as `path_heading`). |

### `RRT`, `RRTStar`

Trees grown from the robot; `RRTStar` also rewires for lower cost. Keys, besides the shared ones:

| Key | Type | Default | Meaning |
|-----|------|---------|---------|
| `sample_max_tries` | int | `1000` | At most this many sampling attempts per expansion round; `0` = unlimited; must be ≥ 0. A robot model whose `samplePosition()` fails ends the round without a node. |

## Evaluators

### `Evaluator` Keys

Read by every evaluator.

| Key | Type | Default | Meaning | Shipped |
|-----|------|---------|---------|---------|
| `bounding_volume_args` | string | `<ns>/bounding_volume` | [Bounding volume](#bounding-volumes) of the target: only voxels inside it earn gain (`PointsEvaluator`: only points inside it are loaded). | `/target_bounding_volume` (task evaluator) |

### `SimulatedSensorEvaluator` Keys

Read by `NaiveEvaluator`, `SurfaceFrontierEvaluator`, `VoxelWeightEvaluator` and `PointsEvaluator`,
which ray-cast a simulated view from each node.

| Key | Type | Default | Meaning | Shipped |
|-----|------|---------|---------|---------|
| `sensor_model_args` | string | `<ns>/sensor_model` | Namespace of the [sensor model](#sensor-models). | default |
| `clear_from_parents` | bool | `false` | Count only voxels no tree ancestor sees; tree generators only. Under `SimpleYawPlanningEvaluator` an ancestor's chosen orientation counts. | default |
| `visualize_sensor_view` | bool | `false` | Draw the sensor view of the selected path's first evaluated node. | `true` |
| `frontier_min_unknown_ratio` | double | `0.0` | A frontier node sees more than this fraction of unknown voxels. | `0.8` |
| `frontier_min_visible_ratio` | double | `0.0` | …and more than this fraction of the most voxels any node has seen. Both below `1e-4` disable frontiers. | `0.6` |
| `update_gain` | double | `0.0` | Re-evaluate only nodes with more gain than this, or frontier nodes. | default |
| `only_update_for_updated_map` | bool | `true` | Re-evaluate only nodes whose view overlaps the region the map updated. | default |

### `NaiveEvaluator`

Gain: one per unknown voxel in view. Accumulated gain: the map's explored volume. No keys beyond
the `SimulatedSensorEvaluator` ones.

### `SurfaceFrontierEvaluator`

Gain: `frontier_voxel_weight` per unknown voxel that borders the mapped surface,
`new_voxel_weight` per other unknown voxel. Accumulated gain: the mapped surface area. Requires an
`OccupancyMap`: setup throws `std::runtime_error` on another map type. Keys, besides the
`SimulatedSensorEvaluator` ones:

| Key | Type | Default | Meaning | Shipped |
|-----|------|---------|---------|---------|
| `accurate_frontiers` | bool | `false` | Check all 26 neighbors instead of the 6 face neighbors. | `true` (`task:=recon`) |
| `surface_frontiers` | bool | `true` | The first observed neighbor (fixed scan order) must be occupied, not just observed; `false` accepts any observed neighbor. | default |
| `checking_distance` | double | `1.0` | Neighbor offset [voxels]. | `2.0` (`task:=recon`) |
| `frontier_voxel_weight` | double | `1.0` | Gain of a surface-frontier voxel. | default |
| `new_voxel_weight` | double | `0.01` | Gain of any other unknown voxel. | `0.0` (`task:=recon`) |

### `VoxelWeightEvaluator`

`SurfaceFrontierEvaluator` that also credits occupied voxels by how much a new observation would
add to their TSDF weight. Requires a `TsdfMap`: setup throws `std::runtime_error` on another map
type. Keys, besides the `SurfaceFrontierEvaluator` ones:

| Key | Type | Default | Meaning |
|-----|------|---------|---------|
| `min_impact_factor` | double | `0.0` | Occupied voxels with a smaller relative weight gain earn nothing. |
| `ray_angle_x` | double | `0.0025` | Horizontal angle between rays [rad]. |
| `ray_angle_y` | double | `0.0025` | Vertical angle between rays [rad]. |

Not shipped: no shipped setup selects `VoxelWeightEvaluator` or sets these keys.

### `SimpleYawPlanningEvaluator`

Wraps a following evaluator and keeps each node's best yaw; tree generators only (one node per
call). It reads the `Evaluator` key but never uses its own volume: the following evaluator's
`bounding_volume_args` bounds the gain. Keys, besides the `Evaluator` one:

| Key | Type | Default | Meaning | Shipped |
|-----|------|---------|---------|---------|
| `following_evaluator_args` | string | `<ns>/following_evaluator` | Namespace of the evaluator that scores each yaw. It must be a `SimulatedSensorEvaluator` (`NaiveEvaluator`, `SurfaceFrontierEvaluator`, `VoxelWeightEvaluator` or `PointsEvaluator`), or setup throws `std::runtime_error`. | `/task_evaluator` (`rrt.yaml`, `rrtstar.yaml`) |
| `update_gain` | double | `0.0` | Re-evaluate only yaws with at least this gain, or frontier yaws. | default |
| `visualize_followup` | bool | `true` | Also draw the following evaluator's markers for the selected yaw. | default |

### `PointsEvaluator`

In `open_apero_ros`. Gain from a set of points to observe and collect. Keys, besides the
`SimulatedSensorEvaluator` ones:

| Key | Type | Default | Meaning | Shipped |
|-----|------|---------|---------|---------|
| `points_file` | string | empty | CSV with rows `x,y,z,radius,gain` (`gain` an integer); points outside the bounding volume are dropped, and a file that cannot be opened logs an error and loads none. | `data/points_3d.csv` (`tasks/points.yaml`) |

## Sensor Models

### `SensorModel` Keys

Read by every sensor model.

| Key | Type | Default | Meaning | Shipped |
|-----|------|---------|---------|---------|
| `ray_length` | double | `5.0` | Maximum ray length [m], the mapping range; `run_experiment_sim.launch` sets it from its `ray_length`. Must be > 0. | `3.0` |
| `log_cast_timing` | bool | `false` | Time every ray cast and log the mean and standard deviation [ms] every 20 casts; safe when an evaluator casts from several threads. | default |

### `SimpleRayCaster`, `IterativeRayCaster`

Pinhole camera models (`CameraModel`); the field of view is `2·atan(resolution / (2·focal_length))`
per axis. `SimpleRayCaster` marches one fixed grid of rays; `IterativeRayCaster` starts coarse and
doubles the ray density in each farther distance section. Both stop a ray at the first occupied voxel, which counts as
visible.
Keys, besides the `SensorModel` ones:

| Key | Type | Default | Meaning | Shipped |
|-----|------|---------|---------|---------|
| `resolution_x` | int | `640` | Image width [px]; must be > 0. | `480` |
| `resolution_y` | int | `480` | Image height [px]; must be > 0. | `640` |
| `focal_length` | double | `320.0` | Focal length [px]; must be > 0. | `625.58` |
| `observation_boundary` | string | `spherical` | Where the view ends: `spherical` (distance `ray_length`, the paper's model) or `planar` (camera depth `ray_length`, a depth camera's max range), as the map integrates; `run_experiment_sim.launch` sets it and voxblox's from its `observation_boundary`. | `planar` |
| `ray_step` | double | map voxel size | Step along each ray [m]; must be > 0. | default |
| `downsampling_factor` | double | `1.0` | Ray spacing at the observation boundary, in voxels; the ray count never exceeds the resolution. Must be > 0, and the resulting grid must have at least 2 rays per axis. | `3.0` |

### `IterativeRayCasterLidar`

Spinning LiDAR model (`LidarModel`). Keys, besides the `SensorModel` ones:

| Key | Type | Default | Meaning |
|-----|------|---------|---------|
| `resolution_x` | int | `1024` | Horizontal beams; must be > 0. |
| `resolution_y` | int | `128` | Vertical beams; must be > 0. |
| `field_of_view_x` | double | `360.0` | Horizontal field of view [degrees]; must be > 0. |
| `field_of_view_y` | double | `30.0` | Vertical field of view [degrees]; must be > 0. |
| `ray_step` | double | map voxel size | Step along each ray [m]; must be > 0. |
| `downsampling_factor` | double | `1.0` | Ray spacing at `ray_length`, in voxels; the ray count never exceeds the resolution. Must be > 0, and the resulting grid must have at least 2 rays per axis. |

## Path Planners

### `PathPlanner` Keys

Read by every path planner except `NoOpPathPlanner`. `GraphTspSearch` reads all four and uses none; its
`objective` is optional (default `path_gain`) but must still be one of the four names.

| Key | Type | Default | Meaning | Shipped |
|-----|------|---------|---------|---------|
| `objective` | string | **required** (`GraphTspSearch`: `path_gain`) | `path_gain`, `path_ratio`, `expected_gain` or `table_gain`; see [selection objectives](modules/path-planners.md#selection-objectives). | `expected_gain` |
| `is_gain_correlated` | bool | `false` | Count a gain anchor shared by several nodes once per path. | `true` (`task:=points`) |
| `gain_suppression_radius` | double | `0.0` | Radius [m] of the [non-maximum suppression](modules/evaluators.md#gain-suppression) over node clusters before each search: a cluster closer than this to a better one is a waypoint worth 0. `0` suppresses nothing; must be finite and ≥ 0. | default |
| `concave_envelope` | bool | `true` | Reduce the `table_gain` table to its concave hull. | default |

### `GraphNodeWiseBeamSearch`, `GraphDepthWiseBeamSearch`

Beam search over the graph: each step NBS keeps `beam_width` paths per end node, DBS
`beam_width` paths in total. Keys, besides the `PathPlanner` ones:

| Key | Type | Default | Meaning | Shipped NBS / DBS |
|-----|------|---------|---------|-------------------|
| `beam_depth` | int | `10` | Maximum path length [edges]; must be ≥ 1. With `check_previous_path`, a reused previous path can exceed it. | `100` / `100` |
| `beam_width` | int | `1` | Paths kept per step; must be ≥ 1. | default / `100` |
| `check_previous_path` | bool | `true` | Also score the rest of the previous path and its one-edge extensions. | `false` / `false` |

### `GraphShortestPathTreeSearch`, `GraphTspSearch`

Both route to candidate nodes: the nodes whose gain lies in the top `top_node_percentage` of the
gain range, plus frontier nodes. SPT scores the shortest path to each. TSP returns an open route from the
robot through all of them that minimizes the summed edge cost (a random neighbor if the solver finds
none), and ignores `objective` and the cost budget, which the agent enforces during execution. Keys,
besides the `PathPlanner` ones:

| Key | Type | Default | Meaning | Shipped SPT / TSP |
|-----|------|---------|---------|-------------------|
| `top_node_percentage` | double | `1.0` | Fraction of the gain range, from the top, whose nodes are candidates; `1.0` = all nodes. | default / `0.5` |
| `cost_scale` | double | `1.0` | TSP only: shortest-path costs between nodes are multiplied by it before they are truncated to the integers OR-Tools takes; `1000` gives millisecond resolution for `TrajectoryTime`, while the shipped `1.0` gives whole seconds. Must be finite and > 0. | default |

### `TreeBestValue`

Tree planner for `RRT` and `RRTStar`: scores the path from the root to every node. No keys beyond the
`PathPlanner` ones. With a graph generator such as `RRAG`, planning throws.

### `NoOpPathPlanner`

Placeholder for layers that select their own step; reads no keys and returns no path.

## Backtrackers

### `RotateInPlace`

Rotates on the spot.

| Key | Type | Default | Meaning | Shipped (as `RotateReverse`) |
|-----|------|---------|---------|---------|
| `rotation_step` | double | π | Yaw swept by one recovery trajectory [rad], rounded up to whole steps of `max_yaw_rate` · `time_step` (shipped: one step of 0.32 rad); must be > 0. | `0.3` |
| `initial_rotate_angle` | double | 2π | Total yaw of the initial sweep at episode start [rad]; `0` = none; must be ≥ 0. | `6.28` |

### `Reverse`

Replays the most recent executed edge backwards.

| Key | Type | Default | Meaning |
|-----|------|---------|---------|
| `queue_size` | int | `100` | Executed edges remembered; must be > 0. |

### `RotateReverse`

`RotateInPlace`, then `Reverse` ([Backtrackers](modules/robot-models.md#backtrackers)). Reads the
`RotateInPlace` keys (shipped: see above) plus:

| Key | Type | Default | Meaning |
|-----|------|---------|---------|
| `queue_size` | int | `100` | Executed edges remembered; must be > 0. |
