# Robot Models, Cost Computers and Backtrackers

The parts that describe the robot and its motion: the robot model, the bounding volumes that limit where it plans,
the cost computer that prices every edge, and the backtrackers that produce recovery motions. The classes are on the
[Plugin Interfaces](../api/plugin-interfaces.md) and [Built-in Plugins](../api/builtin-plugins.md) API pages, the
keys under [Robot Models](../parameters.md#robot-models), [Bounding Volumes](../parameters.md#bounding-volumes),
[Cost Computers](../parameters.md#cost-computers) and [Backtrackers](../parameters.md#backtrackers) in the parameter
reference.

## Robot Models

The robot model is a shared service: generators sample and steer through it, and every layer checks traversability
with it. A state is `kTraversable`, `kPotentiallyTraversable` (not yet observed well enough) or `kUntraversable`.
A potentially traversable state counts as traversable only within `safe_radius` of the initial sensor position
(default `0`). Nodes must also keep the robot's preferred obstacle margin.

`SimpleRobot` is a body of spheres checked against a `TsdfMap`'s distance field:

- **Collision spheres** (`collision_sphere_{x,y,z,r}`, base frame) must not collide, and nodes keep the extra
  `collision_sphere_safe_margins`. **Contact spheres** (`contact_sphere_*`) must touch an obstacle such as the
  ground. A state is `kTraversable` once `min_observed_spheres` spheres (default all) were observed passing.
- **Sampling** draws from the bounding volume, which must be active; `sample_z: false` keeps the initial height.
- **Steering** is a straight line: rotate to face the travel direction, translate at `max_linear_velocity`, rotate
  to the goal orientation. Start and goal must be level (zero pitch); a pitched goal throws.
- **Body bounds** (`getBodyBounds()`) are the box over every sphere grown by its safe margin; the generators re-check a
  node or edge only where the updated map region meets them ([Generators](generators.md)).

`SimpleFallbackRobot`, the shipped robot, falls back to an OMPL planner (shipped `RRTstar`, 2 ms per query) when the
straight line is blocked but both ends are traversable; `random_seed` ≥ 0 seeds OMPL
([Random Seeding](agent.md#random-seeding)).

### Bounding Volumes

`SimpleRobot` and every `Evaluator` build a `BoundingVolume` from the namespace their `bounding_volume_args` names: the
robot's (`/map_bounding_volume`) bounds where it plans, an evaluator's (`/target_bounding_volume`) where gain counts. A
volume is a stage mesh's bounding box (`mesh_folder`, `scene_id`) grown by `margin`, as in the HSSD scenes, or the box
`x_min` … `z_max` in the map frame, either rotated by `rotation` degrees about z. Unless max > min on every axis it is
inactive and contains every point.

## Cost Computers

The agent's one cost computer prices every edge of every layer, recoveries included, in the budget's unit. No cost
computer has keys.

| Type | Edge cost |
|------|-----------|
| `IdentityCost` | `1` per edge |
| `TrajectoryLength` | Length of the sensor path [m]; a rotation in place costs `0`, so a path of rotations alone has an infinite ratio |
| `TrajectoryTime` | Duration of the trajectory [s]; shipped |

`getCostLowerBound()` is a cost no planned edge between two states undercuts, which lets a search skip planning an
edge that cannot win: the straight-line distance for `TrajectoryLength`, the larger of that distance over
`max_linear_velocity` and the yaw difference over `max_yaw_rate` for `TrajectoryTime`, and `0` otherwise.

## Backtrackers

The [`BacktrackingLayer`](layers.md#backtrackinglayer)'s strategy for a recovery motion. Each recovery trajectory
becomes one edge flagged `is_backtrack`, which backtrackers never undo.

| Type | Recovery |
|------|----------|
| `RotateInPlace` | Turns on the spot by `rotation_step`; at episode start it first sweeps `initial_rotate_angle` |
| `Reverse` | Replays the most recent executed edge backwards; remembers up to `queue_size` edges |
| `RotateReverse` | Rotates during the initial sweep and when no edge is remembered, else reverses the most recent edge. Shipped, with `rotation_step: 0.3` and `initial_rotate_angle: 6.28` |
