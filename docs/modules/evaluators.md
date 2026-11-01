# Evaluators and Sensor Models

How a candidate's gain is estimated: the sensor model simulates the view from a node, and the evaluator turns it
into gain anchors and a frontier flag, refreshed as the map grows. The classes are under
[Evaluators](../api/plugin-interfaces.md#evaluators) and [Sensor Model](../api/plugin-interfaces.md#sensor-model),
the keys under [`Evaluator`](../parameters.md#evaluator-keys),
[`SimulatedSensorEvaluator`](../parameters.md#simulatedsensorevaluator-keys) and
[Sensor Models](../parameters.md#sensor-models).

## Gain

- The `SensorModel` ray-casts a view from the node's sensor pose, and `evaluator::SimulatedSensorEvaluator` keeps the
  visible voxels inside the target bounding volume (`bounding_volume_args`). The evaluator turns them into the
  node's **gain anchors**; `Node::getSingleNodeGain()` sums them.
- A path's gain sums its nodes after the start, each node once. With the path planner's `is_gain_correlated`, which
  `task:=points` sets, an anchor shared by several nodes of the path also counts once.
- By default a node the robot has reached is spent (`Node::markObserved()`) and is never evaluated again.
- A node is a **frontier node** (`is_at_frontier`) when more than `frontier_min_unknown_ratio` of the voxels it sees are
  unknown and its view exceeds `frontier_min_visible_ratio` times the largest view seen so far this episode. Both
  default to `0`, which turns detection off; the shipped task evaluator sets `0.8` and `0.6`. Frontier nodes feed
  `expected_gain`, `table_gain`, SPT and TSP ([Path Planners](path-planners.md)).
- `getRealAccumulatedGain()` measures what the episode has actually gained, logged as
  `<layer>.RealAccumulatedGain` and as `real_accumulated_gain` in `episode_summary.yaml`.

## The Evaluator of Each Task

Every shipped planner reads its evaluator from `/task_evaluator`, set by [`task:=`](ros.md#launch-files):

| Task (`task:=`) | Evaluator | A node's gain | Real accumulated gain |
|-----------------|-----------|---------------|-----------------------|
| Volumetric exploration (`explore`, default) | `NaiveEvaluator` | Unknown voxels in view | Explored volume [m³] |
| Surface reconstruction (`recon`) | `SurfaceFrontierEvaluator` | Unknown voxels in view, weighted by `frontier_voxel_weight` next to the mapped surface, else `new_voxel_weight` | Mapped surface area [m²] |
| Point collection (`points`) | `PointsEvaluator` (`open_apero_ros`) | One anchor per observed (voxel mapped free), uncollected target point within its collection radius, shared by every node in range; collected once an executed waypoint puts the sensor within the radius | Summed gain of the collected points |

`SimpleYawPlanningEvaluator` wraps a `SimulatedSensorEvaluator` (`following_evaluator_args`) and keeps each tree
node's best orientation; the `rrt` and `rrtstar` configs use it. `VoxelWeightEvaluator` also credits occupied voxels
by how much a new view would add to their TSDF weight; no shipped setup uses it. `PointsEvaluator`'s CSV input is in
the [`open_apero_ros` README](https://github.com/OpenApero/openapero/blob/main/open_apero_ros/README.md).

## Gain Suppression

With the path planner's `gain_suppression_radius` above its default `0`, each search first suppresses every node
cluster (the nodes at one sensor position) closer than the radius to a higher-scoring kept cluster
(`Node::is_gain_suppressed`). Suppressed nodes stay in the graph as waypoints worth 0; the start node is never
suppressed. The beam searches, SPT and `TreeBestValue` honor it; TSP does not.

## Re-Evaluation on Map Updates

Each cycle `PlanningLayer::update()` hands every node cluster to `Evaluator::updateEvaluation()`. The simulated-sensor
evaluators re-evaluate a traversable node whose gain is above `update_gain` or that is a frontier node, and, with
`only_update_for_updated_map` (default `true`), only if its view box overlaps the map region updated since the last
cycle (`Map::getUpdatedMapRegion()`).

## Sensor Models

Rays run from the sensor pose to `ray_length` or the first occupied voxel, which counts as visible; a camera with
`observation_boundary: planar`, shipped to match the depth camera and Voxblox, ends them at camera depth
`ray_length` instead. `run_experiment_sim.launch` sets `ray_length` (shipped `3.0` m) and `observation_boundary`
from its arguments, as for Voxblox; a robot integration sets them to what its map integrates with.
`downsampling_factor` (shipped `3.0`) sets the ray spacing at the observation boundary in voxels, trading evaluation
time against accuracy. Every sensor model requires an `OccupancyMap`.

| Type | Model | Rays |
|------|-------|------|
| `SimpleRayCaster` | Pinhole camera (`CameraModel`) | One fixed grid |
| `IterativeRayCaster` | Pinhole camera (`CameraModel`); shipped 480 × 640 px, `focal_length` 625.58 (42° horizontal) | Coarse first, doubling the density in each farther distance section |
| `IterativeRayCasterLidar` | Spinning LiDAR (`LidarModel`), `field_of_view_x` × `field_of_view_y` degrees | As `IterativeRayCaster` |
