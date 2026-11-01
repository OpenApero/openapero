# Maps

The world model the planner queries: the `Map`, `OccupancyMap` and `TsdfMap` contracts in `open_apero_core`, and the
Voxblox backend `VoxbloxMap` in `open_apero_voxblox`. The classes are on the [Map API page](../api/maps.md),
`VoxbloxMap`'s parameters and topics in the
[`open_apero_voxblox` README](https://github.com/OpenApero/openapero/blob/main/open_apero_voxblox/README.md).

## The Map Contracts

The map is one of the agent's shared services, built from the `map` block of the planner config. Each plugin asks
for the interface it needs and refuses a poorer map at setup:

| Class | Adds | Required by |
|-------|------|-------------|
| `Map` | Explored volume, mapped surface area, the update window, snapshots, markers | The agent |
| `map::OccupancyMap` | Voxel occupancy (`kFree`, `kOccupied`, `kUnknown`), voxel size and center | Every sensor model and simulated-sensor evaluator |
| `map::TsdfMap` | Distance-field queries, a sphere collision test, voxel observation weights | `SimpleRobot` and `VoxelWeightEvaluator` |

Every `Map` method has a default (a no-op, `false` or `0`; `true` for `isStateReachable()`); `OccupancyMap` and
`TsdfMap` make their core queries pure virtual. The **update window** holds what the backend integrated since the
agent's last `resetMapUpdateStatus()`, called once per cycle: the updated region, whose nodes the evaluators
re-evaluate, and the integrated frames, which the [settle gate](agent.md#the-settle-gate) waits for with
`wait_for_map_updated` and `HistoryData` records as observed poses. Run a backend that reports no frames with
`wait_for_map_updated: false`.

## `VoxbloxMap`

The map of every shipped planner config. It owns a `voxblox::EsdfServer` in the planner node's private namespace and
builds only against the [OpenApero voxblox fork](https://github.com/OpenApero/voxblox). In the shipped setup it
receives the TSDF and ESDF layers that `voxblox_node` integrates. Occupancy comes from the ESDF (`kOccupied` where
the distance is below one voxel). Explored volume counts the observed ESDF voxels, mapped surface area those within
one voxel of a surface. `resetEpisode()` clears both layers and the RViz mesh; with the agent's `num_maps_to_save`,
`saveMap()` writes `maps/<NNNNN>.vxblx` into the run directory.

`run_experiment_sim.launch` sets the server's own keys (`tsdf_voxel_size`, `tsdf_voxels_per_side`, …) to match
`voxblox_node`: both servers need the same voxel size and voxels per block. `open_apero_node` loads
`libopen_apero_voxblox.so` by default (`plugin_libraries`, see
[In Your Own Package](../extending.md#in-your-own-package)).

## Writing a Map Backend

Derive from `map::OccupancyMap`, or from `map::TsdfMap` for `SimpleRobot`, implement the pure virtual queries, and
register the class like any plugin ([Extending Open Apero](../extending.md)). Override `getExploredVolume()` or
`getMappedSurfaceArea()` for the evaluators that report them, the update-window methods for re-evaluation and the
settle gate, and `agentParamsReadByBackend()` for keys the backend reads beside the agent's. A ROS-dependent backend
goes in its own package, as `open_apero_voxblox` does.
