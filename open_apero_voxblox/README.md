# open_apero_voxblox

[Voxblox](https://github.com/ethz-asl/voxblox) TSDF/ESDF map backend for Open Apero: the `VoxbloxMap` plugin that every
shipped planner configuration selects (`map: type: "VoxbloxMap"`). It builds only against the
[OpenApero voxblox fork](https://github.com/OpenApero/voxblox). `VoxbloxMap` owns a `voxblox::EsdfServer` in the
planner node's private namespace, which in the shipped setup mirrors the layers of the `voxblox_node` mapping server
instead of integrating point clouds itself. `open_apero_node` loads the library by default through its
`plugin_libraries` parameter ([Extending Open Apero](../docs/extending.md#in-your-own-package)).

`VoxbloxMap` implements the `TsdfMap` interface of `open_apero_core`: [Maps](../docs/modules/maps.md#voxbloxmap)
describes its queries, the [Map API](../docs/api/maps.md) its methods and the
[fork README](https://github.com/OpenApero/voxblox#what-this-fork-adds) what the fork adds for them.

## ROS interface

Relative to `open_apero_node`'s private namespace unless absolute; the embedded server also advertises the standard
`voxblox_ros` `esdf_server` topics and services there.

| Kind | Name | Type | Description |
|---|---|---|---|
| Subscribed | `~tsdf_map_in`, `~esdf_map_in` | `voxblox_msgs/Layer` | TSDF and ESDF layers; `run_experiment_sim.launch` remaps them to `voxblox_node`'s `tsdf_map_out` and `esdf_map_out` |
| Subscribed | `~pointcloud`, `~sync_pointcloud`, `~sync_camera_transform`, `~transform` | `sensor_msgs/PointCloud2`, `geometry_msgs/TransformStamped` | The server's own inputs, private so that a robot's clouds stay out; unused, as the map mirrors `voxblox_node` |
| Service called | `/voxblox_mesh_display/reset` | `std_srvs/Trigger` | On `resetEpisode()`, if the RViz mesh display offers it |

## Parameters

Read by the embedded server from the planner node's private namespace; `voxblox_node` reads its own copy. Code default
is voxblox's, Launch value what `run_experiment_sim.launch` sets on `open_apero_node`.

| Parameter | Code default | Launch value | Description |
|---|---|---|---|
| `tsdf_voxel_size` | `0.2` | `0.1` (`voxel_size` arg) | Voxel size [m]; must match `voxblox_node` |
| `tsdf_voxels_per_side` | `16` | `8` (`voxels_per_side` arg) | Voxels per block side, a power of two; must match `voxblox_node` |
| `use_tf_transforms` | `true` | `false` | Poses of `~pointcloud` from TF instead of `~transform` |
| `update_esdf_every_n_sec`, `publish_map_every_n_sec`, `update_mesh_every_n_sec` | `1.0` | `0.0` | ESDF update, map publish and mesh update timers [s]; 0 disables them |
| `max_weight` | `10000` | unset | Returned by `getMaximumWeight()` |

## Dependencies

[`open_apero_core`](../open_apero_core/README.md); `voxblox`, `voxblox_ros` from the fork; `roscpp`, `std_srvs`.

## Tests

```bash
catkin build open_apero_voxblox --no-deps --catkin-make-args run_tests --   # from the workspace root
```

`test_open_apero_voxblox` checks the `TsdfMap` registration, construction and `nodeParamsReadByVoxblox()`; setting the
map up needs a ROS master, which it does not start ([Running Tests](../CONTRIBUTING.md#running-tests)).
