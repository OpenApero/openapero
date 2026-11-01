# open_apero (metapackage)

Catkin metapackage that groups every Open Apero package; it holds no code. See the
[top-level README](../README.md) and the [documentation](https://openapero.github.io/openapero/).

```bash
catkin build open_apero   # builds every package below and their dependencies
```

## Dependencies

`open_apero_core`, `open_apero_voxblox`, `open_apero_msgs`, `open_apero_ros_bridge`, `open_apero_ros`,
`open_apero_controllers`, `open_apero_habitat`, `open_apero_analysis` and `open_apero_graph_benchmark`.

## Tests

None; each package runs its own ([Running Tests](../CONTRIBUTING.md#running-tests)).
