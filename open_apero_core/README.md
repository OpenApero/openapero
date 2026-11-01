# open_apero_core

The planning library of Open Apero: the planning cycle (`Agent`), the layers, the plugin interfaces and registry, and
the built-in plugins, in C++17 with no ROS runtime dependency (it builds with catkin). `open_apero_ros` runs it under
ROS; any other program can drive it through the in-memory `PluginFactoryConfig`
([Testing Without ROS](../docs/extending.md#testing-without-ros)). [How Open Apero Works](../docs/concepts.md) explains
the cycle, [Extending Open Apero](../docs/extending.md) how to add a plugin, the
[Parameter Reference](../docs/parameters.md) every key read and the
[API Reference](https://openapero.github.io/openapero/api/index.html) the classes.

```cmake
# CMakeLists.txt of a package that builds on the core (package.xml: <depend>open_apero_core</depend>)
find_package(catkin REQUIRED COMPONENTS open_apero_core)
```

## Contents

Headers under `include/open_apero_core/`, sources under `src/`:

| Directory | Contents |
|---|---|
| `agent/` | `Agent` (runs the cycle; owns the map, robot model, cost computer and `HistoryData`), `AgentInterface`, `BenchmarkAgent` |
| `plugin/` | `Plugin`, `Configurable`, `PluginRegistry`, `PluginFactory`, `PluginFactoryConfig`, `loadPluginLibrary()` |
| `plugin/<role>/` | Each role's base class and built-in types: `generator/`, `evaluator/`, `path_planner/`, `sensor_model/`, `robot_model/`, `cost_computer/`, `backtracker/`, `layer/`, `map/` (the Voxblox backend is [`open_apero_voxblox`](../open_apero_voxblox/README.md)) |
| `graph/` | `Node`, `TreeNode`, `GraphNode`, `Edge`, `Path`, `PathSteps`, `PathLedger`, the KD-tree |
| `state/` | `RobotState`, `RobotTrajectory`, `HistoryData`, `ObservedViewpoints` |
| `gain/`, `geometry/` | `GainAnchor`, `GainCostTable`, gain suppression; `Transform3d`, bounding volumes, collision spheres, primitives |
| `logging/` | `Metrics`, `PerformanceLog`, `RunManifest`, `EpisodeSummary`, `ScopedTimer`, the logging sink |
| `visualization/` | Middleware-free markers, converted to ROS by `open_apero_ros_bridge` |
| `common/`, `third_party/` | Frames, hashing, seeding, utilities; nanoflann and tinycolormap, vendored unchanged |

## Dependencies

- `eigen_catkin` (linear algebra), a catkin package
- Boost (Boost.Graph), OMPL (the fallback planner of `SimpleFallbackRobot`) and Assimp (mesh loading for bounding
  volumes), installed through rosdep
- OR-Tools v9.11 (the TSP solver), copied by hand under `open_apero_core/ortools/` before the first build
  ([Installation](../docs/installation.md#or-tools-setup))
- GoogleTest, for the tests only

## Tests

```bash
catkin build open_apero_core --no-deps --catkin-make-args run_tests --   # from the workspace root
```

`test_open_apero_core` (unit tests) and `test_open_apero_core_plugins` (plugin contracts, including the conformance
test of every registered core type) need no ROS ([Running Tests](../CONTRIBUTING.md#running-tests)).
