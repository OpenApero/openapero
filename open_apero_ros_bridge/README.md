# open_apero_ros_bridge

Header-only adapters between `open_apero_core` types and ROS, for a C++ node that publishes core markers, exchanges
robot states with [`open_apero_msgs`](../open_apero_msgs/README.md) or routes core logging to rosconsole without
depending on the heavier [`open_apero_ros`](../open_apero_ros/README.md). Used by `open_apero_node`,
`benchmark_node` and `generate_graph_instance_node`.

```cpp
#include <open_apero_ros_bridge/log_level.h>
#include <open_apero_ros_bridge/robot_state_conversion.h>

ros::init(argc, argv, "my_node");
ros::NodeHandle nh_private("~");
open_apero::installRosconsoleSink();           // before anything logs
open_apero::setLogLevelFromParam(nh_private);  // reads ~log_level

open_apero::RobotState state;
open_apero_msgs::RobotState msg;
open_apero::ros::robotStateToMsg(state, &msg);
```

## Headers

| Header | Functions | Needs beyond `open_apero_core` |
|---|---|---|
| `eigen_conversion.h` | `vectorEigenToMsg`, `vectorEigenFromMsg` (`Vector3` and `Point`), `quaternionEigenToMsg`, `quaternionEigenFromMsg` | `geometry_msgs` |
| `marker_conversion.h` | `visualizationMarkerToMsg` (one marker), `visualizationMarkersToMsg` (appends to `msg->markers`, leaving out an ADD of a point-list marker without points, which RViz rejects) | `visualization_msgs`, `std_msgs`, `geometry_msgs` |
| `robot_state_conversion.h` | `robotStateToMsg`; `robotStateFromMsg` from a `RobotState` or a base and a sensor `Transform`, `false` for a non-finite or non-unit quaternion (never normalizes) | `open_apero_msgs`, `geometry_msgs` |
| `log_level.h` | `installRosconsoleSink` (core logging to rosconsole and `/rosout`; without it the core prints to `std::cout`/`std::cerr`), `setLogLevelFromParam` (rosconsole's and the core's level from `~log_level`: `debug`, `info`, `warn`, `error`, `fatal`; default `info`) | `roscpp` |
| `status_overlay.h` | `buildStatusOverlay` (the RViz status box of `open_apero_node` and `benchmark_node`) | `jsk_rviz_plugins` |

## Dependencies

[`open_apero_core`](../open_apero_core/README.md), [`open_apero_msgs`](../open_apero_msgs/README.md), `eigen_catkin`,
`roscpp`, `geometry_msgs`, `std_msgs`, `visualization_msgs`, `jsk_rviz_plugins`.

## Tests

None of its own: `test_open_apero_ros` covers `marker_conversion.h` and `status_overlay.h`; the other headers are
exercised only through the nodes.
