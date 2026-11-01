# open_apero_msgs

ROS messages (no services) exchanged between the Open Apero planner and the robot or simulator; any node that talks to
[`open_apero_node`](../open_apero_ros/README.md) needs them.
[`open_apero_ros_bridge`](../open_apero_ros_bridge/README.md) converts `RobotState` to and from the core type.

| Message | Content | Published by | Consumed by |
|---|---|---|---|
| `RobotState` | One robot state | `dummy_controller_node` on `robot_state` | `open_apero_node` (with `use_tf: false`) |
| `RobotTrajectory` | A trajectory to execute, one `RobotState` per waypoint | `open_apero_node` on `command/trajectory`; `keyboard_teleop_node` | `dummy_controller_node` |
| `CommandResult` | The final state of one completed `RobotTrajectory` | `dummy_controller_node` on `command/result` | `open_apero_node` (with `use_tf: true`) |
| `CollectiblePoint`, `CollectiblePointArray` | Point-collection points, drawn as rings in RViz | `PointsEvaluator` on `collectible_points` | `collectible_points_visualization_node` |

## RobotState.msg

`open_apero_node` drops a message whose quaternions are not finite with norm 1 ± 1e-7; it never normalizes, so a
rounded quaternion such as (0, 0, 0.7071, 0.7071) is dropped.

| Field | Type | Meaning |
|---|---|---|
| `base_transform` | `geometry_msgs/Transform` | Base pose in the map frame |
| `sensor_transform` | `geometry_msgs/Transform` | Sensor (camera) pose in the map frame; its x axis is the viewing direction |
| `joint_position` | `float64[]` | Joint positions; empty, as no shipped robot model sets them |
| `time_from_start` | `duration` | Time of this state within its trajectory; 0 for the first point of the planner's trajectories |

## RobotTrajectory.msg

| Field | Type | Meaning |
|---|---|---|
| `header` | `Header` | `frame_id`: the map frame of every point |
| `command_sequence` | `uint32` | Correlation ID, echoed by the `CommandResult`; unlike `Header.seq` not rewritten by ROS. `0`: no result expected |
| `points` | `RobotState[]` | Waypoints, in order of increasing `time_from_start` |

## CommandResult.msg

| Field | Type | Meaning |
|---|---|---|
| `header` | `Header` | `frame_id` must equal the planner's `map_frame`; a result in another frame is dropped |
| `command_sequence` | `uint32` | The `command_sequence` of the trajectory this result completes |
| `state` | `RobotState` | Robot state after the last waypoint |

## CollectiblePoint.msg

| Field | Type | Meaning |
|---|---|---|
| `id` | `int32` | Index of the point among those loaded inside the bounding volume |
| `x`, `y`, `z` | `float64` | Position in the map frame [m] |
| `radius` | `float64` | Collection radius [m] |
| `gain` | `int32` | Gain awarded when the point is collected |
| `status` | `int32` | `UNSEEN` (0) until the voxel at the point is mapped free, then `OBSERVED` (1); `COLLECTED` (2), from either, once an executed waypoint puts the sensor within `radius` |

`CollectiblePointArray.msg` has one field, `points` (`CollectiblePoint[]`): every point of the episode.

## Dependencies

`std_msgs`, `geometry_msgs`; `message_generation` to build, `message_runtime` to run. No tests: the definitions hold no
code.
