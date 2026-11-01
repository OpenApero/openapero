# open_apero_controllers

Stand-in robot controllers for simulated Open Apero experiments: `dummy_controller_node` executes the planner's
trajectories without physics, and `keyboard_teleop_node` sends single-waypoint trajectories by hand.
[`run_experiment_sim.launch`](../open_apero_ros/README.md#run_experiment_simlaunch) starts the dummy controller; a real
robot replaces it with its own controller ([Integrating a Robot](../docs/integrating-a-robot.md)).
[Simulation](../docs/modules/simulation.md#the-stand-in-controllers) describes how both behave.

```bash
rosparam set /initial_position_x 0.0   # and every other /initial_* parameter below: dummy_controller_node requires them
rosrun open_apero_controllers dummy_controller_node _publish_camera_transform:=all _map_frame:=world
rosrun open_apero_controllers keyboard_teleop_node _map_frame:=world
```

## dummy_controller_node

Publishes the initial pose every 0.1 s until the first trajectory arrives, then each waypoint at
`time_from_start / speedup_factor`. It publishes no odometry.

| Topic | Type | Description |
|---|---|---|
| `/command/trajectory` (subscribed) | `open_apero_msgs/RobotTrajectory` | Trajectory to execute; one arriving mid-execution from the same publisher runs after the current one |
| `robot_state` | `open_apero_msgs/RobotState` | Every waypoint, and the initial pose until the first trajectory |
| `command/result` | `open_apero_msgs/CommandResult` | Once per trajectory, after its last waypoint; stamped in `map_frame` |
| `/camera_transform` | `geometry_msgs/TransformStamped` | `camera_optical_frame` in `map_frame` per published state; only with `~publish_camera_transform: all` |
| `/tf` | `tf2_msgs/TFMessage` | `map_frame` → `base_frame`, `camera_frame` and `camera_optical_frame` |

Launch value is what `run_experiment_sim.launch` sets, "scene file" what the
[scene files](../open_apero_ros/README.md#scene-files) set.

| Parameter | Code default | Launch value | Description |
|---|---|---|---|
| `~publish_camera_transform` | `none` | `all` | `all` also publishes each state's optical frame on `/camera_transform` |
| `~speedup_factor` | `1.0` | `5` (`speedup_factor` arg) | Waypoint timing divisor |
| `~map_frame` | `map` | `world` (`map_frame` arg) | Parent frame of every transform and result |
| `~base_frame`, `~camera_frame`, `~camera_optical_frame` | `base`, `camera_frame`, `camera_optical_frame` | `base`, `camera_frame`, `camera_optical_frame` | Base, sensor (x forward, z up) and optical (z forward, y down) frames |
| `/initial_position_x`, `_y`, `_z`, `/initial_quaternion_x`, `_y`, `_z`, `_w` | required | scene file | Initial base pose (position [m], quaternion of norm 1 within 1e-7, or `open_apero_node` drops every state); the sensor starts at the base pose |

## keyboard_teleop_node

Publishes each key's pose as a one-waypoint trajectory on `/command/trajectory` for `dummy_controller_node` to
execute: `w`/`s` 0.5 m forward/back along the heading, `a`/`d` turn 0.1 rad left/right, `r`/`f` 0.2 m up/down,
ESC (alone) or Ctrl-C quits, any other key republishes the current pose. Lowercase only; started by no launch file.

| Parameter | Code default | Description |
|---|---|---|
| `~map_frame` | `map` | `header.frame_id` of the published trajectories |
| `/initial_position_x`, `_y`, `_z`, `/initial_quaternion_x`, `_y`, `_z`, `_w` | `4.0`, `4.0`, `1.0`, `0`, `0`, `0`, `1` | Start pose (position [m], quaternion); only the yaw is used |

## Dependencies

[`open_apero_msgs`](../open_apero_msgs/README.md), `rospy`, `geometry_msgs`, `std_msgs`, `tf2_ros`, SciPy (rotations).

## Tests

```bash
python3 -m pytest test   # from the package root, with the workspace sourced
```
