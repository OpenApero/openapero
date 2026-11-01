# Integrating a Robot

How to run `open_apero_node` on a robot of your own: the nodes to keep, what the robot provides and what to configure.
[ROS Node, Launch Files and Messages](modules/ros.md) describes the node.

Start from a copy of `run_experiment_sim.launch`: keep `voxblox_node`, `open_apero_node` and
`mission_coordinator_node`, and drop the HSSD scene-file loads, `habitat_renderer_node`, `dummy_controller_node` and,
unless an RGB-D camera publishes exactly synchronized camera info, RGB and 32-bit float depth,
`rgbd_to_pointcloud_node`. Give the coordinator `<param name="wait_for_simulation" value="false"/>`, or it waits for
Habitat and ends the launch after 30 s.

## What the Robot Provides

- Commands: execute each `/command/trajectory` (`open_apero_msgs/RobotTrajectory`, in `map_frame`, a launch argument,
  `world` by default). Its states hold the poses of `base_frame` and `camera_frame` (x forward, z up) and a
  `time_from_start`, `robot.time_step` (0.2 s) apart; a planning command has at most `replan_num_steps` + 1 (6 shipped)
  of them. The next command comes once this one is done.
- Pose with `use_tf: false` (shipped, for a lockstep simulator): publish `/robot_state` (`open_apero_msgs/RobotState`,
  the same two poses) while executing a command and, until the first one, the initial pose, which
  `dummy_controller_node` repeats every 0.1 s. Each message counts as the outcome of the latest command.
- Pose with `use_tf: true` (for an asynchronous robot): publish TF `map_frame` → `base_frame` and `map_frame` →
  `camera_frame`, and after each command one `/command/result` (`open_apero_msgs/CommandResult`) with its
  `command_sequence`, `header.frame_id` = `map_frame` and the final state. TF alone never completes a command.
- Point clouds: publish `/open_apero_camera/point_cloud` (`sensor_msgs/PointCloud2` in `camera_optical_frame`) and
  `/camera_transform` (`geometry_msgs/TransformStamped`, that frame in `map_frame`) with identical stamps, or set
  `voxblox_node`'s `use_tf_transforms` to `true` and remap its `pointcloud` input to the robot's cloud. Give pixels
  with no return as points beyond `ray_length`, like Habitat's `far_depth_sentinel`: only these clear free space along
  rays that hit nothing.

A command is done once the sensor pose is within `target_position_threshold` (0.10 m) and `target_angle_threshold`
(0.20 rad) of its last state and, with `wait_for_map_updated: true`, a cloud taken there in the optical frame of
`camera_frame` is mapped ([The Settle Gate](modules/agent.md#the-settle-gate)). For a cloud in another frame, such as
a LiDAR's, set `wait_for_map_updated: false`; for a robot that cannot track within the thresholds, raise them. Poses
with a non-unit quaternion are dropped. Of a `RobotState` on `/robot_state` or `/command/result`, the planner keeps the
camera pose, roll zeroed, and re-derives the base from it through the robot model's sensor mount.

## Adapt the Configuration

- `robot` in `config/planners/_common.yaml`: velocity limits, collision spheres, `sample_z: false` for a ground robot
  and, with `use_tf: false`, the sensor mount ([Robot Models](modules/robot-models.md)).
- `/map_bounding_volume` (where the robot may plan) and `/target_bounding_volume` (where gain counts), boxes in
  `map_frame` ([Configuration Files](modules/ros.md#configuration-files)).
- `sensor_model` in `config/tasks/_common.yaml`: the camera's resolution and focal length. A LiDAR takes
  `IterativeRayCasterLidar`, which is spherical: give it `resolution_x`/`_y` (rays) and `field_of_view_x`/`_y` [deg]
  instead of `focal_length` and `observation_boundary`, and drop the launch's `sensor_model/observation_boundary`
  param, or they are reported unread ([Sensor Models](modules/evaluators.md#sensor-models)). The launch arguments
  `ray_length` and `observation_boundary` set the camera's range and `voxblox_node`'s; set them to what the robot's map
  integrates with.

## Start and Stop

The coordinator creates the run directory, sets the planner's `~run_dir` and runs it until `time_limit` minutes pass or
a message arrives on `/open_apero/collision` (`std_msgs/String`), the hook for a bumper or safety node. To start the
planner by hand, give it an existing, empty run directory first, which it requires with `log_data:=true` (default):

```bash
mkdir -p ~/open_apero_data/my_robot/run_01
rosparam set /open_apero_node/run_dir ~/open_apero_data/my_robot/run_01
rosservice call /open_apero_node/toggle_running "data: true"   # false stops, writes the summary
```

The [`open_apero_node` reference](https://github.com/OpenApero/openapero/blob/main/open_apero_ros/README.md#open_apero_node)
lists every topic, service and parameter. A platform without ROS derives its own agent from `Agent` and implements
`sendMovement()` and `publishVisualization()` ([The Loop](modules/agent.md#the-loop)).
