# Simulation

The simulated side of the experiments: the Habitat-sim renderer, the stand-in controllers that execute trajectories
without physics, and the HSSD scenes. Node parameters and topics are in the
[`open_apero_habitat`](https://github.com/OpenApero/openapero/blob/main/open_apero_habitat/README.md) and
[`open_apero_controllers`](https://github.com/OpenApero/openapero/blob/main/open_apero_controllers/README.md)
READMEs; [Installation](../installation.md#habitat-setup-optional) sets up Habitat-sim and the dataset.

## The Habitat Renderer

`habitat_renderer_node` runs in Habitat-sim's own Python environment. For every camera pose the controller publishes
on `/camera_transform` it renders, in arrival order, a depth image (`32FC1`, planar z, in meters), a color image and
the camera info, stamped with the pose's stamp. Once up, it publishes `/open_apero/simulation_ready`, which
`mission_coordinator_node` waits for before it starts the planner.

- **Depth**, in order: NaN and infinite readings become 0, readings above `max_depth` become `far_depth_sentinel`
  (so voxblox clears free space up to the observation boundary), readings below `min_depth` become 0.
- **Intrinsics**: `fx = fy = width / (2 tan(hfov / 2))`, `cx = (width - 1) / 2`, `cy = (height - 1) / 2`.

## The Stand-In Controllers

`dummy_controller_node` publishes the scene's initial pose until the first trajectory arrives, then steps through its
waypoints at `time_from_start / speedup_factor`, publishing each as TF, `robot_state` and, with
`publish_camera_transform: all`, `/camera_transform`, which makes Habitat render a frame. After the last waypoint it
publishes `command/result` with the trajectory's `command_sequence` and final state. `keyboard_teleop_node` drives
by hand: each key moves, turns, raises or lowers the pose and sends it to `dummy_controller_node` as a one-waypoint
trajectory.

## Scenes and the HSSD Dataset

The scenes come from the HSSD dataset (HSSD-hab on Hugging Face, CC BY-NC 4.0, non-commercial use only), found
through `HSSD_DATA_ROOT` (set in `datasets.env`). `open_apero_ros/config/scenes/HSSD/` holds one file per scene, the
paper's eight and a ninth, `103997562`, each setting the stage and the start position; `scene_id` (default
`102816036`) selects one. The README's
[scene files](https://github.com/OpenApero/openapero/blob/main/open_apero_ros/README.md#scene-files) section lists
them, and [Reproducing the Paper](../paper.md#active-perception-experiments) gives the paper's budgets.
