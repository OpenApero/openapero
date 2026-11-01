# open_apero_habitat

RGB-D renderer for simulated Open Apero experiments, built on [AI Habitat (Meta)](https://aihabitat.org/)'s
Habitat-sim: `habitat_renderer_node` renders one frame for every camera pose it receives, in arrival order.
[`run_experiment_sim.launch`](../open_apero_ros/README.md#run_experiment_simlaunch) starts it; a real robot replaces it
with its camera driver. Habitat-sim runs in its own Python environment (conda in the reference setup), not the ROS
Noetic system Python: install it as in [Installation](../docs/installation.md#habitat-setup-optional) or build the
[Habitat image](../docker/README.md#habitat-image). [Simulation](../docs/modules/simulation.md) explains the depth
processing and the intrinsics.

```bash
conda activate habitat && source ~/catkin_ws/devel/setup.bash && source ~/catkin_ws/src/open_apero/datasets.env
roslaunch open_apero_ros run_experiment_sim.launch scene_id:=102816852   # starts this node
```

## habitat_renderer_node

Published topics are latched, the images and camera info stamped with the incoming transform's stamp in the frame
`camera_optical_frame`; the names are the defaults of the `*_topic` parameters.

| Topic | Type | Description |
|---|---|---|
| `/camera_transform` (subscribed) | `geometry_msgs/TransformStamped` | `camera_optical_frame` in `map_frame`; renders one frame. Other frames are dropped |
| `/open_apero_camera/depth_image` | `sensor_msgs/Image` | Depth, `32FC1` [m], planar z |
| `/open_apero_camera/rgb_image` | `sensor_msgs/Image` | Color, `rgb8` |
| `/open_apero_camera/camera_info` | `sensor_msgs/CameraInfo` | Pinhole intrinsics, no distortion |
| `/open_apero/simulation_ready` | `std_msgs/String` | Once the simulator is up; [`mission_coordinator_node`](../open_apero_ros/README.md#mission_coordinator_node) waits for it |

Private parameters; Launch value is what `run_experiment_sim.launch` sets (unset = not set by it).

| Parameter | Code default | Launch value | Description |
|---|---|---|---|
| `width`, `height` | `480`, `640` | `480`, `640` | Image size [px] |
| `hfov` | `42.0` | `42.0` | Horizontal field of view [deg] |
| `color_sensor`, `depth_sensor` | `true` | unset | Render and publish RGB, depth |
| `min_depth` | `0.0` | unset | Readings below it become 0 (invalid) [m] |
| `max_depth` | `5.0` | `3.0` (`ray_length` arg) | Largest valid reading [m] |
| `far_depth_sentinel` | `6.0` | `10.0` (`far_depth_sentinel` arg) | Written for readings beyond `max_depth` [m] |
| `model_type` | `ground_truth` | unset | Depth model; `ground_truth` is the only one |
| `scene_id` | `102816036` | `scene_id` arg | Scene to load; an HM3D id (`<5 digits>-<11 characters>`, the whole value) resolves to `<mesh_folder>/<id>/<11 characters>.glb`, anything else goes to Habitat unchanged |
| `scene_dataset_config_file` | `/scene_dataset_config_file`, else empty | unset (global from `config/scenes/HSSD/_common.yaml`) | Habitat dataset config that resolves `scene_id` |
| `mesh_folder` | `/mesh_folder`, else empty | unset | Scene folder for HM3D ids |
| `camera_optical_frame` | `camera_optical_frame` | `camera_optical_frame` | Required child frame of incoming transforms |
| `map_frame` | `map` | `world` (`map_frame` arg) | Required parent frame of incoming transforms |
| `rgb_image_topic`, `depth_image_topic`, `camera_info_topic`, `camera_transform_topic`, `simulation_ready_topic` | The topics above | unset | Topic names |

## Dependencies

`habitat-sim` 0.2.3 (brings NumPy and SciPy) in the conda environment of [`environment.yml`](environment.yml); `rospy`,
`std_msgs`, `sensor_msgs`, `geometry_msgs`.

## Tests

```bash
python3 -m pytest test   # from the package root, with the workspace sourced; habitat_sim not needed
```
