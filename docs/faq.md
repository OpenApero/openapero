# FAQ and Troubleshooting

Known problems with installing, running and benchmarking Open Apero, each with its cause and fix.

## Installation

### `rosdep install` cannot locate a ROS package

`Cannot locate rosdep definition for [<ROS package>]`: rosdep was updated without the end-of-life Noetic distro. Run
`rosdep update --include-eol-distros`, then `rosdep install` again.

### `rosdep install` cannot locate `eigen_catkin`, `catkin_simple` or `voxblox`

The repository was cloned without `--recursive`, so the voxblox fork and its vendored dependencies, which have no
rosdep keys, are missing: `git -C ~/catkin_ws/src/open_apero submodule update --init --recursive`.

### CMake stops: OR-Tools is not installed

`open_apero_core` links `open_apero_core/ortools/lib/libortools.so` directly. Copy OR-Tools as in
[OR-Tools Setup](installation.md#or-tools-setup), then run `catkin build` again.

### CMake stops: `empy` not found under conda

`Unable to find either executable 'empy' or Python module 'em'`: the package was configured with the `python3` of
conda's auto-activated base environment. Run `conda deactivate`, remove `~/catkin_ws/build/<pkg>`, which keeps that
`python3`, and run `catkin build` again; `conda config --set auto_activate false` (conda before 25.5:
`auto_activate_base`) keeps base inactive in new shells, but not in the current one.

### `open_apero_voxblox` does not compile, or a voxblox connection is dropped

A missing `getUpdatedMapRegion`, or an md5sum mismatch on `voxblox_msgs/Layer`, means an upstream voxblox is in the
workspace or an underlay. Remove it; the fork ships as the `deps/voxblox` submodule.

### roslaunch aborts: `HSSD_DATA_ROOT` is not set

The scene files read the dataset location from it. Run `source datasets.env`
([HSSD Dataset](installation.md#hssd-dataset-optional)).

### The Habitat node cannot import `habitat_sim`

`ModuleNotFoundError: No module named 'habitat_sim'`: the first line of
`~/catkin_ws/devel/lib/open_apero_habitat/habitat_renderer_node` names `/usr/bin/python3`, not the Habitat
environment's `python3`. Rebuild `open_apero_habitat` as in [Habitat Setup](installation.md#habitat-setup-optional).

### The Habitat node exits: `unable to find EGL device for CUDA device 0`

habitat-sim's CUDA build cannot initialize CUDA, so it finds no GPU to render with. Check CUDA with `nvidia-smi` and,
where PyTorch is installed, `python3 -c "import torch; print(torch.cuda.is_available())"`; if either fails, reload the
`nvidia_uvm` module (`sudo rmmod nvidia_uvm && sudo modprobe nvidia_uvm`) or reboot.

### `pip install habitat-sim` finds no package

`habitat-sim` has no Linux build on PyPI; install 0.2.3 from conda as in
[Habitat Setup](installation.md#habitat-setup-optional).

## Running

| Symptom | Cause and fix |
|---------|---------------|
| A red `REQUIRED process [...] has died!` with `process has finished cleanly` at the end | The normal end of a graph-benchmark sweep or a simulated run: the node is `required`, so roslaunch exits with it |
| The planner waits and logs `[settle]` warnings | The [settle gate](modules/agent.md#the-settle-gate) has not passed; the warning names what is missing. For a sensor whose cloud is not in the camera's optical frame, such as a LiDAR, set `wait_for_map_updated: false` ([Integrating a Robot](integrating-a-robot.md)) |
| `toggle_running` refuses to start the planner | With `log_performance` on, `open_apero_node` needs a writable `~run_dir`; set it first, as in [Start and Stop](integrating-a-robot.md#start-and-stop) |
| `<type> ignores parameters: a, b` | The plugin's setup did not read these keys of its namespace: usually a misspelt key or a key of another type; `strict_params: true` makes it fatal ([How Parameters Are Read](parameters.md#how-parameters-are-read)) |
| `rosparam load` leaves `$(env …)` in a parameter | The configs are roslaunch-templated; load them through the launch files ([Configuration Files](modules/ros.md#configuration-files)) |
| A graph-benchmark dataset does not reproduce after a partial rebuild | A `benchmark_node` older than `libopen_apero_core.so` can disagree through a stale vtable: `catkin build` both packages and rerun ([If a Dataset Does Not Reproduce](paper.md#if-a-dataset-does-not-reproduce)) |

### A Docker run joins another ROS master

The container shares the host's network, so it joins a master already on port 11311. Give the run its own port with
`roslaunch -p 11411 ...` and `export ROS_MASTER_URI=http://localhost:11411` for the other ROS tools in that shell. The
batch tools pick their own: `paper_sweep.py` from `--master-port-base`, `run_experiments.sh` from `SWEEP_MASTER_URI`.
