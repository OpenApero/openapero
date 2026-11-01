# Docker

ROS Noetic is released for Ubuntu 20.04; on any other x86_64 Linux host, this image is the supported way to build and
run Open Apero. It runs the steps of [Installation](../docs/installation.md), then builds the workspace.

## Build

From the repository root, with submodules initialized (`git submodule update --init --recursive`) and BuildKit, the
default builder since Docker 23.0:

```bash
docker build -f docker/Dockerfile -t open_apero .    # about 5 minutes on 12 cores, 5.0 GB
```

| Build arg | Default | Meaning |
|---|---|---|
| `ROS_IMAGE` | `ros:noetic-ros-base-focal` | base image of every stage |
| `ORTOOLS_URL` | OR-Tools v9.11, Ubuntu 20.04 amd64 | prebuilt OR-Tools tarball, x86_64 (arm64 is untested: [OR-Tools Setup](../docs/installation.md#or-tools-setup)); `--build-context ortools=DIR` with it as `DIR/ortools.tar.gz` replaces the download |
| `WS` | `/root/catkin_ws` | workspace in the image, exported as `$WS` |
| `BUILD_TYPE` | `RelWithDebInfo` | `CMAKE_BUILD_TYPE` |
| `SLIM` | `true` | drop `build/` and strip binaries; `false` keeps `build/` for incremental rebuilds (7.2 GB) |
| `JOBS` | `nproc` | parallel build jobs |

## Run

```bash
docker/run.sh                                  # shell in the sourced workspace
docker/run.sh -- roslaunch open_apero_graph_benchmark run_benchmark.launch \
  graph_type:=op_5m_25x25_clustered_gain_0to100_cost_50 replan_type:=everystep objective:=expected_gain
```

`run.sh` runs `open_apero:latest` (`--image NAME` for another tag) as the host user, in a container removed on exit,
and mounts the host's data root (`$OPEN_APERO_DATA_ROOT`, else `$HOME/open_apero_data`) at `~/open_apero_data`, where
runs write their output; `--root` runs as root, e.g. to rebuild the workspace. Sharing the host's network, a run joins
a ROS master already on port 11311 unless it takes [its own port](../docs/faq.md#a-docker-run-joins-another-ros-master).

### RViz on the host

RViz in the container renders poorly through X11 forwarding. RViz on the host needs ROS Noetic and this workspace built
on the host (in practice Ubuntu 20.04): start the experiment with `rviz:=false`, which keeps the visualization topics,
and run `docker/rviz_host.sh` on the host, with `OPEN_APERO_WS_SETUP` naming the host workspace's `devel/setup.bash`
unless this checkout is its `src/open_apero`. It opens `experiment.rviz` on the container's master (`localhost:11311`
or `$ROS_MASTER_URI`), and the end-of-run RViz screenshot is not saved. Without Noetic and the workspace on the host,
`xhost +local:docker` and `docker/run.sh --gui ...` show the container's own RViz on the host display.

## Habitat image

`Dockerfile.habitat` adds headless habitat-sim to `open_apero:latest` (build arg `BASE_IMAGE`), in the conda environment
of [`open_apero_habitat/environment.yml`](../open_apero_habitat/environment.yml) (`MINIFORGE_URL` picks the Miniforge
installer), and rebuilds `open_apero_habitat` on it. `--gpu` needs an NVIDIA GPU and the NVIDIA Container Toolkit on the
host, mounts `$HSSD_DATA_ROOT` read-only (its `scene_datasets` link must be relative) and does not select the image. GPU
rendering in the container is tested with NVIDIA Container Toolkit 1.20.

```bash
docker build -f docker/Dockerfile.habitat -t open_apero:habitat .   # 2 minutes, 1.6 GB more
docker/run.sh --gpu --image open_apero:habitat -- \
  roslaunch open_apero_ros run_experiment_sim.launch planner:=nbs rviz:=false   # RViz on the host, see above
```

## Test

What CI's [`docker.yml`](../.github/workflows/docker.yml) runs:

```bash
docker run --rm open_apero bash -c 'cd $WS &&
  OWNED="open_apero_analysis open_apero_controllers open_apero_core open_apero_ros open_apero_graph_benchmark open_apero_habitat open_apero_voxblox" &&
  catkin build $OWNED --no-deps --no-status --catkin-make-args run_tests -- &&
  for pkg in $OWNED; do catkin_test_results "build/$pkg" || exit 1; done'
```
