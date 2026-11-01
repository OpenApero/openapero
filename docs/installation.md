# Installation

How to install Open Apero natively on Ubuntu 20.04 with ROS Noetic, optionally with Habitat-sim and the HSSD dataset for
the simulated experiments, or in [Docker](#docker) on any other x86_64 Linux host. The [FAQ](faq.md#installation) has
the fix of each install problem; [Getting Started](getting-started.md) then runs the graph benchmark and a simulated
experiment.

## Prerequisites

- Ubuntu 20.04 with ROS Noetic (tested on x86_64); on any other OS release, including Ubuntu 22.04 and 24.04, use
  [Docker](#docker) (RoboStack and ROS-from-source builds are not supported)
- A C++17 compiler (GCC 9.4, the Ubuntu 20.04 default, is the tested one) and Python 3.8, the ROS Noetic system Python
- For the simulated experiments only: an NVIDIA GPU and Habitat-sim in its own Python environment

## Workspace and Dependencies

Install ROS Noetic following the [official instructions](https://wiki.ros.org/noetic/Installation/Ubuntu);
`ros-noetic-ros-base` is enough, as rosdep installs RViz and the other ROS dependencies. Then, in a shell with ROS
sourced (`source /opt/ros/noetic/setup.bash`), create the workspace, clone the repository and install the dependencies:

```bash
sudo apt-get update
sudo apt-get install python3-catkin-tools libassimp-dev git wget autoconf libtool \
  ros-noetic-ompl ros-noetic-jsk-rviz-plugins
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
catkin config --extend /opt/ros/noetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

cd ~/catkin_ws/src
git clone --recursive https://github.com/OpenApero/openapero.git open_apero

cd ~/catkin_ws
sudo rosdep init          # once per machine; skip if already done
rosdep update --include-eol-distros   # Noetic is end-of-life
rosdep install --from-paths src --ignore-src -y --rosdistro noetic
```

`--recursive` also fetches the [voxblox fork](https://github.com/OpenApero/voxblox) (`deps/voxblox`) and its vendored
dependencies. Keep no other voxblox in the workspace or its underlays: its messages do not match the fork's.

## OR-Tools Setup

The TSP path planner uses [Google OR-Tools](https://developers.google.com/optimization) v9.11, which has no rosdep key.
Install it before the first build; without it, the CMake configuration of `open_apero_core` stops:

```bash
cd /tmp
wget https://github.com/google/or-tools/releases/download/v9.11/or-tools_amd64_ubuntu-20.04_cpp_v9.11.4210.tar.gz
tar -xzf or-tools_amd64_ubuntu-20.04_cpp_v9.11.4210.tar.gz
mkdir -p ~/catkin_ws/src/open_apero/open_apero_core/ortools
cp -r or-tools_*/{include,lib} ~/catkin_ws/src/open_apero/open_apero_core/ortools/
```

The archive is x86_64, the only tested architecture. arm64, such as a Jetson, is untested: v9.11 has no Linux arm64
archive among the [OR-Tools releases](https://github.com/google/or-tools/releases), so OR-Tools would have to be built
from source.

## Build

The build needs network access, as some of voxblox's vendored dependencies download their sources. The graph
benchmark of [Getting Started](getting-started.md#run-the-graph-benchmark) then checks the install.

```bash
cd ~/catkin_ws
catkin build
source devel/setup.bash   # in every new shell, or add it to ~/.bashrc
```

## Habitat Setup (Optional)

The simulated experiments render with [Habitat](https://aihabitat.org/) in the conda environment of
[`open_apero_habitat/environment.yml`](https://github.com/OpenApero/openapero/blob/main/open_apero_habitat/environment.yml):
Python 3.8 and `habitat-sim` 0.2.3, which is not on PyPI. Any conda distribution creates it, preferably
[Miniforge](https://github.com/conda-forge/miniforge). Keep conda's base environment inactive in new shells, so that
catkin configures packages with the system `python3` ([FAQ](faq.md#cmake-stops-empy-not-found-under-conda)).

```bash
# Miniforge, unless conda is installed: accept its conda init, then open a new shell
wget "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
bash Miniforge3-$(uname)-$(uname -m).sh
conda config --set auto_activate false   # conda before 25.5: auto_activate_base

conda env create -f ~/catkin_ws/src/open_apero/open_apero_habitat/environment.yml
conda activate habitat
# Without a display: switch habitat-sim to its headless (EGL) build
conda install headless --override-channels -c aihabitat -c conda-forge

# Rebuild the Habitat node's launcher on this environment's python3
cd ~/catkin_ws
catkin build open_apero_habitat --no-deps \
  --cmake-args -DPYTHON_EXECUTABLE="$CONDA_PREFIX/bin/python3"
head -1 devel/lib/open_apero_habitat/habitat_renderer_node   # expect #!<env>/bin/python3
```

The rebuild moves only the Habitat node to this Python, and later builds keep it until a `catkin clean`; the next
plain `catkin build` reconfigures every package once. A plain virtual environment works too: build `habitat-sim`
v0.2.3 from source ([instructions](https://github.com/facebookresearch/habitat-sim/blob/v0.2.3/BUILD_FROM_SOURCE.md)),
`pip install` the other packages of `environment.yml`, and rebuild with
`-DPYTHON_EXECUTABLE="$VIRTUAL_ENV/bin/python3"`.

## HSSD Dataset (Optional)

The scene configs find the HSSD dataset through `HSSD_DATA_ROOT`. HSSD-hab is on
[Hugging Face](https://huggingface.co/datasets/hssd/hssd-hab) under CC BY-NC 4.0 (non-commercial use only); the clone
takes about 20 GB, half of it in `.git`.

```bash
cd ~/catkin_ws/src/open_apero
cp datasets.env.example datasets.env   # datasets.env is gitignored
${EDITOR:-nano} datasets.env           # set HSSD_DATA_ROOT
source datasets.env                    # before every roslaunch, or add to ~/.bashrc

sudo apt-get install git-lfs && git lfs install
git clone https://huggingface.co/datasets/hssd/hssd-hab "$HSSD_DATA_ROOT/versioned_data/hssd-hab"
ln -s versioned_data "$HSSD_DATA_ROOT/scene_datasets"   # keep it relative: the Habitat image mounts it elsewhere
```

## Docker

`docker/Dockerfile` runs the steps from Workspace and Dependencies to Build on `ros:noetic-ros-base-focal`;
`docker/Dockerfile.habitat` adds the Habitat environment. From the repository root, with submodules initialized:

```bash
docker build -f docker/Dockerfile -t open_apero .                    # about 5 minutes on 12 cores, 5.0 GB
docker build -f docker/Dockerfile.habitat -t open_apero:habitat .    # optional, for the simulated experiments
docker/run.sh                                                        # shell in the sourced workspace
```

`docker/run.sh` runs as the host user and mounts the host's data root (`$OPEN_APERO_DATA_ROOT`, else
`$HOME/open_apero_data`); `--gpu --image open_apero:habitat` runs the simulated experiments. RViz renders poorly
through the container's X11 forwarding, so with ROS Noetic and this workspace built on the host, launch with
`rviz:=false` and open RViz with `docker/rviz_host.sh` (`OPEN_APERO_WS_SETUP` names the host workspace's
`devel/setup.bash` unless this checkout is its `src/open_apero`); otherwise `--gui` shows the container's RViz
([Run a Simulated Experiment](getting-started.md#run-a-simulated-experiment)). Since the container shares the host's
network, give a run its own [ROS master port](faq.md#a-docker-run-joins-another-ros-master) when another is running;
[`docker/README.md`](https://github.com/OpenApero/openapero/blob/main/docker/README.md) has every option.
