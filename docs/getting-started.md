# Getting Started

Run the graph benchmark, which needs no simulator and no dataset, then a simulated experiment, and read what a run
writes. [Installation](installation.md) covers the native install and the Habitat and HSSD setup.

## Run the Graph Benchmark

With Docker, on any x86_64 Linux host (BuildKit, the default since Docker 23.0; the build takes about 5 minutes and
5.0 GB):

```bash
git clone --recursive https://github.com/OpenApero/openapero.git open_apero
cd open_apero
docker build -f docker/Dockerfile -t open_apero .
docker/run.sh -- roslaunch open_apero_graph_benchmark run_benchmark.launch \
  graph_type:=op_5m_25x25_clustered_gain_0to100_cost_50 replan_type:=everystep objective:=expected_gain
```

In a native install, run the same `roslaunch` after `source ~/catkin_ws/devel/setup.bash`.

It runs NBS, DBS, SPT and TSP at three settings each on five graph instances, 60 runs in under a minute, and writes
results that match the paper's to `~/open_apero_data/graph_benchmark/` (with Docker, on the host). The red
`REQUIRED process [benchmark_node-...] has died!` banner at the end is the normal exit. [Graph
Benchmark](modules/graph-benchmark.md) explains the benchmark and [Reproducing the Paper](paper.md) the full sweep.

## Run a Simulated Experiment

This needs an NVIDIA GPU, [Habitat-sim](installation.md#habitat-setup-optional) and the
[HSSD dataset](installation.md#hssd-dataset-optional). In a native install:

```bash
conda activate habitat
source ~/catkin_ws/devel/setup.bash
source ~/catkin_ws/src/open_apero/datasets.env
roslaunch open_apero_ros run_experiment_sim.launch planner:=nbs
```

With Docker, build the Habitat image on top of `open_apero`; the runs need `HSSD_DATA_ROOT` set on the host and the
NVIDIA Container Toolkit:

```bash
docker build -f docker/Dockerfile.habitat -t open_apero:habitat .
```

We recommend RViz on the host, since through the container's X11 forwarding it renders poorly. This needs ROS Noetic
and this workspace built natively on the host (in practice Ubuntu 20.04). Start the run with `rviz:=false`, which keeps
the visualization topics, and open RViz on the container's ROS master (the container shares the host network).
`OPEN_APERO_WS_SETUP` names the host workspace's `devel/setup.bash`; leave it out when this checkout is that
workspace's `src/open_apero`:

```bash
docker/run.sh --gpu --image open_apero:habitat -- \
  roslaunch open_apero_ros run_experiment_sim.launch planner:=nbs rviz:=false
OPEN_APERO_WS_SETUP=~/catkin_ws/devel/setup.bash docker/rviz_host.sh    # in a second terminal on the host
```

Without ROS on the host, show the container's own RViz on the host display instead:

```bash
xhost +local:docker
docker/run.sh --gui --gpu --image open_apero:habitat -- \
  roslaunch open_apero_ros run_experiment_sim.launch planner:=nbs
```

The run ends when its cost budget (600 s of trajectory time) is spent, when no layer offers a step, or `time_limit` (10)
minutes after the planner starts; either way, the red `REQUIRED process [...] has died!` banner is the normal exit.
`planner:=` picks one of the six [planners](modules/path-planners.md#the-shipped-planners), `task:=` the gain measure
(`explore`, `recon` or `points`) and `scene_id:=` the scene; the [`open_apero_ros`
README](https://github.com/OpenApero/openapero/blob/main/open_apero_ros/README.md#run_experiment_simlaunch) lists every
argument.

## Reading a Run's Results

Each run writes a directory `~/open_apero_data/HSSD/102816036/nbs/<YYYYmmdd_HHMMSS>/`:

- `episode_summary.yaml`: how the episode ended and its totals; the result is `real_accumulated_gain` (m³, m² or
  the summed gain of the collected points, by task) against `total_cost`, the budget spent.
- `performance_log.csv`: one row per planning cycle; `TotalCost` against `planning.RealAccumulatedGain` is the
  gain-versus-cost curve.
- `run_manifest.yaml` and `rosparams.yaml`: what produced the run.
- `rviz_screenshot.png`: with RViz beside the planner, its view when the budget ran out or the run was stopped (such as
  at `time_limit`).

Two scripts plot them (Matplotlib and seaborn; no display needed but for `--open`, which in Docker finds no viewer: the
figures are on the host):

```bash
# Gain-versus-cost curves of every planner on the scene
rosrun open_apero_ros plot_performance_curves.py HSSD/102816036 --open
# CPU-time breakdown of the newest nbs run
rosrun open_apero_ros evaluate_experiment.py "${OPEN_APERO_DATA_ROOT:-$HOME/open_apero_data}/HSSD/102816036/nbs" --open
```

Figures go below `~/open_apero_data/figures/`. [Run Output and Analysis](modules/run-output.md) describes every file
and column, and the Python API for your own analysis.

## Next Steps

- [How Open Apero Works](concepts.md): the core design.
- [Integrating a Robot](integrating-a-robot.md): run the planner on your own platform.
- [Extending Open Apero](extending.md): add a planner, a gain measure or a robot model.
