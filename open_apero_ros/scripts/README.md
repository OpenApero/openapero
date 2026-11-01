# Active-perception scripts

Hand-run tools for the simulated active-perception experiments: the point-collection input, per-run evaluation,
batch sweeps of [`run_experiment_sim.launch`](../README.md#run_experiment_simlaunch), and the figures of the paper
([Reproducing the Paper](../../docs/paper.md)). The graph-benchmark tools are in
[`open_apero_graph_benchmark/scripts`](../../open_apero_graph_benchmark/scripts/README.md).

Run them with the workspace sourced; each Python tool's `--help` lists every flag, and its code is in
`python/open_apero_ros/<tool>.py`. Every Python tool but `paper_sweep.py` and `compare_paper_sweep.py`, which need only
the standard library, imports [`open_apero_analysis`](../../open_apero_analysis/README.md); the plotting tools need
NumPy, Matplotlib and seaborn, but no display. Every output goes below the data root, `$OPEN_APERO_DATA_ROOT` or
`~/open_apero_data`; the figure tools write below `<data root>/figures/` at their input's path, with a `.yaml` record
beside each figure ([Figures](../../docs/modules/run-output.md#figures)), and take `--out-dir` and `--open`.

| Tool | Purpose | Output |
|---|---|---|
| `generate_points.py` | Random point-collection input in the CSV format `PointsEvaluator` reads | `<data root>/points_3d_temp.csv` |
| `evaluate_experiment.py` | CPU-time performance overview of finished runs | `<data root>/figures/<run>/performance_overview.png` |
| `plot_performance_curves.py` | Mean gain-vs-cost curves per planner for one scene, with a ranked per-run table | `<data root>/figures/<scene>/curves_<y axes>_vs_<x axis>[_top<k>][_<planners>].pdf`, `_top<k>` with `--top-k`, the planners named when `--planners` differs from the shipped six |
| `plot_normalized_gain.py` | The paper's normalized-gain bar chart, from the published numbers kept in its module | `<data root>/figures/normalized_gain_barplots.pdf` |
| `run_experiments.sh` | Batch sweep of `run_experiment_sim.launch`, one `gnome-terminal` per run | Run directories, plus `$DATA_ROOT/sweep_logs/<stamp>/` |
| `paper_sweep.py` | Headless replication of the paper's active-perception sweep over parallel ROS masters | Run directories under `<data-root>/<task>/HSSD/`, plus `runs.csv`, `sweep.log`, `logs/` |
| `compare_paper_sweep.py` | A `paper_sweep.py` data root beside the paper's means and ground truths | Markdown on stdout, and in `--output` |

## `generate_points.py` and `evaluate_experiment.py`

```bash
rosrun open_apero_ros generate_points.py --num-points 5000 --seed 7 --no-show
rosrun open_apero_ros evaluate_experiment.py "${OPEN_APERO_DATA_ROOT:-$HOME/open_apero_data}/HSSD/102816852/nbs" \
  --method all
```

The points file has `x,y,z,radius,gain` rows and is not the tracked `data/points_3d.csv`, which every `task:=points` run
reads: inspect it before copying it over. `--method` reads the directory as one run (`single`), its newest run
(`recent`, the default) or every run (`all`).

## `plot_performance_curves.py`

```bash
rosrun open_apero_ros plot_performance_curves.py HSSD/102816852 --y-axes "Accumulated Gain" "Num Nodes"
rosrun open_apero_ros plot_performance_curves.py recon/HSSD/102816852 \
  --data-root ~/open_apero_data/paper_full --ground-truth 725
```

The scene folder, below `--data-root`, holds one directory per planner: `HSSD/<scene>` for runs of
`run_experiment_sim.launch`, `<task>/HSSD/<scene>` for a `paper_sweep.py` root. The tool reads the six shipped planners,
or those `--planners` names, such as a planner of your own, and plots every run there on one gain axis; as
`run_experiment_sim.launch` puts every task's runs in the same `HSSD/<scene>/<planner>`, give each task its own data
root, e.g. `data_root:=<data root>/recon`, and plot `recon/HSSD/<scene>`, which also labels the axis. On the default
`--x-axis Cost`, a run whose costs do not span 50, where the curves start, is skipped with a `[skip]` line.
`--ground-truth` adds a hand-supplied legend entry; `--help` lists the quantities `--x-axis` and `--y-axes` take.

## `plot_normalized_gain.py`

```bash
rosrun open_apero_ros plot_normalized_gain.py --open
```

The values in `TASK_PERFORMANCE` are the paper's measurements rounded to two decimals, not an input to recompute.
The FUEL bars are labeled "paper only": FUEL is not part of this release.

## `run_experiments.sh`

```bash
PLANNERS="nbs dbs" SCENE_SET=small TRIALS=3 rosrun open_apero_ros run_experiments.sh
TASK=points COST_BUDGET=300 rosrun open_apero_ros run_experiments.sh
```

Each run opens its own `gnome-terminal`, so it needs a desktop session with `gnome-terminal` and util-linux `script`;
`paper_sweep.py` is the headless alternative. It is configured by environment variables:

| Variable | Default | Meaning |
|---|---|---|
| `PLANNERS` | `nbs dbs spt tsp rrtstar rrt` | Space-separated planners |
| `SCENE_SET`, `SCENES` | `large` | `large` (5 scenes), `small` (3) or `all`; `SCENES` lists scene IDs instead |
| `TRIALS` | `15` | Runs per (scene, planner) |
| `PLAN_LIMIT` | `6` | `time_limit` of each run [min]; the hard cap is `PLAN_LIMIT` × 60 + 30 s |
| `TASK`, `COST_BUDGET` | unset | `task:=` and `cost_budget:=` of every run |
| `DATA_ROOT` | `$OPEN_APERO_DATA_ROOT`, else `$HOME/open_apero_data` | `data_root:=` of every run (`$DATA_ROOT/$TASK` with `TASK` set) |
| `SCENE_DATASET` | `HSSD` | `scene_dataset:=` of every run |
| `KEEP_RUN_LOGS` | `all` | `failed` deletes the console logs of runs that ended cleanly |
| `SWEEP_MASTER_URI` | `http://localhost:11411` | Master of every run; the caller's `ROS_MASTER_URI` is not used |
| `OPEN_APERO_WS` | found above the script | The workspace's `src/` directory |

`$DATA_ROOT/sweep_logs/<start stamp>/` keeps `sweep.log` (settings, every `roslaunch` command and outcome),
`runs.csv` (one row per run, `status` `ok`, `timeout`, `failed` or `no-status`) and each run's console log.

## `paper_sweep.py`

Source the workspace, the Habitat environment and `datasets.env` (`HSSD_DATA_ROOT`) first.

```bash
mkdir -p ~/open_apero_data/paper_full
nohup setsid rosrun open_apero_ros paper_sweep.py \
  --data-root ~/open_apero_data/paper_full --parallel 3 --trials 10 --speedup-factor 3 \
  > ~/open_apero_data/paper_full/driver.log 2>&1 &
# Print the first commands; this writes only logs/, runs.csv and sweep.log into the data root
rosrun open_apero_ros paper_sweep.py --data-root /tmp/sweep --dry-run
```

Every (task, scene, planner, trial) is one `roslaunch` of `run_experiment_sim.launch` with `task:=`, `cost_budget:=`
and the paper's per-cell wall cap as `time_limit:=` (`PAPER_WALL_CAPS` in `open_apero_ros.paper_protocol`); each
parallel lane runs its own ROS master. After each run the sweep kills what is left of it, such as the master and nodes
of a run killed at its cap; a lane whose port stays busy for 3 minutes stops and leaves its jobs to the other lanes.
The defaults are 1440 runs (8 scenes × 6 planners × 3 tasks × 10 trials), 2–3 days on three lanes. `kill -INT <pid>`
stops dispatching and lets the running lanes finish; restart with the same `--data-root` and `--resume` to schedule
only the missing trials. The sweep exits with 1 when it left jobs unrun or a lane crashed; `sweep.log` says which.

## `compare_paper_sweep.py`

```bash
rosrun open_apero_ros compare_paper_sweep.py --data-root ~/open_apero_data/paper_full \
  --output ~/open_apero_data/paper_full/report.md
```

For every task × scene × planner it reports the runs' gain at the budget (n, mean, std) beside the paper's mean, the
normalized gains and the mean cost reached, then the mean normalized gain per planner and the rankings, ours beside
the paper's.

## Tests

```bash
MPLBACKEND=Agg python3 -m pytest test   # from open_apero_ros/, with the workspace sourced
```

`test/test_tool_clis.py` covers the tools' argument handling and the sweep tools' readers; `test/test_ros_imports.py`
checks that every Python tool in `scripts/` is in its `TOOLS`, installed and answers `--help`.
