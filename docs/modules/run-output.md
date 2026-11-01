# Run Output and Analysis

What a run writes and how to read it: the run directory, its YAML files, the columns of `performance_log.csv`, the
Python loader and the figure tools. The loader's full API is in the
[`open_apero_analysis` README](https://github.com/OpenApero/openapero/blob/main/open_apero_analysis/README.md), the
plotting tools in the
[`open_apero_ros/scripts` README](https://github.com/OpenApero/openapero/blob/main/open_apero_ros/scripts/README.md).

## Run Directory

`run_experiment_sim.launch` writes one directory per run,
`<data_root>/<scene_dataset>/<scene_id>/<planner>/<YYYYmmdd_HHMMSS>/` (`run_experiments.sh` with `TASK` set prefixes
`<task>/`; `paper_sweep.py` writes below `<--data-root>/<task>/`, by default `<data root>/paper_sweep/`). The data root
is `$OPEN_APERO_DATA_ROOT`, else `~/open_apero_data`; the launch files and tools read it, so the runs, the benchmark
results and the [figures](#figures) share one tree. `mission_coordinator_node` creates the directory and sets the
planner's `~run_dir` to it. All files but `rosparams.yaml` are written by `open_apero_node` (`rviz_screenshot.png` by
RViz, at its request):

| File | When |
|------|------|
| `run_manifest.yaml` | At the first `toggle_running(true)` |
| `performance_log.csv` | Every planning cycle, with `log_performance: true` (launch argument `log_data`, default `true`) |
| `episode_summary.yaml` | When the episode ends, and when the platform stops the planner (`toggle_running(false)`) or `open_apero_node` shuts down (Ctrl+C) during a started episode |
| `rosparams.yaml` | `mission_coordinator_node` runs `rosparam dump /` right after the planner starts |
| `map_log.csv`, `maps/<NNNNN>.vxblx` | With `num_maps_to_save` = N > 0 and a finite `cost_budget`: map k (k = 0 … N−1) at the first replan after the spent cost passed k/N of the budget, at most one per cycle, so map 0 follows the first step and the final map is not saved; with an unbounded budget, map 0 when the episode ends or is stopped. `map_log.csv` holds each map's `TotalPathLength` and `TotalPathDuration` |
| `rviz_screenshot.png` | When the budget is spent or `toggle_running(false)` stops a started episode, with `visualize: true` and RViz running |

## `run_manifest.yaml`

What produced the run: `format_version`, `schema_fingerprint` (a hash over the ordered columns), the build provenance
(`build_git_sha`, `build_git_dirty`, `build_components`), `start_utc`, the agent's `cost_budget` (absent when unbounded)
and `random_seed`, `layers` (instance name → layer kind) and `columns`, one `{name, type, unit, parent}` per CSV column
in CSV order, where `parent` names the timer a sub-timer nests in.

## `episode_summary.yaml`

How the episode ended and its totals: `format_version`, `end_reason` (`budget_exhausted`, `goal_reached`, `no_step` or
`stopped`; see [Ending an Episode](agent.md#ending-an-episode)), `cycles` (the number of CSV rows), `total_cost`,
`cost_budget`, `total_path_length` [m], `total_path_duration` [s], `num_visited_nodes` (executed edges),
`elapsed_wall_time_s` and `real_accumulated_gain` (layer instance name → gain: m³ explored, m² of surface or the summed
gain of the collected points, by task), the run's result against `total_cost`. The last CSV row precedes the last
executed edge, so the totals are only here.

## `performance_log.csv`

One row per planning cycle; `TotalCost` against `planning.RealAccumulatedGain` is the gain-versus-cost curve. The
agent's columns come first (`RosAgent` adds `ReplanTimeRos`, `RosCallbacks` and `RosTimeSec`), then every layer's,
prefixed with its instance name in alphabetical order (`backtracking.`, `planning.` in the shipped configs). Timers are
steady-clock wall time in seconds per cycle and run only with `log_performance`; `Total*` and `TimesSelected` are
cumulative.

| Column | Unit | Meaning |
|--------|------|---------|
| `TotalVisitedNodes` | count | Executed edges (steps), recovery steps included |
| `TotalPathLength`, `TotalPathDuration` | m, s | Executed trajectory length and duration |
| `TotalCost` | cost | Executed edge cost, in the cost computer's unit (s for `TrajectoryTime`) |
| `PlanPathTotal` | s | `planPath` time over every layer tried this cycle |
| `Visualization`, `RosCallbacks` | s | Marker publishing; time in `ros::spinOnce()` |
| `ReplanTimeWall`, `ReplanTimeRos` | s | Wall and ROS time since the previous cycle |
| `CycleIndex` | none | Zero-based cycle index |
| `ElapsedWallTimeSec`, `RosTimeSec` | s | Steady-clock time since the episode started; ROS clock at this cycle |
| `CommandsDispatched`, `FramesMapped` | count | Commands published and frames mapped since the previous cycle |
| `<layer>.ExpandTime`, `<layer>.UpdateTime` | s | Graph expansion; graph and gain update |
| `<layer>.PlanPathTime` | s | This layer's `planPath` |
| `<layer>.TimesSelected` | count | Cycles this layer won |
| `planning.GenerateNode`, `GenerateEdge`, `EvaluateNode` | s | Node generation, edge generation, gain of new nodes |
| `planning.GeneratorUpdate`, `EvaluatorUpdate` | s | Generator update, gain re-evaluation |
| `planning.StartNodeUpdate` | s | Moving the start node to the reached node |
| `planning.PlannedValue` | none | Value of the selected candidate |
| `planning.RealAccumulatedGain` | task unit | The evaluator's accumulated gain when the cycle starts; 0 in the first row |
| `planning.NumExpansions` | count | Expansion rounds this cycle |
| `planning.NumNodesAfterNewStart`, `NumNodesAfterExpansion`, `NumNodesAtSelection` | count | Graph size after the start-node update, after expansion, at selection |

`GenerateNode`, `GenerateEdge` and `EvaluateNode` run inside `<layer>.ExpandTime`, `GeneratorUpdate` and
`EvaluatorUpdate` inside `<layer>.UpdateTime`, and every `<layer>.PlanPathTime` inside `PlanPathTotal`; the sum
`PlanPathTotal + Visualization + RosCallbacks + Σ(<layer>.ExpandTime + <layer>.UpdateTime + <layer>.StartNodeUpdate)`,
the last term for planning layers only, is a per-cycle total without overlap. To add columns, a layer installs a
`LayerMetrics` subclass with its own `declareFields()`; a backtracker overrides `Backtracker::declareMetrics()`.

## Reading Runs in Python

`open_apero_analysis` is a ROS-free Python package that loads run directories: `load_run(run_dir)` reads one into a
`RunData`, `load_runs(roots)` every run below one or more roots, and `aggregate_report(runs)` summarizes how many
loaded cleanly. A run is `PARSED`, `DEGRADED` (columns missing, or no valid manifest) or `SKIPPED` (no usable
data); the loader never raises for a bad CSV.

```python
from pathlib import Path
from open_apero_analysis import aggregate_report, load_run, load_runs

scene = Path.home() / "open_apero_data/HSSD/102816852"
run = load_run(max(p for p in (scene / "nbs").iterdir() if p.is_dir()))  # the newest nbs run
print(run.status.value, run.num_rows, run.columns["planning.RealAccumulatedGain"][-1])

print(aggregate_report(load_runs(scene)))
```

The plotting tools build on the package ([Getting Started](../getting-started.md#reading-a-runs-results)).

## Figures

The figure tools write below `<data root>/figures/`, at the path of the input each was drawn from (a figure drawn from
no run, such as `plot_normalized_gain.py`'s, at its top): the curves of the runs in `paper_full/explore/HSSD/102816852/`
go to `figures/paper_full/explore/HSSD/102816852/`. They share `--out-dir DIR` (the root of the tree), `--output FILE`
(one exact file; not in `evaluate_experiment.py`) and `--open` (the system viewer). Each runs without a display, prints
`Wrote <path>` and writes a `.yaml` record of the same name beside the figure: the command, the input directories, the
time and the code version.
