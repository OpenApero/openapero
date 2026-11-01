# open_apero_analysis

ROS-free Python loader for Open Apero run directories: it reads a run's `performance_log.csv`, `rosparams.yaml`,
`run_manifest.yaml` and `episode_summary.yaml` into one run object, and holds the series interpolation and
CPU-time accounting of the plotting tools in [`open_apero_ros`](../open_apero_ros/scripts/README.md) and
[`open_apero_graph_benchmark`](../open_apero_graph_benchmark/scripts/README.md). It depends on no other Open Apero
package. [Run Output and Analysis](../docs/modules/run-output.md) describes the run files and
[shows the loader in use](../docs/modules/run-output.md#reading-runs-in-python).

## Library API

Everything below is exported from `open_apero_analysis`.

| Name | Purpose |
|---|---|
| `load_run(run_dir)` | One run directory → `RunData`; a bad CSV gives a skipped run with a reason, never an exception |
| `load_runs(roots, dedup=True)` | Every run below one or more roots; across roots the earlier one wins a duplicate `rel_id` |
| `iter_run_dirs(root)` | Every directory below `root` holding a `performance_log.csv` |
| `aggregate_report(runs)` | Text summary: counts per status, shadowed duplicates, degraded and skipped reasons |
| `RunStatus` | `PARSED`, `DEGRADED` (columns missing, or no valid manifest), `SKIPPED` (no usable data) |
| `schema_fingerprint(columns)` | The manifest fingerprint of `(name, type, unit, parent)` tuples, byte-compatible with the C++ writer |
| `data_root()`, `figures_root(root=None)` | `$OPEN_APERO_DATA_ROOT` (`~` expanded), else `~/open_apero_data`; `<root>/figures` |
| `paths.figure_dir(source, out_dir=None)` | A run, scene or dataset's figure directory: its path below the data root, under `out_dir` |
| `load_timer_metadata(manifest, csv_header=None)`, `TimerMetaState` | The manifest's validated column schema → `TimerMetadata` `(state, columns, layers, diagnostics)`; `state` is `ABSENT`, `INVALID` or `VALID` |
| `root_timers(metadata)` | The parentless timer columns of a `VALID` schema, layer and agent-level alike; the caller picks which to sum |
| `ColumnMeta` | One manifest column: name, type, unit, parent, layer_instance, layer_type; `is_timer`, `is_root`, `is_agent_level` |
| `TimerSelection` | `(columns, diagnostics)` of the timer selectors: the columns of a valid manifest, else none and why |
| `calculate_mean_and_std_interpolated(xs, ys, num_steps, start=None)` | Mean and population std on a uniform grid over the x range all runs share |
| `interpolate_onto_grid(x, y, grid, mode="linear")` | One series onto a grid; `mode="previous"` holds the last value; NaN outside the series |
| `paths`, `series`, `timers` | The modules themselves |

### `RunData` attributes

| Attribute | Content |
|---|---|
| `path`, `timestamp`, `algorithm`, `scene`, `task` | Run directory and the names of it and its parents; `task` is the `explore`, `recon` or `points` folder above the experiment, else `None` |
| `rel_id`, `shadowed` | Last four path components (five with a `task`), the identity `load_runs` de-duplicates on; the skipped duplicates |
| `status`, `reason`, `loaded` | `RunStatus`, why, and whether columns were loaded |
| `columns` | Column name → list of floats; empty and `nan` cells are NaN |
| `num_rows`, `raw_header`, `notes` | Row count, the CSV header row, load notes |
| `rosparams`, `manifest`, `episode_summary`, `timer_metadata` | Parsed YAML files (`episode_summary` is `None` without one) and the validated manifest schema |
| `cost_budget`, `cost_computer_type` | From the manifest, else `rosparams`; from `rosparams` (`cost_computer/type`) |
| `has_columns(names)`, `missing_columns(names)` | Capability checks for a plot |

### Experiment-run accounting and figure output

Imported explicitly, not exported from the package: `cpu_time` (what the whole node costs, for
`evaluate_experiment.py`), `planning_time` (what planning alone costs, for `plot_performance_curves.py`),
`performance_overview` (the stacked CPU-time chart; needs Matplotlib) and `figures`, the figure tools' shared
`--out-dir`/`--output`/`--open` arguments and `<figure>.yaml` record ([Figures](../docs/modules/run-output.md#figures)).

## Dependencies

PyYAML and NumPy; Matplotlib for `performance_overview` and `figures`.

## Tests

```bash
python3 -m pytest test   # from the package root, with the workspace sourced
```
