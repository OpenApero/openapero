# Reproducing the Paper

How to rerun the experiments of Qu et al., "An Efficient Beam Search Algorithm for Active Perception in Mobile
Robotics", IJRR 2026 ([doi:10.1177/02783649261455911](https://doi.org/10.1177/02783649261455911)).

The graph benchmark reproduces the paper's results exactly, apart from the computation times, which depend on the
machine. The active perception experiments reproduce statistically, without the external FUEL baseline; the
differences from the paper's runs are listed at the end.

## Graph Benchmark

`open_apero_graph_benchmark/data/` holds the paper's eight datasets, each with five graph instances and a
`reference.json` of the paper's results. The verifier reruns each dataset and compares it with `reference.json`:

```bash
rosrun open_apero_graph_benchmark verify_benchmark_results.py --all --parallel 4
```

This takes about 6 h, most of it for the two `ap_50x50_*` datasets; name datasets instead of `--all` to run a subset.
Reports go to `~/open_apero_data/graph_benchmark/verify/<dataset>/`, and `--help` lists every flag.

### If a Dataset Does Not Reproduce

The verifier prints `REPRODUCED (exact)`, `REPRODUCED (figure-level)` (some result files differ, every figure value
agrees) or `MISMATCH`. Ties are broken by enumeration order, so another compiler can pick a different tied path. On
`MISMATCH`, rebuild `open_apero_core` and `open_apero_graph_benchmark` and rerun; then rerun in the Docker image, whose
toolchain is the tested one; if it still differs, open an issue with the dataset's `report.md`.

### Rendering the Figures

```bash
VERIFY=${OPEN_APERO_DATA_ROOT:-$HOME/open_apero_data}/graph_benchmark/verify
rosrun open_apero_graph_benchmark plot_final_gain_ranges.py \
  $VERIFY/ap_25x25_clustered_gain_0to100_cost_50/ap_25x25_clustered_gain_0to100_cost_50
rosrun open_apero_graph_benchmark plot_computation_time_ranges.py \
  $VERIFY/ap_25x25_scattered_gain_0to100_cost_50/ap_25x25_scattered_gain_0to100_cost_50
rosrun open_apero_graph_benchmark render_graph.py ap_25x25_clustered_gain_0to100_cost_50/graph_0
```

The first plots the range of final gains of each planner per replanning mode and objective, the second their
computation times, and the third draws a graph instance with its gains. The paper's figures use `--gain-tick` 250 for
the 25×25 clustered datasets, 1000 for the 50×50 scattered ones and 500 otherwise. Figures go below
`~/open_apero_data/figures/` ([Run Output](modules/run-output.md#figures)).

## Active Perception Experiments

Three tasks (`task:=points`, `recon`, `explore`) on eight HSSD scenes, ten trials per planner, scene and task: five
large scenes with a 600 s budget and three small ones with 300 s. With the workspace, the Habitat environment and
`datasets.env` sourced:

```bash
rosrun open_apero_ros paper_sweep.py --data-root ~/open_apero_data/paper_full
rosrun open_apero_ros compare_paper_sweep.py --data-root ~/open_apero_data/paper_full \
  --output ~/open_apero_data/paper_full/report.md
```

The sweep's defaults are the paper's protocol, including its per-run wall-clock caps: 1440 runs, 2–3 days on one GPU
with three parallel lanes. Run it detached (`nohup setsid` or tmux); `--resume` continues an interrupted sweep.
`compare_paper_sweep.py` puts the results beside the paper's, `plot_performance_curves.py` plots gain against cost per
planner, and `plot_normalized_gain.py` redraws the paper's normalized-gain chart from its published numbers ([scripts
README](https://github.com/OpenApero/openapero/blob/main/open_apero_ros/scripts/README.md)).

### Known Differences from the Paper's Runs

- Depth integration clears free space up to the camera's planar `z = ray_length` boundary, and the planner's camera
  model ends its view at the same plane (`observation_boundary: planar`); the paper's runs used a sphere of radius
  `ray_length` for both. This raises reconstruction and exploration gains.
- DBS and NBS keep each path as a chain of steps it shares with the paths it extends, instead of a copy, and check
  revisits against marks on the graph. They return the paper's paths at a lower time complexity, so planning is
  faster, the more so the longer the paths ([Beam Search Complexity](modules/path-planners.md#beam-search-complexity)).
