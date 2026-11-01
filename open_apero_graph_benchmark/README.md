# open_apero_graph_benchmark

Offline benchmark of the Open Apero graph path planners (NBS, DBS, SPT, TSP) on synthetic graph instances, with no
simulator and no map: the paper's graph-benchmark experiments and a regression check for path-planner changes in
[`open_apero_core`](../open_apero_core/README.md). [Graph Benchmark](../docs/modules/graph-benchmark.md) explains a
run, [Reproducing the Paper](../docs/paper.md#graph-benchmark) the verification and the figures, and
[`scripts/`](scripts/README.md) lists the Python tools.

```bash
rosrun open_apero_graph_benchmark verify_benchmark_results.py op_5m_25x25_scattered_gain_0to100_cost_50  # a few minutes
roslaunch open_apero_graph_benchmark run_benchmark.launch \
  graph_type:=op_5m_25x25_clustered_gain_0to100_cost_50 replan_type:=everystep objective:=expected_gain
```

The verifier reruns a dataset's sweep in a scratch tree and checks it against the dataset's `reference.json`. A sweep
ends with roslaunch's red `REQUIRED process [benchmark_node-N] has died!` banner and exit status 0; its results
overwrite those in `<data root>/graph_benchmark/<dataset>/` (`$OPEN_APERO_DATA_ROOT`, else `~/open_apero_data`).

## Datasets

`data/` holds the tracked inputs: eight datasets `<prefix><W>x<H>_<gain pattern>_gain_0to100_cost_<budget>` of five
instances `graph_0` … `graph_4/graph.txt` and a `reference.json` each. `ap_` is a priori (the whole graph is visible, so
`perception_radius` must reach the `<W>x<H>` diagonal: 35.4 for `25x25`, 70.7 for `50x50`), `op_<R>m_` online within
radius R. The launch file reads the radius and `cost_budget` from the name (else 100); the node warns when the radius
does not fit the name, not on a `cost_budget` override.

| File | Format |
|---|---|
| `graph_N/graph.txt`, `data/graph_topology.txt` | One node per line: `id x y gain neighbor_id ...`; node 0 is the start |
| `data/gain_model.txt` | One region per line: `x y radius min_gain max_gain`; a node takes the first region containing it, and an all-zero center and radius contains every node |
| `reference.json` | Digests and range tables of the result files, see [scripts/README.md](scripts/README.md#reproducibility) |

Each run writes `<results_folder>/graph_N/<replan>/<objective>/<algorithm>_<parameter>.txt`: `dbs_<B>.txt`,
`nbs_<B>.txt` (beam width), `spt_<α>.txt`, `tsp_<α>.txt` (`top_node_percentage`, two decimals). One row
`gain cost computation_time node_id num_nodes` per sample: cumulative gain, cost and planning time [s], current node,
perceived graph size. The first row is the start state, then one row before and one after each executed edge, and one
before the edge a run stops at, if any.

## benchmark_node

Runs the sweep over the instances in `graph_type_folder`: per graph, objective and replanning mode a block of one run
per DBS and NBS beam width and SPT and TSP `top_node_percentage` (launch defaults: 60 blocks of 12 runs). It checks its
parameters and loads every instance before writing a result; a failure is a `ROS_FATAL` with exit status 1.
`run_benchmark.launch` passes each argument to the private parameter of the same name:

| Parameter | Launch default | Meaning |
|---|---|---|
| `graph_type` | `ap_50x50_clustered_gain_0to100_cost_100` | Launch only: dataset name below `data/` |
| `graph_type_folder` | `data/<graph_type>` | Dataset folder, read only |
| `data_root` | `$OPEN_APERO_DATA_ROOT`, else `$HOME/open_apero_data` | Launch only: root of every run output |
| `results_folder` | `<data_root>/graph_benchmark/<dataset>` | Root of the result files |
| `replan_type` | `try_every_option` | `none` (one plan), `goal` (replan at the plan's end), `everystep` (after every edge), or all three |
| `objective` | `try_every_option` | `path_gain`, `path_ratio`, `expected_gain`, `table_gain`, or all four |
| `perception_radius` | R for `op_<R>m_`, else `100.0` | Visibility radius [m]; must match the dataset name |
| `cost_budget` | N from `_cost_<N>`, else `100.0` | Cost budget per run, finite |
| `beam_depth` | `100` | Beam depth of DBS and NBS |
| `min_beam_width_dbs`, `max_beam_width_dbs`, `beam_width_step_dbs` | `1`, `10000`, `100` | Geometric DBS sweep: 1, 100, 10000 |
| `min_beam_width_nbs`, `max_beam_width_nbs`, `beam_width_step_nbs` | `1`, `100`, `10` | Geometric NBS sweep: 1, 10, 100 |
| `top_node_percentage_min`, `top_node_percentage_max`, `percentage_step` | `0.5`, `1.0`, `0.25` | Linear SPT and TSP sweep: 0.50, 0.75, 1.00 |
| `random_seed` | `0` | Seeds the planners' tie-break RNGs; negative is nondeterministic |
| `visualize`, `visualize_edges`, `visualization_rate` | `false`, `true`, `5.0` | Publish markers and the overlay (the launch file also starts RViz), with edges, at this rate [Hz] |
| `debug_diagnostics` | `false` | Per-step diagnostic log lines; slower, same results |
| `log_level` | `info` | `debug`, `info`, `warn`, `error` or `fatal`, for the node and the core |

| Topic | Type | Description |
|---|---|---|
| `~visualization` | `visualization_msgs/MarkerArray` | Perceived graph colored by gain, the current node in magenta; latched |
| `~status_overlay` | `jsk_rviz_plugins/OverlayText` | Planner, instance, objective, mode, step, node, run phase, gain, cost against the budget; latched |

At `info` the node logs a line per graph and two per block, `start` and `progress`; the latter adds, when they apply,
`(N skipped)`, `reasons=` how the runs ended if not all alike (`noplan`, `overbudget`, `exhausted`, `oneplan`,
`nodegone`, `noedge`) and `(N new)`. A `[benchmark_stale]` warning lists result files this sweep did not write;
`log_level:=debug` adds a line per result file.

## generate_graph_instance_node

Samples a gain from `gain_model_file` for each node of `topology_file` reachable from node 0 (the rest are dropped) and
writes `<output_folder>/graph_<N+1>/graph.txt`, N the highest existing `graph_<N>`; an error is a `ROS_FATAL`, exit 1.

```bash
roslaunch open_apero_graph_benchmark generate_graph_instance.launch random_seed:=0
```

| Parameter | Launch default | Meaning |
|---|---|---|
| `topology_file` | `data/graph_topology.txt` | Topology to sample, read only |
| `gain_model_file` | `data/gain_model.txt` | Gain model to sample from, read only |
| `data_root` | `$OPEN_APERO_DATA_ROOT`, else `$HOME/open_apero_data` | Launch only: root of `output_folder` |
| `output_folder` | `<data_root>/graph_benchmark/generated` | Where the new `graph_<N+1>/` goes |
| `random_seed` | `-1` | Seeds gain sampling; negative is nondeterministic |

## Dependencies

[`open_apero_core`](../open_apero_core/README.md) (planners), `roscpp`, `visualization_msgs`, `jsk_rviz_plugins`,
[`open_apero_ros_bridge`](../open_apero_ros_bridge/README.md) (markers, logging); `rviz` at run time; for the scripts
[`open_apero_analysis`](../open_apero_analysis/README.md), NumPy, Matplotlib, seaborn and, in an install space, rospkg.

## Tests

```bash
catkin build open_apero_graph_benchmark --no-deps --catkin-make-args run_tests --   # from the workspace root
```

This runs the gtest binary and the pytest suites under `test/`; see [Running Tests](../CONTRIBUTING.md#running-tests).
