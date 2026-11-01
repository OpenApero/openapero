# Graph-benchmark scripts

Hand-run tools around [`benchmark_node`](../README.md#benchmark_node): generate graph inputs, rerun and verify the
paper's sweep on the tracked instances, and render the graph-benchmark figures.
[Reproducing the Paper](../../docs/paper.md#graph-benchmark) walks through
verifying and plotting, [Graph Benchmark](../../docs/modules/graph-benchmark.md) explains the benchmark, and
[`open_apero_ros/scripts`](../../open_apero_ros/scripts/README.md) holds the active-perception tools.

Each tool is installed with the package (`rosrun open_apero_graph_benchmark <tool>`), and `--help` lists every flag.
Inputs come from the package's `data/`, which only `benchmark_reference.py` writes to. Outputs go below the data root
(`$OPEN_APERO_DATA_ROOT`, else `~/open_apero_data`): results in `graph_benchmark/`, generated inputs in
`graph_benchmark/generated/` and figures in `figures/` ([Run Output](../../docs/modules/run-output.md#figures)).

| Tool | Purpose |
|---|---|
| `verify_benchmark_results.py` | Rerun the sweep on the tracked instances and check it against each dataset's `reference.json` |
| `benchmark_reference.py` | Write a dataset's `reference.json` from result files, or check result files against it |
| `compare_result_trees.py` | Check that two result trees match on everything but the measured time |
| `plot_final_gain_ranges.py` | Min/max final gain per replanning mode, objective and algorithm |
| `plot_computation_time_ranges.py` | Min/max computation time on a log scale, `ap_` datasets only |
| `render_graph.py` | Draw a graph instance with gain-colored nodes |
| `generate_random_graph_topology.py` | Random node positions and adjacency |
| `generate_clustered_gain_model.py` | Random non-overlapping gain regions |

Each tool's code and `main(argv=None)` live in `python/open_apero_graph_benchmark/<tool>.py`, which builds on
[`open_apero_analysis`](../../open_apero_analysis/README.md); `scripts/<tool>.py` only calls that `main`. A new tool
is also listed in `catkin_install_python()` and in `TOOLS` of `test/test_graph_benchmark_imports.py`.

## Reproducibility

The paper's 40 `graph_N/graph.txt` instances are tracked, their 5760 result files are not: each dataset ships a
`reference.json` that `benchmark_reference.py` wrote from them.

| Key | Content |
|---|---|
| `digests` | Per `<replan>/<objective>/<algorithm>`, a SHA-256 truncated to 128 bits over the group's file paths and their `gain`, `cost`, `node_id` and `num_nodes` columns |
| `final_gain_ranges`, `computation_time_ranges` | The [min, max] final gain and computation time per algorithm, for the cells the paper's figures show; times are machine-dependent and never compared |
| `result_files`, `digest_columns`, `format_version`, `dataset`, `generator` | Bookkeeping |

`benchmark_node` is deterministic for a graph file, `random_seed` and build; before trusting a mismatch, follow
[If a Dataset Does Not Reproduce](../../docs/paper.md#if-a-dataset-does-not-reproduce).

## `verify_benchmark_results.py`

Copies each dataset's `graph_N/graph.txt` into `<work-dir>/<dataset>/<dataset>`, runs `run_benchmark.launch` there over
every replanning mode and objective, and compares; it never writes into `data/`. `--work-dir` defaults to
`<data root>/graph_benchmark/verify`; each dataset gets `report.md`, `status` and `roslaunch.log` there.

```bash
rosrun open_apero_graph_benchmark verify_benchmark_results.py --all --parallel 4    # 4 datasets at once
rosrun open_apero_graph_benchmark verify_benchmark_results.py <dataset>... --detach  # returns at once
# hours later, once every <work-dir>/<dataset>/status is written:
rosrun open_apero_graph_benchmark verify_benchmark_results.py <dataset>... --skip-run
```

Against the dataset's `reference.json` it reports `REPRODUCED (exact)` when every digest matches,
`REPRODUCED (figure-level)` when only digests differ and every final-gain range cell agrees within 1e-9, and
`MISMATCH` otherwise. It exits 0 only when every dataset reproduced.

## `benchmark_reference.py`

```bash
rosrun open_apero_graph_benchmark benchmark_reference.py --all              # write data/<dataset>/reference.json
rosrun open_apero_graph_benchmark benchmark_reference.py <dataset> --check  # compare result files, write nothing
```

A dataset name reads its results from `<data root>/graph_benchmark/<dataset>`; for a path, the reference sits beside the
results. Both need every replanning mode and objective, not only the single run of Getting Started. `--check` prints
`exact`, `figure-level` or `mismatch` by the verifier's rule and exits 1 on `mismatch`. Writing needs seaborn and
`open_apero_analysis`. Write references only from result files that reproduce the paper.

## `compare_result_trees.py`

```bash
rosrun open_apero_graph_benchmark compare_result_trees.py /tmp/baseline_results /tmp/candidate_results
```

Proves that a refactor changed nothing: the same result files, row counts and `gain`, `cost`, `node_id` and
`num_nodes` columns as raw text, and a finite `computation_time`. Other files are ignored.

## Figures

```bash
rosrun open_apero_graph_benchmark plot_final_gain_ranges.py <results> --gain-tick 250
rosrun open_apero_graph_benchmark plot_computation_time_ranges.py <results>
rosrun open_apero_graph_benchmark render_graph.py ap_25x25_clustered_gain_0to100_cost_50/graph_0
```

`<results>` is a folder of result files for every replanning mode and objective, typically the verifier's
`<work-dir>/<dataset>/<dataset>`, or a dataset name below `<data root>/graph_benchmark/`; `render_graph.py` takes a
folder holding `graph.txt` or a `<dataset>/graph_N` name from `data/`. Each writes a PDF and its `.yaml` record below
`<data root>/figures/` at its input's path; [Rendering the Figures](../../docs/paper.md#rendering-the-figures) lists the
paper's `--gain-tick` values.

## Generators

`generate_random_graph_topology.py` places up to 1000 nodes in 25 × 25 at least 1.5 apart, with an edge between every
pair within 3.0 and node 0 at the origin; `generate_clustered_gain_model.py` places 8 regions in 25 × 25 with gains
0–100 (settings hard-coded). Both write below `<data root>/graph_benchmark/generated/` and are unseeded without
`--seed N`. [`generate_graph_instance_node`](../README.md#generate_graph_instance_node) samples an instance there:

```bash
G=${OPEN_APERO_DATA_ROOT:-$HOME/open_apero_data}/graph_benchmark/generated
rosrun open_apero_graph_benchmark generate_random_graph_topology.py --seed 0
rosrun open_apero_graph_benchmark generate_clustered_gain_model.py --seed 0
for seed in 0 1 2 3 4; do   # one instance per seed; the same seed draws the same gains
  roslaunch open_apero_graph_benchmark generate_graph_instance.launch \
    topology_file:=$G/graph_topology.txt gain_model_file:=$G/gain_model.txt random_seed:=$seed
done
```

To track a new dataset, copy its `graph_N/graph.txt` files to `data/<dataset>/`, run the sweep on it and write its
`reference.json` with `benchmark_reference.py <dataset>`.

## Tests

```bash
python3 -m pytest -q test   # from the package root, with the workspace sourced; covers every tool
```
