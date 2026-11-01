"""Load graph-benchmark results and render min-max range charts."""

import re
from collections import OrderedDict, defaultdict
from pathlib import Path

import matplotlib.lines as mlines
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np
import seaborn as sns

from open_apero_analysis import calculate_mean_and_std_interpolated
from open_apero_graph_benchmark.paths import RESULTS_ROOT

ALGORITHM_ORDER = ("nbs", "dbs", "spt", "tsp")

# Keep the result-file pattern centralized so the plot scripts cannot drift apart.
GROUPING_PATTERN = re.compile(r"(nbs|dbs|spt|tsp)_(\d+(\.\d+)?)")

ALGORITHM_LABELS = {
    "nbs": "NBS",
    "dbs": "DBS",
    "spt": "SPT",
    "tsp": "TSP",
}

# What the paper's final-gain figures aggregate, per dataset type: (replan types, objectives,
# skipped (replan type, objective) pairs). plot_final_gain_ranges.py plots it and
# benchmark_reference.py tables it, so a reference checks exactly what the figure shows.
FIGURE_SELECTION = {
    "ap": (["none", "goal", "everystep"], ["path_gain", "path_ratio"], [("none", "path_ratio")]),
    "op": (["goal", "everystep"], ["path_gain", "path_ratio", "expected_gain"], []),
}

# Points on the common cost grid that per-graph series are interpolated onto before averaging.
INTERPOLATION_SAMPLE_COUNT = 100

# tab20c groups four shades per hue, so stepping by four picks one distinct hue per algorithm.
# Enumerating the palette directly would give four shades of the same blue and silently
# restyle every published figure.
PALETTE_STRIDE = 4


class MissingResultsError(FileNotFoundError):
    """A dataset folder lacks the benchmark result files a range summary reads."""


def missing_results(folder):
    """The error for a dataset folder, or one of its result folders, without results."""
    return MissingResultsError(
        f"no benchmark results in {folder}. A range summary reads every replanning mode and "
        "objective of the paper's figures, and the result files are not shipped (a fresh clone "
        "has only graph_N/graph.txt and reference.json). verify_benchmark_results.py <dataset> "
        "runs and checks them all; the figure scripts take the tree it prints as 'results:'."
    )


def resolve_dataset_folder(dataset):
    """Resolve an existing path, or a dataset name to its results below ``RESULTS_ROOT``."""
    dataset_folder = Path(dataset)
    if not dataset_folder.is_absolute() and not dataset_folder.exists():
        dataset_folder = RESULTS_ROOT / dataset_folder
    return dataset_folder


def classify_dataset_folder(dataset_folder):
    """Return ``"ap"`` or ``"op"`` from a benchmark dataset directory name."""
    dataset_name = Path(dataset_folder).name
    if dataset_name.startswith("ap_"):
        return "ap"
    if dataset_name.startswith("op_"):
        return "op"
    raise ValueError(f"dataset directory name must start with 'ap_' or 'op_': {dataset_name}")


def load_and_group_files(folder_path, pattern):
    """Group matching ``.txt`` results by algorithm and numeric parameter."""
    grouped_files = {algorithm: [] for algorithm in ALGORITHM_ORDER}

    for file_path in folder_path.iterdir():
        if file_path.suffix != ".txt":
            continue
        match = pattern.match(file_path.stem)
        if not match:
            continue
        algorithm = match.group(1)
        if algorithm in grouped_files:
            grouped_files[algorithm].append((float(match.group(2)), file_path.name))

    for files in grouped_files.values():
        files.sort()

    return OrderedDict(
        (algorithm, grouped_files[algorithm])
        for algorithm in ALGORITHM_ORDER
        if grouped_files[algorithm]
    )


def process_files(plot_data, grouped_files, folder_path, graph_name):
    """Load gain, cost, computation-time, node-ID, and node-count columns into ``plot_data``.

    Mutates ``plot_data`` in place and returns nothing.
    """
    for algorithm, files in grouped_files.items():
        for parameter, file_name in files:
            file_path = folder_path / file_name
            try:
                data = np.loadtxt(file_path)
            except (OSError, ValueError) as exc:
                raise ValueError(f"failed to load result file {file_path}: {exc}") from exc
            if data.ndim != 2 or data.shape[1] < 5:
                raise ValueError(
                    f"result file {file_path} has shape {getattr(data, 'shape', None)!r}; "
                    "expected 2-D with >=5 columns "
                    "(gain, cost, computation_time, node_id, num_nodes)"
                )
            plot_data[algorithm][parameter][graph_name] = {
                "file_name": file_path.stem,
                "gain": data[:, 0],
                "cost": data[:, 1],
                "computation_time": data[:, 2],
                "node_id": [str(int(value)) for value in data[:, 3]],
                "num_nodes": data[:, 4],
            }


def graph_result_folders(data_folder, replan_types):
    """Return the sorted names of the graph folders holding results for any of ``replan_types``.

    A graph folder holds its results as ``<graph>/<replan_type>/<objective>/``.
    """
    return sorted(
        folder.name
        for folder in data_folder.iterdir()
        if folder.is_dir() and any((folder / replan_type).is_dir() for replan_type in replan_types)
    )


def summarize_final_value_ranges(data_folder, replan_types, objectives, skip, value_key):
    """Summarize the min/max final value per algorithm across a dataset's graph instances.

    For each (replan type, objective) pair, loads every graph instance's results, interpolates
    ``value_key`` against cost onto a common grid, takes the final interpolated value per
    algorithm and parameter, and reduces those to a (min, max) range.

    Args:
        data_folder: Dataset folder holding one subfolder per graph instance.
        replan_types: Replanning modes to read.
        objectives: Objectives to read.
        skip: (replan type, objective) pairs to leave out.
        value_key: Column to summarize -- ``"gain"`` or ``"computation_time"``.

    Returns:
        An ordered ``{(replan_type, objective): {algorithm: (minimum, maximum)}}`` mapping, in
        reverse discovery order so the plot reads top-to-bottom the way the paper figures do.
    """
    ranges = defaultdict(lambda: defaultdict(lambda: defaultdict(dict)))

    for replan_type in replan_types:
        for objective in objectives:
            if (replan_type, objective) in skip:
                continue

            plot_data = defaultdict(lambda: defaultdict(dict))
            graph_names = graph_result_folders(data_folder, replan_types)
            if not graph_names:
                raise missing_results(data_folder)
            for graph_name in graph_names:
                graph_folder = data_folder / graph_name / replan_type / objective
                if not graph_folder.is_dir():
                    raise missing_results(graph_folder)
                grouped_files = load_and_group_files(graph_folder, GROUPING_PATTERN)
                if not grouped_files:
                    raise missing_results(graph_folder)
                process_files(plot_data, grouped_files, graph_folder, graph_name)

            for algorithm, per_parameter in plot_data.items():
                for _parameter, per_graph in per_parameter.items():
                    cost_series = []
                    value_series = []
                    for _graph_name, graph_data in per_graph.items():
                        cost_series.append(graph_data["cost"])
                        value_series.append(graph_data[value_key])

                    _grid, mean_value, _std = calculate_mean_and_std_interpolated(
                        cost_series, value_series, INTERPOLATION_SAMPLE_COUNT
                    )

                    final_value = mean_value[-1]
                    algorithm_range = ranges[replan_type][objective][algorithm]
                    algorithm_range["min"] = min(
                        algorithm_range.get("min", float("inf")), final_value
                    )
                    algorithm_range["max"] = max(
                        algorithm_range.get("max", float("-inf")), final_value
                    )

    summary = defaultdict(dict)
    for replan_type, per_objective in ranges.items():
        for objective, per_algorithm in per_objective.items():
            for algorithm, bounds in per_algorithm.items():
                summary[(replan_type, objective)][algorithm] = (bounds["min"], bounds["max"])
    return dict(reversed(list(summary.items())))


def plot_horizontal_ranges(
    summary_data,
    algorithms,
    objective_labels,
    replan_labels,
    output_path,
    *,
    figsize,
    xlabel,
    tick_step=None,
    log_x=False,
    title=None,
):
    """Render grouped min-max ranges and save them as a PDF.

    Saves but does not show: whether a window opens is the entry point's decision, not this
    module's. Callers that want one call ``plt.show()`` themselves.
    """
    sns.set_style("darkgrid")
    fig, ax = plt.subplots(figsize=figsize)

    width = 0.18
    palette = sns.color_palette("tab20c")
    algorithm_colors = {
        algorithm: palette[index * PALETTE_STRIDE]
        for index, algorithm in enumerate(ALGORITHM_ORDER)
    }

    y_positions = []
    y_labels = []
    for index, ((replan_type, objective), algorithm_ranges) in enumerate(summary_data.items()):
        drew_any = False
        for offset_index, algorithm in enumerate(algorithms):
            if algorithm not in algorithm_ranges:
                continue
            drew_any = True
            minimum, maximum = algorithm_ranges[algorithm]
            y = index + offset_index * width
            color = algorithm_colors[algorithm]
            ax.hlines(y, minimum, maximum, color=color, linewidth=4)
            ax.vlines(
                minimum,
                y - width * 0.3,
                y + width * 0.3,
                color=color,
                linewidth=4,
            )
            ax.vlines(
                maximum,
                y - width * 0.3,
                y + width * 0.3,
                color=color,
                linewidth=4,
            )

        if drew_any:
            y_positions.append(index + width * (len(algorithms) - 1) / 2)
            y_labels.append(
                f"{replan_labels.get(replan_type, replan_type)}\n"
                f"{objective_labels.get(objective, objective)}"
            )

    ax.set_yticks(y_positions)
    ax.set_yticklabels(
        y_labels,
        rotation=90,
        va="center",
        ha="center",
        rotation_mode="anchor",
    )
    ax.tick_params(axis="y", pad=20)
    ax.set_xlabel(xlabel)

    if log_x:
        ax.set_xscale("log")
        ax.xaxis.set_major_locator(ticker.LogLocator(base=10.0, numticks=10))
        ax.xaxis.set_minor_locator(
            ticker.LogLocator(base=10.0, subs=np.arange(2, 10), numticks=100)
        )
        ax.grid(True, which="both", axis="x")
    else:
        ax.grid(True, axis="x")
        if tick_step is not None:
            ax.xaxis.set_major_locator(ticker.MultipleLocator(tick_step))
        ax.set_xlim(left=0)

    if title:
        ax.set_title(title)

    legend_handles = [
        mlines.Line2D(
            [],
            [],
            color=algorithm_colors[algorithm],
            linewidth=5,
            label=ALGORITHM_LABELS.get(algorithm, algorithm),
        )
        for algorithm in algorithms
    ]
    ax.legend(
        handles=legend_handles[::-1],
        labels=[handle.get_label() for handle in legend_handles][::-1],
        title="Algorithms",
        loc="upper left",
        bbox_to_anchor=(1.02, 1),
        frameon=False,
        ncol=1,
    )

    fig.tight_layout()
    fig.savefig(output_path, format="pdf", dpi=600, bbox_inches="tight")
