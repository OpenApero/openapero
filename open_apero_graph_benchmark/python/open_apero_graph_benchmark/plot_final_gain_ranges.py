"""Plot final-gain ranges (min/max) as horizontal bars per algorithm, objective and replanning mode.

Averages each algorithm configuration's final accumulated gain over the dataset's graph
instances, takes the minimum and maximum of that mean over the algorithm's parameter sweep, and
renders the ranges as a horizontal bar chart saved as a PDF, into the figure tree at the
dataset's path below the data root (``<data root>/figures/.../<dataset>/final_gain_ranges.pdf``).
"""

import argparse
import math

import matplotlib.pyplot as plt

from open_apero_analysis.figures import (
    add_figure_arguments,
    command_line,
    figure_path,
    finish_figure,
    opens_viewer,
    use_file_backend,
)
from open_apero_graph_benchmark.benchmark_ranges import (
    ALGORITHM_ORDER,
    FIGURE_SELECTION,
    MissingResultsError,
    classify_dataset_folder,
    plot_horizontal_ranges,
    resolve_dataset_folder,
    summarize_final_value_ranges,
)

plt.rcParams.update({"font.size": 18})

FIGURE_NAME = "final_gain_ranges.pdf"


def positive_finite_float(value):
    """Parse a finite, strictly positive floating-point CLI value."""
    try:
        parsed = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"not a floating-point value: {value}") from exc
    if not math.isfinite(parsed) or parsed <= 0:
        raise argparse.ArgumentTypeError("must be finite and greater than zero")
    return parsed


def plot_dataset(data_folder, dataset_type, output_path, gain_tick=None):
    """Aggregate and plot final-gain ranges for one resolved dataset folder into ``output_path``."""
    if dataset_type == "ap":
        objective_labels = {
            "path_gain": r"$g_e=g$",
            "path_ratio": r"$r$",
        }
    elif dataset_type == "op":
        objective_labels = {
            "path_gain": r"$g$",
            "path_ratio": r"$r$",
            "expected_gain": r"$g_e$",
        }
    else:
        # The CLI classifies this value first; keep the guard for direct Python callers.
        raise ValueError(f"unsupported dataset type: {dataset_type}")
    replan_types, objectives, skip = FIGURE_SELECTION[dataset_type]

    replan_labels = {
        "everystep": "node",
        "goal": "goal",
        "none": "no",
    }

    summary_data = summarize_final_value_ranges(
        data_folder, replan_types, objectives, skip, value_key="gain"
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    plot_horizontal_ranges(
        summary_data,
        list(ALGORITHM_ORDER),
        objective_labels,
        replan_labels,
        output_path=output_path,
        figsize=(8, 8),
        xlabel="Final gain",
        tick_step=gain_tick,
    )


def main(argv=None):
    use_file_backend()
    parser = argparse.ArgumentParser(
        description="Plot final-gain ranges for a graph-benchmark dataset."
    )
    parser.add_argument(
        "dataset",
        help="Dataset directory holding results, e.g. the tree verify_benchmark_results.py "
        "prints as 'results:', or a dataset name, resolved below "
        "<data root>/graph_benchmark/; its basename must start with 'ap_' or 'op_'.",
    )
    parser.add_argument(
        "--gain-tick",
        type=positive_finite_float,
        default=None,
        help="Fixed major tick spacing for the gain axis (default: automatic).",
    )
    add_figure_arguments(parser)
    args = parser.parse_args(argv)

    data_folder = resolve_dataset_folder(args.dataset)
    if not data_folder.is_dir():
        parser.error(f"not a dataset directory: {data_folder}")
    try:
        dataset_type = classify_dataset_folder(data_folder)
    except ValueError as exc:
        parser.error(str(exc))

    output_path = figure_path(args, data_folder, FIGURE_NAME)
    try:
        plot_dataset(data_folder, dataset_type, output_path, gain_tick=args.gain_tick)
    except MissingResultsError as exc:
        parser.error(str(exc))
    finish_figure(
        output_path,
        [data_folder],
        command_line("plot_final_gain_ranges.py", argv),
        open_file=opens_viewer(args),
        extra={"dataset_type": dataset_type},
    )


if __name__ == "__main__":
    main()
