"""Plot total computation time ranges as horizontal bars for a-priori graph benchmarks.

Aggregates computation time across multiple graph instances and algorithm
configurations, then renders min-max horizontal bar charts on a log scale, saved as a PDF
into the figure tree at the dataset's path below the data root
(``<data root>/figures/.../<dataset>/computation_time_ranges.pdf``).
"""

import argparse

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
    MissingResultsError,
    classify_dataset_folder,
    plot_horizontal_ranges,
    resolve_dataset_folder,
    summarize_final_value_ranges,
)

plt.rcParams.update({"font.size": 18})

FIGURE_NAME = "computation_time_ranges.pdf"


def plot_dataset(data_folder, output_path):
    """Aggregate and plot computation-time ranges for one a-priori dataset into ``output_path``."""
    replan_types = ["none", "everystep"]
    objectives = ["path_gain"]
    objective_labels = {"path_gain": r"$g_e = g$"}

    replan_labels = {
        "everystep": "node",
        "none": "no",
    }

    summary_data = summarize_final_value_ranges(
        data_folder, replan_types, objectives, skip=[], value_key="computation_time"
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    plot_horizontal_ranges(
        summary_data,
        list(ALGORITHM_ORDER),
        objective_labels,
        replan_labels,
        output_path=output_path,
        figsize=(8, 4),
        xlabel="Computation time (s, log scale)",
        log_x=True,
    )


def main(argv=None):
    use_file_backend()
    parser = argparse.ArgumentParser(
        description="Plot computation-time ranges for an a-priori graph dataset."
    )
    parser.add_argument(
        "dataset",
        help="A-priori dataset directory holding results, e.g. the tree "
        "verify_benchmark_results.py prints as 'results:', or a dataset name, resolved below "
        "<data root>/graph_benchmark/; its basename must start with 'ap_'.",
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
    if dataset_type != "ap":
        parser.error(f"computation-time ranges require an 'ap_' dataset: {data_folder.name}")

    output_path = figure_path(args, data_folder, FIGURE_NAME)
    try:
        plot_dataset(data_folder, output_path)
    except MissingResultsError as exc:
        parser.error(str(exc))
    finish_figure(
        output_path,
        [data_folder],
        command_line("plot_computation_time_ranges.py", argv),
        open_file=opens_viewer(args),
    )


if __name__ == "__main__":
    main()
