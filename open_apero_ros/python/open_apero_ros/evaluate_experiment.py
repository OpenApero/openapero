"""Write the CPU-time performance overview for recorded experiment runs.

Reads each run's ``performance_log.csv`` and ``run_manifest.yaml`` through the
open_apero_analysis loader and renders the overview plot into the figure tree, at the
run's path below the data root
(``<data root>/figures/<run>/performance_overview.png``). Expects the directory layout
``mission_coordinator_node`` records: one ``<YYYYmmdd_HHMMSS>`` run directory per
experiment below a planner-level directory.
"""

import argparse
import re
from pathlib import Path

from open_apero_analysis import cpu_time, load_run, performance_overview
from open_apero_analysis.figures import (
    add_figure_arguments,
    command_line,
    figure_path,
    finish_figure,
    opens_viewer,
    use_file_backend,
)

FIGURE_NAME = "performance_overview.png"

# Columns the performance-overview plot requires. These are the display
# categories; the accounted-CPU-time *denominator* is read from the run's
# manifest by open_apero_analysis.cpu_time.ros_cpu_timer_selection (so it
# includes StartNodeUpdate).
REQUIRED_PERF_COLUMNS = [
    "ReplanTimeRos",
    "PlanPathTotal",
    "Visualization",
    "RosCallbacks",
    "planning.ExpandTime",
    "planning.UpdateTime",
    "planning.StartNodeUpdate",
    "planning.PlanPathTime",
    "planning.GenerateNode",
    "planning.GenerateEdge",
    "planning.EvaluateNode",
    "planning.GeneratorUpdate",
    "planning.EvaluatorUpdate",
    "planning.NumExpansions",
    "planning.NumNodesAtSelection",
]

# mission_coordinator_node names run directories <YYYYmmdd_HHMMSS>.
RUN_DIR_EXPRESSION = re.compile(r"\d{8}_\d{6}")


def select_run_directories(target_directory, method):
    """Run directories `method` selects at or below `target_directory`.

    Raises ValueError when the selection comes up empty, so the CLI can turn
    that into an argument error instead of silently evaluating nothing.
    """
    if method == "single":
        return [target_directory]
    subdirs = sorted(child for child in target_directory.iterdir() if child.is_dir())
    if method == "recent":
        subdirs = [child for child in subdirs if RUN_DIR_EXPRESSION.match(child.name)][-1:]
    if not subdirs:
        raise ValueError(
            "no %s directories under %s"
            % ("timestamped run" if method == "recent" else "run", target_directory)
        )
    return subdirs


def evaluate_run(run_directory, args, command):
    """Plot the performance overview for one experiment run directory."""
    print("Evaluating run '%s'." % run_directory)

    run = load_run(run_directory)
    if not run.loaded:
        print("  [skip] no usable 'performance_log.csv' (%s)." % run.reason)
        return
    missing = run.missing_columns(REQUIRED_PERF_COLUMNS)
    if missing:
        print(
            "  [skip] 'performance_log.csv' is missing columns %s (available: %s)."
            % (missing, sorted(run.columns.keys()))
        )
        return
    if run.num_rows < 2:
        print("  [skip] too few entries in 'performance_log.csv'.")
        return

    selection = cpu_time.ros_cpu_timer_selection(run)
    if not selection.columns:
        print("  [skip] no CPU-time timer scope (%s)." % "; ".join(selection.diagnostics))
        return

    overview = performance_overview.compute_performance_overview(
        dict(run.columns), selection.columns
    )
    figure = figure_path(args, run_directory, FIGURE_NAME)
    figure.parent.mkdir(parents=True, exist_ok=True)
    performance_overview.render_performance_overview(overview, figure)
    finish_figure(figure, [run_directory], command, open_file=opens_viewer(args))


def main(argv=None):
    use_file_backend()
    parser = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "target_directory",
        type=Path,
        help="What it points at depends on --method: one run directory for "
        "'single', a planner-level directory holding timestamped run "
        "directories otherwise.",
    )
    parser.add_argument(
        "--method",
        choices=("single", "recent", "all"),
        default="recent",
        help="Evaluate target_directory as one run, only its newest run, or every run below it.",
    )
    add_figure_arguments(parser, single_file=False)
    args = parser.parse_args(argv)

    if not args.target_directory.is_dir():
        parser.error("not a directory: %s" % args.target_directory)
    try:
        run_directories = select_run_directories(args.target_directory, args.method)
    except ValueError as exc:
        parser.error(str(exc))

    command = command_line("evaluate_experiment.py", argv)
    for run_directory in run_directories:
        evaluate_run(run_directory, args, command)


if __name__ == "__main__":
    main()
