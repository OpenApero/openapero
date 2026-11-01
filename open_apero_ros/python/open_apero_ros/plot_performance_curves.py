"""Plot the performance curves of the planners on one experiment scene.

Reads the runs' performance logs through the open_apero_analysis loader, interpolates
and averages results across runs, and plots accumulated gain versus cost for different
algorithms (by default the shipped NBS, DBS, SPT, TSP, RRT*, RRT; --planners names others).

The scene folder holds one sub-directory per planner, as run_experiment_sim.launch
writes them (``<data-root>/HSSD/<scene>/<planner>/<stamp>``) and paper_sweep.py
(``<data-root>/<task>/HSSD/<scene>/<planner>/<stamp>``). A run whose x values miss the
start of the curves' common grid is skipped.

Each algorithm folder prints how many runs were loaded and skipped and why, a ranked
per-run table (final values, timings, provenance) and a config census, so an outlier
run can be identified and its directory removed by hand. Final values come from the
run's episode_summary.yaml when it has one, else from the last performance_log.csv row.
"""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

from open_apero_analysis import RunStatus, calculate_mean_and_std_interpolated, data_root, load_run
from open_apero_analysis.figures import (
    add_figure_arguments,
    command_line,
    figure_path,
    finish_figure,
    opens_viewer,
    use_file_backend,
)
from open_apero_analysis.paths import DATA_ROOT_VARIABLE
from open_apero_analysis.planning_time import plan_path_selection, planning_time_selection

# Display name -> the column(s) it reads; a list is summed per row.
COLUMN_MAP = {
    "Accumulated Gain": ["planning.RealAccumulatedGain"],
    "Cost": ["TotalCost"],
    "Num Expansions": ["planning.NumExpansions"],
    "Num Nodes": ["planning.NumNodesAtSelection"],
    "Path Length": ["TotalPathLength"],
    "Path Duration": ["TotalPathDuration"],
    "Planned Value": ["planning.PlannedValue"],
    "Num Visited Nodes": ["TotalVisitedNodes"],
    # "Planning Time" is resolved per run by planning_time_selection (see extract_series).
    "Runtime": ["ReplanTimeWall"],
}

# Keys accumulated across rows (per-cycle values summed into a running total).
NEED_ACCUMULATION = {"Planning Time", "Runtime"}

# The common interpolation grid starts here on the cost axis, past the start-up cycles every
# run spends at the origin. Any other x axis starts where the runs' common interval does.
COST_GRID_START = 50.0


def interpolation_start(x_axis):
    """Where the common grid of ``x_axis`` starts; None lets the runs' common interval decide."""
    return COST_GRID_START if x_axis == "Cost" else None


def figure_name(x_axis, y_axes, top_k=0, planners=None):
    """The figure's file name, which says what it plots.

    ``curves_<y>..._vs_<x>[_top<k>][_<planners>].pdf``: planners other than the shipped six appear
    joined by ``-``, so each selection keeps its own file.
    """

    def slug(label):
        return label.lower().replace(" ", "-")

    suffix = "_top%d" % top_k if top_k else ""
    if planners and list(planners) != list(PLANNER_FOLDERS.values()):
        suffix += "_" + "-".join(planners)
    return "curves_%s_vs_%s%s.pdf" % ("_".join(slug(y) for y in y_axes), slug(x_axis), suffix)


# Per-run summary table: (header, summarize_run key, width, format). Read from
# the run's columns rather than the plotted series, so the table is the same
# whatever --x-axis/--y-axes select.
SUMMARY_COLUMNS = [
    ("rank", "rank", 4, "%d"),
    ("run", "run", 15, "%s"),
    ("rows", "rows", 5, "%d"),
    ("gain", "gain", 7, "%.1f"),
    ("cost", "cost", 7, "%.1f"),
    ("length", "length", 7, "%.1f"),
    ("dur", "duration", 7, "%.1f"),
    ("planPath[s]", "plan_path_s", 11, "%.1f"),
    ("wall[s]", "wall_s", 8, "%.1f"),
    ("start(UTC)", "start", 11, "%s"),
    ("build", "build", 8, "%s"),
]

# Config fields censused per algorithm folder: drift between runs of the same
# folder is an outlier signal in itself.
CONFIG_FIELDS = [
    ("cost_budget", lambda run: run.cost_budget),
    ("cost_computer", lambda run: run.cost_computer_type),
    ("schema", lambda run: (run.manifest or {}).get("schema_fingerprint", "")[:8] or None),
    ("format_version", lambda run: (run.manifest or {}).get("format_version")),
]


def _final(run, name, end=None):
    """``end`` (the episode summary's value) if given, else ``name``'s last-row value, else None."""
    if end is not None:
        return end
    if run.columns.get(name):
        return run.columns[name][-1]
    return None


def _total(run, names):
    """Sum of every row of every named column, or None if none are present."""
    present = [n for n in names if n in run.columns]
    if not present:
        return None
    return sum(sum(run.columns[n]) for n in present)


def _run_flags(run):
    """Human-readable caveats about this run's provenance, worst first."""
    flags = []
    if run.status is RunStatus.DEGRADED and run.reason:
        flags.append("DEGRADED: %s" % run.reason)
    meta = run.timer_metadata
    if meta is not None and not meta.is_valid:
        flags.append("timer metadata %s" % meta.state.value)
    flags.extend(run.notes)
    return flags


def summarize_run(run):
    """One run's identity, final values, timings and provenance for the table."""
    manifest = run.manifest or {}
    sha = manifest.get("build_git_sha")
    build = None
    if sha:
        build = sha[:7] + ("*" if manifest.get("build_git_dirty") else "")
    start = manifest.get("start_utc")
    if isinstance(start, str) and len(start) >= 16:
        start = start[5:16].replace("T", " ")  # 2026-08-01T08:23:01Z -> 08-01 08:23
    # The last row precedes the last executed edge; the summary holds the episode's end.
    summary = run.episode_summary or {}
    end_gain = (summary.get("real_accumulated_gain") or {}).get("planning")
    return {
        "run": run.timestamp,
        "rows": run.num_rows,
        "gain": _final(run, "planning.RealAccumulatedGain", end_gain),
        "cost": _final(run, "TotalCost", summary.get("total_cost")),
        "length": _final(run, "TotalPathLength", summary.get("total_path_length")),
        "duration": _final(run, "TotalPathDuration", summary.get("total_path_duration")),
        # Planning layer's planPath only; ElapsedWallTimeSec already measures
        # the whole episode, so per-cycle ReplanTimeWall adds nothing here.
        "plan_path_s": _total(run, plan_path_selection(run).columns),
        "wall_s": _final(run, "ElapsedWallTimeSec", summary.get("elapsed_wall_time_s")),
        "start": start,
        "build": build,
        "flags": _run_flags(run),
    }


# Planner display name -> run-directory name.
PLANNER_FOLDERS = {
    "NBS": "nbs",
    "DBS": "dbs",
    "SPT": "spt",
    "TSP": "tsp",
    "RRT*": "rrtstar",
    "RRT": "rrt",
}


def planner_label(folder):
    """A planner folder's legend label: a shipped planner's display name, else the folder name."""
    return {name: label for label, name in PLANNER_FOLDERS.items()}.get(folder, folder)


# Task -> y-axis label; the task is read from the folder path (paper_sweep.py's <task>/ level).
TASK_LABELS = {
    "points": "Collected point gain",
    "recon": r"Surface reconstruction [$\mathrm{m^2}$]",
    "explore": r"Explored volume [$\mathrm{m^3}$]",
}


def resolve_scene_folder(folder_name):
    """The scene folder below the data root, where run_experiment_sim.launch writes runs."""
    return data_root() / folder_name


def infer_task(folder_name):
    """The task a folder path names as a component (paper_sweep.py's <task>/), else ``explore``."""
    parts = Path(folder_name).parts
    for task in TASK_LABELS:
        if task in parts:
            return task
    return "explore"


def generate_config(base_settings, data_root=None):
    """Resolve data folders, colors, and the axis columns from base settings."""
    config = dict(base_settings)
    if data_root is not None:
        data_folder = Path(data_root) / config["folder_name"]
    else:
        data_folder = resolve_scene_folder(config["folder_name"])
    if not data_folder.is_dir():
        raise FileNotFoundError(
            "no such data folder: %s (record runs with run_experiment_sim.launch or "
            "paper_sweep.py first, or pass --data-root)" % data_folder
        )
    print("Scene folder: %s" % data_folder)
    folders = {planner_label(name): data_folder / name for name in config["planners"]}
    config["data_folder"] = data_folder
    config["folders"] = folders

    sns.set_style("darkgrid")
    palette = sns.color_palette("tab10")
    colors = {
        "NBS": palette[0],
        "DBS": palette[1],
        "SPT": palette[2],
        "TSP": palette[4],
        "RRT*": palette[5],
        "RRT": palette[7],
    }
    spare = [palette[index] for index in (3, 6, 8, 9)]
    custom = [label for label in folders if label not in colors]
    for index, label in enumerate(custom):
        colors[label] = spare[index % len(spare)]
    config["alg_colors"] = colors
    return config


def extract_series(run, display_keys):
    """Extract one run's per-row series for each display key.

    Returns (data, None) on success or (None, reason) if a required column is
    missing, so the caller can account for the skip explicitly.
    """
    resolved = {}
    for key in display_keys:
        if key == "Planning Time":
            # Manifest-driven, layer-aware scope (includes StartNodeUpdate).
            # An empty scope or one lacking PlanPathTotal is not a valid
            # planning-time series -> skip the run instead of plotting zeros.
            selection = planning_time_selection(run)
            if not selection.columns or "PlanPathTotal" not in selection.columns:
                detail = "; ".join(selection.diagnostics) or "no PlanPathTotal / no timer columns"
                return None, "planning-time scope unusable [%s]" % detail
            cols = list(selection.columns)
        else:
            cols = COLUMN_MAP[key]
        missing = run.missing_columns(cols)
        if missing:
            return None, "missing %s (for '%s')" % (", ".join(missing), key)
        resolved[key] = cols

    data = {}
    n_rows = run.num_rows
    for key, cols in resolved.items():
        series = []
        running = 0.0
        for i in range(n_rows):
            value = sum(run.columns[c][i] for c in cols)
            if key in NEED_ACCUMULATION:
                running += value
                value = running
            series.append(value)
        data[key] = series
    return data, None


def _cell(value, fmt, width):
    """Render one table cell, right-aligned, with '-' for an absent value."""
    text = "-" if value is None else fmt % value
    return text.rjust(width)


def print_run_table(summaries, kept):
    """Print the ranked per-run table; ``kept`` is how many --top-k keeps."""
    header = "  ".join(name.rjust(width) for name, _key, width, _fmt in SUMMARY_COLUMNS)
    print("    %s" % header)
    for rank, summary in enumerate(summaries, start=1):
        summary["rank"] = rank
        cells = [_cell(summary[key], fmt, width) for _n, key, width, fmt in SUMMARY_COLUMNS]
        notes = list(summary["flags"])
        if rank > kept:
            notes.insert(0, "dropped: --top-k")
        line = "    %s" % "  ".join(cells)
        if notes:
            line += "  [%s]" % "; ".join(notes)
        print(line)


def print_config_census(runs):
    """Report each config field, calling out the runs that disagree."""
    for name, read in CONFIG_FIELDS:
        by_value = {}
        for run in runs:
            by_value.setdefault(read(run), []).append(run.timestamp)
        if len(by_value) == 1:
            value, _names = next(iter(by_value.items()))
            print("    config %s: %s (all runs)" % (name, "-" if value is None else value))
            continue
        print("    config %s: DIFFERS" % name)
        for value, names in sorted(by_value.items(), key=lambda kv: -len(kv[1])):
            print("      %s: %s" % ("-" if value is None else value, ", ".join(names)))


def read_from_folder(folder_path, config):
    """Load and extract every run under one algorithm folder, with accounting."""
    display_keys = set(config["y_axes"]) | {config["x_axis"]}
    data_list = []
    summaries = []
    gains = []
    loaded = 0
    skipped = []

    for item in sorted(folder_path.iterdir()):
        if not item.is_dir():
            continue
        run = load_run(item)
        if not run.loaded:
            skipped.append((item.name, run.reason))
            continue
        data, reason = extract_series(run, display_keys)
        if data is None:
            skipped.append((item.name, reason))
            continue
        # Interpolation needs at least two points.
        if run.num_rows < 2:
            skipped.append((item.name, "too few rows"))
            continue
        # The curves share one grid, so a run must cover its start.
        start = interpolation_start(config["x_axis"])
        x = data[config["x_axis"]]
        if start is not None and not min(x) <= start <= max(x):
            skipped.append(
                (
                    item.name,
                    "%s spans %g to %g, not the curves' start %g"
                    % (config["x_axis"], min(x), max(x), start),
                )
            )
            continue
        data_list.append((data, run))
        # Ranked on the gain column, not on a plotted series, so the order holds
        # for any --x-axis/--y-axes selection.
        summary = summarize_run(run)
        summaries.append(summary)
        gains.append(summary["gain"] or 0.0)
        loaded += 1

    print("  %s: loaded %d, skipped %d" % (folder_path.name, loaded, len(skipped)))
    for name, reason in skipped:
        print("    [skip] %s: %s" % (name, reason))

    top_k = config["top_k"]
    order = np.argsort(-np.array(gains)) if gains else np.array([], dtype=int)
    if data_list:
        print_run_table([summaries[i] for i in order], top_k if top_k else len(order))
        print_config_census([data_list[i][1] for i in order])
    if top_k:
        order = order[:top_k]
    return [data_list[i] for i in order]


def plot_data(data_dicts, config, std_factor=1.0):
    """Plot interpolated mean curves with std bands, plus an optional ground-truth legend entry."""
    x_axis = config["x_axis"]
    y_axes = config["y_axes"]
    # Supplied by the caller only; a run never records it (0 = no ground-truth entry).
    ground_truth = config["ground_truth"]

    fig, axes = plt.subplots(1, len(y_axes), figsize=(4 * len(y_axes), 3), squeeze=False)

    for y_axis, ax in zip(y_axes, axes.flat):
        for label, runs in data_dicts.items():
            x_list = [data[x_axis] for data, _run in runs]
            y_list = [data[y_axis] for data, _run in runs]

            fixed_x, mean_values, std_values = calculate_mean_and_std_interpolated(
                x_list, y_list, config["num_steps"], start=interpolation_start(x_axis)
            )
            color = config["alg_colors"][label]
            legend_label = "%d: %s" % (int(round(mean_values[-1])), label)
            ax.plot(fixed_x, mean_values, label=legend_label, color=color)
            ax.fill_between(
                fixed_x,
                mean_values - std_factor * std_values,
                mean_values + std_factor * std_values,
                color=color,
                alpha=0.2,
            )

        ax.set_xlabel(x_axis)
        if y_axis == "Accumulated Gain":
            ax.set_ylabel(TASK_LABELS[config["task"]])
        else:
            ax.set_ylabel(y_axis)

        main_legend = ax.legend(loc="upper left")
        ax.add_artist(main_legend)

        # The ground-truth legend entry appears only when a value is given.
        if ground_truth:
            (dummy_handle,) = ax.plot([], [], linestyle="None", label="GT = %s" % ground_truth)
            ax.legend(handles=[dummy_handle], loc="lower right", handlelength=0, handletextpad=0)

        ax.grid(True)
        ax.autoscale(tight=True)

    plt.tight_layout()
    config["output_path"].parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(config["output_path"], format="pdf", dpi=600, bbox_inches="tight")


def main(argv=None):
    """Read experiment data from all algorithm folders and generate plots."""
    use_file_backend()
    parser = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "folder_name",
        nargs="?",
        default="HSSD/102816852",
        help="Scene folder holding one sub-directory per algorithm, e.g. "
        "HSSD/<scene> or, for a paper_sweep.py root, <task>/HSSD/<scene>.",
    )
    parser.add_argument(
        "--data-root",
        type=Path,
        default=argparse.SUPPRESS,
        help="Directory containing folder_name (default: the data root, $%s or "
        "~/open_apero_data)." % DATA_ROOT_VARIABLE,
    )
    parser.add_argument(
        "--task",
        choices=sorted(TASK_LABELS),
        default=argparse.SUPPRESS,
        help="Task that labels the gain axis (default: read from folder_name, else explore).",
    )
    parser.add_argument(
        "--planners",
        nargs="+",
        default=list(PLANNER_FOLDERS.values()),
        metavar="FOLDER",
        help="Planner folders below folder_name to read; one not of a shipped planner is "
        "labeled with its name.",
    )
    parser.add_argument(
        "--ground-truth",
        type=float,
        default=0.0,
        help="Ground-truth value shown as a legend entry, supplied by hand; 0 disables it.",
    )
    parser.add_argument(
        "--num-steps", type=int, default=100, help="Samples on the shared interpolation grid."
    )
    parser.add_argument(
        "--top-k", type=int, default=0, help="Keep only the k best runs per algorithm; 0 keeps all."
    )
    parser.add_argument(
        "--x-axis",
        default="Cost",
        choices=sorted(set(COLUMN_MAP) | {"Planning Time"}),
        help="Quantity on the x axis.",
    )
    parser.add_argument(
        "--y-axes",
        nargs="+",
        default=["Accumulated Gain"],
        choices=sorted(set(COLUMN_MAP) | {"Planning Time"}),
        help="One plot per named quantity.",
    )
    add_figure_arguments(parser)
    args = parser.parse_args(argv)

    if args.num_steps <= 0:
        parser.error("--num-steps must be positive")
    if args.top_k < 0:
        parser.error("--top-k cannot be negative")

    settings = {
        "folder_name": args.folder_name,
        "task": getattr(args, "task", None) or infer_task(args.folder_name),
        "planners": args.planners,
        "top_k": args.top_k,
        "ground_truth": args.ground_truth,
        "num_steps": args.num_steps,
        "x_axis": args.x_axis,
        "y_axes": args.y_axes,
    }
    try:
        config = generate_config(settings, data_root=getattr(args, "data_root", None))
    except FileNotFoundError as exc:
        parser.error(str(exc))
    config["output_path"] = figure_path(
        args,
        config["data_folder"],
        figure_name(args.x_axis, args.y_axes, args.top_k, args.planners),
    )

    data_dicts = {}
    for label, folder in config["folders"].items():
        if not folder.exists():
            print("[WARN] Folder %s does not exist." % folder)
            continue
        print("Reading %s" % folder)
        runs = read_from_folder(folder, config)
        if not runs:
            print("[WARN] No usable runs in %s, omitting '%s'." % (folder, label))
            continue
        data_dicts[label] = runs

    if not data_dicts:
        parser.error("no usable runs in any planner folder under %s" % config["data_folder"])
    plot_data(data_dicts, config, 0.5)
    finish_figure(
        config["output_path"],
        [run.path for runs in data_dicts.values() for _data, run in runs],
        command_line("plot_performance_curves.py", argv),
        open_file=opens_viewer(args),
        extra={"task": config["task"], "x_axis": args.x_axis, "y_axes": list(args.y_axes)},
    )


if __name__ == "__main__":
    main()
