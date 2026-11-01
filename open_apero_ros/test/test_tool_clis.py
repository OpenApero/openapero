"""Command-line behavior of the hand-run tools (open_apero_ros modules, run from scripts/)."""

import csv
import os
import pathlib
import queue
import re
import signal
import socket
import subprocess
import tempfile
import time

import matplotlib

matplotlib.use("Agg")  # Import-time: the tools import pyplot at module scope.

import matplotlib.pyplot as plt  # noqa: E402
import pytest  # noqa: E402

from open_apero_analysis import schema_fingerprint  # noqa: E402
from open_apero_ros import (  # noqa: E402
    compare_paper_sweep,
    evaluate_experiment,
    generate_points,
    paper_sweep,
    plot_normalized_gain,
    plot_performance_curves,
)


@pytest.fixture(autouse=True)
def no_window(monkeypatch):
    """Figure tools write their file and exit; generate_points' preview is off (--no-show) here."""
    monkeypatch.setattr(plt, "show", lambda *args, **kwargs: pytest.fail("a tool opened a window"))


def _run(module, argv):
    """Call a tool's main() and return the SystemExit code, or None if it returned."""
    try:
        module.main(argv)
    except SystemExit as exc:
        return exc.code
    return None


def test_generate_points_writes_requested_count():
    with tempfile.TemporaryDirectory() as folder:
        out = os.path.join(folder, "points.csv")
        assert _run(generate_points, ["--num-points", "7", "--output", out, "--no-show"]) is None
        with open(out) as handle:
            rows = list(csv.reader(handle))
        assert len(rows) == 7
        # x, y, z, collection radius, gain
        assert all(len(row) == 5 for row in rows)


def test_generate_points_is_reproducible_with_a_seed():
    with tempfile.TemporaryDirectory() as folder:
        first = os.path.join(folder, "a.csv")
        second = os.path.join(folder, "b.csv")
        for out in (first, second):
            _run(
                generate_points, ["--num-points", "20", "--seed", "7", "--output", out, "--no-show"]
            )
        assert open(first).read() == open(second).read()


def test_generate_points_honors_the_requested_bounds():
    with tempfile.TemporaryDirectory() as folder:
        out = os.path.join(folder, "points.csv")
        _run(
            generate_points,
            [
                "--num-points",
                "50",
                "--x-range",
                "1",
                "2",
                "--collection-radius",
                "0.25",
                "--output",
                out,
                "--no-show",
            ],
        )
        with open(out) as handle:
            rows = list(csv.reader(handle))
        assert all(1.0 <= float(row[0]) <= 2.0 for row in rows)
        assert all(float(row[3]) == 0.25 for row in rows)


def test_generate_points_creates_a_missing_output_parent():
    with tempfile.TemporaryDirectory() as folder:
        out = os.path.join(folder, "nested", "deeper", "points.csv")
        assert _run(generate_points, ["--num-points", "3", "--output", out, "--no-show"]) is None
        assert os.path.isfile(out)


def test_plot_normalized_gain_writes_the_requested_file_creating_its_parent():
    with tempfile.TemporaryDirectory() as folder:
        out = os.path.join(folder, "nested", "barplots.pdf")
        assert _run(plot_normalized_gain, ["--output", out]) is None
        assert os.path.getsize(out) > 0
        # The record beside it says what drew the figure.
        assert os.path.isfile(os.path.join(folder, "nested", "barplots.yaml"))


def test_plot_normalized_gain_sits_at_the_top_of_the_figure_tree():
    with tempfile.TemporaryDirectory() as folder:
        assert _run(plot_normalized_gain, ["--out-dir", folder]) is None
        assert os.path.getsize(os.path.join(folder, "normalized_gain_barplots.pdf")) > 0


def test_plot_performance_curves_names_the_figure_after_what_it_plots():
    assert (
        plot_performance_curves.figure_name("Cost", ["Accumulated Gain", "Num Nodes"])
        == "curves_accumulated-gain_num-nodes_vs_cost.pdf"
    )
    assert (
        plot_performance_curves.figure_name("Planning Time", ["Accumulated Gain"], top_k=2)
        == "curves_accumulated-gain_vs_planning-time_top2.pdf"
    )
    shipped = list(plot_performance_curves.PLANNER_FOLDERS.values())
    assert plot_performance_curves.figure_name(
        "Cost", ["Accumulated Gain"], planners=shipped
    ) == plot_performance_curves.figure_name("Cost", ["Accumulated Gain"])
    assert (
        plot_performance_curves.figure_name(
            "Cost", ["Accumulated Gain"], planners=["nbs", "my_planner"]
        )
        == "curves_accumulated-gain_vs_cost_nbs-my_planner.pdf"
    )


def test_plot_performance_curves_starts_the_grid_at_50_only_on_the_cost_axis():
    # Planning time never reaches 50 s in a short run, so that axis takes the common interval.
    assert plot_performance_curves.interpolation_start("Cost") == 50.0
    assert plot_performance_curves.interpolation_start("Planning Time") is None


def test_plot_performance_curves_infers_the_task_from_the_folder_path():
    assert plot_performance_curves.infer_task("points/HSSD/102816852") == "points"
    assert plot_performance_curves.infer_task("recon/HSSD/102816852") == "recon"
    assert plot_performance_curves.infer_task("HSSD/102816852") == "explore"


def test_plot_performance_curves_rejects_a_folder_without_runs():
    with tempfile.TemporaryDirectory() as folder:
        os.makedirs(os.path.join(folder, "HSSD", "1", "nbs"))
        assert _run(plot_performance_curves, ["HSSD/1", "--data-root", folder]) == 2


@pytest.mark.parametrize(
    "argv",
    [
        ["--num-points", "0"],
        ["--num-points", "-5"],
        ["--collection-radius", "0"],
        ["--x-range", "5", "1"],
        ["--gain-range", "9", "2"],
    ],
)
def test_generate_points_rejects_invalid_arguments(argv):
    assert _run(generate_points, argv + ["--no-show"]) == 2


@pytest.mark.parametrize(
    "argv",
    [
        ["--num-steps", "0"],
        ["--top-k", "-1"],
        ["--data-root", "/nonexistent-data-root"],
    ],
)
def test_plot_performance_curves_rejects_invalid_arguments(argv):
    assert _run(plot_performance_curves, argv) == 2


def _write_manifest(run_dir, header):
    """Write the run_manifest.yaml of a log with ``header``, every column a root timer [s]."""
    columns = [(name, "double", "s", "") for name in header]
    lines = [
        "format_version: 1",
        'schema_fingerprint: "%s"' % schema_fingerprint(columns),
        "layers:",
        '  "backtracking": "backtracking"',
        '  "planning": "planning"',
        "columns:",
    ]
    lines += ['  - {name: "%s", type: "%s", unit: "%s", parent: "%s"}' % c for c in columns]
    with open(os.path.join(run_dir, "run_manifest.yaml"), "w") as handle:
        handle.write("\n".join(lines) + "\n")


# Enough of the schema for a run to load and for the summary table to fill in.
_SUMMARY_HEADER = (
    "CycleIndex,TotalCost,TotalPathLength,TotalPathDuration,ElapsedWallTimeSec,"
    "ReplanTimeWall,PlanPathTotal,planning.PlanPathTime,"
    "backtracking.PlanPathTime,planning.ExpandTime,planning.UpdateTime,"
    "planning.RealAccumulatedGain"
)


def _write_summary_run(folder, name, gains, costs=None):
    """Write a run whose last row holds the given final values; TotalCost is 100, 101, ... unless
    ``costs`` gives it per row."""
    run_dir = os.path.join(folder, name)
    os.makedirs(run_dir)
    _write_manifest(run_dir, _SUMMARY_HEADER.split(","))
    rows = [_SUMMARY_HEADER]
    for index, gain in enumerate(gains):
        cost = 100.0 + index if costs is None else costs[index]
        rows.append(
            "%d,%f,10.0,%f,%f,1.0,3.0,2.0,1.0,0.5,0.25,%f"
            % (index, cost, 100.0 + index, 50.0 + index, gain)
        )
    with open(os.path.join(run_dir, "performance_log.csv"), "w") as handle:
        handle.write("\n".join(rows) + "\n")
    return run_dir


def test_plot_performance_curves_summarizes_a_run_from_its_columns():
    with tempfile.TemporaryDirectory() as folder:
        run_dir = _write_summary_run(folder, "20260101_000000", [1.0, 7.5])
        summary = plot_performance_curves.summarize_run(plot_performance_curves.load_run(run_dir))
        assert summary["run"] == "20260101_000000"
        assert summary["rows"] == 2
        assert summary["gain"] == 7.5
        assert summary["cost"] == 101.0
        assert summary["duration"] == 101.0
        assert summary["wall_s"] == 51.0  # elapsed episode wall clock, last row
        # Planning layer's own planPath, summed over both rows: neither the
        # backtracking layer nor the expand/update graph work counts.
        assert summary["plan_path_s"] == 4.0
        assert summary["start"] is None and summary["build"] is None  # not in this manifest
        assert any("DEGRADED" in flag for flag in summary["flags"])


def test_plot_performance_curves_prints_one_ranked_table_line_per_loaded_run(capsys):
    with tempfile.TemporaryDirectory() as folder:
        _write_summary_run(folder, "20260101_000000", [1.0, 3.0], costs=[0.0, 100.0])
        _write_summary_run(folder, "20260102_000000", [1.0, 9.0], costs=[0.0, 100.0])
        config = {"x_axis": "Cost", "y_axes": ["Accumulated Gain"], "top_k": 1}
        kept = plot_performance_curves.read_from_folder(pathlib.Path(folder), config)

        assert len(kept) == 1  # --top-k still trims the returned runs
        lines = [line for line in capsys.readouterr().out.splitlines() if "202601" in line]
        assert len(lines) == 2
        # Ranked by final gain, and the run --top-k drops is marked, not hidden.
        assert lines[0].split()[:2] == ["1", "20260102_000000"]
        assert lines[1].split()[:2] == ["2", "20260101_000000"]
        assert "dropped: --top-k" in lines[1] and "dropped: --top-k" not in lines[0]


def test_plot_performance_curves_skips_a_run_whose_cost_misses_the_grid_start(capsys):
    with tempfile.TemporaryDirectory() as folder:
        data_root = os.path.join(folder, "data")
        planner = os.path.join(data_root, "HSSD", "1", "nbs")
        _write_summary_run(planner, "20260101_000000", [1.0, 9.0], costs=[0.0, 100.0])
        _write_summary_run(
            planner, "20260102_000000", [1.0, 3.0], costs=[0.0, 42.2]
        )  # stopped early
        _write_summary_run(planner, "20260103_000000", [1.0, 3.0], costs=[60.0, 100.0])

        figure = os.path.join(folder, "curves.pdf")
        argv = ["HSSD/1", "--data-root", data_root, "--output", figure]
        assert _run(plot_performance_curves, argv) is None

        out = capsys.readouterr().out
        assert "nbs: loaded 1, skipped 2" in out
        assert "[skip] 20260102_000000: Cost spans 0 to 42.2, not the curves' start 50" in out
        assert "[skip] 20260103_000000: Cost spans 60 to 100, not the curves' start 50" in out
        assert os.path.getsize(figure) > 0


def test_plot_performance_curves_reads_the_planner_folders_named(capsys):
    assert plot_performance_curves.planner_label("rrtstar") == "RRT*"
    assert plot_performance_curves.planner_label("my_planner") == "my_planner"
    with tempfile.TemporaryDirectory() as folder:
        planner = os.path.join(folder, "HSSD", "1", "my_planner")
        _write_summary_run(planner, "20260101_000000", [1.0, 9.0], costs=[0.0, 100.0])
        argv = ["HSSD/1", "--data-root", folder, "--output", os.path.join(folder, "curves.pdf")]

        assert _run(plot_performance_curves, argv) == 2  # the shipped planners' folders only
        assert _run(plot_performance_curves, argv + ["--planners", "my_planner"]) is None
        assert "Reading %s" % planner in capsys.readouterr().out


# As the planner writes it: the last executed edge is in the summary but in no row.
_EPISODE_SUMMARY = """format_version: 1
end_reason: budget_exhausted
cycles: 2
total_cost: 108.5
cost_budget: .inf
total_path_length: 12.5
total_path_duration: 108.5
num_visited_nodes: 3
elapsed_wall_time_s: 55.25
real_accumulated_gain:
  "planning": 11.5
"""


def _write_episode_summary(run_dir):
    with open(os.path.join(run_dir, "episode_summary.yaml"), "w") as handle:
        handle.write(_EPISODE_SUMMARY)


def test_plot_performance_curves_takes_final_values_from_the_episode_summary():
    with tempfile.TemporaryDirectory() as folder:
        run_dir = _write_summary_run(folder, "20260101_000000", [1.0, 7.5])
        _write_episode_summary(run_dir)
        summary = plot_performance_curves.summarize_run(plot_performance_curves.load_run(run_dir))
        assert summary["gain"] == 11.5
        assert summary["cost"] == 108.5
        assert summary["length"] == 12.5
        assert summary["duration"] == 108.5
        assert summary["wall_s"] == 55.25
        assert summary["rows"] == 2


def test_paper_sweep_reads_the_episode_summary():
    with tempfile.TemporaryDirectory() as folder:
        run_dir = _write_summary_run(folder, "20260101_000000", [1.0, 7.5])
        _write_episode_summary(run_dir)
        summary = paper_sweep.read_episode_summary(run_dir)
        assert summary["end_reason"] == "budget_exhausted"
        assert summary["total_cost"] == 108.5
        assert summary["cost_budget"] == float("inf")
        assert summary["real_accumulated_gain"] == {"planning": 11.5}
        assert paper_sweep.read_episode_summary(folder) is None


def test_paper_sweep_final_cost_prefers_the_summary_over_the_last_row():
    with tempfile.TemporaryDirectory() as folder:
        run_dir = _write_summary_run(folder, "20260101_000000", [1.0, 7.5])
        # A run without a summary: the last row's TotalCost.
        assert paper_sweep.read_final_cost(run_dir) == 101.0
        _write_episode_summary(run_dir)
        assert paper_sweep.read_final_cost(run_dir) == 108.5
        assert paper_sweep.read_final_cost(folder) is None


def test_paper_sweep_with_nothing_to_run_waits_for_no_lane(monkeypatch):
    monkeypatch.setattr(paper_sweep.signal, "signal", lambda *_args: None)  # keep pytest's handlers
    with tempfile.TemporaryDirectory() as folder:
        try:
            start = time.monotonic()
            assert (
                paper_sweep.main(["--data-root", folder, "--trials", "0", "--parallel", "3"]) == 0
            )
            assert time.monotonic() - start < 10.0  # a lane start used to wait 20 s regardless
        finally:
            paper_sweep.stop_dispatch.clear()
        with open(os.path.join(folder, "sweep.log")) as handle:
            assert "SWEEP END 0 runs executed" in handle.read()


def test_paper_sweep_kills_a_run_s_processes_in_sessions_of_their_own():
    # roslaunch starts the master and each node like this, so a killpg of roslaunch misses them.
    tag = "test.%d" % os.getpid()
    tagged = subprocess.Popen(
        ["sleep", "60"], env=dict(os.environ, **{paper_sweep.RUN_TAG: tag}), start_new_session=True
    )
    untagged = subprocess.Popen(["sleep", "60"], start_new_session=True)
    try:
        assert paper_sweep.kill_tagged(tag) == 1
        assert tagged.wait(timeout=5) == -signal.SIGKILL
        assert untagged.poll() is None
    finally:
        for proc in (tagged, untagged):
            proc.kill()
            proc.wait()


def test_paper_sweep_lane_stops_at_a_busy_port_and_requeues_its_job(monkeypatch):
    wait_for_port = paper_sweep.wait_for_port
    monkeypatch.setattr(paper_sweep, "wait_for_port", lambda *args: wait_for_port(*args, limit=0))
    monkeypatch.setattr(paper_sweep, "launch", lambda *_args: pytest.fail("launched onto a master"))
    job = {"trial": 1, "scene": "102816852", "task": "points", "planner": "nbs"}
    jobs = queue.Queue()
    jobs.put(job)
    with socket.socket() as master, tempfile.TemporaryDirectory() as folder:
        master.bind(("", 0))
        master.listen()
        sweep_log = os.path.join(folder, "sweep.log")
        paper_sweep.worker(0, master.getsockname()[1], jobs, None, None, sweep_log, {})
        assert jobs.get_nowait() is job
        with open(sweep_log) as handle:
            assert "still busy after 0s: the lane stops" in handle.read()


def test_paper_sweep_logs_and_counts_a_lane_crash(monkeypatch):
    def crash(*_args):
        raise FileNotFoundError("ss")

    monkeypatch.setattr(paper_sweep, "worker", crash)
    stats = {}
    with tempfile.TemporaryDirectory() as folder:
        sweep_log = os.path.join(folder, "sweep.log")
        paper_sweep.guarded_worker(2, 11413, None, None, None, sweep_log, stats)
        with open(sweep_log) as handle:
            text = handle.read()
    assert "lane2 CRASHED" in text and "FileNotFoundError: ss" in text
    assert stats == {"crashed": 1}


def test_compare_paper_sweep_reads_the_gain_at_the_episode_end_nearest_the_budget():
    with tempfile.TemporaryDirectory() as folder:
        run_dir = _write_summary_run(folder, "20260101_000000", [1.0, 7.5])
        log = os.path.join(run_dir, "performance_log.csv")
        assert compare_paper_sweep.gain_at_budget(log, 110.0) == (7.5, 101.0)
        _write_episode_summary(run_dir)
        assert compare_paper_sweep.gain_at_budget(log, 110.0) == (11.5, 108.5)
        assert compare_paper_sweep.gain_at_budget(log, 100.0) == (1.0, 100.0)


def test_compare_paper_sweep_takes_the_budget_from_the_episode_summary():
    with tempfile.TemporaryDirectory() as folder:
        run_dir = _write_summary_run(folder, "20260101_000000", [1.0, 7.5])
        # No summary, then one whose cost_budget is .inf: the protocol's budget for the scene.
        assert compare_paper_sweep.run_budget(run_dir, "102344280") == 600.0
        _write_episode_summary(run_dir)
        assert compare_paper_sweep.run_budget(run_dir, "102816852") == 300.0
        with open(os.path.join(run_dir, "episode_summary.yaml"), "w") as handle:
            handle.write(_EPISODE_SUMMARY.replace("cost_budget: .inf", "cost_budget: 450.0"))
        assert compare_paper_sweep.run_budget(run_dir, "102344280") == 450.0


@pytest.mark.parametrize(
    "argv",
    [
        ["/nonexistent-run-dir"],
        ["--method", "bogus", "."],
    ],
)
def test_evaluate_experiment_rejects_invalid_arguments(argv):
    assert _run(evaluate_experiment, argv) == 2


def test_evaluate_experiment_requires_a_timestamped_run_for_recent():
    with tempfile.TemporaryDirectory() as folder:
        assert _run(evaluate_experiment, [folder]) == 2


def test_evaluate_experiment_skips_a_run_without_a_performance_log():
    with tempfile.TemporaryDirectory() as folder:
        assert _run(evaluate_experiment, [folder, "--method", "single"]) is None


def _write_overview_run(folder, name, cycles=6, manifest=True):
    """Write a run holding every column the overview needs, one simulated second per cycle."""
    run_dir = os.path.join(folder, name)
    os.makedirs(run_dir)
    header = list(evaluate_experiment.REQUIRED_PERF_COLUMNS)
    if manifest:
        _write_manifest(run_dir, header)
    rows = [",".join(header)]
    for index in range(cycles):
        rows.append(
            ",".join("%d" % (index + 1) if c.startswith("planning.Num") else "1.0" for c in header)
        )
    with open(os.path.join(run_dir, "performance_log.csv"), "w") as handle:
        handle.write("\n".join(rows) + "\n")
    return run_dir


def test_evaluate_experiment_writes_into_the_figure_tree_and_only_reads_the_run(monkeypatch):
    with tempfile.TemporaryDirectory() as folder:
        data_root = os.path.join(folder, "data")
        monkeypatch.setenv("OPEN_APERO_DATA_ROOT", data_root)
        planner = os.path.join(data_root, "HSSD", "1", "nbs")
        run_dir = _write_overview_run(planner, "20260101_000000")

        assert _run(evaluate_experiment, [planner]) is None

        figures = os.path.join(data_root, "figures", "HSSD", "1", "nbs", "20260101_000000")
        assert os.path.getsize(os.path.join(figures, "performance_overview.png")) > 0
        assert os.path.isfile(os.path.join(figures, "performance_overview.yaml"))
        assert sorted(os.listdir(run_dir)) == [
            "performance_log.csv",
            "run_manifest.yaml",
        ], "the run is read only"

        # --out-dir moves the tree; the run's path below the data root stays.
        out_dir = os.path.join(folder, "elsewhere")
        assert (
            _run(evaluate_experiment, [run_dir, "--method", "single", "--out-dir", out_dir]) is None
        )
        assert os.path.isfile(
            os.path.join(out_dir, "HSSD", "1", "nbs", "20260101_000000", "performance_overview.png")
        )


def test_evaluate_experiment_skips_a_run_without_a_manifest(monkeypatch, capsys):
    with tempfile.TemporaryDirectory() as folder:
        monkeypatch.setenv("OPEN_APERO_DATA_ROOT", folder)
        run_dir = _write_overview_run(folder, "20260101_000000", manifest=False)
        assert _run(evaluate_experiment, [run_dir, "--method", "single"]) is None
        assert "[skip] no CPU-time timer scope" in capsys.readouterr().out
        assert not os.path.exists(os.path.join(folder, "figures"))


def test_tools_expose_a_main_accepting_argv():
    # main(argv=None) is what lets these tests drive the tools in-process.
    for module in (
        evaluate_experiment,
        generate_points,
        plot_normalized_gain,
        plot_performance_curves,
    ):
        assert callable(module.main)
        assert _run(module, ["--help"]) == 0


def _option_blocks(help_text):
    """Split --help output into {option: its help text}."""
    blocks = {}
    for block in re.split(r"\n(?=  -)", help_text.split("optional arguments:")[-1]):
        words = block.split()
        if words and words[0].startswith("-"):
            blocks[words[1] if words[0] == "-h," else words[0]] = " ".join(words)
    return blocks


@pytest.mark.parametrize("module", [paper_sweep, compare_paper_sweep])
def test_sweep_tools_document_every_option_with_its_default(module, capsys):
    assert _run(module, ["--help"]) == 0
    blocks = _option_blocks(capsys.readouterr().out)
    blocks.pop("--help")
    assert len(blocks) >= 5
    for option, text in blocks.items():
        required = module is compare_paper_sweep and option == "--data-root"
        assert ("(default:" in text) != required, text
