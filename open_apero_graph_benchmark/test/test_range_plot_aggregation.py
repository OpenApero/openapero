"""Pin range-plot aggregation, CLI configuration, and dataset-family behavior."""

import os
import tempfile
from pathlib import Path

import matplotlib

# Must precede any pyplot import: the suite has to run identically with and without a DISPLAY.
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pytest

from open_apero_graph_benchmark import (
    benchmark_ranges,
    plot_computation_time_ranges,
    plot_final_gain_ranges,
)


# Gain climbs to 100 while computation_time climbs to 7, so a test can tell which column
# an aggregation actually read.
def result_rows(gain_scale):
    """Five columns: gain, cost, computation_time, node_id, num_nodes."""
    return "".join(f"{gain_scale * step} {step} {step * 0.7} {step} 40\n" for step in range(1, 11))


FINAL_GAIN = 100.0
FINAL_TIME = 7.0

# What plot_final_gain_ranges.main() reads for an "ap_" dataset: every replan/objective
# pair except the skipped ("none", "path_ratio").
FINAL_GAIN_COMBINATIONS = (
    ("none", "path_gain"),
    ("goal", "path_gain"),
    ("goal", "path_ratio"),
    ("everystep", "path_gain"),
    ("everystep", "path_ratio"),
)

# plot_computation_time_ranges.main() reads path_gain for both supported replan types.
COMPUTATION_TIME_COMBINATIONS = (("none", "path_gain"), ("everystep", "path_gain"))


def build_dataset(root, combinations, graphs=("graph_0", "graph_1"), filename="nbs_1.txt"):
    """Create ``<graph>/<replan>/<objective>/<filename>`` under ``root``."""
    for graph in graphs:
        for replan_type, objective in combinations:
            folder = root / graph / replan_type / objective
            folder.mkdir(parents=True, exist_ok=True)
            (folder / filename).write_text(result_rows(10))
    return root


class NoWindow:
    """Context manager that fails the test if a script's main() opens a window.

    A figure tool writes its file and exits; ``--open`` hands the file to the system viewer.
    """

    def __enter__(self):
        self._original_show = plt.show

        def show(*args, **kwargs):
            raise AssertionError("a figure tool opened a window")

        plt.show = show
        return self

    def __exit__(self, *exc_info):
        plt.show = self._original_show
        return False


class Recorder:
    """Stands in for plot_horizontal_ranges and keeps what it was handed."""

    def __init__(self):
        self.summary = None
        self.calls = 0
        self.kwargs = None
        self.main_result = None

    def __call__(self, summary_data, *args, **kwargs):
        self.summary = summary_data
        self.calls += 1
        self.kwargs = kwargs


# Both scripts classify the resolved dataset directory by its ``ap_``/``op_`` basename.
A_PRIORI_DATASET_PREFIX = "ap_test_fixture"


def run_script(
    module,
    combinations,
    dataset_prefix=A_PRIORI_DATASET_PREFIX,
    monkeypatch_target="plot_horizontal_ranges",
    extra_args=(),
):
    """Run ``module.main()`` against a synthetic dataset and return the recorded summary.

    The figure tree goes into the temporary directory too, so a test writes only there.
    """
    with tempfile.TemporaryDirectory(prefix=f"{dataset_prefix}_") as directory:
        root = build_dataset(Path(directory) / dataset_prefix, combinations)
        recorder = Recorder()
        original_plotter = getattr(module, monkeypatch_target)
        try:
            setattr(module, monkeypatch_target, recorder)
            with NoWindow():
                result = module.main(
                    [str(root), "--out-dir", os.path.join(directory, "figures"), *extra_args]
                )
        finally:
            setattr(module, monkeypatch_target, original_plotter)
        recorder.main_result = result
        return recorder


def test_final_gain_script_aggregates_the_gain_column():
    """The final-gain plot must summarize column 1, not the timing column."""
    recorder = run_script(plot_final_gain_ranges, FINAL_GAIN_COMBINATIONS)
    assert recorder.calls == 1
    assert recorder.main_result is None
    for key, per_algorithm in recorder.summary.items():
        minimum, maximum = per_algorithm["nbs"]
        assert minimum == maximum, f"identical graphs must collapse to a point for {key}"
        assert abs(minimum - FINAL_GAIN) < 1e-6, f"{key}: expected the gain column"


def test_computation_time_script_aggregates_the_timing_column():
    """The computation-time plot must summarize column 3."""
    recorder = run_script(plot_computation_time_ranges, COMPUTATION_TIME_COMBINATIONS)
    assert recorder.calls == 1
    for key, per_algorithm in recorder.summary.items():
        minimum, maximum = per_algorithm["nbs"]
        assert abs(minimum - FINAL_TIME) < 1e-6, f"{key}: expected the computation_time column"


def test_final_gain_script_covers_every_unskipped_combination():
    recorder = run_script(plot_final_gain_ranges, FINAL_GAIN_COMBINATIONS)
    assert set(recorder.summary) == set(FINAL_GAIN_COMBINATIONS)
    assert ("none", "path_ratio") not in recorder.summary, "skip list must be honored"


def test_computation_time_script_reads_only_path_gain():
    recorder = run_script(plot_computation_time_ranges, COMPUTATION_TIME_COMBINATIONS)
    assert set(recorder.summary) == set(COMPUTATION_TIME_COMBINATIONS)


def test_summary_order_is_reversed_before_plotting():
    """The scripts hand the plotter the combinations in reverse discovery order."""
    recorder = run_script(plot_final_gain_ranges, FINAL_GAIN_COMBINATIONS)
    keys = list(recorder.summary)
    # Discovery order is replan-major, matching the replan_types/objectives loops.
    discovered = [
        combination for combination in FINAL_GAIN_COMBINATIONS if combination in set(keys)
    ]
    assert keys == list(reversed(discovered))


def test_final_gain_uses_automatic_ticks_unless_explicitly_configured():
    automatic = run_script(plot_final_gain_ranges, FINAL_GAIN_COMBINATIONS)
    fixed = run_script(
        plot_final_gain_ranges,
        FINAL_GAIN_COMBINATIONS,
        extra_args=("--gain-tick", "250"),
    )
    assert automatic.kwargs["tick_step"] is None
    assert fixed.kwargs["tick_step"] == 250.0


def test_missing_result_directory_is_a_cli_error(capsys):
    """A dataset missing a replan/objective folder fails loudly rather than plotting less."""
    with tempfile.TemporaryDirectory(prefix="ap_test_fixture_") as directory:
        root = build_dataset(Path(directory), (("everystep", "path_gain"),))
        original_plotter = plot_final_gain_ranges.plot_horizontal_ranges
        try:
            plot_final_gain_ranges.plot_horizontal_ranges = Recorder()
            with pytest.raises(SystemExit) as error, NoWindow():
                plot_final_gain_ranges.main([str(root)])
        finally:
            plot_final_gain_ranges.plot_horizontal_ranges = original_plotter
    assert error.value.code == 2
    assert "verify_benchmark_results.py" in capsys.readouterr().err


@pytest.mark.parametrize("module", [plot_final_gain_ranges, plot_computation_time_ranges])
def test_dataset_with_only_graph_instances_is_a_cli_error(module, capsys):
    """A fresh dataset holding only graph_N/graph.txt names the missing results."""
    with tempfile.TemporaryDirectory(prefix="ap_test_fixture_") as directory:
        (Path(directory) / "graph_0").mkdir()
        (Path(directory) / "graph_0" / "graph.txt").write_text("")
        with pytest.raises(SystemExit) as error:
            module.main([directory])
    assert error.value.code == 2
    assert "no benchmark results" in capsys.readouterr().err


def test_single_row_result_file_is_rejected():
    """A one-row result file, which numpy squeezes to 1-D, is refused."""
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        (root / "nbs_1.txt").write_text("0 0 0 0 740\n")
        grouped = benchmark_ranges.load_and_group_files(root, benchmark_ranges.GROUPING_PATTERN)
        with pytest.raises(ValueError, match="expected 2-D with >=5 columns"):
            benchmark_ranges.process_files({}, grouped, root, "graph_0")


def test_plot_horizontal_ranges_writes_its_output_file():
    """Pins that the plotter saves to the path it is given, under a headless backend."""
    summary = {("everystep", "path_gain"): {"nbs": (1.0, 2.0), "dbs": (1.5, 3.0)}}
    with tempfile.TemporaryDirectory() as directory:
        output_path = Path(directory) / "ranges.pdf"
        benchmark_ranges.plot_horizontal_ranges(
            summary,
            list(benchmark_ranges.ALGORITHM_ORDER),
            {"path_gain": "g"},
            {"everystep": "node"},
            output_path=output_path,
            figsize=(4, 3),
            xlabel="Final gain",
        )
        assert output_path.is_file() and output_path.stat().st_size > 0


def test_plot_horizontal_ranges_supports_a_log_axis():
    """The computation-time figure's log axis must work on Focal's matplotlib 3.1."""
    summary = {("none", "path_gain"): {"nbs": (0.01, 2.0)}}
    with tempfile.TemporaryDirectory() as directory:
        output_path = Path(directory) / "ranges.pdf"
        benchmark_ranges.plot_horizontal_ranges(
            summary,
            list(benchmark_ranges.ALGORITHM_ORDER),
            {"path_gain": "g"},
            {"none": "no"},
            output_path=output_path,
            figsize=(4, 3),
            xlabel="Computation time",
            log_x=True,
        )
        assert output_path.is_file() and output_path.stat().st_size > 0


def test_algorithm_colors_keep_their_published_hues():
    """Palette indices 0/4/8/12 are what the paper figures use.

    tab20c holds four shades per hue, so enumerating the palette directly would hand every
    algorithm a shade of the same blue. Pinned because the change would be invisible in code
    review and obvious only in the figures.
    """
    import seaborn as sns

    palette = sns.color_palette("tab20c")
    expected = {"nbs": palette[0], "dbs": palette[4], "spt": palette[8], "tsp": palette[12]}
    derived = {
        algorithm: palette[index * benchmark_ranges.PALETTE_STRIDE]
        for index, algorithm in enumerate(benchmark_ranges.ALGORITHM_ORDER)
    }
    assert derived == expected


def test_online_dataset_basename_selects_the_other_branch():
    """An ``op_`` directory basename selects online replanning/objective combinations."""
    combinations = (
        ("goal", "path_gain"),
        ("goal", "path_ratio"),
        ("goal", "expected_gain"),
        ("everystep", "path_gain"),
        ("everystep", "path_ratio"),
        ("everystep", "expected_gain"),
    )
    recorder = run_script(plot_final_gain_ranges, combinations, dataset_prefix="op_5m_test_fixture")
    assert set(recorder.summary) == set(combinations)
    assert not any(
        replan == "none" for replan, _ in recorder.summary
    ), "the non-a-priori branch does not read the 'none' replan type"


def test_resolve_dataset_folder_accepts_existing_paths_and_results_root_names():
    with tempfile.TemporaryDirectory(prefix="ap_existing_path_") as directory:
        existing = Path(directory)
        assert benchmark_ranges.resolve_dataset_folder(existing) == existing

    dataset_name = "ap_25x25_clustered_gain_0to100_cost_50"
    assert benchmark_ranges.resolve_dataset_folder(dataset_name) == (
        benchmark_ranges.RESULTS_ROOT / dataset_name
    )


@pytest.mark.parametrize("value", ["0", "-1", "nan", "inf", "-inf"])
def test_final_gain_rejects_non_positive_or_non_finite_ticks(value, capsys):
    with pytest.raises(SystemExit) as error:
        plot_final_gain_ranges.main(
            ["ap_missing_but_tick_validation_runs_first", f"--gain-tick={value}"]
        )
    assert error.value.code == 2
    assert "argument --gain-tick: must be finite and greater than zero" in (capsys.readouterr().err)


def test_missing_dataset_root_is_a_cli_error():
    with pytest.raises(SystemExit) as error:
        plot_final_gain_ranges.main(["ap_definitely_missing_cli_dataset"])
    assert error.value.code == 2


def test_unclassified_dataset_basename_is_a_cli_error():
    with tempfile.TemporaryDirectory(prefix="results_fixture_") as directory:
        with pytest.raises(SystemExit) as error:
            plot_final_gain_ranges.main([directory])
    assert error.value.code == 2


def test_computation_time_rejects_online_dataset():
    with tempfile.TemporaryDirectory(prefix="op_test_fixture_") as directory:
        with pytest.raises(SystemExit) as error:
            plot_computation_time_ranges.main([directory])
    assert error.value.code == 2
