"""Performance-overview computation and rendering."""

import os
import tempfile

import matplotlib

matplotlib.use("Agg")  # Import-time: the render test must not need a display.

import numpy as np  # noqa: E402

from open_apero_analysis import performance_overview  # noqa: E402


def _columns(cycles=6, value=1.0):
    """Per-cycle columns where every band contributes `value` seconds a cycle."""
    band_columns = {column: [value] * cycles for _, column, _ in performance_overview.STACK_BANDS}
    band_columns.update(
        {
            "ReplanTimeRos": [1.0] * cycles,
            "RosCallbacks": [value] * cycles,
            "planning.ExpandTime": [value] * cycles,
            "planning.UpdateTime": [value] * cycles,
            "planning.PlanPathTime": [value] * cycles,
            "planning.NumNodesAtSelection": list(range(10, 10 + cycles)),
            "planning.NumExpansions": [2] * cycles,
        }
    )
    return band_columns


def _timer_columns():
    """Accounted-CPU columns summing to one second per band plus ROS."""
    return tuple(column for _, column, _ in performance_overview.STACK_BANDS) + (
        performance_overview.ROS_COLUMN,
    )


def test_shares_account_for_all_cpu_time():
    overview = performance_overview.compute_performance_overview(_columns(), _timer_columns())
    total = sum(overview.shares) + overview.residual_share + overview.ros_share
    assert abs(total - 100.0) < 1e-9


def test_equal_contributions_split_evenly():
    # Every band plus ROS contributes one second per cycle, so each gets an equal share.
    overview = performance_overview.compute_performance_overview(_columns(), _timer_columns())
    expected = 100.0 / (len(performance_overview.STACK_BANDS) + 1)
    for share in overview.shares:
        assert abs(share - expected) < 1e-9
    assert abs(overview.ros_share - expected) < 1e-9
    assert abs(overview.residual_share) < 1e-9


def test_rerooting_is_its_own_band_and_planning_layer_time():
    # StartNodeUpdate nests in no other timer: without a band it would fall into the residual,
    # and the planning layer's CPU usage would miss it (RRT* rewires its whole tree there).
    columns = _columns()
    columns["planning.StartNodeUpdate"] = [0.0] * 6
    without = performance_overview.compute_performance_overview(columns, _timer_columns())
    columns["planning.StartNodeUpdate"] = [1.0] * 6
    with_reroot = performance_overview.compute_performance_overview(columns, _timer_columns())

    assert abs(with_reroot.residual_share) < 1e-9
    np.testing.assert_allclose(with_reroot.planning_cpu_usage, without.planning_cpu_usage + 1.0)


def test_band_edges_are_cumulative_and_ordered():
    overview = performance_overview.compute_performance_overview(_columns(), _timer_columns())
    assert len(overview.band_edges) == len(performance_overview.STACK_BANDS)
    previous = np.zeros_like(overview.band_edges[0])
    for edge in overview.band_edges:
        assert np.all(edge >= previous - 1e-12)
        previous = edge
    # The ROS band is measured down from the top, so it must sit above the stack.
    assert np.all(overview.ros_edge >= previous - 1e-12)


def test_step_time_starts_at_zero_and_doubles_samples():
    cycles = 5
    overview = performance_overview.compute_performance_overview(_columns(cycles), _timer_columns())
    assert overview.step_time.size == 2 * cycles
    assert overview.step_time[0] == 0
    # Each cycle holds its value until the next begins, so samples come in pairs.
    assert overview.step_time[1] == overview.step_time[2]


def test_smoothing_buckets_reach_the_threshold():
    # Half-second cycles: four of them fill one two-second bucket.
    replan = [0.5] * 12
    cpu = [np.full(12, 0.25)]
    times, usages = performance_overview.smoothed_cpu_usage(replan, cpu, threshold=2.0)
    assert times.size == usages[0].size
    assert times.size > 0
    # 0.25 CPU-seconds per 0.5 simulated seconds is half a core throughout.
    assert np.allclose(usages[0], 0.5)


def test_smoothing_handles_runs_shorter_than_one_bucket():
    times, usages = performance_overview.smoothed_cpu_usage(
        [0.1, 0.1], [np.array([0.05, 0.05])], threshold=2.0
    )
    assert times.size == 0
    assert usages[0].size == 0


def test_smoothing_of_empty_input_is_empty():
    times, usages = performance_overview.smoothed_cpu_usage([], [np.array([])])
    assert times.size == 0
    assert usages[0].size == 0


def test_render_writes_a_png():
    overview = performance_overview.compute_performance_overview(_columns(20), _timer_columns())
    with tempfile.TemporaryDirectory() as folder:
        path = os.path.join(folder, "PerformanceOverview.png")
        performance_overview.render_performance_overview(overview, path)
        assert os.path.isfile(path)
        assert os.path.getsize(path) > 0


def test_bands_and_legend_cannot_drift_apart():
    # One tuple drives stacking order, colors and legend text; if a band were
    # added without a color or label this would be the first thing to fail.
    for label, column, color in performance_overview.STACK_BANDS:
        assert label and column
        assert color.startswith("#")
