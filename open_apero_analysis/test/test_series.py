"""ROS-free interpolation and series aggregation."""

import numpy as np
import pytest

from open_apero_analysis import calculate_mean_and_std_interpolated, interpolate_onto_grid
from open_apero_analysis.series import _sanitize_monotonic


def _assert_value_error(fragment, callback):
    with pytest.raises(ValueError) as error:
        callback()
    assert fragment in str(error.value)


def test_default_start_is_common_lower_bound():
    grid, _mean, _std = calculate_mean_and_std_interpolated(
        [[0, 1, 2, 3, 4], [2, 3, 4, 5]],
        [[0, 1, 2, 3, 4], [2, 3, 4, 5]],
        num_steps=3,
    )
    np.testing.assert_allclose(grid, [2, 3, 4])


def test_explicit_start_within_interval():
    grid, _mean, _std = calculate_mean_and_std_interpolated(
        [[0, 50, 100], [0, 50, 100]],
        [[0, 5, 10], [0, 5, 10]],
        num_steps=2,
        start=50,
    )
    assert grid[0] == 50.0


def test_invalid_common_grid_inputs_raise():
    _assert_value_error(
        "outside the common interval",
        lambda: calculate_mean_and_std_interpolated(
            [[0, 1, 2], [0, 1, 2]], [[0, 1, 2], [0, 1, 2]], 3, start=50
        ),
    )
    _assert_value_error(
        "no common x interval",
        lambda: calculate_mean_and_std_interpolated([[0, 1], [5, 6]], [[0, 1], [5, 6]], 3),
    )
    _assert_value_error(
        "len(x)",
        lambda: calculate_mean_and_std_interpolated([[0, 1, 2]], [[0, 1]], 3),
    )
    _assert_value_error(
        "num_steps",
        lambda: calculate_mean_and_std_interpolated([[0, 1], [0, 1]], [[0, 1], [0, 1]], 0),
    )
    _assert_value_error(
        "non-finite",
        lambda: calculate_mean_and_std_interpolated([[0, 1, np.inf]], [[0, 1, 2]], 3),
    )


def test_single_run_std_is_zero():
    _grid, _mean, std = calculate_mean_and_std_interpolated([[0, 1, 2]], [[0, 1, 2]], num_steps=3)
    np.testing.assert_allclose(std, 0)


def test_duplicate_x_collapsed_keeping_last():
    x, y = _sanitize_monotonic([0, 1, 1, 2], [5, 6, 7, 8])
    np.testing.assert_allclose(x, [0, 1, 2])
    np.testing.assert_allclose(y, [5, 7, 8])


def test_decreasing_x_rejected():
    _assert_value_error("non-decreasing", lambda: _sanitize_monotonic([0, 2, 1], [0, 0, 0]))


def test_interpolate_linear_uses_nan_outside():
    result = interpolate_onto_grid([0, 1, 2], [0, 10, 20], [0, 1, 2, 3], mode="linear")
    np.testing.assert_allclose(result[:3], [0, 10, 20])
    assert np.isnan(result[3])


def test_interpolate_previous_holds_step_value():
    result = interpolate_onto_grid([0, 2], [5, 9], [0, 1, 2, 3], mode="previous")
    np.testing.assert_allclose(result[:3], [5, 5, 9])
    assert np.isnan(result[3])
