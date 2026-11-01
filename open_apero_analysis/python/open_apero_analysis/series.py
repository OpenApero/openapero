"""ROS-free interpolation and aggregation for run time series."""

import numpy as np


def _sanitize_monotonic(x, y, allow_empty=False):
    """Return a strictly increasing series, keeping the last duplicate x.

    Args:
        x: One-dimensional, monotonic non-decreasing sample positions.
        y: Values corresponding one-to-one with ``x``.
        allow_empty: Return empty arrays instead of rejecting an empty series.
    """
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    if x.ndim != 1 or y.ndim != 1:
        raise ValueError("x and y must be one-dimensional")
    if x.size != y.size:
        raise ValueError("len(x)=%d != len(y)=%d" % (x.size, y.size))
    if x.size == 0:
        if allow_empty:
            return x, y
        raise ValueError("empty x")
    if not np.all(np.isfinite(x)):
        raise ValueError("x contains non-finite values")

    keep = np.ones(x.size, dtype=bool)
    for index in range(1, x.size):
        if x[index] < x[index - 1]:
            raise ValueError(
                "x is not non-decreasing at index %d (%r < %r)" % (index, x[index], x[index - 1])
            )
        if x[index] == x[index - 1]:
            keep[index - 1] = False
    return x[keep], y[keep]


def interpolate_onto_grid(x, y, grid, mode="linear"):
    """Interpolate one series onto ``grid`` and use NaN outside its span.

    ``mode="linear"`` is intended for continuous metrics. ``mode="previous"``
    holds the last value until the next sample and is intended for cumulative or
    step metrics. Extrapolation is never performed.
    """
    x, y = _sanitize_monotonic(x, y, allow_empty=True)
    grid = np.asarray(grid, dtype=float)
    if grid.ndim != 1:
        raise ValueError("grid must be one-dimensional")

    out = np.full(grid.shape, np.nan)
    if x.size == 0:
        return out

    inside = (grid >= x[0]) & (grid <= x[-1])
    if not np.any(inside):
        return out
    grid_inside = grid[inside]

    if mode == "linear":
        out[inside] = np.interp(grid_inside, x, y)
    elif mode == "previous":
        indices = np.searchsorted(x, grid_inside, side="right") - 1
        out[inside] = y[np.clip(indices, 0, x.size - 1)]
    else:
        raise ValueError("unknown interpolation mode: %r" % (mode,))
    return out


def calculate_mean_and_std_interpolated(xs, ys, num_steps, start=None):
    """Interpolate runs on their common interval and return grid, mean, std.

    The grid begins at the largest per-run minimum unless ``start`` is supplied,
    and ends at the smallest per-run maximum. An explicit start must lie in that
    common interval. Standard deviation is the population value (``ddof=0``).
    """
    if not xs or not ys or len(xs) != len(ys):
        raise ValueError("xs and ys must be non-empty and the same length")
    if num_steps <= 0:
        raise ValueError("num_steps must be positive, got %r" % (num_steps,))

    sanitized = []
    for index, (x, y) in enumerate(zip(xs, ys)):
        try:
            sanitized.append(_sanitize_monotonic(x, y))
        except ValueError as exc:
            raise ValueError("dataset %d: %s" % (index, exc)) from exc

    common_lower = max(float(np.min(x)) for x, _ in sanitized)
    common_upper = min(float(np.max(x)) for x, _ in sanitized)
    if common_lower > common_upper:
        raise ValueError(
            "datasets share no common x interval "
            "(common lower %r > common upper %r)" % (common_lower, common_upper)
        )

    if start is None:
        start = common_lower
    elif not common_lower <= start <= common_upper:
        raise ValueError(
            "start=%r is outside the common interval [%r, %r]" % (start, common_lower, common_upper)
        )

    grid = np.linspace(start, common_upper, num_steps)
    stack = np.vstack([interpolate_onto_grid(x, y, grid, mode="linear") for x, y in sanitized])
    return grid, np.mean(stack, axis=0), np.std(stack, axis=0)
