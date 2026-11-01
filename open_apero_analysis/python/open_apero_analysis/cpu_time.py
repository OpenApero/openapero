"""CPU-time accounting of the whole planner node, for ``evaluate_experiment.py``.

All helpers are pure (no ROS or Matplotlib) so they remain unit-testable.
"""

import numpy as np

from open_apero_analysis.timers import TimerSelection, load_timer_metadata, root_timers

__all__ = [
    "accounted_cpu_time_per_cycle",
    "ros_cpu_timer_selection",
]

# Agent-level (non-layer) timers summed alongside every layer's root timers: the
# whole node's CPU cost (planning_time scopes what planning alone costs).
# Excludes the cycle totals/denominators (ReplanTimeRos/Wall, ElapsedWallTimeSec).
ROS_AGENT_CPU_COMPONENTS = ("PlanPathTotal", "Visualization", "RosCallbacks")


def ros_cpu_timer_selection(run):
    """Ordered timer columns whose sum is the accounted planner CPU time.

    Every layer's root timers in the manifest's structural timer hierarchy plus
    the agent-level components. Empty, with the reason in ``diagnostics``, unless
    the run's manifest is valid (:class:`~open_apero_analysis.TimerMetaState`
    VALID).

    Returns:
        :class:`~open_apero_analysis.TimerSelection`.
    """
    meta = load_timer_metadata(run.manifest, csv_header=run.raw_header)
    if not meta.is_valid:
        return TimerSelection((), meta.diagnostics)

    present = set(run.columns)
    agent = [c for c in ROS_AGENT_CPU_COMPONENTS if c in present]
    layer_roots = [
        c.name for c in root_timers(meta) if c.layer_instance is not None and c.name in present
    ]
    return TimerSelection(tuple(agent + layer_roots), ())


def accounted_cpu_time_per_cycle(columns, timer_columns):
    """Per-cycle accounted CPU time [s]: the row-wise sum of ``timer_columns``.

    Args:
        columns: mapping column name -> sequence of per-cycle values.
        timer_columns: the column names to sum (e.g. from
            :func:`ros_cpu_timer_selection`).

    Returns:
        1-D float array with one value per cycle; empty if none of ``timer_columns``
        is present.
    """
    present = [c for c in timer_columns if c in columns]
    if not present:
        return np.zeros(0)
    total = None
    for name in present:
        values = np.asarray(columns[name], dtype=float)
        total = values.copy() if total is None else total + values
    return total
