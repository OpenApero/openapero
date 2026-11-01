"""ROS-free analysis library for Open Apero run artifacts.

Consumed by the experiment tools in open_apero_ros/scripts and by the graph benchmark's range
plots without a dependency cycle. See :func:`run_data.load_run`.

The experiment-run accounting is imported explicitly: :mod:`.cpu_time` (the whole node's CPU
time), :mod:`.planning_time` (the planning layer's timer scope) and :mod:`.performance_overview`
(the overview chart, which needs Matplotlib). So is :mod:`.figures`, the figure tools' shared
output arguments and provenance record; :mod:`.paths` (the data root and the figure tree) is
exported.
"""

from open_apero_analysis import paths, series, timers
from open_apero_analysis.fingerprint import schema_fingerprint
from open_apero_analysis.paths import data_root, figures_root
from open_apero_analysis.run_data import (
    RunData,
    RunStatus,
    aggregate_report,
    iter_run_dirs,
    load_run,
    load_runs,
)
from open_apero_analysis.series import calculate_mean_and_std_interpolated, interpolate_onto_grid
from open_apero_analysis.timers import (
    ColumnMeta,
    TimerMetadata,
    TimerMetaState,
    TimerSelection,
    load_timer_metadata,
    root_timers,
)

__all__ = [
    "RunData",
    "RunStatus",
    "load_run",
    "load_runs",
    "aggregate_report",
    "iter_run_dirs",
    "schema_fingerprint",
    "paths",
    "data_root",
    "figures_root",
    "series",
    "calculate_mean_and_std_interpolated",
    "interpolate_onto_grid",
    "timers",
    "ColumnMeta",
    "TimerMetadata",
    "TimerMetaState",
    "TimerSelection",
    "load_timer_metadata",
    "root_timers",
]
