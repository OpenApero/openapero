"""Planning-time timer scope of an experiment run (manifest-driven, layer-aware).

Read from the manifest's structural timer hierarchy, so ``StartNodeUpdate`` is
included and custom PlanningLayer instance names are honored. A run without a
valid manifest has an empty scope.

The scope differs from :func:`open_apero_analysis.cpu_time.ros_cpu_timer_selection`:

===================  ===========================  ============================
                     planning time (this module)  node CPU time (cpu_time)
===================  ===========================  ============================
question             what does planning cost?     what does the node cost?
agent components     ``PlanPathTotal``            + ``Visualization``,
                                                  ``RosCallbacks``
layer roots          the planning layer only      every layer
===================  ===========================  ============================
"""

from open_apero_analysis.timers import TimerSelection, load_timer_metadata, root_timers

# Agent-level component summed alongside the planning-layer roots. Required: a
# planning-time series without it is meaningless, so its absence skips the run.
PLANNING_REQUIRED_AGENT_COMPONENT = "PlanPathTotal"

# A layer's own planPath timer, nested under the agent's PlanPathTotal.
PLAN_PATH_SUFFIX = ".PlanPathTime"


def planning_time_selection(run):
    """Timer scope whose per-row sum is a run's planning CPU time.

    Returns a :class:`~open_apero_analysis.TimerSelection` (columns,
    diagnostics). ``columns`` contains only columns present in the run; it is
    empty, with the reason in ``diagnostics``, without a valid manifest. The
    caller must treat an empty selection, or one missing ``PlanPathTotal``, as a
    reason to skip the run rather than silently plotting a zero series.
    """
    meta = load_timer_metadata(run.manifest, csv_header=run.raw_header)
    if not meta.is_valid:
        return TimerSelection((), meta.diagnostics)

    present = set(run.columns)
    agent = (
        [PLANNING_REQUIRED_AGENT_COMPONENT] if PLANNING_REQUIRED_AGENT_COMPONENT in present else []
    )
    roots = [
        column.name
        for column in root_timers(meta)
        if column.layer_type == "planning" and column.name in present
    ]
    return TimerSelection(tuple(agent + roots), ())


def plan_path_selection(run):
    """The planning layer's own ``planPath`` timer -- a narrower scope.

    Distinct from :func:`planning_time_selection` on both counts: it excludes
    the graph work (expand/update) and it excludes every other layer, where the
    agent-level ``PlanPathTotal`` sums planPath over all layers attempted (e.g.
    backtracking). Used for the per-run summary table, not for the plotted
    series. Returns a :class:`~open_apero_analysis.TimerSelection`, empty without
    a valid manifest.
    """
    meta = load_timer_metadata(run.manifest, csv_header=run.raw_header)
    if not meta.is_valid:
        return TimerSelection((), meta.diagnostics)

    columns = [
        column.name
        for column in meta.columns
        if column.layer_type == "planning"
        and column.name.endswith(PLAN_PATH_SUFFIX)
        and column.name in run.columns
    ]
    return TimerSelection(tuple(columns), ())
