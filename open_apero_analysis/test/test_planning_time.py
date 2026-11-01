"""Planning-time timer scope, read from the run's manifest."""

from open_apero_analysis.fingerprint import schema_fingerprint
from open_apero_analysis.planning_time import plan_path_selection, planning_time_selection


class _Run(object):
    def __init__(self, columns, manifest=None, raw_header=None):
        self.columns = columns
        self.manifest = manifest
        self.raw_header = raw_header


_COLUMNS = [
    ("PlanPathTotal", "double", "s", ""),
    ("planning.ExpandTime", "double", "s", ""),
    ("planning.UpdateTime", "double", "s", ""),
    ("planning.StartNodeUpdate", "double", "s", ""),
    ("planning.GenerateNode", "double", "s", "planning.ExpandTime"),  # nested
    ("backtracking.ExpandTime", "double", "s", ""),  # other layer
]


def _manifest(columns=_COLUMNS, layers=None):
    if layers is None:
        layers = {"planning": "planning", "backtracking": "backtracking"}
    return {
        "format_version": 1,
        "layers": layers,
        "schema_fingerprint": schema_fingerprint(columns),
        "columns": [{"name": n, "type": t, "unit": u, "parent": p} for (n, t, u, p) in columns],
    }


def _cols(names):
    return {n: [1.0, 1.0] for n in names}


def test_manifest_scope_planning_roots_plus_planpathtotal_incl_startnode():
    names = [c[0] for c in _COLUMNS]
    run = _Run(_cols(names), manifest=_manifest(), raw_header=names)
    cols = planning_time_selection(run).columns
    assert set(cols) == {
        "PlanPathTotal",
        "planning.ExpandTime",
        "planning.UpdateTime",
        "planning.StartNodeUpdate",
    }
    assert "planning.GenerateNode" not in cols  # nested
    assert "backtracking.ExpandTime" not in cols  # other layer


def test_custom_planning_instance_name_honored():
    cols_def = [
        ("PlanPathTotal", "double", "s", ""),
        ("explore.ExpandTime", "double", "s", ""),
        ("explore.UpdateTime", "double", "s", ""),
    ]
    names = [c[0] for c in cols_def]
    run = _Run(
        _cols(names), manifest=_manifest(cols_def, layers={"explore": "planning"}), raw_header=names
    )
    cols = planning_time_selection(run).columns
    assert set(cols) == {"PlanPathTotal", "explore.ExpandTime", "explore.UpdateTime"}


def test_scope_is_empty_without_a_manifest():
    # No manifest -> caller must skip (not plot zeros).
    names = [c[0] for c in _COLUMNS]
    sel = planning_time_selection(_Run(_cols(names), manifest=None, raw_header=names))
    assert not sel.columns
    assert sel.diagnostics


def test_selection_missing_planpathtotal_but_has_timers():
    cols_def = [
        ("planning.ExpandTime", "double", "s", ""),
        ("planning.UpdateTime", "double", "s", ""),
    ]
    names = [c[0] for c in cols_def]
    run = _Run(_cols(names), manifest=_manifest(cols_def), raw_header=names)
    sel = planning_time_selection(run)
    assert set(sel.columns) == {"planning.ExpandTime", "planning.UpdateTime"}
    assert "PlanPathTotal" not in sel.columns  # caller treats this as unusable


_PLAN_PATH_COLUMNS = _COLUMNS + [
    ("planning.PlanPathTime", "double", "s", "PlanPathTotal"),
    ("backtracking.PlanPathTime", "double", "s", "PlanPathTotal"),
]


def test_plan_path_scope_is_the_planning_layer_only():
    names = [c[0] for c in _PLAN_PATH_COLUMNS]
    run = _Run(_cols(names), manifest=_manifest(_PLAN_PATH_COLUMNS), raw_header=names)
    sel = plan_path_selection(run)
    assert set(sel.columns) == {"planning.PlanPathTime"}
    # Neither the other layer's planPath nor the agent-wide sum of both.
    assert "backtracking.PlanPathTime" not in sel.columns
    assert "PlanPathTotal" not in sel.columns
    # Nor the graph work that planning_time_selection also charges to planning.
    assert "planning.ExpandTime" not in sel.columns


def test_plan_path_scope_honors_a_custom_planning_instance_name():
    cols_def = [
        ("PlanPathTotal", "double", "s", ""),
        ("explore.PlanPathTime", "double", "s", "PlanPathTotal"),
    ]
    names = [c[0] for c in cols_def]
    run = _Run(
        _cols(names), manifest=_manifest(cols_def, layers={"explore": "planning"}), raw_header=names
    )
    assert set(plan_path_selection(run).columns) == {"explore.PlanPathTime"}


def test_plan_path_scope_is_empty_without_a_manifest():
    names = ["planning.PlanPathTime", "backtracking.PlanPathTime"]
    run = _Run(_cols(names), manifest=None, raw_header=names)
    sel = plan_path_selection(run)
    assert not sel.columns and sel.diagnostics
