"""Node CPU-time timer selection and accounting."""

import numpy as np

from open_apero_analysis import cpu_time
from open_apero_analysis.fingerprint import schema_fingerprint


class _Run(object):
    """Minimal stand-in for RunData for the timer-selection tests."""

    def __init__(self, columns, manifest=None, raw_header=None):
        self.columns = columns
        self.manifest = manifest
        self.raw_header = raw_header


_MANIFEST_COLUMNS = [
    ("PlanPathTotal", "double", "s", ""),
    ("Visualization", "double", "s", ""),
    ("RosCallbacks", "double", "s", ""),
    ("ReplanTimeRos", "double", "s", ""),
    ("planning.ExpandTime", "double", "s", ""),
    ("planning.UpdateTime", "double", "s", ""),
    ("planning.StartNodeUpdate", "double", "s", ""),
    ("planning.GenerateNode", "double", "s", "planning.ExpandTime"),
    ("backtracking.ExpandTime", "double", "s", ""),
]


def _valid_manifest():
    return {
        "format_version": 1,
        "layers": {"planning": "planning", "backtracking": "backtracking"},
        "schema_fingerprint": schema_fingerprint(_MANIFEST_COLUMNS),
        "columns": [
            {"name": n, "type": t, "unit": u, "parent": p} for (n, t, u, p) in _MANIFEST_COLUMNS
        ],
    }


def _columns_for(names):
    return {n: [1.0, 1.0] for n in names}


def test_timer_selection_manifest_includes_startnodeupdate():
    names = [c[0] for c in _MANIFEST_COLUMNS]
    run = _Run(_columns_for(names), manifest=_valid_manifest(), raw_header=names)
    sel = cpu_time.ros_cpu_timer_selection(run)
    assert not sel.diagnostics
    assert "planning.StartNodeUpdate" in sel.columns
    # Agent components + all layer roots; excludes nested + denominators.
    assert set(sel.columns) == {
        "PlanPathTotal",
        "Visualization",
        "RosCallbacks",
        "planning.ExpandTime",
        "planning.UpdateTime",
        "planning.StartNodeUpdate",
        "backtracking.ExpandTime",
    }
    assert "planning.GenerateNode" not in sel.columns  # nested
    assert "ReplanTimeRos" not in sel.columns  # denominator


def test_timer_selection_is_empty_without_a_valid_manifest():
    names = [c[0] for c in _MANIFEST_COLUMNS]
    for manifest in (None, dict(_valid_manifest(), schema_fingerprint="0000000000000000")):
        run = _Run(_columns_for(names), manifest=manifest, raw_header=names)
        sel = cpu_time.ros_cpu_timer_selection(run)
        assert sel.columns == ()
        assert sel.diagnostics  # says why


def test_startnodeupdate_root_timer_adds_to_the_accounted_time():
    # StartNodeUpdate is a root timer: nested in no other timer, so the accounted time grows by
    # exactly its value, per cycle.
    cols = {
        "PlanPathTotal": [1.0, 2.0],
        "Visualization": [0.5, 0.5],
        "RosCallbacks": [0.1, 0.1],
        "planning.ExpandTime": [1.0, 1.0],
        "planning.UpdateTime": [0.2, 0.2],
        "planning.StartNodeUpdate": [0.3, 0.7],
    }
    other_roots = [
        "PlanPathTotal",
        "Visualization",
        "RosCallbacks",
        "planning.ExpandTime",
        "planning.UpdateTime",
    ]
    without = cpu_time.accounted_cpu_time_per_cycle(cols, other_roots)
    with_start = cpu_time.accounted_cpu_time_per_cycle(
        cols, other_roots + ["planning.StartNodeUpdate"]
    )
    np.testing.assert_allclose(with_start, without + np.array(cols["planning.StartNodeUpdate"]))


def test_accounted_cpu_time_ignores_absent_columns():
    cols = {"PlanPathTotal": [1.0, 2.0]}
    total = cpu_time.accounted_cpu_time_per_cycle(cols, ["PlanPathTotal", "Missing"])
    np.testing.assert_allclose(total, [1.0, 2.0])
