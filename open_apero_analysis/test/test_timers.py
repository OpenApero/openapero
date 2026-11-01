"""Manifest trust model + structural (root) timer selection."""

from open_apero_analysis import (
    TimerMetaState,
    load_timer_metadata,
    root_timers,
)
from open_apero_analysis.fingerprint import schema_fingerprint

# A hand-authored current-schema column set. Root timers (unit 's', empty
# parent) are enumerated by hand in the assertions below.
_COLUMNS = [
    ("PlanPathTotal", "double", "s", ""),
    ("Visualization", "double", "s", ""),
    ("RosCallbacks", "double", "s", ""),
    ("ReplanTimeRos", "double", "s", ""),
    ("planning.ExpandTime", "double", "s", ""),
    ("planning.UpdateTime", "double", "s", ""),
    ("planning.StartNodeUpdate", "double", "s", ""),  # a root timer
    ("planning.GenerateNode", "double", "s", "planning.ExpandTime"),  # nested
    ("planning.PlanPathTime", "double", "s", "PlanPathTotal"),  # nested
    ("backtracking.ExpandTime", "double", "s", ""),
    ("TotalCost", "double", "cost", ""),  # not a timer
    ("NumExpansions", "int", "count", ""),  # not a timer
]

_LAYERS = {"planning": "planning", "backtracking": "backtracking"}


def _manifest(columns=_COLUMNS, layers=_LAYERS, version=1, fingerprint=True, include_columns=True):
    """Build a parsed-manifest dict with a correct (or omitted) fingerprint."""
    m = {"format_version": version, "layers": dict(layers)}
    if include_columns:
        m["columns"] = [{"name": n, "type": t, "unit": u, "parent": p} for (n, t, u, p) in columns]
        if fingerprint:
            m["schema_fingerprint"] = schema_fingerprint(columns)
    return m


def test_absent_when_no_manifest():
    assert load_timer_metadata(None).state is TimerMetaState.ABSENT


def test_invalid_when_manifest_has_no_columns():
    # The writer always records the column schema.
    meta = load_timer_metadata({"format_version": 1, "cost_budget": 600})
    assert meta.state is TimerMetaState.INVALID
    assert meta.diagnostics == ("manifest has no column schema",)


def test_invalid_unsupported_version():
    assert load_timer_metadata(_manifest(version=99)).state is TimerMetaState.INVALID


def test_invalid_non_integer_versions_do_not_raise():
    for version in ([1], {"major": 1}, True, 1.0, "1"):
        meta = load_timer_metadata(_manifest(version=version))
        assert meta.state is TimerMetaState.INVALID
        assert "format_version is not an integer" in meta.diagnostics[0]


def test_invalid_bad_fingerprint():
    m = _manifest()
    m["schema_fingerprint"] = "0000000000000000"
    assert load_timer_metadata(m).state is TimerMetaState.INVALID


def test_invalid_duplicate_column_name():
    cols = [
        ("planning.ExpandTime", "double", "s", ""),
        ("planning.ExpandTime", "double", "s", ""),
    ]
    assert load_timer_metadata(_manifest(columns=cols)).state is TimerMetaState.INVALID


def test_invalid_csv_header_mismatch():
    m = _manifest()
    wrong = ["PlanPathTotal", "somethingelse"]
    assert load_timer_metadata(m, csv_header=wrong).state is TimerMetaState.INVALID


def test_invalid_missing_fingerprint():
    # A v1 writer always emits a fingerprint; its absence is not trustworthy.
    m = _manifest(fingerprint=False)
    assert load_timer_metadata(m).state is TimerMetaState.INVALID


def test_invalid_non_dict_manifest_vs_absent_none():
    assert load_timer_metadata(None).state is TimerMetaState.ABSENT
    assert load_timer_metadata(["not", "a", "mapping"]).state is TimerMetaState.INVALID
    meta = load_timer_metadata("garbage")
    assert meta.state is TimerMetaState.INVALID
    assert isinstance(meta.diagnostics, tuple) and len(meta.diagnostics) == 1


def test_invalid_malformed_layers_does_not_raise():
    m = _manifest()
    m["layers"] = ["planning", "backtracking"]  # a list, not a mapping
    # Must classify INVALID, not raise AttributeError.
    assert load_timer_metadata(m).state is TimerMetaState.INVALID


def test_invalid_bad_column_field_type():
    m = _manifest()
    m["columns"][0]["unit"] = 5  # unit must be a string
    m["schema_fingerprint"] = schema_fingerprint(_COLUMNS)  # fp irrelevant; caught first
    assert load_timer_metadata(m).state is TimerMetaState.INVALID


def test_custom_layer_instance_and_agent_level_columns():
    # A custom layer instance ('explore' typed 'planning') owns its prefixed
    # columns; an unprefixed column is agent-level.
    cols = [
        ("NewAgentTimer", "double", "s", ""),
        ("explore.ExpandTime", "double", "s", ""),
    ]
    m = _manifest(columns=cols, layers={"explore": "planning"})
    meta = load_timer_metadata(m)
    assert meta.state is TimerMetaState.VALID
    assert [c.name for c in meta.columns] == ["NewAgentTimer", "explore.ExpandTime"]
    by_name = {c.name: c for c in meta.columns}
    assert by_name["explore.ExpandTime"].layer_type == "planning"
    assert by_name["NewAgentTimer"].is_agent_level


def test_invalid_layer_key_or_value_not_string():
    m = _manifest()
    m["layers"] = {1: "planning"}  # non-string key
    assert load_timer_metadata(m).state is TimerMetaState.INVALID
    m2 = _manifest()
    m2["layers"] = {"planning": ["bad"]}  # non-string value
    assert load_timer_metadata(m2).state is TimerMetaState.INVALID


def test_malformed_manifest_marker_is_invalid():
    assert load_timer_metadata({"__malformed__": True}).state is TimerMetaState.INVALID


def test_version_ordering_edge():
    # Unsupported version with NO columns -> INVALID for the version, not the columns.
    meta = load_timer_metadata({"format_version": 99})
    assert meta.state is TimerMetaState.INVALID
    assert "format_version" in meta.diagnostics[0]
    # Columns present but no version -> INVALID.
    m = _manifest()
    del m["format_version"]
    assert load_timer_metadata(m).state is TimerMetaState.INVALID


def test_valid_and_csv_header_match():
    raw_names = [c[0] for c in _COLUMNS]
    meta = load_timer_metadata(_manifest(), csv_header=raw_names)
    assert meta.state is TimerMetaState.VALID


def test_root_timers_include_startnodeupdate_exclude_nested_and_nontimers():
    meta = load_timer_metadata(_manifest())
    roots = {c.name for c in root_timers(meta)}
    assert roots == {
        "PlanPathTotal",
        "Visualization",
        "RosCallbacks",
        "ReplanTimeRos",
        "planning.ExpandTime",
        "planning.UpdateTime",
        "planning.StartNodeUpdate",
        "backtracking.ExpandTime",
    }
    # Nested sub-phase timers and non-timers are excluded structurally.
    assert "planning.GenerateNode" not in roots  # parent = ExpandTime
    assert "planning.PlanPathTime" not in roots  # parent = PlanPathTotal
    assert "TotalCost" not in roots and "NumExpansions" not in roots


def test_root_timers_are_ordered_by_manifest():
    meta = load_timer_metadata(_manifest())
    names = [c.name for c in root_timers(meta)]
    assert names == [
        "PlanPathTotal",
        "Visualization",
        "RosCallbacks",
        "ReplanTimeRos",
        "planning.ExpandTime",
        "planning.UpdateTime",
        "planning.StartNodeUpdate",
        "backtracking.ExpandTime",
    ]


def test_layer_identity_and_agent_level_split():
    meta = load_timer_metadata(_manifest())
    by_name = {c.name: c for c in meta.columns}
    expand = by_name["planning.ExpandTime"]
    assert expand.layer_instance == "planning" and expand.layer_type == "planning"
    assert not expand.is_agent_level
    bt = by_name["backtracking.ExpandTime"]
    assert bt.layer_type == "backtracking"
    ppt = by_name["PlanPathTotal"]
    assert ppt.is_agent_level and ppt.layer_instance is None


def test_layer_scoping_for_planning_time_and_ros():
    meta = load_timer_metadata(_manifest())
    roots = root_timers(meta)
    # Planning-time scope: planning-layer roots + PlanPathTotal.
    planning_time = [
        c.name for c in roots if c.layer_type == "planning" or c.name == "PlanPathTotal"
    ]
    assert planning_time == [
        "PlanPathTotal",
        "planning.ExpandTime",
        "planning.UpdateTime",
        "planning.StartNodeUpdate",
    ]
    # ROS scope: every layer root + explicit agent-level components.
    agent = {"PlanPathTotal", "Visualization", "RosCallbacks"}
    ros = [c.name for c in roots if c.layer_instance is not None or c.name in agent]
    assert "planning.StartNodeUpdate" in ros and "backtracking.ExpandTime" in ros
    assert "ReplanTimeRos" not in ros  # a denominator, not a summed component


def test_parent_is_kept_and_a_root_has_an_empty_parent():
    cols = [
        ("planning.ExpandTime", "double", "s", ""),  # root
        ("planning.GenerateNode", "double", "s", "planning.ExpandTime"),
    ]
    meta = load_timer_metadata(_manifest(columns=cols, layers={"planning": "planning"}))
    assert meta.state is TimerMetaState.VALID
    by_name = {c.name: c for c in meta.columns}
    assert by_name["planning.GenerateNode"].parent == "planning.ExpandTime"
    assert by_name["planning.ExpandTime"].parent == ""
    assert by_name["planning.ExpandTime"].is_root
    assert not by_name["planning.GenerateNode"].is_root
