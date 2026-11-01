"""Load a single Open Apero run into one self-describing object.

`load_run` bundles everything an analysis tool needs from a run directory:

* the per-cycle ``performance_log.csv`` columns;
* the ``rosparams.yaml`` parameter snapshot (cost computer, budget, scene, ...);
* the ``run_manifest.yaml`` self-description (column schema, units, schema
  fingerprint, git provenance);
* the ``episode_summary.yaml`` end reason and final totals, read on access.

Each run gets a terminal status — ``parsed``, ``degraded`` (loaded, but missing
columns relative to the reference schema or without a valid manifest), or
``skipped`` (no usable data) — plus a human reason. `load_runs` +
`aggregate_report` turn a tree of runs into an explicit accounting so that
dropped runs are counted, never silently lost.
"""

import csv
import enum
import os
from collections import Counter
from pathlib import Path

from open_apero_analysis.timers import load_timer_metadata

try:  # PyYAML is a declared exec dependency; degrade gracefully if absent.
    import yaml
except ImportError:  # pragma: no cover
    yaml = None

PERFORMANCE_LOG = "performance_log.csv"
ROSPARAMS = "rosparams.yaml"
MANIFEST = "run_manifest.yaml"
EPISODE_SUMMARY = "episode_summary.yaml"

# Task folders paper_sweep.py and run_experiments.sh put above the experiment folder:
# <data_root>/<task>/<experiment>/<scene>/<algo>/<timestamp>.
TASK_FOLDERS = ("explore", "recon", "points")

# The reference schema: a run missing any of its columns is 'degraded'.
REFERENCE_AGENT_COLUMNS = frozenset(
    {
        "TotalVisitedNodes",
        "TotalPathLength",
        "TotalPathDuration",
        "PlanPathTotal",
        "Visualization",
        "ReplanTimeWall",
        "ReplanTimeRos",
        "RosCallbacks",
    }
)

# Reference leaves of one layer instance, by layer type (the values of the
# manifest's ``layers`` map). A layer of any other type is held to the leaves
# every layer writes.
REFERENCE_LAYER_LEAVES = ("ExpandTime", "UpdateTime", "PlanPathTime", "TimesSelected")
REFERENCE_LEAVES_BY_LAYER_TYPE = {
    "planning": REFERENCE_LAYER_LEAVES
    + (
        "GenerateNode",
        "GenerateEdge",
        "EvaluateNode",
        "GeneratorUpdate",
        "EvaluatorUpdate",
        "PlannedValue",
        "RealAccumulatedGain",
        "NumExpansions",
        "NumNodesAfterExpansion",
        "NumNodesAfterNewStart",
        "NumNodesAtSelection",
    ),
}

# The shipped layer instances (instance -> layer type): the reference for a run
# without a trustworthy manifest ``layers`` map.
SHIPPED_LAYERS = {"backtracking": "backtracking", "planning": "planning"}


def reference_schema_columns(layers):
    """The reference schema of a run whose layers are ``layers`` (instance -> layer type)."""
    columns = set(REFERENCE_AGENT_COLUMNS)
    for instance, layer_type in layers.items():
        leaves = REFERENCE_LEAVES_BY_LAYER_TYPE.get(layer_type, REFERENCE_LAYER_LEAVES)
        columns.update("%s.%s" % (instance, leaf) for leaf in leaves)
    return frozenset(columns)


# The reference schema of a run with the shipped layer instances.
REFERENCE_SCHEMA_COLUMNS = reference_schema_columns(SHIPPED_LAYERS)


class RunStatus(enum.Enum):
    """Terminal load outcome for one run. 'discovered' is an aggregate, not here."""

    PARSED = "parsed"
    DEGRADED = "degraded"
    SKIPPED = "skipped"


def _find_key(tree, key):
    """Depth-first search for the first value of ``key`` anywhere in a dict tree.

    ``rosparams.yaml`` is a full ``rosparam dump /``, so parameters are nested
    under node namespaces; callers should not need to know the exact path.
    """
    if isinstance(tree, dict):
        if key in tree:
            return tree[key]
        for value in tree.values():
            found = _find_key(value, key)
            if found is not None:
                return found
    return None


class RunData:
    """One run: columns + provenance + derived metadata."""

    def __init__(
        self,
        path,
        status,
        reason="",
        columns=None,
        rosparams=None,
        manifest=None,
        num_rows=0,
        notes=None,
        raw_header=None,
        timer_metadata=None,
    ):
        self.path = Path(path)
        self.status = status
        self.reason = reason
        self.columns = columns or {}
        self.rosparams = rosparams
        self.manifest = manifest
        self.num_rows = num_rows
        self.notes = notes or []
        # The CSV header row in file order, so a consumer can cross-check the
        # manifest's column order against the actual log. None if unknown.
        self.raw_header = raw_header
        # Validated manifest column schema, already checked against raw_header.
        # None only when the run never got as far as reading a header.
        self.timer_metadata = timer_metadata
        # Run directories of later roots with this run's rel_id, which load_runs skipped for it.
        self.shadowed = []

    # --- path-derived identity ([<task>/]<experiment>/<scene>/<algo>/<timestamp>) ---
    @property
    def timestamp(self):
        return self.path.name

    @property
    def algorithm(self):
        return self.path.parent.name if self.path.parent else None

    @property
    def scene(self):
        parent = self.path.parent
        return parent.parent.name if parent and parent.parent else None

    @property
    def task(self):
        """The task folder above the experiment folder (a TASK_FOLDERS name), else None."""
        parts = self.path.parts
        if len(parts) >= 5 and parts[-5] in TASK_FOLDERS:
            return parts[-5]
        return None

    @property
    def rel_id(self):
        """Identity used for cross-root dedup: [task/]experiment/scene/algo/timestamp."""
        parts = self.path.parts[-5:] if self.task else self.path.parts[-4:]
        return "/".join(parts)

    # --- config metadata (None when unknown) ---
    @property
    def cost_computer_type(self):
        cc = _find_key(self.rosparams or {}, "cost_computer")
        if isinstance(cc, dict):
            return cc.get("type")
        return None

    @property
    def cost_budget(self):
        if self.manifest and self.manifest.get("cost_budget") is not None:
            return self.manifest["cost_budget"]
        return _find_key(self.rosparams or {}, "cost_budget")

    @property
    def episode_summary(self):
        """The parsed ``episode_summary.yaml`` (``.inf`` loads as inf), or None when absent."""
        summary = _load_yaml(self.path / EPISODE_SUMMARY)
        return summary if isinstance(summary, dict) else None

    # --- capability API: let each plot ask for exactly what it needs ---
    def has_columns(self, names):
        return all(n in self.columns for n in names)

    def missing_columns(self, names):
        return [n for n in names if n not in self.columns]

    @property
    def loaded(self):
        return self.status in (RunStatus.PARSED, RunStatus.DEGRADED)

    def __repr__(self):
        return "RunData(%s, %s, rows=%d)" % (self.rel_id, self.status.value, self.num_rows)


def _load_yaml(path):
    if yaml is None or not path.exists():
        return None
    try:
        with open(path, "r") as handle:
            return yaml.safe_load(handle)
    except (yaml.YAMLError, OSError):
        return None


def _load_manifest(path):
    """Load run_manifest.yaml, distinguishing absent from malformed.

    Returns ``None`` when the file is absent, a ``{"__malformed__": True}``
    marker when it exists but cannot be parsed into a mapping (so downstream
    trust checks classify it INVALID rather than ABSENT), or the parsed mapping.
    """
    if not path.exists():
        return None
    if yaml is None:  # dependency missing: cannot judge; treat as absent
        return None
    try:
        with open(path, "r") as handle:
            data = yaml.safe_load(handle)
    except (yaml.YAMLError, OSError) as exc:
        return {"__malformed__": True, "__reason__": str(exc)}
    if not isinstance(data, dict):
        return {"__malformed__": True, "__reason__": "manifest is empty or not a mapping"}
    return data


def load_run(run_dir):
    """Load one run directory into a :class:`RunData`.

    Never raises for a malformed/absent CSV: returns a ``skipped`` RunData with
    a reason instead, so a batch load accounts for every directory.
    """
    run_dir = Path(run_dir)
    rosparams = _load_yaml(run_dir / ROSPARAMS)
    manifest = _load_manifest(run_dir / MANIFEST)
    log_file = run_dir / PERFORMANCE_LOG

    if not log_file.exists():
        return RunData(
            run_dir,
            RunStatus.SKIPPED,
            reason="no performance_log.csv",
            rosparams=rosparams,
            manifest=manifest,
        )

    try:
        with open(log_file, "r", newline="") as handle:
            reader = csv.reader(handle)
            rows = list(reader)
    except (OSError, UnicodeDecodeError, csv.Error) as exc:
        # csv.Error: a NUL byte, as in a log cut off by a crash, before Python 3.11.
        return RunData(
            run_dir,
            RunStatus.SKIPPED,
            reason="unreadable CSV: %s" % exc,
            rosparams=rosparams,
            manifest=manifest,
        )

    if not rows:
        return RunData(
            run_dir, RunStatus.SKIPPED, reason="empty CSV", rosparams=rosparams, manifest=manifest
        )

    raw_header = rows[0]
    data_rows = rows[1:]

    notes = []
    # A repeated column name keeps its first column.
    seen = {}
    for index, name in enumerate(raw_header):
        if name in seen:
            notes.append("duplicate column '%s' (cols %d and %d)" % (name, seen[name], index))
        else:
            seen[name] = index

    if not data_rows:
        # A lone header with no rows.
        return RunData(
            run_dir,
            RunStatus.SKIPPED,
            reason="no data rows (header only)",
            rosparams=rosparams,
            manifest=manifest,
            notes=notes,
            raw_header=raw_header,
        )

    columns = {name: [] for name in seen}
    for row_number, row in enumerate(data_rows):
        for name, index in seen.items():
            if index < len(row):
                cell = row[index].strip()
                if cell in ("", "nan"):
                    columns[name].append(float("nan"))
                    continue
                try:
                    columns[name].append(float(cell))
                except ValueError:
                    # A malformed numeric cell skips the whole run, with its
                    # (row, column, value) as the reason. Empty/'nan' are handled
                    # above and are not malformed.
                    reason = "malformed numeric cell at data row %d, column '%s': %r" % (
                        row_number,
                        name,
                        cell,
                    )
                    return RunData(
                        run_dir,
                        RunStatus.SKIPPED,
                        reason=reason,
                        rosparams=rosparams,
                        manifest=manifest,
                        notes=notes,
                        raw_header=raw_header,
                    )
            else:
                columns[name].append(float("nan"))

    timer_metadata = load_timer_metadata(manifest, csv_header=raw_header)

    # The per-layer reference columns follow the layer instances a trustworthy
    # manifest lists; without one, the shipped instance names.
    reasons = []
    reference = reference_schema_columns(timer_metadata.layers or SHIPPED_LAYERS)
    missing = sorted(reference - set(columns))
    if missing:
        reasons.append("missing %d column(s): %s" % (len(missing), ", ".join(missing)))

    # Without a valid manifest the columns have no trustworthy units, provenance
    # or timer structure, so the run is DEGRADED rather than PARSED: the numbers
    # loaded fine, their description did not.
    if not timer_metadata.is_valid:
        detail = "; ".join(timer_metadata.diagnostics) or "unspecified"
        reasons.append("no valid manifest: %s" % detail)
        notes.append("manifest rejected for structural use: %s" % detail)

    status = RunStatus.DEGRADED if reasons else RunStatus.PARSED

    return RunData(
        run_dir,
        status,
        reason="; ".join(reasons),
        columns=columns,
        rosparams=rosparams,
        manifest=manifest,
        num_rows=len(data_rows),
        notes=notes,
        raw_header=raw_header,
        timer_metadata=timer_metadata,
    )


def iter_run_dirs(root):
    """Yield run directories under ``root`` (any dir containing a performance log)."""
    root = Path(root)
    for log_file in sorted(root.rglob(PERFORMANCE_LOG)):
        yield log_file.parent


def load_runs(roots, dedup=True):
    """Load every run under one or more roots.

    Args:
        roots: a path or iterable of paths. When several roots are given, a run
            is identified by its ``rel_id`` ([task/]experiment/scene/algo/timestamp);
            on collision the EARLIER root in the list wins and the duplicate is
            recorded in the returned run's ``shadowed`` and notes — never silently
            merged. Runs under one root are all kept: two of its directories are two
            runs even when their ``rel_id`` matches (two trees above the task
            folders).
        dedup: set False to keep every directory even across roots.

    Returns:
        list[RunData] in load order (roots in the order given).
    """
    if isinstance(roots, (str, os.PathLike)):
        roots = [roots]

    runs = []
    # rel_id -> run, over the roots before the current one: only another root shadows a run.
    earlier_roots = {}
    for root in roots:
        this_root = {}
        for run_dir in iter_run_dirs(root):
            run = load_run(run_dir)
            if dedup and run.rel_id in earlier_roots:
                winner = earlier_roots[run.rel_id]
                winner.shadowed.append(run_dir)
                winner.notes.append("duplicate rel_id shadowed at %s (earlier root wins)" % run_dir)
                continue
            this_root.setdefault(run.rel_id, run)
            runs.append(run)
        earlier_roots.update(this_root)
    return runs


def aggregate_report(runs):
    """Human-readable accounting of a batch load, grouped by status."""
    # Shadowed duplicates were discovered too, so the counts below add up to the first line.
    shadowed = sum(len(r.shadowed) for r in runs)
    by_status = Counter(r.status.value for r in runs)
    lines = ["Discovered %d run(s):" % (len(runs) + shadowed)]
    for status in (RunStatus.PARSED, RunStatus.DEGRADED, RunStatus.SKIPPED):
        count = by_status.get(status.value, 0)
        lines.append("  %-8s %d" % (status.value + ":", count))
    if shadowed:
        lines.append("  %-8s %d (same rel_id under an earlier root)" % ("shadowed:", shadowed))

    degraded = [r for r in runs if r.status is RunStatus.DEGRADED]
    if degraded:
        lines.append("  degraded by reason:")
        for reason, count in Counter(r.reason for r in degraded).most_common():
            lines.append("    [%d] %s" % (count, reason))

    skipped = [r for r in runs if r.status is RunStatus.SKIPPED]
    if skipped:
        lines.append("  skipped by reason:")
        for reason, count in Counter(r.reason for r in skipped).most_common():
            lines.append("    [%d] %s" % (count, reason))

    return "\n".join(lines)
