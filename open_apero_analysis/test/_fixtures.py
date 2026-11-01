"""Test helpers: load the header fixture and synthesize run dirs."""

import importlib.util
from pathlib import Path

from open_apero_analysis.fingerprint import schema_fingerprint

_FIXTURE_FILE = Path(__file__).parent / "fixtures" / "headers.py"
_spec = importlib.util.spec_from_file_location("_apero_headers", _FIXTURE_FILE)
_module = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_module)

current_header = _module.current_header


def write_run(
    root,
    header,
    num_rows=2,
    rel=("HSSD", "scene1", "nbs", "20260101_000000"),
    rosparams=None,
    manifest=None,
):
    """Create a run directory containing a performance_log.csv (and optional yaml).

    Returns the run directory Path.
    """
    run_dir = Path(root)
    for part in rel:
        run_dir = run_dir / part
    run_dir.mkdir(parents=True, exist_ok=True)

    ncols = len(header.split(","))
    lines = [header]
    for r in range(num_rows):
        lines.append(",".join(str(float(r * ncols + c)) for c in range(ncols)))
    (run_dir / "performance_log.csv").write_text("\n".join(lines) + "\n")

    if rosparams is not None:
        (run_dir / "rosparams.yaml").write_text(rosparams)
    if manifest is not None:
        (run_dir / "run_manifest.yaml").write_text(manifest)
    return run_dir


def manifest_for(
    header, format_version=1, fingerprint=None, columns=None, layers=("backtracking", "planning")
):
    """Render a v1 manifest describing ``header``, as the C++ writer would.

    Every column is declared a second-valued root timer, which is enough for the
    reader's structural checks and keeps the fixture independent of the real
    timer tree. ``columns`` overrides the (name, type, unit, parent) tuples so a
    test can reorder or corrupt one field; ``fingerprint`` overrides the
    computed value so a test can force a mismatch. ``layers`` is a sequence of
    instance names, each its own layer type, or an instance -> type mapping.
    """
    if columns is None:
        columns = [(name, "double", "s", "") for name in header.split(",")]
    if fingerprint is None:
        fingerprint = schema_fingerprint(columns)

    lines = [
        "format_version: %s" % format_version,
        'schema_fingerprint: "%s"' % fingerprint,
        "layers:",
    ]
    items = layers.items() if isinstance(layers, dict) else ((name, name) for name in layers)
    lines += ['  "%s": "%s"' % item for item in items]
    lines.append("columns:")
    lines += [
        '  - {name: "%s", type: "%s", unit: "%s", parent: "%s"}' % column for column in columns
    ]
    return "\n".join(lines) + "\n"
