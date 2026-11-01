"""Manifest-driven column metadata and structural timer selection.

The C++ writer records, per column, ``{name, type, unit, parent}`` in
``run_manifest.yaml`` (see ``open_apero_core/.../run_manifest.cpp``). ``parent``
is the *qualified parent timer* a sub-phase timer nests inside; a **root timer**
records an empty parent. Because the parents are authored explicitly for
non-overlapping summation, the manifest — not a hard-coded timer tree — is the
source of truth for which timers may be summed without double counting.

This module:

* validates whether a manifest's column schema can be *trusted* for structural
  selection (:class:`TimerMetaState`): its schema fingerprint, and its columns
  against the CSV header;
* turns each column into a :class:`ColumnMeta` (name, parent, unit, and the
  owning layer instance/type derived from the manifest ``layers`` map); and
* returns the ordered set of **all** structural root timers
  (:func:`root_timers`). It embeds no consumer scope: a caller (the node
  CPU-time scope, the planning-time scope) filters roots by layer identity and
  adds its own agent-level components.
"""

import enum
from collections import namedtuple

from open_apero_analysis.fingerprint import schema_fingerprint

# Manifest format versions this reader understands (kRunManifestFormatVersion).
SUPPORTED_FORMAT_VERSIONS = frozenset({1})

# A timer column is one measured in seconds.
TIMER_UNIT = "s"


class TimerMetaState(enum.Enum):
    """Trust level of a run's manifest column schema for timer selection."""

    ABSENT = "absent"  # no manifest file / no manifest mapping at all
    INVALID = "invalid"  # malformed / unsupported / no column schema / bad fingerprint / mismatch
    VALID = "valid"  # safe for structural timer selection


class ColumnMeta(
    namedtuple(
        "ColumnMeta",
        [
            "name",
            "type",
            "unit",
            "parent",
            "layer_instance",
            "layer_type",
        ],
    )
):
    """Metadata for one manifest column.

    ``parent`` is ``""`` for a root timer (never ``None``-vs-``""`` ambiguity).
    ``layer_instance``/``layer_type`` are ``None`` for agent-level columns (those
    with no owning layer prefix).
    """

    __slots__ = ()

    @property
    def is_timer(self):
        return self.unit == TIMER_UNIT

    @property
    def is_root(self):
        """A root timer: measured in seconds and nesting under no parent."""
        return self.is_timer and not self.parent

    @property
    def is_agent_level(self):
        return self.layer_instance is None


class TimerMetadata(namedtuple("TimerMetadata", ["state", "columns", "layers", "diagnostics"])):
    """Outcome of validating a manifest's column schema.

    ``columns`` is an ordered tuple of :class:`ColumnMeta` (empty unless
    ``state`` is VALID). ``layers`` maps layer *instance* name to layer *type*.
    ``diagnostics`` explains a non-VALID state.
    """

    __slots__ = ()

    @property
    def is_valid(self):
        return self.state is TimerMetaState.VALID


TimerSelection = namedtuple("TimerSelection", ["columns", "diagnostics"])
"""Ordered timer column names, read from a valid manifest.

``columns`` preserves manifest/CSV order. Without a valid manifest it is empty
and ``diagnostics`` says why.
"""


def _layer_for(name, layers):
    """Return (instance, type) for a column via the longest matching layer prefix.

    A column ``planning.ExpandTime`` belongs to instance ``planning``; matching
    the longest recorded instance prefix keeps it correct even if one instance
    name is a prefix of another.
    """
    best_instance = None
    for instance in layers:
        prefix = instance + "."
        if name.startswith(prefix):
            if best_instance is None or len(instance) > len(best_instance):
                best_instance = instance
    if best_instance is None:
        return None, None
    return best_instance, layers[best_instance]


def _validated_layers(raw_layers):
    """The manifest ``layers`` mapping (instance name -> layer type), validated.

    Raises :class:`ValueError` if a key/value is not a non-empty string.
    """
    for instance, layer_type in raw_layers.items():
        if not isinstance(instance, str) or not instance:
            raise ValueError("layer instance name is not a non-empty string: %r" % (instance,))
        if not isinstance(layer_type, str) or not layer_type:
            raise ValueError(
                "layer type for %r is not a non-empty string: %r" % (instance, layer_type)
            )
    return dict(raw_layers)


def load_timer_metadata(manifest, csv_header=None):
    """Validate a manifest's column schema for timer selection.

    Args:
        manifest: the parsed ``run_manifest.yaml`` mapping (or ``None``).
        csv_header: the ordered CSV header list. When given, the manifest's
            column names must match it exactly (order included); a mismatch
            invalidates *structural* selection.

    Returns:
        A :class:`TimerMetadata`. Only ``VALID`` carries usable ``columns``.
    """
    # Only a genuinely absent manifest is ABSENT; a present-but-non-mapping value
    # is corrupt content -> INVALID.
    if manifest is None:
        return TimerMetadata(TimerMetaState.ABSENT, (), {}, ("manifest is absent",))
    if not isinstance(manifest, dict):
        return TimerMetadata(
            TimerMetaState.INVALID,
            (),
            {},
            ("manifest is not a mapping: %r" % (type(manifest).__name__,),),
        )

    def invalid(reason):
        return TimerMetadata(TimerMetaState.INVALID, (), {}, (reason,))

    # A manifest file that exists but failed to parse is corrupt, not absent
    # (run_data marks it so). It is not trustworthy.
    if manifest.get("__malformed__"):
        return invalid("manifest failed to parse")

    version = manifest.get("format_version")
    if version is not None and type(version) is not int:
        return invalid("format_version is not an integer: %r" % (version,))
    if version not in SUPPORTED_FORMAT_VERSIONS:
        return invalid("missing/unsupported format_version: %r" % (version,))

    # The writer always records the column schema.
    raw_columns = manifest.get("columns")
    if raw_columns is None:
        return invalid("manifest has no column schema")
    if not isinstance(raw_columns, list) or not raw_columns:
        return invalid("columns is not a non-empty list")

    raw_tuples = []
    for entry in raw_columns:
        if not isinstance(entry, dict):
            return invalid("column entry is not a mapping: %r" % (entry,))
        name = entry.get("name")
        if not isinstance(name, str) or not name:
            return invalid("column entry has no valid 'name': %r" % (entry,))
        for field in ("type", "unit", "parent"):
            value = entry.get(field)
            if value is not None and not isinstance(value, str):
                return invalid("column '%s' field '%s' is not a string: %r" % (name, field, value))
        raw_tuples.append(
            (
                name,
                entry.get("type") or "",
                entry.get("unit") or "",
                entry.get("parent") or "",
            )
        )

    # A v1 writer always emits a schema_fingerprint; its absence means the
    # manifest was not produced by a trusted writer -> not trustworthy.
    stored_fp = manifest.get("schema_fingerprint")
    if stored_fp is None:
        return invalid("missing schema_fingerprint")
    # A mismatch means the manifest and its own schema are inconsistent.
    if str(stored_fp) != schema_fingerprint(raw_tuples):
        return invalid("schema_fingerprint mismatch")

    names = [t[0] for t in raw_tuples]
    if csv_header is not None and list(csv_header) != names:
        return invalid("manifest columns disagree with CSV header")

    raw_layers = manifest.get("layers")
    if raw_layers is None:
        raw_layers = {}
    elif not isinstance(raw_layers, dict):
        return invalid("layers is not a mapping: %r" % (type(raw_layers).__name__,))

    try:
        layers = _validated_layers(raw_layers)
    except ValueError as exc:
        return invalid("invalid layers mapping: %s" % (exc,))

    columns = []
    seen = set()
    for name, type_, unit, parent in raw_tuples:
        if name in seen:
            return invalid("duplicate column '%s'" % name)
        seen.add(name)
        instance, layer_type = _layer_for(name, layers)
        columns.append(
            ColumnMeta(
                name=name,
                type=type_,
                unit=unit,
                parent=parent,
                layer_instance=instance,
                layer_type=layer_type,
            )
        )

    return TimerMetadata(TimerMetaState.VALID, tuple(columns), layers, ())


def root_timers(metadata):
    """Ordered tuple of all structural root-timer :class:`ColumnMeta`.

    Empty unless ``metadata`` is VALID. Includes both layer roots (e.g.
    ``planning.ExpandTime``, ``planning.StartNodeUpdate``) and agent-level
    timers (e.g. ``PlanPathTotal``); the caller applies its own scope.
    """
    if not metadata.is_valid:
        return ()
    return tuple(c for c in metadata.columns if c.is_root)
