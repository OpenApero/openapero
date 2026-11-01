"""Deterministic schema fingerprint, byte-compatible with the C++ writer.

A run's ``run_manifest.yaml`` records a ``schema_fingerprint`` so a consumer can
tell two CSV schemas apart without diffing headers. The fingerprint must be
identical whether computed by the C++ logger or recomputed here from the
manifest, so the encoding is pinned exactly and reimplemented on both sides.

Canonical encoding
------------------
The schema is an ordered list of columns; each column contributes four string
fields in this order: ``name``, ``type``, ``unit``, ``qualified_parent`` (empty
strings for absent unit/parent). Every field is serialized as::

    <4-byte big-endian unsigned length><UTF-8 bytes>

and all fields of all columns are concatenated in column order. The length
prefix makes ``:`` or newline in any field unable to collide with a delimiter,
so no escaping is needed. The resulting byte stream is hashed with 64-bit
FNV-1a and rendered as lowercase zero-padded hex (16 chars).

``std::hash`` is deliberately NOT used: it is not a portable, stable contract.
"""

import struct

# 64-bit FNV-1a constants (spelled out identically in the C++ implementation).
FNV_OFFSET_BASIS = 14695981039346656037
FNV_PRIME = 1099511628211
_MASK_64 = (1 << 64) - 1


def fnv1a_64(data):
    """Return the 64-bit FNV-1a hash of ``data`` (bytes) as an int."""
    h = FNV_OFFSET_BASIS
    for byte in data:
        h ^= byte
        h = (h * FNV_PRIME) & _MASK_64
    return h


def _encode_field(text):
    """Length-prefixed UTF-8 encoding of a single field."""
    raw = (text or "").encode("utf-8")
    return struct.pack(">I", len(raw)) + raw


def encode_schema(columns):
    """Serialize an ordered column schema to its canonical byte stream.

    Args:
        columns: iterable of (name, type, unit, qualified_parent) tuples;
            unit/qualified_parent may be None or "".

    Returns:
        The canonical bytes that get hashed.
    """
    buf = bytearray()
    for name, type_, unit, parent in columns:
        buf += _encode_field(name)
        buf += _encode_field(type_)
        buf += _encode_field(unit)
        buf += _encode_field(parent)
    return bytes(buf)


def schema_fingerprint(columns):
    """Return the lowercase-hex 64-bit fingerprint of a column schema.

    Args:
        columns: iterable of (name, type, unit, qualified_parent) tuples.
    """
    return "{:016x}".format(fnv1a_64(encode_schema(columns)))
