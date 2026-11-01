"""Fingerprint: fixed FNV-1a vectors, determinism, and delimiter safety."""

from open_apero_analysis import fingerprint


def test_fnv1a_known_vectors():
    # Canonical FNV-1a 64-bit test vectors.
    assert fingerprint.fnv1a_64(b"") == fingerprint.FNV_OFFSET_BASIS
    assert fingerprint.fnv1a_64(b"a") == 0xAF63DC4C8601EC8C
    assert fingerprint.fnv1a_64(b"foobar") == 0x85944171F73967E8


def test_fingerprint_is_16_char_lowercase_hex():
    fp = fingerprint.schema_fingerprint([("PlanPathTotal", "double", "s", "")])
    assert len(fp) == 16
    assert fp == fp.lower()
    int(fp, 16)  # parses as hex


def test_fingerprint_is_deterministic():
    cols = [("a", "double", "s", ""), ("b", "int", "", "a")]
    assert fingerprint.schema_fingerprint(cols) == fingerprint.schema_fingerprint(cols)


def test_order_sensitive():
    a = [("x", "double", "", ""), ("y", "double", "", "")]
    b = [("y", "double", "", ""), ("x", "double", "", "")]
    assert fingerprint.schema_fingerprint(a) != fingerprint.schema_fingerprint(b)


def test_length_prefix_prevents_delimiter_collision():
    # Field boundaries must not be forgeable by injecting ':' or newlines.
    a = [("x:y", "double", "", "")]
    b = [("x", "y:double", "", "")]  # same bytes if ':' were the separator
    assert fingerprint.schema_fingerprint(a) != fingerprint.schema_fingerprint(b)

    c = [("x\ny", "double", "", "")]
    d = [("x", "y\ndouble", "", "")]
    assert fingerprint.schema_fingerprint(c) != fingerprint.schema_fingerprint(d)


def test_none_and_empty_unit_parent_equivalent():
    assert fingerprint.schema_fingerprint(
        [("x", "double", None, None)]
    ) == fingerprint.schema_fingerprint([("x", "double", "", "")])


def test_cross_language_reference_constant():
    # This exact schema and value are pinned in the C++ test
    # (open_apero_core/test/test_logging.cpp, kReferenceFingerprint). Both sides
    # must move together if the encoding ever changes.
    cols = [
        ("TotalCost", "double", "cost", ""),
        ("planning.GenerateNode", "double", "s", "planning.ExpandTime"),
        ("planning.PlanPathTime", "double", "s", "PlanPathTotal"),
        ("NumExpansions", "int", "count", ""),
    ]
    assert fingerprint.schema_fingerprint(cols) == "8da7ad974ddeb2ba"
