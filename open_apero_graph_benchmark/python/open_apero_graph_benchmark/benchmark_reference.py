"""Write, or check, the reference.json that stands in for a dataset's result files.

The benchmark result files are not shipped. Each dataset instead ships a reference.json with:

* ``digests``: per ``<replan>/<objective>/<algorithm>``, a SHA-256 (truncated to 128 bits) over
  every result file of that group (all graph instances and parameters, in path order): each
  file's relative path, then its gain, cost, node_id and num_nodes columns as raw text tokens.
  The measured computation_time column is left out.
* ``final_gain_ranges``: the paper-figure [min, max] final gain per
  ``<replan>/<objective>/<algorithm>`` over the parameter sweep, as
  benchmark_ranges.summarize_final_value_ranges computes it.
* ``computation_time_ranges``: the same for computation time [s], to 4 significant digits.
  Machine-dependent, so it is information only and never compared.

verify_benchmark_results.py compares a rerun against it. Write one from the
result files on disk; writing needs the plotting stack (seaborn and the open_apero_analysis
package) for the range tables. A dataset name reads the results run_benchmark.launch wrote below
~/open_apero_data/graph_benchmark/ and writes the shipped data/<dataset>/reference.json; a path
reads the results there and keeps the reference beside them::

    rosrun open_apero_graph_benchmark benchmark_reference.py --all            # every dataset
    rosrun open_apero_graph_benchmark benchmark_reference.py <dataset> --out-dir /tmp/refs
    rosrun open_apero_graph_benchmark benchmark_reference.py <dataset> --check  # vs reference
"""

import argparse
import hashlib
import json
import math
import re
import sys
from pathlib import Path

from open_apero_graph_benchmark.compare_result_trees import (
    RESULT_FILENAME_PATTERN,
    TIME_COLUMN_INDEX,
    find_result_files,
    read_rows,
)
from open_apero_graph_benchmark.paths import INPUT_ROOT, RESULTS_ROOT

REFERENCE_NAME = "reference.json"

FORMAT_VERSION = 1

DIGEST_COLUMNS = ("gain", "cost", "node_id", "num_nodes")

# Hex digits kept of each SHA-256: 128 bits, ample to detect a change, half the file size.
DIGEST_LENGTH = 32

# Significant digits kept of the informational computation-time ranges.
TIME_DIGITS = 4

# Range cells closer than this count as equal, so a reference survives floating-point noise in
# the interpolation without letting a real change through.
RANGE_TOLERANCE = 1e-9


def dataset_type(dataset_name):
    """Return ``"ap"`` or ``"op"`` for a dataset folder name."""
    if dataset_name.startswith("ap_"):
        return "ap"
    if dataset_name.startswith("op_"):
        return "op"
    raise ValueError("dataset name must start with 'ap_' or 'op_': {}".format(dataset_name))


def group_digests(dataset_folder):
    """Return ``({"<replan>/<objective>/<algorithm>": digest}, result file count)``."""
    hashes = {}
    files = find_result_files(dataset_folder)
    for relative_path in files:
        parts = relative_path.parts
        if len(parts) != 4:
            raise ValueError(
                "result file not at graph_N/<replan>/<objective>/: {}".format(relative_path)
            )
        algorithm = RESULT_FILENAME_PATTERN.match(parts[3]).group(1)
        key = "{}/{}/{}".format(parts[1], parts[2], algorithm)
        digest = hashes.setdefault(key, hashlib.sha256())
        digest.update((relative_path.as_posix() + "\n").encode())
        for row in read_rows(dataset_folder / relative_path):
            kept = (token for index, token in enumerate(row) if index != TIME_COLUMN_INDEX)
            digest.update((" ".join(kept) + "\n").encode())
    return {key: hashes[key].hexdigest()[:DIGEST_LENGTH] for key in sorted(hashes)}, len(files)


def range_table(dataset_folder, value_key, digits=None):
    """Return the paper-figure ranges as ``{"<replan>/<objective>/<algorithm>": [min, max]}``.

    ``digits`` rounds to that many significant digits. Imports the plotting stack lazily, so the
    digests work without it.
    """
    from open_apero_graph_benchmark.benchmark_ranges import (
        FIGURE_SELECTION,
        summarize_final_value_ranges,
    )

    replan_types, objectives, skip = FIGURE_SELECTION[dataset_type(dataset_folder.name)]
    summary = summarize_final_value_ranges(
        dataset_folder, replan_types, objectives, skip, value_key=value_key
    )
    table = {}
    for (replan_type, objective), per_algorithm in summary.items():
        for algorithm, (minimum, maximum) in per_algorithm.items():
            bounds = [float(minimum), float(maximum)]
            if digits is not None:
                bounds = [float("{:.{}g}".format(value, digits)) for value in bounds]
            table["{}/{}/{}".format(replan_type, objective, algorithm)] = bounds
    return table


def nested_ranges(table):
    """Turn a flat range table into ``{(replan, objective): {algorithm: (min, max)}}``."""
    nested = {}
    for key, (minimum, maximum) in table.items():
        replan_type, objective, algorithm = key.split("/")
        nested.setdefault((replan_type, objective), {})[algorithm] = (minimum, maximum)
    return nested


def reference_ranges(dataset_folder):
    """Return the reference's two range tables; raises ImportError without the plotting stack."""
    return {
        "final_gain_ranges": range_table(dataset_folder, "gain"),
        "computation_time_ranges": range_table(dataset_folder, "computation_time", TIME_DIGITS),
    }


def build_reference(dataset_folder, with_ranges=True):
    """Return the reference dict for a dataset folder holding result files.

    With ``with_ranges`` false the range tables are ``None``; otherwise a missing plotting stack
    raises ImportError.
    """
    dataset_folder = Path(dataset_folder)
    digests, count = group_digests(dataset_folder)
    if not count:
        raise ValueError("no result files under {}".format(dataset_folder))
    reference = {
        "format_version": FORMAT_VERSION,
        "dataset": dataset_folder.name,
        "generator": "scripts/benchmark_reference.py",
        "result_files": count,
        "digest_columns": list(DIGEST_COLUMNS),
        "digests": digests,
        "final_gain_ranges": None,
        "computation_time_ranges": None,
    }
    if with_ranges:
        reference.update(reference_ranges(dataset_folder))
    return reference


def dumps_reference(reference):
    """Serialize a reference as indented JSON with each list of scalars on one line."""
    text = json.dumps(reference, indent=2)
    return (
        re.sub(
            r"\[\s+([^\[\]{}]+?)\s+\]",
            lambda match: "[" + ", ".join(item.strip() for item in match.group(1).split(",")) + "]",
            text,
        )
        + "\n"
    )


def write_reference(reference, path):
    Path(path).write_text(dumps_reference(reference))


def load_reference(path):
    with open(path) as reference_file:
        reference = json.load(reference_file)
    if reference.get("format_version") != FORMAT_VERSION:
        raise ValueError(
            "{} has format_version {!r}, expected {}".format(
                path, reference.get("format_version"), FORMAT_VERSION
            )
        )
    return reference


def compare_references(reference, candidate, tolerance=RANGE_TOLERANCE):
    """Compare a rerun's reference with the shipped one.

    Returns ``(verdict, failures, notes)``: ``"exact"`` when every digest and the file count match;
    ``"mismatch"`` when the file counts differ or a digest group exists on one side only (listed in
    ``failures``); otherwise, when some digest differs (listed in ``notes``), ``"figure-level"``
    if every final-gain range cell agrees within ``tolerance`` and ``"mismatch"`` with the
    differing range cells in ``failures`` if not.
    """
    failures = []
    notes = []
    if reference["result_files"] != candidate["result_files"]:
        failures.append(
            "result files: reference {}, rerun {}".format(
                reference["result_files"], candidate["result_files"]
            )
        )
    for key in sorted(set(reference["digests"]) | set(candidate["digests"])):
        expected = reference["digests"].get(key)
        actual = candidate["digests"].get(key)
        if expected is None:
            failures.append("digest {}: not in the reference".format(key))
        elif actual is None:
            failures.append("digest {}: missing from the rerun".format(key))
        elif expected != actual:
            notes.append("digest {}: differs".format(key))
    if failures:
        return "mismatch", failures, notes
    if not notes:
        return "exact", [], []

    expected_ranges = reference.get("final_gain_ranges")
    actual_ranges = candidate.get("final_gain_ranges")
    if expected_ranges is None or actual_ranges is None:
        return (
            "mismatch",
            ["final-gain range tables unavailable, cannot compare at figure level"],
            notes,
        )

    for key in list(expected_ranges) + [k for k in actual_ranges if k not in expected_ranges]:
        expected = expected_ranges.get(key)
        actual = actual_ranges.get(key)
        if expected is None or actual is None:
            failures.append(
                "{}: reference {}, rerun {}".format(key, expected or "missing", actual or "missing")
            )
            continue
        if any(
            not math.isclose(e, a, rel_tol=0.0, abs_tol=tolerance) for e, a in zip(expected, actual)
        ):
            failures.append(
                "{}: reference [{!r}, {!r}], rerun [{!r}, {!r}]".format(key, *expected, *actual)
            )
    return ("figure-level" if not failures else "mismatch"), failures, notes


def resolve_dataset(dataset):
    """Return ``(results folder, folder of its reference.json)`` for a dataset name or path.

    An existing path is both. A name reads its results below ``RESULTS_ROOT``, where
    run_benchmark.launch writes them, and its reference in ``INPUT_ROOT``, where it ships.
    """
    candidate = Path(dataset)
    if candidate.is_dir():
        folder = candidate.resolve()
        return folder, folder
    results = RESULTS_ROOT / candidate
    if results.is_dir():
        shipped = INPUT_ROOT / candidate
        return results.resolve(), (shipped if shipped.is_dir() else results).resolve()
    raise ValueError("no such dataset directory: {} (no results in {})".format(dataset, results))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "datasets", nargs="*", help="Dataset names (results below %s), or paths." % RESULTS_ROOT
    )
    parser.add_argument("--all", action="store_true", help="Every ap_/op_ dataset under data/.")
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=None,
        help="Write <out-dir>/<dataset>/reference.json instead of the dataset's own reference.",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="Compare the result files on disk with the dataset's reference.json; write nothing.",
    )
    args = parser.parse_args(argv)

    datasets = list(args.datasets)
    if args.all:
        datasets = sorted(
            entry.name
            for entry in INPUT_ROOT.iterdir()
            if entry.is_dir() and entry.name[:3] in ("ap_", "op_")
        )
    if not datasets:
        parser.error("name at least one dataset, or pass --all")

    status = 0
    for dataset in datasets:
        try:
            folder, reference_folder = resolve_dataset(dataset)
            candidate = build_reference(folder, with_ranges=not args.check)
            reference = load_reference(reference_folder / REFERENCE_NAME) if args.check else None
            if args.check:
                try:
                    candidate.update(reference_ranges(folder))
                except ImportError:
                    pass  # compare_references reports the ranges as unavailable.
        except (ValueError, OSError) as error:
            # OSError covers benchmark_ranges.MissingResultsError: a partial results tree.
            print("ERROR: {}: {}".format(dataset, error))
            status = 1
            continue
        except ImportError as error:
            print(
                "ERROR: {}: writing {} needs the plotting stack (seaborn and the "
                "open_apero_analysis package) for its final-gain ranges: {}".format(
                    dataset, REFERENCE_NAME, error
                )
            )
            status = 1
            continue
        if args.check:
            verdict, failures, notes = compare_references(reference, candidate)
            print("{}: {}".format(folder.name, verdict))
            for line in failures + notes:
                print("  " + line)
            status |= verdict == "mismatch"
            continue
        target = (args.out_dir / folder.name if args.out_dir else reference_folder) / REFERENCE_NAME
        target.parent.mkdir(parents=True, exist_ok=True)
        write_reference(candidate, target)
        print(
            "Wrote {} ({} result files, {} bytes)".format(
                target, candidate["result_files"], target.stat().st_size
            )
        )
    return status


if __name__ == "__main__":
    sys.exit(main())
