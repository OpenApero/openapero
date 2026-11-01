"""Compare two graph-benchmark result trees for everything except measured time.

Benchmark result files cannot be byte-identical across runs: column 3 is wall-clock
computation time. Everything else is deterministic for a fixed graph and seed, so a
refactor that leaves planning untouched must reproduce columns 1, 2, 4 and 5 exactly.

What is compared:

* Only files whose name matches the result pattern ``<algorithm>_<parameter>.txt``.
  Inputs copied alongside the results (``graph.txt``, ``graph_topology.txt``,
  ``gain_model.txt``), logs and figures are ignored, so a copied input can never be
  reported as an extra file.
* The set of result files and their paths relative to each tree root, which pins the
  per-planner parameter formatting (``dbs_1.txt`` versus ``spt_0.50.txt``).
* Row counts.
* Columns 1, 2, 4 and 5 (gain, cost, node_id, num_nodes) as **raw text tokens**.
  Parsing to float would let ``1`` and ``1.0`` compare equal, and a change in emitted
  precision is exactly the drift this check exists to catch.
* Column 3 for validity only, **on both sides**: every value must parse and be finite --
  a corrupt timing invalidates the comparison whichever tree it is in. Values are never
  required to match between the trees. A decreasing cumulative time is reported but not failed:
  ``std::chrono::high_resolution_clock`` is not guaranteed monotonic (libstdc++ aliases
  it to ``system_clock``, which can be stepped backwards), so non-monotonicity is a
  platform artifact rather than evidence of a code change.

Usage:
    rosrun open_apero_graph_benchmark compare_result_trees.py <baseline_dir> <candidate_dir>

Exit status is 0 when the trees agree on everything required, 1 otherwise.
"""

import argparse
import math
import re
import sys
from pathlib import Path

# Stricter than benchmark_ranges.GROUPING_PATTERN and kResultPattern (isBenchmarkResultFilename in
# include/open_apero_graph_benchmark/benchmark_sweep.h), which prefix-match the stem: only exact
# ``<algorithm>_<parameter>.txt`` names are compared, so ``dbs_1_copy.txt`` is not.
RESULT_FILENAME_PATTERN = re.compile(r"^(nbs|dbs|spt|tsp)_(\d+(?:\.\d+)?)\.txt$")

EXPECTED_COLUMN_COUNT = 5

# Column 3 (index 2) is measured time; every other column must match exactly.
TIME_COLUMN_INDEX = 2

COLUMN_NAMES = ("gain", "cost", "computation_time", "node_id", "num_nodes")


def find_result_files(root):
    """Return result-file paths relative to ``root``, sorted, ignoring non-results."""
    return sorted(
        path.relative_to(root)
        for path in root.rglob("*.txt")
        if RESULT_FILENAME_PATTERN.match(path.name)
    )


def read_rows(path):
    """Return each non-empty line of ``path`` split into whitespace-separated tokens."""
    rows = []
    with open(path) as result_file:
        for line in result_file:
            tokens = line.split()
            if tokens:
                rows.append(tokens)
    return rows


def check_row_widths(rows, relative_path, side, failures):
    """Report any row that is not exactly five columns. Returns True when all rows are.

    Runs before anything indexes a column, so a truncated row is reported as a comparison
    failure rather than raising IndexError out of the middle of the run.
    """
    all_valid = True
    for row_index, row in enumerate(rows, start=1):
        if len(row) != EXPECTED_COLUMN_COUNT:
            failures.append(
                f"{relative_path}:{row_index}: {side} row has {len(row)} columns, "
                f"expected {EXPECTED_COLUMN_COUNT}"
            )
            all_valid = False
    return all_valid


def check_timing_column(rows, relative_path, side, failures, reports):
    """Validate column 3 on one side, without requiring the two sides to match.

    Applied to the baseline as well as the candidate: an unparseable or non-finite timing
    means that tree is corrupt, which invalidates the comparison whichever side it is on.

    Rows too short to have a timing column are skipped: check_row_widths has already
    reported them, and re-reporting here would just duplicate the failure.
    """
    previous = None
    for row_index, row in enumerate(rows, start=1):
        if len(row) <= TIME_COLUMN_INDEX:
            continue
        token = row[TIME_COLUMN_INDEX]
        try:
            value = float(token)
        except ValueError:
            failures.append(
                f"{relative_path}:{row_index}: {side} computation_time {token!r} is not a number"
            )
            continue
        if not math.isfinite(value):
            failures.append(
                f"{relative_path}:{row_index}: {side} computation_time {token!r} is not finite"
            )
            continue
        if previous is not None and value < previous:
            # Reported, never failed: see the module docstring on clock monotonicity.
            reports.append(
                f"{relative_path}:{row_index}: {side} cumulative computation_time decreased "
                f"({previous} -> {value})"
            )
        previous = value


def compare_rows(baseline_rows, candidate_rows, relative_path, failures):
    """Compare row counts and every column except the measured time.

    Assumes both sides have already passed check_row_widths.
    """
    if len(baseline_rows) != len(candidate_rows):
        failures.append(
            f"{relative_path}: row count changed "
            f"({len(baseline_rows)} -> {len(candidate_rows)})"
        )
        return

    for row_index, (baseline_row, candidate_row) in enumerate(
        zip(baseline_rows, candidate_rows), start=1
    ):
        for column_index in range(EXPECTED_COLUMN_COUNT):
            if column_index == TIME_COLUMN_INDEX:
                continue
            if baseline_row[column_index] != candidate_row[column_index]:
                failures.append(
                    f"{relative_path}:{row_index}: {COLUMN_NAMES[column_index]} changed "
                    f"({baseline_row[column_index]!r} -> {candidate_row[column_index]!r})"
                )


def compare_trees(baseline_root, candidate_root):
    """Return ``(failures, reports)`` for two result trees."""
    failures = []
    reports = []

    baseline_files = find_result_files(baseline_root)
    candidate_files = find_result_files(candidate_root)

    for missing in sorted(set(baseline_files) - set(candidate_files)):
        failures.append(f"{missing}: present in baseline, missing from candidate")
    for extra in sorted(set(candidate_files) - set(baseline_files)):
        failures.append(f"{extra}: present in candidate, missing from baseline")

    for relative_path in sorted(set(baseline_files) & set(candidate_files)):
        baseline_rows = read_rows(baseline_root / relative_path)
        candidate_rows = read_rows(candidate_root / relative_path)

        # Width first: everything below indexes fixed columns, so a truncated row has to be
        # reported as a failure here rather than raising out of the comparison.
        baseline_ok = check_row_widths(baseline_rows, relative_path, "baseline", failures)
        candidate_ok = check_row_widths(candidate_rows, relative_path, "candidate", failures)
        if baseline_ok and candidate_ok:
            compare_rows(baseline_rows, candidate_rows, relative_path, failures)
        check_timing_column(baseline_rows, relative_path, "baseline", failures, reports)
        check_timing_column(candidate_rows, relative_path, "candidate", failures, reports)

    if not baseline_files and not candidate_files:
        failures.append(
            "no result files found in either tree; check the paths and the result pattern"
        )

    return failures, reports


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Compare two graph-benchmark result trees on every column except "
        "the measured computation time."
    )
    parser.add_argument("baseline", type=Path, help="Reference result tree.")
    parser.add_argument("candidate", type=Path, help="Result tree to check.")
    args = parser.parse_args(argv)

    for root in (args.baseline, args.candidate):
        if not root.is_dir():
            parser.error(f"not a directory: {root}")

    failures, reports = compare_trees(args.baseline, args.candidate)

    for report in reports:
        print(f"note: {report}")
    for failure in failures:
        print(f"FAIL: {failure}")

    compared = len(set(find_result_files(args.baseline)) & set(find_result_files(args.candidate)))
    if failures:
        print(f"\n{len(failures)} difference(s) across {compared} shared result file(s).")
        return 1

    print(f"OK: {compared} result file(s) match on gain, cost, node_id and num_nodes.")
    if reports:
        print(f"({len(reports)} timing note(s) above, not treated as failures.)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
