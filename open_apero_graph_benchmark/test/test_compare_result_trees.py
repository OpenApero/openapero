"""The result-tree comparator must detect real drift and tolerate measured time.

A comparator that passes when handed two copies of the same tree proves nothing: it
would also pass if it compared nothing at all. These tests pin the negative cases --
what it must catch -- alongside the differences it must forgive.
"""

import tempfile
from pathlib import Path

from open_apero_graph_benchmark.compare_result_trees import compare_trees

BASELINE_ROWS = "10 1.5 0.001 3 40\n20 2.5 0.002 7 40\n"

RESULT_PATH = "everystep/path_gain/dbs_1.txt"


def write_tree(root, files):
    """Create ``files`` (relative path -> contents) under ``root`` and return it."""
    for relative_path, contents in files.items():
        path = root / relative_path
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(contents)
    return root


def compare(baseline_files, candidate_files):
    """Build two throwaway trees and return ``(failures, reports)``."""
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        baseline = write_tree(root / "baseline", baseline_files)
        candidate = write_tree(root / "candidate", candidate_files)
        return compare_trees(baseline, candidate)


def test_identical_trees_pass():
    failures, reports = compare({RESULT_PATH: BASELINE_ROWS}, {RESULT_PATH: BASELINE_ROWS})
    assert failures == []
    assert reports == []


def test_differing_timing_is_accepted():
    """Column 3 may differ freely -- it is measured wall-clock time."""
    failures, _ = compare(
        {RESULT_PATH: "10 1.5 0.001 3 40\n20 2.5 0.002 7 40\n"},
        {RESULT_PATH: "10 1.5 9.999 3 40\n20 2.5 12.5 7 40\n"},
    )
    assert failures == []


def test_missing_result_file_is_detected():
    failures, _ = compare({RESULT_PATH: BASELINE_ROWS}, {"placeholder.md": ""})
    assert any("missing from candidate" in failure for failure in failures)


def test_extra_result_file_is_detected():
    failures, _ = compare(
        {RESULT_PATH: BASELINE_ROWS},
        {RESULT_PATH: BASELINE_ROWS, "everystep/path_gain/dbs_2.txt": BASELINE_ROWS},
    )
    assert any("missing from baseline" in failure for failure in failures)


def test_changed_row_count_is_detected():
    failures, _ = compare({RESULT_PATH: BASELINE_ROWS}, {RESULT_PATH: "10 1.5 0.001 3 40\n"})
    assert any("row count changed" in failure for failure in failures)


def test_integer_versus_float_spelling_is_detected():
    """``1`` and ``1.0`` are the same number but a different emitted precision."""
    failures, _ = compare({RESULT_PATH: "10 1 0.001 3 40\n"}, {RESULT_PATH: "10 1.0 0.001 3 40\n"})
    assert any("cost changed" in failure for failure in failures)


def test_changed_gain_and_node_id_are_detected():
    failures, _ = compare(
        {RESULT_PATH: "10 1.5 0.001 3 40\n"}, {RESULT_PATH: "11 1.5 0.001 9 40\n"}
    )
    assert any("gain changed" in failure for failure in failures)
    assert any("node_id changed" in failure for failure in failures)


def test_changed_num_nodes_is_detected():
    failures, _ = compare(
        {RESULT_PATH: "10 1.5 0.001 3 40\n"}, {RESULT_PATH: "10 1.5 0.001 3 41\n"}
    )
    assert any("num_nodes changed" in failure for failure in failures)


def test_non_finite_timing_is_detected():
    failures, _ = compare({RESULT_PATH: "10 1.5 0.001 3 40\n"}, {RESULT_PATH: "10 1.5 inf 3 40\n"})
    assert any("not finite" in failure for failure in failures)


def test_decreasing_timing_is_reported_not_failed():
    """Non-monotonic cumulative time is a clock artifact, not a code change."""
    rows = "10 1.5 5.0 3 40\n20 2.5 1.0 7 40\n"
    failures, reports = compare({RESULT_PATH: rows}, {RESULT_PATH: rows})
    assert failures == []
    assert any("decreased" in report for report in reports)


def test_wrong_column_count_is_detected():
    failures, _ = compare({RESULT_PATH: "10 1.5 0.001 3 40\n"}, {RESULT_PATH: "10 1.5 0.001 3\n"})
    assert any("columns, expected 5" in failure for failure in failures)


def test_non_result_files_are_ignored():
    """Copied inputs and stray files must not register as tree differences."""
    failures, _ = compare(
        {
            RESULT_PATH: BASELINE_ROWS,
            "graph_0/graph.txt": "0 0 0 0 1\n",
            "gain_model.txt": "0 0 0 0 100\n",
        },
        {
            RESULT_PATH: BASELINE_ROWS,
            "graph_topology.txt": "0 0 0 0 1\n",
            "run.log": "noise\n",
            "gain_ranges_horizontal.pdf": "%PDF\n",
        },
    )
    assert failures == []


def test_both_trees_empty_is_a_failure():
    """An empty comparison must not silently look like success."""
    failures, _ = compare({"notes.md": ""}, {"notes.md": ""})
    assert any("no result files found" in failure for failure in failures)


def test_spt_filename_format_is_pinned():
    """``spt_0.50.txt`` and ``spt_0.5.txt`` are different files, not the same one."""
    failures, _ = compare(
        {"everystep/path_gain/spt_0.50.txt": BASELINE_ROWS},
        {"everystep/path_gain/spt_0.5.txt": BASELINE_ROWS},
    )
    assert any("missing from candidate" in failure for failure in failures)
    assert any("missing from baseline" in failure for failure in failures)


def test_short_row_fails_cleanly_instead_of_crashing():
    """A row too short to hold a timing column must report, not raise IndexError.

    compare_rows indexes every column directly, so compare_trees validates row widths first;
    check_timing_column skips rows too short to hold a timing column.
    """
    failures, _ = compare({RESULT_PATH: "10 1.5 0.001 3 40\n"}, {RESULT_PATH: "10 1.5\n"})
    assert any("columns, expected 5" in failure for failure in failures)


def test_empty_candidate_row_fails_cleanly():
    """A single-token row is the degenerate case of the same problem."""
    failures, _ = compare({RESULT_PATH: "10 1.5 0.001 3 40\n"}, {RESULT_PATH: "10\n"})
    assert any("columns, expected 5" in failure for failure in failures)


def test_short_baseline_row_also_reports():
    """Width validation covers the baseline side too, not just the candidate."""
    failures, _ = compare({RESULT_PATH: "10 1.5\n"}, {RESULT_PATH: "10 1.5 0.001 3 40\n"})
    assert any("baseline row has 2 columns" in failure for failure in failures)


def test_malformed_baseline_timing_is_detected():
    """Timing validity is checked on both sides, not just the candidate.

    A corrupt baseline invalidates the comparison exactly as much as a corrupt candidate, so
    trusting it silently would let a broken reference tree report success.
    """
    failures, _ = compare(
        {RESULT_PATH: "10 1.5 notanumber 3 40\n"}, {RESULT_PATH: "10 1.5 0.001 3 40\n"}
    )
    assert any("baseline computation_time" in failure for failure in failures)


def test_non_finite_baseline_timing_is_detected():
    failures, _ = compare({RESULT_PATH: "10 1.5 nan 3 40\n"}, {RESULT_PATH: "10 1.5 0.001 3 40\n"})
    assert any(
        "baseline computation_time" in failure and "not finite" in failure for failure in failures
    )


def test_candidate_timing_failures_still_name_their_side():
    failures, _ = compare({RESULT_PATH: "10 1.5 0.001 3 40\n"}, {RESULT_PATH: "10 1.5 inf 3 40\n"})
    assert any("candidate computation_time" in failure for failure in failures)
