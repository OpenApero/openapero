"""The reference.json that stands in for the unshipped result files: digests, ranges, verdicts."""

import copy
import json
import shutil
import tempfile
from pathlib import Path
from unittest import mock

import matplotlib

matplotlib.use("Agg")  # benchmark_ranges imports pyplot.

import pytest
from _result_trees import OBJECTIVES, REPLANS, RESULT_FILES, write_results

from open_apero_graph_benchmark import benchmark_reference
from open_apero_graph_benchmark.benchmark_reference import (
    REFERENCE_NAME,
    build_reference,
    compare_references,
    dumps_reference,
    load_reference,
    main,
    write_reference,
)


@pytest.fixture
def root():
    with tempfile.TemporaryDirectory() as directory:
        yield Path(directory)


def test_digests_cover_every_group_and_ignore_the_time_column(root):
    first = build_reference(write_results(root / "a" / "ap_test"))
    second = build_reference(write_results(root / "b" / "ap_test", time_scale=0.5))
    assert first["result_files"] == 2 * len(REPLANS) * len(OBJECTIVES) * len(RESULT_FILES)
    assert len(first["digests"]) == len(REPLANS) * len(OBJECTIVES) * 2
    assert "goal/path_gain/nbs" in first["digests"]
    assert first["digests"] == second["digests"]
    assert compare_references(first, second) == ("exact", [], [])
    # Computation time is recorded for information and differs, but is never compared.
    assert first["computation_time_ranges"] != second["computation_time_ranges"]


def test_a_changed_gain_changes_only_its_group_digest(root):
    reference = build_reference(write_results(root / "ap_test"))
    changed = root / "ap_test" / "graph_1" / "goal" / "path_gain" / "tsp_0.50.txt"
    changed.write_text(changed.read_text().replace("100.0 5", "100.5 5"))
    rerun = build_reference(root / "ap_test")
    differing = [
        key for key in reference["digests"] if reference["digests"][key] != rerun["digests"][key]
    ]
    assert differing == ["goal/path_gain/tsp"]


def test_ranges_hold_the_paper_figure_cells(root):
    reference = build_reference(write_results(root / "ap_test"))
    ranges = reference["final_gain_ranges"]
    # a-priori selection: every replan x {path_gain, path_ratio}, without none/path_ratio.
    assert "none/path_ratio/nbs" not in ranges
    assert ranges["goal/path_ratio/nbs"] == pytest.approx([50.0, 50.0])
    assert ranges["goal/path_ratio/tsp"] == pytest.approx([100.0, 100.0])


def test_a_corrupted_digest_falls_back_to_the_figure_level(root):
    reference = build_reference(write_results(root / "ap_test"))
    stored = copy.deepcopy(reference)
    stored["digests"]["everystep/table_gain/nbs"] = "0" * 32
    verdict, failures, notes = compare_references(stored, reference)
    assert verdict == "figure-level"
    assert failures == []
    assert notes == ["digest everystep/table_gain/nbs: differs"]


def test_a_range_cell_off_by_more_than_the_tolerance_is_a_mismatch(root):
    reference = build_reference(write_results(root / "ap_test"))
    stored = copy.deepcopy(reference)
    stored["digests"]["goal/path_gain/nbs"] = "0" * 32
    stored["final_gain_ranges"]["goal/path_gain/nbs"][1] += 1e-6
    verdict, failures, _notes = compare_references(stored, reference)
    assert verdict == "mismatch"
    assert len(failures) == 1 and failures[0].startswith("goal/path_gain/nbs: reference")

    within = copy.deepcopy(reference)
    within["digests"]["goal/path_gain/nbs"] = "0" * 32
    within["final_gain_ranges"]["goal/path_gain/nbs"][1] += 1e-12
    assert compare_references(within, reference)[0] == "figure-level"


def test_a_missing_group_is_a_mismatch(root):
    reference = build_reference(write_results(root / "ap_test"))
    stored = copy.deepcopy(reference)
    stored["digests"]["none/table_gain/dbs"] = "f" * 32
    verdict, failures, notes = compare_references(stored, reference)
    assert verdict == "mismatch"
    assert failures == ["digest none/table_gain/dbs: missing from the rerun"]
    assert notes == []


def test_a_group_only_in_the_rerun_is_a_mismatch(root):
    reference = build_reference(write_results(root / "ap_test"))
    rerun = copy.deepcopy(reference)
    rerun["digests"]["none/table_gain/dbs"] = "f" * 32
    verdict, failures, _notes = compare_references(reference, rerun)
    assert verdict == "mismatch"
    assert failures == ["digest none/table_gain/dbs: not in the reference"]


def test_a_different_result_file_count_is_a_mismatch(root):
    reference = build_reference(write_results(root / "ap_test"))
    rerun = copy.deepcopy(reference)
    rerun["result_files"] -= 1
    rerun["digests"]["goal/path_gain/nbs"] = "0" * 32
    verdict, failures, notes = compare_references(reference, rerun)
    assert verdict == "mismatch"
    count = reference["result_files"]
    assert failures == ["result files: reference {}, rerun {}".format(count, count - 1)]
    assert notes == ["digest goal/path_gain/nbs: differs"]


def test_a_rerun_missing_whole_groups_is_a_mismatch_even_with_equal_ranges(root):
    reference = build_reference(write_results(root / "full" / "ap_test"))
    partial = write_results(root / "partial" / "ap_test")
    for graph in ("graph_0", "graph_1"):
        shutil.rmtree(partial / graph / "none")
        for replan in ("goal", "everystep"):
            shutil.rmtree(partial / graph / replan / "table_gain")
    rerun = build_reference(partial, with_ranges=False)
    rerun["final_gain_ranges"] = reference["final_gain_ranges"]
    verdict, failures, notes = compare_references(reference, rerun)
    assert verdict == "mismatch"
    assert failures[0] == "result files: reference 48, rerun 24"
    assert "digest none/path_gain/nbs: missing from the rerun" in failures
    assert "digest goal/table_gain/tsp: missing from the rerun" in failures
    assert notes == []


def test_the_file_round_trips_exactly_and_stays_small(root):
    reference = build_reference(write_results(root / "ap_test"))
    path = root / REFERENCE_NAME
    write_reference(reference, path)
    assert load_reference(path) == json.loads(json.dumps(reference))
    assert '"digest_columns": ["gain", "cost", "node_id", "num_nodes"]' in path.read_text()
    assert dumps_reference(reference).count("\n") < 100


def test_a_newer_format_is_refused(root):
    path = root / REFERENCE_NAME
    path.write_text('{"format_version": 99}')
    with pytest.raises(ValueError, match="format_version"):
        load_reference(path)


def test_cli_writes_then_checks_a_reference(root, capsys):
    dataset = write_results(root / "ap_test")
    assert main([str(dataset)]) == 0
    assert (dataset / REFERENCE_NAME).is_file()
    assert main([str(dataset), "--check"]) == 0
    assert "ap_test: exact" in capsys.readouterr().out

    out_dir = root / "elsewhere"
    assert main([str(dataset), "--out-dir", str(out_dir)]) == 0
    assert (out_dir / "ap_test" / REFERENCE_NAME).is_file()


def test_writing_without_the_plotting_stack_fails_instead_of_storing_no_ranges(root, capsys):
    dataset = write_results(root / "ap_test")
    missing = ImportError("No module named 'seaborn'")
    with mock.patch.object(benchmark_reference, "range_table", side_effect=missing):
        with pytest.raises(ImportError):
            build_reference(dataset)
        assert main([str(dataset)]) == 1
    out = capsys.readouterr().out.strip().splitlines()
    assert len(out) == 1 and out[0].startswith("ERROR: {}: writing".format(dataset))
    assert "seaborn" in out[0]
    assert not (dataset / REFERENCE_NAME).exists()


def test_checking_without_the_plotting_stack_still_compares_the_digests(root, capsys):
    dataset = write_results(root / "ap_test")
    assert main([str(dataset)]) == 0
    capsys.readouterr()
    missing = ImportError("No module named 'seaborn'")
    with mock.patch.object(benchmark_reference, "range_table", side_effect=missing):
        assert main([str(dataset), "--check"]) == 0
    assert "ap_test: exact" in capsys.readouterr().out


def test_a_dataset_name_reads_the_results_root_and_writes_the_shipped_reference(root, capsys):
    results_root, data_root = root / "results", root / "data"
    results = write_results(results_root / "ap_test")
    (data_root / "ap_test").mkdir(parents=True)
    with mock.patch.object(benchmark_reference, "RESULTS_ROOT", results_root), mock.patch.object(
        benchmark_reference, "INPUT_ROOT", data_root
    ):
        assert benchmark_reference.resolve_dataset("ap_test") == (
            results.resolve(),
            (data_root / "ap_test").resolve(),
        )
        assert main(["ap_test"]) == 0
        assert main(["ap_test", "--check"]) == 0
    assert (data_root / "ap_test" / REFERENCE_NAME).is_file()
    assert not (results / REFERENCE_NAME).exists()
    assert "ap_test: exact" in capsys.readouterr().out


def test_cli_reports_a_dataset_without_results(root, capsys):
    (root / "ap_empty" / "graph_0").mkdir(parents=True)
    assert main([str(root / "ap_empty")]) == 1
    assert "no result files" in capsys.readouterr().out


def test_cli_check_reports_a_partial_results_tree_in_one_line(root, capsys):
    dataset = write_results(root / "ap_test")
    assert main([str(dataset)]) == 0
    capsys.readouterr()
    # A single-mode run (e.g. the Quick Start's everystep/expected_gain) leaves figure cells empty.
    for graph in ("graph_0", "graph_1"):
        shutil.rmtree(dataset / graph / "goal")
    assert main([str(dataset), "--check"]) == 1
    out = capsys.readouterr().out.strip().splitlines()
    assert len(out) == 1
    assert out[0].startswith("ERROR: {}: no benchmark results in".format(dataset))


def test_cli_check_reports_a_missing_reference_in_one_line(root, capsys):
    dataset = write_results(root / "ap_test")
    assert main([str(dataset), "--check"]) == 1
    out = capsys.readouterr().out.strip().splitlines()
    assert len(out) == 1 and out[0].startswith("ERROR: ") and REFERENCE_NAME in out[0]
