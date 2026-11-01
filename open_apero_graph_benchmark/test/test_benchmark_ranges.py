"""Graph-benchmark range-result loading and grouping."""

import tempfile
from collections import defaultdict
from pathlib import Path

import pytest

from open_apero_graph_benchmark.benchmark_ranges import (
    ALGORITHM_LABELS,
    ALGORITHM_ORDER,
    GROUPING_PATTERN as PATTERN,
    graph_result_folders,
    load_and_group_files,
    process_files,
)


def test_grouping_preserves_algorithm_order_and_sorts_parameters():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        for name in ("tsp_1.txt", "nbs_10.txt", "nbs_2.txt", "ignored.csv"):
            (root / name).touch()
        grouped = load_and_group_files(root, PATTERN)
        assert list(grouped) == ["nbs", "tsp"]
        assert grouped["nbs"] == [(2.0, "nbs_2.txt"), (10.0, "nbs_10.txt")]


def test_every_ordered_algorithm_is_styleable_and_labeled():
    """Every algorithm emitted by the grouping layer has a display label."""
    for algorithm in ALGORITHM_ORDER:
        assert algorithm in ALGORITHM_LABELS


def test_process_files_loads_five_column_format():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        (root / "nbs_2.txt").write_text("1 2 3 4 5\n6 7 8 9 10\n")
        grouped = load_and_group_files(root, PATTERN)
        data = defaultdict(lambda: defaultdict(dict))
        process_files(data, grouped, root, "graph_0")
        run = data["nbs"][2.0]["graph_0"]
        assert list(run["gain"]) == [1.0, 6.0]
        assert run["node_id"] == ["4", "9"]


def test_process_files_rejects_malformed_shape():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        (root / "nbs_2.txt").write_text("1 2 3 4\n5 6 7 8\n")
        grouped = load_and_group_files(root, PATTERN)
        data = defaultdict(lambda: defaultdict(dict))
        with pytest.raises(ValueError, match="expected 2-D with >=5 columns"):
            process_files(data, grouped, root, "graph_0")


def test_graph_folders_are_sorted_and_hold_results():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        for folder in ("graph_1/goal", "graph_0/everystep", "figures", "graph_2"):
            (root / folder).mkdir(parents=True)
        assert graph_result_folders(root, ["goal", "everystep"]) == ["graph_0", "graph_1"]
