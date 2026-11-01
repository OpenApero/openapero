"""Where the graph benchmark reads its inputs and writes its outputs."""

import os
import subprocess
import sys

from open_apero_graph_benchmark.paths import INPUT_ROOT, PACKAGE_ROOT

PRINT_ROOTS = (
    "from open_apero_graph_benchmark.paths import RESULTS_ROOT, GENERATED_ROOT;"
    "print(RESULTS_ROOT); print(GENERATED_ROOT)"
)


def roots_under(environment):
    """RESULTS_ROOT and GENERATED_ROOT as computed by a fresh interpreter under ``environment``."""
    completed = subprocess.run(
        [sys.executable, "-c", PRINT_ROOTS],
        stdout=subprocess.PIPE,
        universal_newlines=True,
        env=environment,
        check=True,
    )
    return completed.stdout.split()


def test_outputs_follow_the_data_root(tmp_path):
    environment = dict(os.environ, OPEN_APERO_DATA_ROOT=str(tmp_path))
    assert roots_under(environment) == [
        str(tmp_path / "graph_benchmark"),
        str(tmp_path / "graph_benchmark" / "generated"),
    ]


def test_without_a_data_root_outputs_go_below_home(tmp_path):
    environment = {k: v for k, v in os.environ.items() if k != "OPEN_APERO_DATA_ROOT"}
    environment["HOME"] = str(tmp_path)
    assert roots_under(environment) == [
        str(tmp_path / "open_apero_data" / "graph_benchmark"),
        str(tmp_path / "open_apero_data" / "graph_benchmark" / "generated"),
    ]


def test_inputs_are_the_package_s_tracked_data():
    assert (PACKAGE_ROOT / "package.xml").is_file()
    assert INPUT_ROOT == PACKAGE_ROOT / "data"
    assert (INPUT_ROOT / "graph_topology.txt").is_file()
    assert (INPUT_ROOT / "gain_model.txt").is_file()
    datasets = sorted(path.name for path in INPUT_ROOT.iterdir() if path.is_dir())
    assert len(datasets) == 8
    for name in datasets:
        assert name.startswith(("ap_", "op_")), name
        assert (INPUT_ROOT / name / "reference.json").is_file(), name
        assert len(list((INPUT_ROOT / name).glob("graph_*/graph.txt"))) == 5, name
