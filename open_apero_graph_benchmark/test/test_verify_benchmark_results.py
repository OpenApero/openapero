"""The reproduction checker's dataset derivation, scratch tree and ignore rules.

The derivation tests are what stop a dataset from being re-run under the wrong perception
model: an ``ap_`` dataset run at 5 m silently produces online-perception results in a folder
whose name claims a-priori visibility, and the comparison would then fail for a reason that
has nothing to do with the code under test.
"""

import contextlib
import io
import math
import shutil
import socket
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ElementTree
from pathlib import Path
from unittest import mock

import pytest
from _result_trees import write_results

from open_apero_graph_benchmark import verify_benchmark_results
from open_apero_graph_benchmark.benchmark_reference import (
    build_reference,
    load_reference,
    write_reference,
)
from open_apero_graph_benchmark.compare_result_trees import compare_trees, find_result_files
from open_apero_graph_benchmark.verify_benchmark_results import (
    AP_PERCEPTION_RADIUS,
    INPUT_ROOT,
    MODULE_NAME,
    all_dataset_names,
    build_roslaunch_command,
    create_scratch_dataset,
    derive_cost_budget,
    derive_dataset_parameters,
    derive_perception_radius,
    largest_range_difference,
    main,
    require_free_master_port,
    require_reference,
    wait_for_node_then_stop,
)

ROWS = "10 1.5 0.001 3 40\n20 2.5 0.002 7 40\n"


def test_online_perception_radius_is_the_name_s_radius_exactly():
    assert derive_perception_radius("op_5m_25x25_clustered_gain_0to100_cost_50") == 5.0
    assert derive_perception_radius("op_5m_50x50_scattered_gain_0to100_cost_100") == 5.0


def test_a_priori_radius_is_the_launch_value_and_spans_the_named_extent():
    small = derive_perception_radius("ap_25x25_clustered_gain_0to100_cost_50")
    large = derive_perception_radius("ap_50x50_scattered_gain_0to100_cost_100")
    assert small == large == AP_PERCEPTION_RADIUS
    # benchmark_node warns below the diagonal.
    assert small >= math.hypot(25.0, 25.0)
    assert large >= math.hypot(50.0, 50.0)


def test_cost_budget_comes_from_the_name_suffix():
    assert derive_cost_budget("ap_25x25_clustered_gain_0to100_cost_50") == 50.0
    assert derive_cost_budget("op_5m_50x50_clustered_gain_0to100_cost_100") == 100.0


def test_both_parameters_are_derived_together():
    radius, cost_budget = derive_dataset_parameters("op_5m_25x25_scattered_gain_0to100_cost_50")
    assert (radius, cost_budget) == (5.0, 50.0)


def test_unrecognized_dataset_names_are_rejected():
    for name in ("clustered_gain_0to100_cost_50", "ap_clustered_cost_50", "op_25x25_cost_50"):
        with pytest.raises(ValueError):
            derive_perception_radius(name)


def test_a_dataset_without_a_reference_is_refused_before_running():
    with tempfile.TemporaryDirectory() as root:
        dataset = Path(root) / "ap_25x25_clustered_gain_0to100_cost_50"
        (dataset / "graph_0").mkdir(parents=True)
        (dataset / "graph_0" / "graph.txt").write_text("0 0.0 0.0 0.0 1\n")
        with pytest.raises(RuntimeError, match="no reference.json"):
            require_reference(dataset)
        output = io.StringIO()
        with mock.patch.object(
            verify_benchmark_results.subprocess, "Popen", side_effect=AssertionError("spawned")
        ):
            with contextlib.redirect_stdout(output):
                status = main([str(dataset), "--work-dir", str(Path(root) / "work")])
        assert status == 1
        assert "no reference.json" in output.getvalue()
        assert "benchmark_reference.py" in output.getvalue()

        (dataset / "reference.json").write_text('{"format_version": 1, "result_files": 1}')
        require_reference(dataset)


def _fresh_clone(root):
    """A dataset with the paper-style results, and its fresh-clone copy: graph.txt + reference."""
    results = write_results(root / "results" / "ap_25x25_scattered_gain_0to100_cost_50")
    fresh = root / "fresh" / results.name
    for graph in ("graph_0", "graph_1"):
        (fresh / graph).mkdir(parents=True)
        shutil.copyfile(results / graph / "graph.txt", fresh / graph / "graph.txt")
    write_reference(build_reference(results), fresh / "reference.json")
    return results, fresh


def _verify_with_rerun(results, fresh, work_dir, drop=()):
    """Run main() on the fresh tree, with a benchmark stand-in that writes the stored results.

    ``drop`` lists result folders, relative to each graph folder, the stand-in leaves out.
    """

    def rerun(scratch, *_args):
        for graph in ("graph_0", "graph_1"):
            shutil.copytree(str(results / graph), str(scratch / graph), dirs_exist_ok=True)
            for folder in drop:
                shutil.rmtree(str(scratch / graph / folder))
        return 1.0

    output = io.StringIO()
    with mock.patch.object(verify_benchmark_results, "run_benchmark", side_effect=rerun):
        with mock.patch.object(verify_benchmark_results, "require_free_master_port"):
            with contextlib.redirect_stdout(output):
                status = main([str(fresh), "--work-dir", str(work_dir)])
    return status, output.getvalue()


def test_a_fresh_clone_rerun_matching_the_digests_is_reproduced_exactly():
    with tempfile.TemporaryDirectory() as directory:
        results, fresh = _fresh_clone(Path(directory))
        status, output = _verify_with_rerun(results, fresh, Path(directory) / "work")
        assert status == 0
        assert "REPRODUCED (exact)" in output
        assert "result files compared: 48" in output
        report = (Path(directory) / "work" / fresh.name / "report.md").read_text()
        assert "every digest matches reference.json" in report


def test_a_fresh_clone_rerun_with_a_digest_difference_is_reproduced_at_figure_level():
    with tempfile.TemporaryDirectory() as directory:
        results, fresh = _fresh_clone(Path(directory))
        reference = load_reference(fresh / "reference.json")
        reference["digests"]["everystep/table_gain/tsp"] = "0" * 32
        write_reference(reference, fresh / "reference.json")
        status, output = _verify_with_rerun(results, fresh, Path(directory) / "work")
        assert status == 0
        assert "REPRODUCED (figure-level)" in output
        assert "NOTE: digest everystep/table_gain/tsp: differs" in output


def test_a_fresh_clone_rerun_with_a_different_figure_cell_is_a_mismatch():
    with tempfile.TemporaryDirectory() as directory:
        results, fresh = _fresh_clone(Path(directory))
        reference = load_reference(fresh / "reference.json")
        reference["digests"]["goal/path_gain/nbs"] = "0" * 32
        reference["final_gain_ranges"]["goal/path_gain/nbs"][0] -= 1.0
        write_reference(reference, fresh / "reference.json")
        status, output = _verify_with_rerun(results, fresh, Path(directory) / "work")
        assert status == 1
        assert "MISMATCH" in output
        assert "FAIL: goal/path_gain/nbs: reference" in output


def test_a_fresh_clone_rerun_missing_whole_groups_is_a_mismatch():
    with tempfile.TemporaryDirectory() as directory:
        results, fresh = _fresh_clone(Path(directory))
        drop = ("none", "goal/table_gain", "everystep/table_gain")
        status, output = _verify_with_rerun(results, fresh, Path(directory) / "work", drop)
        assert status == 1
        assert "=== {} : MISMATCH ===".format(fresh.name) in output
        assert "FAIL: result files: reference 48, rerun 24" in output
        assert "FAIL: digest none/path_gain/nbs: missing from the rerun" in output


def test_scratch_tree_copies_only_the_graph_files():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        source = root / "ap_25x25_clustered_gain_0to100_cost_50"
        (source / "graph_0" / "none" / "path_gain").mkdir(parents=True)
        (source / "graph_0" / "graph.txt").write_text("0 0 0 0 1\n")
        (source / "graph_0" / "graph.pdf").write_bytes(b"%PDF")
        (source / "graph_0" / "none" / "path_gain" / "dbs_1.txt").write_text(ROWS)
        (source / "graph_1").mkdir()
        (source / "graph_1" / "graph.txt").write_text("0 0 0 0 1\n")
        (source / "reference.json").write_text("{}")

        scratch = root / "scratch" / source.name
        instances = create_scratch_dataset(source, scratch)

        assert instances == ["graph_0", "graph_1"]
        assert sorted(path.name for path in scratch.iterdir()) == ["graph_0", "graph_1"]
        assert [path.name for path in (scratch / "graph_0").iterdir()] == ["graph.txt"]
        assert find_result_files(scratch) == []


def test_scratch_tree_refresh_drops_an_earlier_run_s_results():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        source = root / "ap_25x25_clustered_gain_0to100_cost_50"
        (source / "graph_0").mkdir(parents=True)
        (source / "graph_0" / "graph.txt").write_text("0 0 0 0 1\n")

        scratch = root / "scratch" / source.name
        create_scratch_dataset(source, scratch)
        stale = scratch / "graph_0" / "none" / "path_gain"
        stale.mkdir(parents=True)
        (stale / "dbs_1.txt").write_text(ROWS)

        create_scratch_dataset(source, scratch)
        assert find_result_files(scratch) == []


def test_non_result_files_are_ignored_by_the_comparison():
    """Non-result files in the baseline (other method names, .tmp leftovers) must not count."""
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        baseline = root / "baseline" / "graph_0" / "none" / "path_gain"
        candidate = root / "candidate" / "graph_0" / "none" / "path_gain"
        baseline.mkdir(parents=True)
        candidate.mkdir(parents=True)

        (baseline / "dbs_1.txt").write_text(ROWS)
        (candidate / "dbs_1.txt").write_text(ROWS)
        # Present only in the baseline, and none of it is the benchmark's own output.
        (baseline / "dbsx_1.txt").write_text(ROWS)
        (baseline / "nbsx_10.txt").write_text(ROWS)
        (baseline / "dbs_100.txt.tmp.6vGVeF").write_text(ROWS)
        (root / "baseline" / "graph_0" / "graph.txt").write_text("0 0 0 0 1\n")

        failures, _ = compare_trees(root / "baseline", root / "candidate")
        assert failures == []


def test_launch_command_requests_every_stored_combination():
    command = build_roslaunch_command(Path("/scratch/ds"), 70.71, 100.0, 0, "info")
    assert "replan_type:=try_every_option" in command
    assert "objective:=try_every_option" in command
    assert "random_seed:=0" in command
    assert "visualize:=false" in command
    assert "graph_type_folder:=/scratch/ds" in command
    # The scratch tree is also where the results go, never the inputs' data/.
    assert "results_folder:=/scratch/ds" in command


def test_dry_run_with_detach_prints_the_child_commands_and_spawns_nothing():
    with tempfile.TemporaryDirectory() as root:
        dataset = Path(root) / "ap_25x25_clustered_gain_0to100_cost_50"
        (dataset / "graph_0").mkdir(parents=True)
        (dataset / "graph_0" / "graph.txt").write_text("0 0.0 0.0 0.0 1\n1 1.0 0.0 0.0 0\n")
        work_dir = Path(root) / "work"
        output = io.StringIO()
        with mock.patch.object(
            verify_benchmark_results.subprocess, "Popen", side_effect=AssertionError("spawned")
        ):
            with contextlib.redirect_stdout(output):
                status = main(
                    [
                        str(dataset),
                        "--detach",
                        "--dry-run",
                        "--work-dir",
                        str(work_dir),
                        "--master-port-base",
                        "11999",
                    ]
                )
        assert status == 0
        assert "would detach" in output.getvalue()
        assert "-m {} ".format(MODULE_NAME) in output.getvalue()
        assert "--master-port-base 11999" in output.getvalue()
        assert not (work_dir / dataset.name / "verify.log").exists()


def test_detach_starts_the_module_in_its_own_session_and_says_how_to_check_later():
    with tempfile.TemporaryDirectory() as root:
        dataset = Path(root) / "ap_25x25_clustered_gain_0to100_cost_50"
        (dataset / "graph_0").mkdir(parents=True)
        (dataset / "graph_0" / "graph.txt").write_text("0 0.0 0.0 0.0 1\n1 1.0 0.0 0.0 0\n")
        (dataset / "reference.json").write_text('{"format_version": 1, "result_files": 1}')
        work_dir = Path(root) / "work"
        output = io.StringIO()
        with mock.patch.object(
            verify_benchmark_results.subprocess, "Popen", return_value=mock.Mock(pid=4242)
        ) as popen, mock.patch.object(verify_benchmark_results, "require_free_master_port"):
            with contextlib.redirect_stdout(output):
                status = main([str(dataset), "--detach", "--work-dir", str(work_dir)])
        assert status == 0
        command = popen.call_args[0][0]
        assert command[:4] == [sys.executable, "-m", MODULE_NAME, str(dataset)]
        assert "--detach" not in command
        assert popen.call_args[1]["start_new_session"] is True
        assert (work_dir / dataset.name / "verify.log").is_file()
        assert "pid 4242" in output.getvalue()
        assert (
            "rosrun open_apero_graph_benchmark verify_benchmark_results.py {} --work-dir {} "
            "--skip-run".format(dataset.name, work_dir.resolve())
        ) in output.getvalue()


def test_a_detached_child_can_start_the_module():
    completed = subprocess.run(
        [sys.executable, "-m", MODULE_NAME, "--help"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        universal_newlines=True,
    )
    assert completed.returncode == 0, completed.stdout
    assert "--skip-run" in completed.stdout


def test_a_busy_master_port_is_refused_and_a_free_one_accepted():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
        listener.bind(("127.0.0.1", 0))
        listener.listen(1)
        port = listener.getsockname()[1]
        try:
            require_free_master_port(port)
        except RuntimeError as error:
            assert str(port) in str(error)
        else:
            raise AssertionError("a port with a listener was accepted")
    require_free_master_port(port)


LAUNCH = Path(__file__).resolve().parents[1] / "launch" / "run_benchmark.launch"


def _launch_default(arguments, name):
    """Evaluate one ``$(eval ...)`` arg of run_benchmark.launch the way roslaunch does."""
    tags = {tag.get("name"): tag for tag in ElementTree.parse(LAUNCH).getroot().findall("arg")}
    expression = tags[name].get("default") or tags[name].get("value")
    assert expression.startswith("$(eval ") and expression.endswith(")")
    namespace = {"__builtins__": {"float": float, "str": str}, "arg": arguments.__getitem__}
    return eval(expression[len("$(eval ") : -1], namespace)


def test_launch_defaults_follow_the_dataset_name_like_the_verifier():
    for name in all_dataset_names():
        arguments = {"graph_type_folder": str(INPUT_ROOT / name) + "/"}
        arguments["dataset_name"] = _launch_default(arguments, "dataset_name")
        assert arguments["dataset_name"] == name
        assert _launch_default(arguments, "cost_budget") == derive_cost_budget(name)
        assert _launch_default(arguments, "perception_radius") == derive_perception_radius(name)


def test_launch_defaults_fall_back_for_a_name_without_the_encoding():
    arguments = {"dataset_name": "my_graphs"}
    assert _launch_default(arguments, "cost_budget") == 100.0
    assert _launch_default(arguments, "perception_radius") == 100.0


# Names whose radius or cost token is not a number; roslaunch evaluates a default even when the
# arg is passed, so these must fall back instead of raising.
NON_NUMERIC_NAMES = (
    "op_custom",
    "mygraph_cost_100_v2",
    "ap_25x25_clustered_gain_0to100_cost_50_copy",
)


def test_launch_defaults_fall_back_for_a_non_numeric_token():
    for name in NON_NUMERIC_NAMES:
        arguments = {"dataset_name": name}
        assert _launch_default(arguments, "cost_budget") == 100.0, name
        assert _launch_default(arguments, "perception_radius") == 100.0, name


def _roslaunch_params(argv):
    """Load run_benchmark.launch with roslaunch's own loader; return the benchmark_node params."""
    xmlloader = pytest.importorskip("roslaunch.xmlloader")
    config_module = pytest.importorskip("roslaunch.config")
    rospkg = pytest.importorskip("rospkg")
    config = config_module.ROSLaunchConfig()
    try:
        xmlloader.XmlLoader().load(str(LAUNCH), config, argv=argv, verbose=False)
    except rospkg.ResourceNotFound as error:
        pytest.skip("open_apero_graph_benchmark is not on ROS_PACKAGE_PATH: {}".format(error))
    prefix = "/benchmark_node/"
    return {
        key[len(prefix) :]: param.value
        for key, param in config.params.items()
        if key.startswith(prefix)
    }


def test_roslaunch_loads_a_non_numeric_name_and_explicit_values_win():
    for name in NON_NUMERIC_NAMES:
        folder = "graph_type_folder:=/scratch/" + name
        params = _roslaunch_params(["roslaunch", folder])
        assert (params["perception_radius"], params["cost_budget"]) == (100.0, 100.0), name
        params = _roslaunch_params(
            ["roslaunch", folder, "perception_radius:=5.0", "cost_budget:=50"]
        )
        assert (params["perception_radius"], params["cost_budget"]) == (5.0, 50), name


def test_benchmark_node_is_required_so_the_launch_ends_with_the_sweep():
    nodes = ElementTree.parse(LAUNCH).getroot().findall("node")
    assert [node.get("required") for node in nodes if node.get("name") == "benchmark_node"] == [
        "true"
    ]


def test_a_launch_that_exits_by_itself_is_not_interrupted():
    process = subprocess.Popen(["sh", "-c", "exit 0"], start_new_session=True)
    with mock.patch.object(verify_benchmark_results, "POLL_SECONDS", 0.01), mock.patch.object(
        verify_benchmark_results, "_stop_process_group", side_effect=AssertionError("interrupted")
    ):
        assert wait_for_node_then_stop(process) == 0


def test_largest_range_difference_is_a_float_and_inf_for_a_missing_cell():
    baseline = {("goal", "path_gain"): {"nbs": (1.0, 5.0), "tsp": (2.0, 4.0)}}
    candidate = {("goal", "path_gain"): {"nbs": (1.5, 5.0), "tsp": (2.0, 3.0)}}
    assert largest_range_difference(baseline, candidate) == 1.0
    missing_algorithm = {("goal", "path_gain"): {"nbs": (1.0, 5.0)}}
    assert largest_range_difference(baseline, missing_algorithm) == float("inf")
    assert largest_range_difference(baseline, {}) == float("inf")
    assert "{:.6g}".format(largest_range_difference(baseline, {})) == "inf"
