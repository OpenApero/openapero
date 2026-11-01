"""generate_graph_instance.launch and its node: defaults below the data root, the instance written.

The node tests start roslaunch on a free port, so they need a built workspace; they skip without
roslaunch or the node binary.
"""

import os
import shutil
import socket
import subprocess
from pathlib import Path

import pytest

from open_apero_graph_benchmark.paths import INPUT_ROOT

LAUNCH = Path(__file__).resolve().parents[1] / "launch" / "generate_graph_instance.launch"

NODE_NAME = "generate_graph_instance_node"

# Nodes 0-2 form a path from the start; node 3 is unreachable and must be dropped.
TOPOLOGY = "0 0.0 0.0 0.0 1\n1 1.5 0.0 0.0 0 2\n2 3.0 0.0 0.0 1\n3 9.0 9.0 0.0\n"
# The all-zero region is the fallback: every node draws an integer gain in [40, 60].
GAIN_MODEL = "0 0 0 40 60\n"


def _launch_config(argv):
    xmlloader = pytest.importorskip("roslaunch.xmlloader")
    config_module = pytest.importorskip("roslaunch.config")
    rospkg = pytest.importorskip("rospkg")
    config = config_module.ROSLaunchConfig()
    try:
        xmlloader.XmlLoader().load(str(LAUNCH), config, argv=["roslaunch"] + argv, verbose=False)
    except rospkg.ResourceNotFound as error:
        pytest.skip("open_apero_graph_benchmark is not on ROS_PACKAGE_PATH: {}".format(error))
    return config


def _node_params(config):
    prefix = "/{}/".format(NODE_NAME)
    return {
        key[len(prefix) :]: param.value
        for key, param in config.params.items()
        if key.startswith(prefix)
    }


def test_defaults_read_the_tracked_inputs_and_write_below_the_data_root(tmp_path, monkeypatch):
    monkeypatch.setenv("OPEN_APERO_DATA_ROOT", str(tmp_path))
    params = _node_params(_launch_config([]))
    assert Path(params["topology_file"]).resolve() == (INPUT_ROOT / "graph_topology.txt").resolve()
    assert Path(params["gain_model_file"]).resolve() == (INPUT_ROOT / "gain_model.txt").resolve()
    assert Path(params["topology_file"]).is_file() and Path(params["gain_model_file"]).is_file()
    assert params["output_folder"] == str(tmp_path / "graph_benchmark" / "generated")
    assert params["random_seed"] == -1


def test_without_a_data_root_the_output_goes_below_home(monkeypatch):
    monkeypatch.delenv("OPEN_APERO_DATA_ROOT", raising=False)
    params = _node_params(_launch_config([]))
    expected = Path(os.environ["HOME"]) / "open_apero_data" / "graph_benchmark" / "generated"
    assert params["output_folder"] == str(expected)


def test_explicit_arguments_reach_the_node(tmp_path):
    params = _node_params(
        _launch_config(
            [
                "topology_file:=/in/topology.txt",
                "gain_model_file:=/in/gains.txt",
                "output_folder:={}".format(tmp_path),
                "random_seed:=7",
            ]
        )
    )
    assert params == {
        "topology_file": "/in/topology.txt",
        "gain_model_file": "/in/gains.txt",
        "output_folder": str(tmp_path),
        "random_seed": 7,
    }


def test_the_node_is_required_so_the_launch_ends_with_it():
    nodes = _launch_config([]).nodes
    assert [(node.name, node.required) for node in nodes] == [(NODE_NAME, True)]


def _free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


@pytest.fixture
def launch(tmp_path):
    """Run the launch file under its own master; return its combined output."""
    if shutil.which("roslaunch") is None:
        pytest.skip("roslaunch is not on PATH")
    roslib_packages = pytest.importorskip("roslib.packages")
    try:
        if not roslib_packages.find_node("open_apero_graph_benchmark", NODE_NAME):
            pytest.skip("{} is not built".format(NODE_NAME))
    except Exception as error:  # noqa: BLE001 - rospkg raises several kinds when not set up
        pytest.skip("cannot look up {}: {}".format(NODE_NAME, error))

    def run(*arguments):
        port = _free_port()
        environment = dict(os.environ)
        environment["ROS_MASTER_URI"] = "http://localhost:{}".format(port)
        environment["ROS_HOME"] = str(tmp_path / "ros_home")
        completed = subprocess.run(
            ["roslaunch", "-p", str(port), str(LAUNCH)] + list(arguments),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            env=environment,
            timeout=120,
        )
        return completed.stdout

    return run


def _read_instance(path):
    nodes = {}
    for line in path.read_text().splitlines():
        values = line.split()
        nodes[int(values[0])] = (
            float(values[1]),
            float(values[2]),
            float(values[3]),
            [int(value) for value in values[4:]],
        )
    return nodes


def test_the_node_samples_reachable_nodes_into_numbered_instances(tmp_path, launch):
    (tmp_path / "topology.txt").write_text(TOPOLOGY)
    (tmp_path / "gains.txt").write_text(GAIN_MODEL)
    output = tmp_path / "out" / "generated"
    arguments = (
        "topology_file:={}".format(tmp_path / "topology.txt"),
        "gain_model_file:={}".format(tmp_path / "gains.txt"),
        "output_folder:={}".format(output),
        "random_seed:=0",
    )

    first = launch(*arguments)
    assert "Wrote new graph instance to {}".format(output / "graph_0" / "graph.txt") in first
    second = launch(*arguments)
    assert "Wrote new graph instance to {}".format(output / "graph_1" / "graph.txt") in second

    instance = _read_instance(output / "graph_0" / "graph.txt")
    assert sorted(instance) == [0, 1, 2]
    assert [(x, y) for x, y, _, _ in instance.values()] == [(0.0, 0.0), (1.5, 0.0), (3.0, 0.0)]
    assert [neighbors for _, _, _, neighbors in instance.values()] == [[1], [0, 2], [1]]
    for _, _, gain, _ in instance.values():
        assert 40 <= gain <= 60 and gain == int(gain)
    # The same seed draws the same gains.
    assert (output / "graph_1" / "graph.txt").read_text() == (
        output / "graph_0" / "graph.txt"
    ).read_text()
    assert (tmp_path / "topology.txt").read_text() == TOPOLOGY


def test_a_missing_input_is_fatal_and_writes_nothing(tmp_path, launch):
    output = tmp_path / "generated"
    printed = launch(
        "topology_file:={}".format(tmp_path / "missing.txt"),
        "output_folder:={}".format(output),
    )
    assert "Cannot generate a graph instance in {}".format(output) in printed
    assert not output.exists()
