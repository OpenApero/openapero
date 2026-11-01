"""The open_apero_ros package, its hand-run tools and its node launcher must be safe to import."""

import importlib
import os
import re
import runpy
import subprocess
import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = PACKAGE_ROOT / "scripts"
NODES = PACKAGE_ROOT / "nodes"

PACKAGE = "open_apero_ros"

MODULES = (
    "open_apero_ros.episode_summary",
    "open_apero_ros.mission_coordinator",
    "open_apero_ros.paper_protocol",
)

# Every Python tool in scripts/ is the entry point of the package module of the same name.
TOOLS = (
    "compare_paper_sweep",
    "evaluate_experiment",
    "generate_points",
    "paper_sweep",
    "plot_normalized_gain",
    "plot_performance_curves",
)

# Every launcher in nodes/ runs the main() of the package module it is named after.
NODE_MODULES = {"mission_coordinator_node": "mission_coordinator"}


def _installed_programs():
    cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text()
    return (
        re.search(r"catkin_install_python\(PROGRAMS(.*?)DESTINATION", cmake, re.S).group(1).split()
    )


def test_package_modules_are_import_safe():
    for module_name in MODULES + tuple("{}.{}".format(PACKAGE, tool) for tool in TOOLS):
        importlib.import_module(module_name)


def test_every_python_tool_is_listed():
    found = sorted(path.stem for path in SCRIPTS.glob("*.py"))
    assert found == sorted(TOOLS)


def test_every_tool_and_node_is_installed():
    expected = ["scripts/{}.py".format(tool) for tool in TOOLS]
    expected += ["nodes/{}".format(node) for node in NODE_MODULES]
    assert sorted(_installed_programs()) == sorted(expected)


def test_every_tool_is_the_entry_point_of_its_module():
    for tool in TOOLS:
        script = SCRIPTS / (tool + ".py")
        assert "from {}.{} import main\n".format(PACKAGE, tool) in script.read_text()
        # Runs the imports without the __main__ block.
        assert callable(runpy.run_path(str(script))["main"])


def test_every_node_launches_its_module():
    assert sorted(path.name for path in NODES.iterdir()) == sorted(NODE_MODULES)
    for node, module in NODE_MODULES.items():
        launcher = NODES / node
        assert os.access(str(launcher), os.X_OK), node
        assert "from {}.{} import main\n".format(PACKAGE, module) in launcher.read_text()


def test_every_tool_runs_as_a_program():
    environment = dict(os.environ, MPLBACKEND="Agg")
    for tool in TOOLS:
        script = SCRIPTS / (tool + ".py")
        assert os.access(str(script), os.X_OK), tool
        completed = subprocess.run(
            [sys.executable, str(script), "--help"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True,
            env=environment,
        )
        assert completed.returncode == 0, completed.stderr
        assert completed.stdout.startswith("usage: {}.py".format(tool)), completed.stdout
