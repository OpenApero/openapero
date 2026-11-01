"""The graph-benchmark package and its hand-run tools must be safe to import."""

import importlib
import os
import re
import runpy
import subprocess
import sys
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"

PACKAGE = "open_apero_graph_benchmark"

MODULES = (
    "open_apero_graph_benchmark.benchmark_ranges",
    "open_apero_graph_benchmark.paths",
)

# Every tool in scripts/ is the entry point of the package module of the same name.
TOOLS = (
    "benchmark_reference",
    "compare_result_trees",
    "generate_clustered_gain_model",
    "generate_random_graph_topology",
    "plot_computation_time_ranges",
    "plot_final_gain_ranges",
    "render_graph",
    "verify_benchmark_results",
)


def test_package_modules_are_import_safe():
    for module_name in MODULES + tuple("{}.{}".format(PACKAGE, tool) for tool in TOOLS):
        importlib.import_module(module_name)


def test_every_python_tool_is_listed():
    found = sorted(path.stem for path in SCRIPTS.glob("*.py"))
    assert found == sorted(TOOLS)


def test_every_tool_is_installed():
    cmake = (SCRIPTS.parent / "CMakeLists.txt").read_text()
    installed = re.search(r"catkin_install_python\(PROGRAMS(.*?)DESTINATION", cmake, re.S).group(1)
    assert sorted(installed.split()) == sorted("scripts/{}.py".format(tool) for tool in TOOLS)


def test_every_tool_is_the_entry_point_of_its_module():
    for tool in TOOLS:
        script = SCRIPTS / (tool + ".py")
        assert "from {}.{} import main\n".format(PACKAGE, tool) in script.read_text()
        # Runs the imports without the __main__ block.
        assert callable(runpy.run_path(str(script))["main"])


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
