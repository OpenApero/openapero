"""Import safety of the Habitat node module.

The node imports habitat_sim only when it builds the simulator, and talks to ROS only from
``HabitatRenderer``/``main()``, so importing the module must succeed in any interpreter with the
workspace sourced, with or without habitat_sim.
"""

import importlib
import os
import re
from pathlib import Path

# Every launcher in nodes/ runs the main() of the module it is named after.
NODE_MODULES = {"habitat_renderer_node": "open_apero_habitat.habitat_renderer"}


def test_habitat_node_imports():
    module = importlib.import_module("open_apero_habitat.habitat_renderer")
    assert callable(module.main)


def test_every_node_launches_its_module():
    nodes = Path(__file__).resolve().parents[1] / "nodes"
    cmake = (nodes.parent / "CMakeLists.txt").read_text()
    installed = (
        re.search(r"catkin_install_python\(PROGRAMS(.*?)DESTINATION", cmake, re.S).group(1).split()
    )
    assert sorted(path.name for path in nodes.iterdir()) == sorted(NODE_MODULES)
    assert sorted(installed) == sorted("nodes/{}".format(node) for node in NODE_MODULES)
    for node, module in NODE_MODULES.items():
        launcher = nodes / node
        assert os.access(str(launcher), os.X_OK), node
        assert "from {} import main\n".format(module) in launcher.read_text()
