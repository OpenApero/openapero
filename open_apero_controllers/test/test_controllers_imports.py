"""Import safety of the controller node modules.

The nodes talk to ROS only from ``__init__``/``main()``, so importing the
modules must succeed in any interpreter with the workspace sourced.
"""

import importlib
import os
import re
from pathlib import Path

# Every launcher in nodes/ runs the main() of the module it is named after.
NODE_MODULES = {
    "dummy_controller_node": "open_apero_controllers.dummy_controller",
    "keyboard_teleop_node": "open_apero_controllers.keyboard_teleop",
}


def test_dummy_controller_imports():
    module = importlib.import_module("open_apero_controllers.dummy_controller")
    assert callable(module.main)


def test_keyboard_teleop_imports():
    module = importlib.import_module("open_apero_controllers.keyboard_teleop")
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
