"""Controller nodes for simulated Open Apero agents.

Each submodule holds one node implementation and is imported explicitly
(e.g. ``from open_apero_controllers import dummy_controller``). Nothing is
re-exported here: every submodule needs rospy and the Open Apero message
types at import time, so an eager import would fail in a ROS-less
interpreter.
"""
