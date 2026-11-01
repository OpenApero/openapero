"""Python side of the Open Apero ROS package.

:mod:`.mission_coordinator` is the node implementation; its ``nodes/`` entry point imports it
explicitly, never this ``__init__``, because it needs rospy at import time. :mod:`.episode_summary`
is the stdlib reader the sweep tools in ``scripts/`` share. The experiment analysis helpers live in
:mod:`open_apero_analysis`.
"""
