"""The performance_log.csv header the loader tests run over, without shipping any runs.

:func:`current_header` is the header this release writes, built from the C++ metric declarations:
``AgentMetrics`` (open_apero_core agent/agent_metrics.h), ``RosAgentMetrics`` (open_apero_ros
agent/ros_agent_metrics.h), then every layer instance in alphabetical order, prefixed with its
name: ``LayerMetrics`` (plugin/layer/layer_metrics.h) and, for a planning layer,
``PlanningLayerMetrics`` (plugin/layer/planning_layer_metrics.h); a backtracking layer's
backtracker declares no columns of its own.
"""

AGENT_COLUMNS = (
    "TotalVisitedNodes",
    "TotalPathLength",
    "TotalPathDuration",
    "TotalCost",
    "PlanPathTotal",
    "Visualization",
    "ReplanTimeWall",
    "CycleIndex",
    "ElapsedWallTimeSec",
    "CommandsDispatched",
    "FramesMapped",
)

ROS_AGENT_COLUMNS = ("ReplanTimeRos", "RosCallbacks", "RosTimeSec")

LAYER_COLUMNS = ("ExpandTime", "UpdateTime", "PlanPathTime", "TimesSelected")

PLANNING_LAYER_COLUMNS = LAYER_COLUMNS + (
    "GenerateNode",
    "GenerateEdge",
    "EvaluateNode",
    "GeneratorUpdate",
    "EvaluatorUpdate",
    "StartNodeUpdate",
    "PlannedValue",
    "RealAccumulatedGain",
    "NumExpansions",
    "NumNodesAfterExpansion",
    "NumNodesAfterNewStart",
    "NumNodesAtSelection",
)

COLUMNS_BY_LAYER_TYPE = {"backtracking": LAYER_COLUMNS, "planning": PLANNING_LAYER_COLUMNS}

SHIPPED_LAYERS = {"backtracking": "backtracking", "planning": "planning"}


def current_header(layers=SHIPPED_LAYERS):
    """The header this release writes for ``layers`` (instance -> layer type)."""
    columns = list(AGENT_COLUMNS + ROS_AGENT_COLUMNS)
    for instance in sorted(layers):
        columns += ["%s.%s" % (instance, leaf) for leaf in COLUMNS_BY_LAYER_TYPE[layers[instance]]]
    return ",".join(columns)
