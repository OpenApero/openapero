"""The experiment launch file selects the active-perception task from the command line."""

import os
import xml.etree.ElementTree as ElementTree

import yaml

LAUNCH = os.path.join(os.path.dirname(__file__), "..", "launch", "run_experiment_sim.launch")
TASK_EVALUATORS = {
    "explore": "NaiveEvaluator",
    "recon": "SurfaceFrontierEvaluator",
    "points": "PointsEvaluator",
}
# SurfaceFrontierEvaluator keys: config/tasks/recon.yaml sets them, the planner files do not.
RECON_KEYS = {
    "accurate_frontiers": True,
    "checking_distance": 2.0,
    "frontier_voxel_weight": 1.0,
    "new_voxel_weight": 0.0,
}
CONFIGS = os.path.join(os.path.dirname(__file__), "..", "config")
PLANNERS = os.path.join(CONFIGS, "planners")
TASKS = os.path.join(CONFIGS, "tasks")
CONFIG_PREFIX = "$(find open_apero_ros)/config/"
TASK_NS = "/task_evaluator"


def _root():
    return ElementTree.parse(LAUNCH).getroot()


def _args():
    return {tag.get("name"): tag for tag in _root().findall("arg")}


def _planner_node():
    for node in _root().findall("node"):
        if node.get("name") == "open_apero_node":
            return node
    raise AssertionError("no open_apero_node in " + LAUNCH)


def _task_group():
    """The one <group> of the launch file: the task evaluator's namespace."""
    groups = _root().findall("group")
    assert len(groups) == 1, len(groups)
    return groups[0]


def _top_level_loads():
    """<rosparam command="load"> tags at the top level or in the task group, in launch order."""
    tags = []
    for tag in _root():
        candidates = list(tag) if tag.tag == "group" else [tag]
        tags += [t for t in candidates if t.tag == "rosparam" and t.get("command") == "load"]
    return tags


def _config_path(tag):
    """The file a load tag reads, relative to config/."""
    return tag.get("file", "").replace(CONFIG_PREFIX, "", 1)


def _task_loads():
    """The three task loads, keyed by their file relative to config/."""
    loads = {}
    for tag in _top_level_loads():
        name = _config_path(tag)
        if name.startswith("tasks/"):
            loads[name] = tag
    assert sorted(loads) == [
        "tasks/$(arg task).yaml",
        "tasks/_common.yaml",
        "tasks/_points_planning.yaml",
    ], sorted(loads)
    return loads


def _load_yaml(path):
    with open(path) as handle:
        return yaml.safe_load(handle)


def _selectable(directory):
    """The YAML files of a config folder that a launch argument can name: no leading underscore."""
    names = os.listdir(directory)
    return sorted(name for name in names if name.endswith(".yaml") and not name.startswith("_"))


def _planner_configs():
    """Planner config files: every config/planners/*.yaml but the shared base."""
    return _selectable(PLANNERS)


def _types(tree):
    """Every `type` value in a parsed YAML tree."""
    if not isinstance(tree, dict):
        return set()
    found = {tree["type"]} if "type" in tree else set()
    for value in tree.values():
        found |= _types(value)
    return found


def test_task_and_budget_args_default_to_empty():
    args = _args()
    for name in ("task", "cost_budget"):
        assert name in args
        assert args[name].get("default") == ""


def test_the_task_evaluator_group_is_unconditional():
    group = _task_group()
    assert group.get("if") is None and group.get("unless") is None


def test_the_task_evaluator_namespace_is_cleared_before_its_loads():
    group = _task_group()
    assert group.get("ns") == "task_evaluator"
    assert group.get("clear_params") == "true"
    files = [_config_path(child) for child in group if child.tag == "rosparam"]
    assert files == ["tasks/_common.yaml", "tasks/$(arg task).yaml"]
    top_level = [_config_path(tag) for tag in _root().findall("rosparam")]
    assert "tasks/_points_planning.yaml" in top_level


def test_the_base_file_loads_unconditionally_before_the_task_file():
    loads = _task_loads()
    base, task = loads["tasks/_common.yaml"], loads["tasks/$(arg task).yaml"]
    for tag in (base, task):
        assert tag.get("ns") is None
        assert tag.get("subst_value") == "true"
    assert base.get("if") is None and base.get("unless") is None
    order = [tag.get("file") for tag in _top_level_loads()]
    assert order.index(base.get("file")) < order.index(task.get("file"))


def test_the_launch_args_set_the_task_sensor_range_after_the_task_loads():
    children = list(_task_group())
    params = {
        child.get("name"): (index, child.get("value"))
        for index, child in enumerate(children)
        if child.tag == "param"
    }
    last_load = max(index for index, child in enumerate(children) if child.tag == "rosparam")
    for key in ("ray_length", "observation_boundary"):
        assert key in _args(), key
        index, value = params["sensor_model/" + key]
        assert value == "$(arg " + key + ")", key
        assert index > last_load, key


def test_the_task_file_loads_only_with_a_task():
    task = _task_loads()["tasks/$(arg task).yaml"]
    assert task.get("if") == "$(eval arg('task') != '')"


def test_the_points_planning_file_loads_into_the_planning_layer_only_for_points():
    planning = _task_loads()["tasks/_points_planning.yaml"]
    assert planning.get("ns") == "open_apero_node/layers/planning"
    assert planning.get("if") == "$(eval arg('task') == 'points')"


def test_each_task_file_sets_its_evaluator_type():
    assert _selectable(TASKS) == sorted(task + ".yaml" for task in TASK_EVALUATORS)
    for task, evaluator in TASK_EVALUATORS.items():
        assert _load_yaml(os.path.join(TASKS, task + ".yaml"))["type"] == evaluator, task


def test_recon_task_sets_the_surface_frontier_keys():
    recon = _load_yaml(os.path.join(TASKS, "recon.yaml"))
    for key, value in RECON_KEYS.items():
        assert recon[key] == value, key


def test_points_task_sets_the_file_and_the_paper_planning_settings():
    points = _load_yaml(os.path.join(TASKS, "points.yaml"))
    assert points["points_file"] == "$(find open_apero_ros)/data/points_3d.csv"
    planning = _load_yaml(os.path.join(TASKS, "_points_planning.yaml"))
    assert planning == {
        "generator": {"min_edge_length": 1.0, "max_edge_length": 2.0},
        "path_planner": {"is_gain_correlated": True},
    }


def test_the_shared_task_evaluator_is_the_explore_task():
    base = _load_yaml(os.path.join(TASKS, "_common.yaml"))
    assert base["type"] == TASK_EVALUATORS["explore"]
    assert base["bounding_volume_args"] == "/target_bounding_volume"
    assert base["sensor_model"]["type"] == "IterativeRayCaster"


def test_planner_configs_read_the_task_evaluator_from_its_namespace():
    assert _planner_configs()
    for name in _planner_configs():
        planning = _load_yaml(os.path.join(PLANNERS, name))["layers"]["planning"]
        evaluator = planning.get("evaluator")
        if evaluator is None:
            assert planning["evaluator_args"] == TASK_NS, name
        else:
            assert "evaluator_args" not in planning, name
            assert evaluator == {
                "type": "SimpleYawPlanningEvaluator",
                "following_evaluator_args": TASK_NS,
            }, name


def test_no_planner_config_holds_an_inline_task_evaluator_type():
    for name in _planner_configs():
        types = _types(_load_yaml(os.path.join(PLANNERS, name)))
        task_types = {
            t for t in types if t.endswith("Evaluator") and t != "SimpleYawPlanningEvaluator"
        }
        assert not task_types, (name, task_types)


def test_the_planner_node_loads_the_shared_base_before_the_planner_file():
    default = _args()["planner_config"].get("default")
    assert default == CONFIG_PREFIX + "planners/$(arg planner).yaml"
    loads = [tag.get("file") for tag in _planner_node().findall("rosparam")]
    assert loads == [CONFIG_PREFIX + "planners/_common.yaml", "$(arg planner_config)"]


def test_the_planner_node_itself_sets_no_task_key():
    for param in _planner_node().findall("param"):
        assert not param.get("name", "").startswith("layers/"), param.get("name")
    budget = [p for p in _planner_node().findall("param") if p.get("name") == "cost_budget"]
    assert len(budget) == 1 and budget[0].get("if")


def test_collectible_points_visualization_runs_only_for_a_visualized_points_task():
    nodes = {node.get("name"): node for node in _root().findall("node")}
    condition = nodes["collectible_points_visualization_node"].get("if")
    assert condition == "$(eval arg('visualize') and arg('task') == 'points')"
    params = {
        param.get("name"): param.get("value")
        for param in nodes["collectible_points_visualization_node"].findall("param")
    }
    assert params["map_frame"] == "$(arg map_frame)"
    assert params["camera_frame"] == "$(arg camera_frame)"


def test_planner_files_hold_no_key_their_evaluator_ignores():
    ignored = tuple(RECON_KEYS) + ("min_impact_factor", "ray_angle_x", "ray_angle_y")
    for name in sorted(os.listdir(PLANNERS)):
        if not name.endswith(".yaml"):
            continue
        with open(os.path.join(PLANNERS, name)) as handle:
            text = handle.read()
        for key in ignored:
            assert key + ":" not in text, (name, key)
