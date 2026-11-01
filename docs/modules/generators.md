# Generators

How the planning layer's candidates come about: the generator samples new viewpoints around the existing ones,
connects them with edges and keeps the graph valid as the map grows. Keys:
[`Generator` and `GraphGenerator` Keys](../parameters.md#generator-and-graphgenerator-keys),
[`RRAG`](../parameters.md#rrag), [`RRT`, `RRTStar`](../parameters.md#rrt-rrtstar); classes:
[Generators](../api/plugin-interfaces.md#generators), [built-in generators](../api/builtin-plugins.md#generators).

Every shipped generator is a `GraphGenerator` that owns its graph or tree. It adds one node cluster per expansion round,
moves the start node when the robot arrives (`setNewStartNode()`) and cuts an edge at the replanning horizon
(`insertIntermediateNode()`). Each cycle `RRAG` re-checks the nodes, edges and missing connections where the map region
updated since the last cycle meets the robot's body bounds (`RobotModel::getBodyBounds()`), and all of them for a robot
without bounds; the trees re-check only their edges and prune the subtrees behind those that fail. Positions,
orientations and edges come from the [robot model](robot-models.md).

## Sampling Rule

`RRAG`, `RRT` and `RRTStar` sample the same way. A position closer than `min_edge_length` to its nearest node is
rejected, and one farther than `max_edge_length` is moved to `min_edge_length` from that node, so new positions land
in an annulus around the existing ones (shipped `1.5` to `3.0` m; `1.0` to `2.0` m with `task:=points`). An
expansion round makes at most `sample_max_tries` (shipped `1000`) attempts.

## `RRAG`

The rapidly-exploring random annulus graph, the generator of the graph planners `nbs`, `dbs`, `spt` and `tsp`. At
each sampled position it builds a node cluster: one node per orientation the robot model offers
(`num_orientation_candidates` evenly spaced yaws for the in-tree robots). Rotation edges join neighboring
orientations of a cluster, and translation edges join clusters within `max_edge_length` of each other.
`translation_adjacency` picks the orientation pair a translation edge joins: the pair best aligned with the chord
between the two positions (`straight_line`, default), with the planned trajectory's end directions
(`path_heading`), or every pair bracketing those directions as the robot model decides (`path_heading_cell`; the
in-tree robots return one pair).

## `RRT` and `RRTStar`

Trees grown from the robot, the generators of the `rrt` and `rrtstar` baselines (the paper's RRAT and RRAT*). Each
sample becomes one tree node whose parent is its nearest node; the planner config wraps the task evaluator in
`SimpleYawPlanningEvaluator`, which picks each node's yaw. `RRTStar` also gives each new node the cheapest parent
within `max_edge_length`, rewires the nodes around it through it where that is cheaper, and rewires the tree again
whenever it reroots. A tree's path planner is `TreeBestValue`, which rejects a graph generator such as `RRAG`.
