# Extension Hooks

Interfaces the shipped setups leave at their default or never call. Each has a default that leaves the shipped setups
unchanged; a custom robot or plugin overrides or reads it. The [API reference](../api/index.md) documents each, and
[Extending Open Apero](../extending.md) shows how to write the plugin that overrides one.

| Hook | Default | What a custom robot or plugin does with it |
|------|---------|--------------------------------------------|
| Hit regions: `Map::registerHit()`, `Map::getHitRegions()`, `Map::isStateReachable()`, `RobotModel::updateHitRegions()`, `RobotModel::isInsideHitRegions()`, `Agent::respondToHit()` | No regions; every state reachable | Record where a collision or a vetoed step happened; the map keeps the blocked regions until `resetEpisode()`, and `SimpleRobot` rejects a state for which `isInsideHitRegions()` returns `true` |
| `Map::getGroundHeight()` | `false` (no ground model) | Report the ground under a state, so the path markers draw its ground track |
| `RobotModel::isStationingPositionAdmissible()`, `StationingCheck` | Admits every position | Restrict where nodes may be placed, separately from traversability; generators exempt only the seed at the robot's own position |
| `RobotModel::attachSharedGainAnchors()` | Attaches none | Give a new cluster's nodes gain anchors they share, which an evaluator that keeps them scores once; `RRAG` calls it once per new cluster |
| `HistoryData::getObservedViewpoints()`, `ObservedViewpoints` | Disabled (both thresholds `0.0`) | Ask whether a pose was already looked from ([History and Observed Viewpoints](agent.md#history-and-observed-viewpoints)) |
| `RobotModel::getGazePitchMin()`, `getGazePitchMax()`, `getSensorMountType()` | −π/2, π/2, `fixed` | Plan the gaze pitch or the full sensor orientation; every shipped plugin plans yaw only |
| `RobotModel::samplePositionNear()`, `RobotModel::getSafeRadius()`, `SensorModel::pointInSensorView()` | `samplePositionNear()` returns `false` (`SimpleRobot` implements it); every point is in view (`CameraModel` implements it) | Sample candidates around a point of interest; test without occlusion whether a point is in a sensor's view |
| `Backtracker::lastProbeSamples()`, `declareMetrics()`, `getVisualization()` | No samples, columns or markers | Report the samples behind a recovery decision, add `<layer>.` columns to `performance_log.csv` and draw markers |
| `RRAG::findNearestExpansionAnchor()`, `isSamplePositionAdmissible()`, `indexNodeCluster()` | The nearest node; every position admissible; one spatial index | A subclass keeps its own expansion anchors and indices, or rejects sample positions early |
| `Edge::is_backtrack` | Set on every recovery edge; the backtrackers and edge revalidation skip such edges | Exempt recovery edges from a platform's own pre-flight checks in `sendMovement()` |
| `RosAgent::statusOverlayLines()`, `RosAgent::onStateAvailable()` | The active layer, distance and time; roots the graph in the first state after a reset | Add lines to the RViz overlay, or react to each arriving state |

`Agent::sendMovement()`, which every platform implements, and the loop hooks `Agent::readyToExpand()` and
`RobotModel::refineBeforeExecute()` are described in [Agent and Execution](agent.md#the-loop) and
[Layers](layers.md#planninglayer).
