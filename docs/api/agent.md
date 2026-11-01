# Agent

The agent owns the map, the robot model, the cost computer and the layer stack, and runs the
plan-execute cycle; [Agent and Execution](../modules/agent.md) explains it. A platform derives from {cpp:class}`open_apero::Agent`, implements
`sendMovement()` and `publishVisualization()`, and feeds measured states in with
`setCurrentState()`; plugins see the agent only through {cpp:class}`open_apero::AgentInterface`.

% The :members: list below names the members shown; a new member of Agent joins it by hand.

```{doxygenclass} open_apero::Agent
:members: State, ExecutionStatus, CommandPhase, StateUpdateKind, EpisodeEnd, Agent, resetEpisode, planningLoop, configureRunOutput, reportUnreadParams, loggingRequested, saveScreenshot, resumeEpisode, getMetrics, getExecutionStatus, isAwaitingExternalInput, reduceCostBudget, setCostBudget, setupFromParamMap, clearEpisodeState, onPlanningStart, loopIteration, requestNextTrajectory, readyToExpand, sendMovement, respondToHit, setCurrentState, zeroSensorRoll, executeTrajectory, onGoalReached, terminateEpisode, finalizeEpisodeOutput, hasSettled, recordObservedPoses, updateFrameLedger, saveMapSnapshot
:show: header-file
```

{cpp:class}`open_apero::BenchmarkAgent` runs the planner without a robot or ROS: movement finishes at once. The graph
benchmark drives the path planners through it.

```{doxygenclass} open_apero::BenchmarkAgent
:members:
:protected-members:
:show: header-file
```

```{doxygenclass} open_apero::AgentInterface
:members:
:show: header-file
```

```{doxygenstruct} open_apero::EdgeCursor
:members:
:show: header-file
```

```{doxygenstruct} open_apero::RunOutputConfig
:members:
:show: header-file
```
