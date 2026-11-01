# Layers

Every cycle the agent asks its layers for a step in descending priority; the first layer whose
`planPath()` succeeds wins. [Layers](../modules/layers.md) explains the arbitration and the two shipped layers.

```{doxygenclass} open_apero::Layer
:members:
:protected-members:
:show: header-file
```

```{doxygenclass} open_apero::layer::PlanningLayer
:members:
:protected-members:
:show: header-file
```

```{doxygenclass} open_apero::layer::BacktrackingLayer
:members:
:show: header-file
```
