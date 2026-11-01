# Logging and Metrics

The core's logging sink and the run output: the metric groups behind `performance_log.csv` and the writers of the run
directory's files. [Run Output and Analysis](../modules/run-output.md) lists every file and column.

## Logging

```{doxygennamespace} open_apero::logging
:members:
```

## Metrics

```{doxygenstruct} open_apero::Metrics
:members:
:show: header-file
```

```{doxygenclass} open_apero::MetricsFields
:members:
:show: header-file
```

```{doxygenstruct} open_apero::ParentRef
:members:
:show: header-file
```

```{doxygenstruct} open_apero::AgentMetrics
:members:
:show: header-file
```

```{doxygenstruct} open_apero::LayerMetrics
:members:
:show: header-file
```

```{doxygenstruct} open_apero::layer::PlanningLayerMetrics
:members:
:show: header-file
```

```{doxygenstruct} open_apero::layer::BacktrackingLayerMetrics
:members:
:show: header-file
```

```{doxygenclass} open_apero::ScopedTimer
:members:
:show: header-file
```

## Run Output

```{doxygenclass} open_apero::PerformanceLog
:members:
:show: header-file
```

```{doxygenstruct} open_apero::RunManifest
:members:
:show: header-file
```

```{doxygenstruct} open_apero::EpisodeSummary
:members:
:show: header-file
```
