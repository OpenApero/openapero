# API Reference

The C++ API of Open Apero, generated from the headers by Doxygen and Breathe. Every class lives in
namespace `open_apero`; most built-in plugins live in a sub-namespace named after their role
(`open_apero::evaluator`, `open_apero::path_planner`, ...). [How Open Apero Works](../concepts.md)
explains how the pieces fit together, and [Extending Open Apero](../extending.md) walks through
writing a plugin.

| I want to ... | Start with |
|---------------|------------|
| write a plugin | {cpp:class}`open_apero::Configurable`, {cpp:class}`open_apero::Plugin`, {cpp:struct}`open_apero::PluginRegistry::Registration`, then the role's base class in [Plugin Interfaces](plugin-interfaces.md) |
| connect a robot or simulator | {cpp:class}`open_apero::Agent` (`sendMovement()`, `setCurrentState()`), {cpp:struct}`open_apero::EdgeCursor`, or the ready-made {cpp:class}`open_apero::ros::RosAgent` |
| run the planner without ROS | {cpp:class}`open_apero::PluginFactoryConfig`, {cpp:class}`open_apero::BenchmarkAgent` |
| add a planner layer | {cpp:class}`open_apero::Layer` |
| add a map backend | {cpp:class}`open_apero::Map`, {cpp:class}`open_apero::map::OccupancyMap`, {cpp:class}`open_apero::map::TsdfMap` |

```{toctree}
:maxdepth: 1

agent
layers
plugin-system
plugin-interfaces
builtin-plugins
maps
data-types
logging-metrics
ros
```
