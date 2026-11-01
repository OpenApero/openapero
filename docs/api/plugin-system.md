# Plugin System

How a `type:` string in YAML becomes a configured object: parameter parsing
({cpp:class}`open_apero::Configurable`), self-registration ({cpp:class}`open_apero::PluginRegistry`),
loading a library of plugins ({cpp:func}`open_apero::loadPluginLibrary`) and construction
({cpp:class}`open_apero::PluginFactory`).

```{doxygenclass} open_apero::Configurable
:members:
:protected-members:
:show: header-file
```

```{doxygenclass} open_apero::Plugin
:members:
:protected-members:
:show: header-file
```

```{doxygenclass} open_apero::PluginRegistry
:members:
:show: header-file
```

```{doxygentypedef} open_apero::PluginConstructor
```

```{doxygenfunction} open_apero::loadPluginLibrary
```

```{doxygenclass} open_apero::PluginFactory
:members:
:protected-members:
:show: header-file
```

```{doxygenclass} open_apero::PluginFactoryConfig
:members:
:show: header-file
```

```{doxygenstruct} open_apero::BoundingVolume
:members:
:show: header-file
```

## Seeded Randomness

```{doxygenfunction} open_apero::makeSeededEngine
```

```{doxygenstruct} open_apero::SeededRng
:members:
:show: header-file
```
