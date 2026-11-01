# Map

The map interface the agent, the robot model and the evaluators query, the contracts that refine it, and the Voxblox
backend; [Maps](../modules/maps.md) explains the contracts.

```{doxygenclass} open_apero::Map
:members:
:show: header-file
```

```{doxygenclass} open_apero::map::OccupancyMap
:members:
:show: header-file
```

```{doxygenclass} open_apero::map::TsdfMap
:members:
:show: header-file
```

## Voxblox Backend

From the `open_apero_voxblox` package.

```{doxygenclass} open_apero::map::VoxbloxMap
:members:
:show: header-file
```

## Enumerations

```{doxygenenum} open_apero::Occupancy
```

```{doxygenenum} open_apero::CollisionStatus
```

## Hit Regions

```{doxygenstruct} open_apero::GeometryPrimitiveCollection
:members:
:show: header-file
```
