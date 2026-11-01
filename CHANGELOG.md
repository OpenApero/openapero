# Changelog

All notable changes to this project are documented in this file. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and versions follow
[Semantic Versioning](https://semver.org/spec/v2.0.0.html): a breaking change to the C++ API or to the
configuration format bumps the major version.

## [1.0.0] - 2026-11-01

The initial public release: the code release of *An Efficient Beam Search
Algorithm for Active Perception in Mobile Robotics* (Qu et al., IJRR 2026).

### Added

- Graph planners NBS, DBS, SPT and TSP on a random annulus graph, and the RRT and RRT\* tree baselines.
- Gain evaluators for volumetric exploration, surface reconstruction and point collection.
- A plugin for every role, selected by `type:` in YAML.
- `open_apero_node`, a ROS Noetic planner node for any robot, on a Voxblox map.
- A Habitat-sim setup for the paper's HSSD experiments, and an offline graph benchmark that
  reproduces the paper's results.
- A Docker image for x86_64 hosts other than Ubuntu 20.04.

[1.0.0]: https://github.com/OpenApero/openapero/releases/tag/v1.0.0
