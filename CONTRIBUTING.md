# Contributing to Open Apero

Bug fixes, plugins, new robot platforms ([Integrating a Robot](docs/integrating-a-robot.md)), documentation, tests and
performance improvements are welcome. By participating, you agree to abide by the
[Code of Conduct](https://github.com/OpenApero/openapero/blob/main/CODE_OF_CONDUCT.md), and by contributing, that your
contributions are licensed under the [BSD 3-Clause License](https://github.com/OpenApero/openapero/blob/main/LICENSE).

## Development Setup

Open Apero builds natively on Ubuntu 20.04 with ROS Noetic; on any other host, develop in the Docker image
([In Docker](#in-docker)). Fork [OpenApero/openapero](https://github.com/OpenApero/openapero), follow
[Installation](docs/installation.md) but clone your fork with upstream as a second remote, and install the commit hooks
of [`.pre-commit-config.yaml`](https://github.com/OpenApero/openapero/blob/main/.pre-commit-config.yaml):

```bash
cd ~/catkin_ws/src
git clone --recursive https://github.com/<your-username>/openapero.git open_apero
git -C open_apero remote add upstream https://github.com/OpenApero/openapero.git
curl -LsSf https://astral.sh/uv/install.sh | sh && source ~/.local/bin/env   # uv, unless installed
uv tool install --python 3.11 'pre-commit<4.6.1'  # 4.6.1 and later need git 2.31; Ubuntu 20.04 has 2.25
cd open_apero && pre-commit install
```

Build (`cd ~/catkin_ws && catkin build && source devel/setup.bash`), run the tests, then create a branch.

## Running Tests

After a full `catkin build`, build and run the tests of Open Apero's own packages, as CI's
[`build_test.yml`](https://github.com/OpenApero/openapero/blob/main/.github/workflows/build_test.yml) does:

```bash
cd ~/catkin_ws
OWNED="open_apero_analysis open_apero_controllers open_apero_core open_apero_ros \
  open_apero_graph_benchmark open_apero_habitat open_apero_voxblox"
catkin build $OWNED --no-deps --catkin-make-args run_tests --
for pkg in $OWNED; do catkin_test_results "build/$pkg"; done
```

`catkin_test_results` exits non-zero when one of a package's tests failed. A workspace-wide `run_tests` would also
build third-party tests, and those of `eigen_checks` fail to build. To iterate on one package, set `OWNED` to it alone.

### In Docker

The image copies the source tree when it is built: rebuild it after every change, then run the
[test command of `docker/README.md`](https://github.com/OpenApero/openapero/blob/main/docker/README.md#test).

## Coding Standards

The commit hooks format C++ with clang-format 10.0.1, which they install themselves (for an editor,
`pip install clang-format==10.0.1`; other versions differ), and
[`.clang-format`](https://github.com/OpenApero/openapero/blob/main/.clang-format), and Python with black and ruff,
configured in [`pyproject.toml`](https://github.com/OpenApero/openapero/blob/main/pyproject.toml).

- **C++**: C++17, built with `-Wall -Wextra -Werror=return-type`, and new code adds no warnings. Headers use
  `#pragma once`. The vendored headers in `open_apero_core/include/open_apero_core/third_party/` stay as upstream's.
- **Naming**: classes and structs `PascalCase`; functions `camelCase`; variables and struct members `snake_case`; class
  members `snake_case_` (`p_<key>_` when bound to a parameter key, `c_*_` when derived from parameters); constants
  `kPascalCase`; namespaces `snake_case`; files the `snake_case` of their class. Acronyms are cased as words (`NodeId`,
  `TsdfMap`), except algorithm names that are plugin IDs (`RRT`, `RRAG`). Spelling is American (`center`, `color`).
- **Includes**: own header (`.cpp` only), C system, C++ standard library, third-party and other packages, same package,
  one blank line apart and alphabetical within; `""` for the same package, `<>` for all others. Include what you use;
  no forward declarations unless one breaks a circular include.
- **Python**: PEP 8, Python 3.8 (the ROS Noetic system Python). Code lives in `<pkg>/python/<pkg>/`; a
  `nodes/<module>_node` or `scripts/<tool>.py` only calls its module's `main()` and is listed in
  `test/test_<pkg>_imports.py` (`<pkg>` without its `open_apero_` prefix). Runs write below `~/open_apero_data`, never
  into the source tree.
- **Folders**: a package holds only `include/`, `src/` (plugins in `plugin/<kind>/`), `python/`, `nodes/`, `scripts/`,
  `launch/`, `config/`, `rviz/`, `meshes/`, `msg/`, `cmake/`, `data/` (tracked input, never run output) and `test/`.
- **ROS and tools**: the executable, the node name and the launch file's `name=` are the same `<module>_node`.
  Parameter keys are `snake_case` with words spelled out (`visualization`, not `viz`); a frame parameter ends in
  `_frame`. Command-line options are kebab-case (`--data-root DIR`, `--out-dir DIR`, `--dry-run`).

## Adding a Plugin

[Extending Open Apero](docs/extending.md) holds the plugin extension contract. A type that ships in this repository
also needs, in the same pull request:

1. Its header and source in `plugin/<role>/` of `open_apero_core` (a ROS-dependent type: `open_apero_ros`), listed in
   the library's sources (`SRC_FILES` of `open_apero_core/CMakeLists.txt`); the class in the namespace of its role
   (`open_apero::generator::RRAG`), registered under its class name.
2. For a core type, a recipe in `open_apero_core/test/test_plugin_conformance.cpp`.
3. Its keys, defaults and shipped values in the [Parameter Reference](docs/parameters.md).
4. A Doxygen `@brief` on the class and its public methods, and a `{doxygenclass}` entry in
   `docs/api/builtin-plugins.md` (a layer: `layers.md`; a map backend: `maps.md`; an `open_apero_ros` type: `ros.md`;
   a new role's base class: `plugin-interfaces.md`).
5. For a new planner, `open_apero_ros/config/planners/<planner>.yaml` holding only its `layers.planning` block.

## Documentation

One fact has one home, and every other place links to it: explanation and guides in `docs/*.md`; plugin and agent YAML
keys in `docs/parameters.md`; a node's parameters, topics and services in its package `README.md` (a tool's flags in
`scripts/README.md`); a launch argument in a comment next to its `<arg>`; the C++ API in Doxygen comments. Diagrams are
Mermaid blocks. Keep pages short: a sentence and a link instead of a repeated default.

Links use relative paths. The site includes `CONTRIBUTING.md` and `CHANGELOG.md` whole, and README passages between
`<!-- <name>-start… -->` and `<!-- <name>-end -->` comments: there, link a `docs/` page by its relative path and any
other file by its `https://github.com/OpenApero/openapero/blob/main/...` URL.

Build the docs for a change to them or to a public header, in a separate environment (Sphinx 8.2 needs Python 3.11 or
newer). Sphinx runs with `-W`, so any warning fails the build, as in CI's
[`docs.yml`](https://github.com/OpenApero/openapero/blob/main/.github/workflows/docs.yml). From the repository root:

```bash
sudo apt-get install doxygen
uv venv -p 3.11 ~/venvs/open_apero_docs && . ~/venvs/open_apero_docs/bin/activate
uv pip install -r docs/requirements.txt
docs/generate_docs.sh    # output: docs/_build/html/index.html
```

## Submitting Changes

1. Run the tests ([Running Tests](#running-tests)) and `pre-commit run --all-files` from the repository root, as CI's
   [`portability.yml`](https://github.com/OpenApero/openapero/blob/main/.github/workflows/portability.yml) does.
2. If you changed documentation or public headers, build the docs ([Documentation](#documentation)).
3. For a user-visible change, add one line to `CHANGELOG.md` under `## [Unreleased]` (add the heading above the latest
   release if missing), in its group (`### Added`, `### Changed`, `### Fixed`, `### Removed`).
4. Rebase on `main` (`git fetch upstream && git rebase upstream/main`), then open one pull request per change with the
   [pull request template](https://github.com/OpenApero/openapero/blob/main/.github/PULL_REQUEST_TEMPLATE.md).

A commit subject is one imperative sentence of at most 72 characters, without a trailing period. A change confined to
one package may prefix it with `[<package>]`, one confined to one class with `<Class>:`
(`Agent: Count dispatched commands and integrated frames per cycle`). A body, after a blank line, gives the motivation.

## Reporting Bugs and Requesting Features

Open an issue with the [bug report](https://github.com/OpenApero/openapero/issues/new?template=bug_report.md) or
[feature request](https://github.com/OpenApero/openapero/issues/new?template=feature_request.md) template; for a bug,
the commit, native or Docker, the launch command and the planner configuration usually decide whether it reproduces.
Report security problems privately ([SECURITY.md](https://github.com/OpenApero/openapero/blob/main/SECURITY.md)).
