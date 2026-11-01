# Citation, License and Acknowledgments

How to cite Open Apero, the terms it is distributed under, and the projects it builds on. For anyone publishing work
that uses it or redistributing it.

## Citation

If you use Open Apero in your research, please cite the paper it was developed for:

```bibtex
@article{qu2026efficient,
  title={An Efficient Beam Search Algorithm for Active Perception in Mobile Robotics},
  author={Qu, Kaixian and Wang, Han and Klemm, Victor and Cadena, Cesar and Hutter, Marco},
  journal={The International Journal of Robotics Research},
  year={2026},
  doi={10.1177/02783649261455911},
  note={Project page: \url{https://efficient-beam-search.github.io/}}
}
```

The same reference is the `preferred-citation` in
[`CITATION.cff`](https://github.com/OpenApero/openapero/blob/main/CITATION.cff), which GitHub's "Cite this repository"
button reads.

## License

This project is licensed under the **BSD 3-Clause License**; see the
[LICENSE](https://github.com/OpenApero/openapero/blob/main/LICENSE) file. The copyright is held by ETH Zürich, and
Open Apero was created by Kaixian Qu and Han Wang at the Robotic Systems Lab (RSL).

Open Apero started in 2024 as a fork of
[mav_active_3d_planning](https://github.com/ethz-asl/mav_active_3d_planning) (BSD 3-Clause, Copyright (c) 2020, ETHZ
ASL) and has since been substantially rewritten: the graph planners, the Habitat simulation and the graph benchmark
are new.

Two exceptions are bundled content that this license does not cover:

- the runner figure in
  [`open_apero_ros/meshes/`](https://github.com/OpenApero/openapero/blob/main/open_apero_ros/meshes/README.md) drawn by
  `collectible_points_visualization_node`, under CC BY 4.0 (attribution in that README);
- the vendored single-header libraries in `open_apero_core/include/open_apero_core/third_party/`: nanoflann 1.7.1
  (BSD 2-Clause) and tinycolormap (MIT), whose Turbo colormap lookup table is Copyright 2019 Google LLC under the
  Apache License 2.0, with its notice retained in the header.

The Open Apero logo (`open_apero_logo.png`) is part of this repository and is distributed under its BSD 3-Clause
license. The HSSD scenes the simulated experiments render are not part of this repository; their dataset is licensed
CC BY-NC 4.0 (see [HSSD Dataset](installation.md#hssd-dataset-optional)).

## Acknowledgments

This project builds upon and has benefited from the following libraries:

- [mav_active_3d_planning](https://github.com/ethz-asl/mav_active_3d_planning) - The original tree-based active 3D
  planning library
- [Voxblox](https://github.com/ethz-asl/voxblox) - Volumetric mapping framework
- [OMPL](https://ompl.kavrakilab.org/) - Open Motion Planning Library
- [OR-Tools](https://developers.google.com/optimization) - Google's optimization suite
- [Habitat](https://aihabitat.org/) - Meta AI's simulation platform
