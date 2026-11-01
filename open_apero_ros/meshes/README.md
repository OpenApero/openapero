# Meshes

Meshes and textures for `collectible_points_visualization_node`, which draws the point-collection task in
RViz through `package://open_apero_ros/meshes/...` URIs: `collectible_ring.dae`, the ring drawn at each point, and
the runner figure. Nothing else depends on them: the node is display-only and the planner runs without it.

The runner figure is third-party and **not** covered by the repository's BSD 3-Clause licence.

## Runner figure

| | |
|---|---|
| Title | ANIMATIONS Classic Sonic - Sonic Runners |
| Creator | clwent (@davetehhedgehog) |
| Source | <https://sketchfab.com/3d-models/animations-classic-sonic-sonic-runners-fa58f582a6f4425ba303c1b0cf3f34c8> (Sketchfab) |
| Licence | [Creative Commons Attribution 4.0 International (CC BY 4.0)][cc-by] |
| Files | `runner_0.dae` … `runner_32.dae` (even frames only), `runner_body_dif.png`, `runner_eye_dif.png`, `runner_mouth_dif.png` |
| Changes | The animation was exported from Blender as one COLLADA file per frame, even frames only, and the files, textures and their internal names were renamed; `collectible_points_visualization_node` cycles through them to animate the running figure |

"ANIMATIONS Classic Sonic - Sonic Runners" by clwent (@davetehhedgehog), licensed under [CC BY 4.0][cc-by],
exported from Blender as one COLLADA file per even animation frame, with files and internal names renamed.

CC BY 4.0 requires that this attribution travel with the files: keep this notice beside them in any
redistribution. The licence covers this model of the character, not the character itself, which remains the
property of its rights holder.

[cc-by]: https://creativecommons.org/licenses/by/4.0/
