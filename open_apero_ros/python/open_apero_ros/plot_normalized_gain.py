"""Compare normalized planner gain across active-perception tasks.

Renders Fig. 17 of Qu et al., IJRR 2026: the numbers below are the published
results, not something read back from a run directory. FUEL is shown as in the
paper, but it is not part of this release and cannot be reproduced with it.

Usage:
    rosrun open_apero_ros plot_normalized_gain.py --help
    rosrun open_apero_ros plot_normalized_gain.py --open
"""

import argparse

import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

from open_apero_analysis.figures import (
    add_figure_arguments,
    command_line,
    figure_path,
    finish_figure,
    opens_viewer,
    use_file_backend,
)

# Drawn from no run, so it sits at the top of the figure tree.
FIGURE_NAME = "normalized_gain_barplots.pdf"

ALGORITHMS = ("NBS", "DBS", "SPT", "TSP", "FUEL\n(paper only)", "RRT*", "RRT")
TASK_PERFORMANCE = {
    "Point collection": (0.50, 0.36, 0.43, 0.22, 0.30, 0.31, 0.31),
    "Surface reconstruction": (0.49, 0.49, 0.41, 0.48, 0.43, 0.40, 0.40),
    "Volumetric exploration": (0.77, 0.75, 0.74, 0.62, 0.59, 0.70, 0.68),
}


def main(argv=None):
    use_file_backend()
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    add_figure_arguments(parser)
    args = parser.parse_args(argv)
    output = figure_path(args, None, FIGURE_NAME)

    sns.set_style("darkgrid")
    plt.rcParams.update({"font.size": 12, "axes.labelsize": 12})

    colors = sns.color_palette("Set2", len(TASK_PERFORMANCE))
    task_colors = dict(zip(TASK_PERFORMANCE, colors))
    x_positions = np.arange(len(ALGORITHMS))
    bar_width = 0.25

    fig, axis = plt.subplots(figsize=(14, 4))
    for task_index, (task, performance) in enumerate(TASK_PERFORMANCE.items()):
        color = task_colors[task]
        positions = x_positions + task_index * bar_width
        axis.bar(
            positions,
            performance,
            width=bar_width,
            label=task,
            color=color,
            edgecolor="black",
            linewidth=0.5,
        )
        axis.axhline(
            y=max(performance),
            color=color,
            linestyle="dashed",
            linewidth=2,
            alpha=0.6,
        )
        for position, value in zip(positions, performance):
            axis.text(
                position,
                value + 0.01,
                f"{value:.2f}",
                ha="center",
                va="bottom",
                fontsize=11,
                color="black",
            )

    axis.set_xticks(x_positions + bar_width)
    axis.set_xticklabels(ALGORITHMS)
    axis.set_ylabel("Normalized gain")
    axis.set_ylim(0, 0.85)
    axis.legend(loc="lower right")

    fig.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, format="pdf", dpi=600, bbox_inches="tight")
    finish_figure(
        output,
        [],
        command_line("plot_normalized_gain.py", argv),
        open_file=opens_viewer(args),
        extra={"source": "the paper's published values, kept in the script"},
    )


if __name__ == "__main__":
    main()
