"""Render a graph benchmark graph as a gain-colored PDF."""

import argparse
from pathlib import Path

import matplotlib.cm as cm
import matplotlib.colors as mcolors
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LinearSegmentedColormap

from open_apero_analysis.figures import (
    add_figure_arguments,
    command_line,
    figure_path,
    finish_figure,
    opens_viewer,
    use_file_backend,
)
from open_apero_graph_benchmark.paths import INPUT_ROOT, RESULTS_ROOT

HEAT_COLORMAP = LinearSegmentedColormap.from_list(
    "heat",
    [
        (0.0, 0.0, 1.0),
        (0.0, 1.0, 1.0),
        (0.0, 1.0, 0.0),
        (1.0, 1.0, 0.0),
        (1.0, 0.0, 0.0),
    ],
    N=256,
)

# Edges shorter than this are skipped: at that separation the arrowhead is larger than the edge
# and the two nodes render on top of each other anyway.
MIN_EDGE_LENGTH = 1e-2

# Mid gray, as a matplotlib grayscale string. Shared by edge lines and arrowheads.
EDGE_COLOR = "0.5"

plt.rcParams.update({"font.size": 15})


class Node:
    """Graph node loaded from the graph file format."""

    def __init__(self, node_id, x, y, gain, successors_id):
        self.id = node_id
        self.x = x
        self.y = y
        self.gain = gain
        self.successors_id = successors_id


def load_graph(file_path):
    """Load a graph from the graph file format."""
    graph = {}
    # Deliberately not caught: a missing file must not become an empty graph, which the caller
    # would only notice later as a far less informative KeyError.
    with open(file_path, "r") as input_file:
        for line in input_file:
            values = line.strip().split(" ")
            node_id = values[0]
            x, y, gain = map(float, values[1:4])
            successors_id = values[4:]
            graph[node_id] = Node(node_id, x, y, gain, successors_id)
    return graph


def plot_graph(
    graph,
    min_gain,
    max_gain,
    axis=None,
    plot_gain=True,
    plot_colorbar=True,
    bidirectional=True,
):
    """Plot graph nodes and directed edges."""
    cmap = HEAT_COLORMAP
    axis = axis if axis is not None else plt.gca()

    for node in graph.values():
        for successor_id in node.successors_id:
            successor_node = graph[successor_id]
            direction = np.array([successor_node.x - node.x, successor_node.y - node.y, 0.0])
            norm = np.linalg.norm(direction)
            if norm < MIN_EDGE_LENGTH:
                continue
            direction /= norm

            # Offset opposing edges so both directions remain visible.
            ref = np.array([0.0, 0.0, 1.0])
            offset_magnitude = 0.1 if bidirectional else 0.0
            offset = np.cross(direction, ref) * offset_magnitude

            edge_tail = np.array([node.x + offset[0], node.y + offset[1], 0.0])
            edge_head = np.array([successor_node.x + offset[0], successor_node.y + offset[1], 0.0])
            x_vals = [edge_tail[0], edge_head[0]]
            y_vals = [edge_tail[1], edge_head[1]]

            axis.plot(x_vals, y_vals, color=EDGE_COLOR, linewidth=1)

            arrowhead_length = 0.25
            arrowhead_width = 0.05
            arrowhead_center = (edge_tail + edge_head) / 2
            arrowhead_tip = arrowhead_center + arrowhead_length * direction / 2
            perp = np.array([-direction[1], direction[0], 0.0]) * arrowhead_width
            arrowhead_left = arrowhead_center - arrowhead_length * direction / 2 + perp
            arrowhead_right = arrowhead_center - arrowhead_length * direction / 2 - perp

            axis.fill(
                [arrowhead_tip[0], arrowhead_left[0], arrowhead_right[0]],
                [arrowhead_tip[1], arrowhead_left[1], arrowhead_right[1]],
                color=EDGE_COLOR,
            )

    for node in graph.values():
        if plot_gain:
            normalized_gain = (node.gain - min_gain) / (max_gain - min_gain)
            color = cmap(normalized_gain)
        else:
            color = "gray"

        axis.plot(
            node.x,
            node.y,
            ".",
            color=color,
            markersize=20,
            markeredgecolor="black",
            markeredgewidth=1.5,
        )

    if plot_colorbar:
        sm = cm.ScalarMappable(cmap=cmap, norm=mcolors.Normalize(vmin=min_gain, vmax=max_gain))
        sm.set_array([])
        axis.figure.colorbar(sm, ax=axis, label="Gain")


def main(argv=None):
    use_file_backend()
    parser = argparse.ArgumentParser(description="Render a graph instance as a gain-colored PDF.")
    parser.add_argument(
        "graph_folder",
        help="Folder holding graph.txt, or <dataset>/graph_N, e.g. "
        "ap_25x25_clustered_gain_0to100_cost_50/graph_0, read from the package data/. The figure "
        "goes to <data root>/figures/<folder path>/graph.pdf, for a name "
        "<data root>/figures/graph_benchmark/<dataset>/graph_N/graph.pdf.",
    )
    parser.add_argument(
        "--margin",
        type=float,
        default=1.0,
        help="Padding added around the node extent, in graph units (default: 1.0).",
    )
    parser.add_argument(
        "--min-gain",
        type=float,
        default=None,
        help="Lower end of the gain color scale (default: the graph's own minimum).",
    )
    parser.add_argument(
        "--max-gain",
        type=float,
        default=None,
        help="Upper end of the gain color scale (default: the graph's own maximum).",
    )
    add_figure_arguments(parser)
    args = parser.parse_args(argv)

    graph_folder = Path(args.graph_folder)
    source = graph_folder
    if not graph_folder.is_absolute() and not graph_folder.exists():
        # A <dataset>/graph_N name: read the tracked input; the figure mirrors the dataset's
        # results.
        source = RESULTS_ROOT / graph_folder
        graph_folder = INPUT_ROOT / graph_folder

    graph = load_graph(graph_folder / "graph.txt")

    # The color scale spans the graph's own gains unless --min-gain/--max-gain set it; the axis
    # extent spans its node positions plus --margin.
    gains = [node.gain for node in graph.values()]
    min_gain = args.min_gain if args.min_gain is not None else min(gains)
    max_gain = args.max_gain if args.max_gain is not None else max(gains)
    if max_gain == min_gain:
        max_gain = min_gain + 1.0

    fig, ax = plt.subplots()
    plot_graph(graph, min_gain, max_gain, axis=ax, plot_gain=True)

    xs = [node.x for node in graph.values()]
    ys = [node.y for node in graph.values()]
    ax.set_xlim(min(xs) - args.margin, max(xs) + args.margin)
    ax.set_ylim(min(ys) - args.margin, max(ys) + args.margin)
    ax.set_aspect("equal")

    fig_width, fig_height = fig.get_size_inches()
    fig.set_size_inches(fig_width * 1.8, fig_height * 1.8)

    output_path = figure_path(args, source, "graph.pdf")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(output_path, format="pdf", dpi=600, bbox_inches="tight")
    finish_figure(
        output_path,
        [graph_folder / "graph.txt"],
        command_line("render_graph.py", argv),
        open_file=opens_viewer(args),
    )


if __name__ == "__main__":
    main()
