"""Generate a random graph topology for the graph benchmark."""

import argparse
import math
import random
from pathlib import Path

from open_apero_graph_benchmark.paths import GENERATED_ROOT

OUTPUT_NAME = "graph_topology.txt"

NUM_NODES = 1000
MAX_TRIES = 5000
X_MIN, X_MAX = 0, 25
Y_MIN, Y_MAX = 0, 25
MIN_DISTANCE = 1.5
MAX_DISTANCE = 3.0
DEFAULT_GAIN = 0.0


def generate_nodes():
    """Generate separated random nodes, always beginning at the origin."""
    nodes = [(0.0, 0.0)]
    tries = 0
    while len(nodes) < NUM_NODES and tries < MAX_TRIES:
        x = random.uniform(X_MIN, X_MAX)
        y = random.uniform(Y_MIN, Y_MAX)
        tries += 1
        if all(math.hypot(x - nx, y - ny) >= MIN_DISTANCE for nx, ny in nodes):
            nodes.append((x, y))
            tries = 0
    return nodes


def build_adjacency(nodes):
    """Connect every node pair within ``MAX_DISTANCE`` symmetrically."""
    graph = {index: [] for index in range(len(nodes))}
    for first in range(len(nodes)):
        for second in range(first + 1, len(nodes)):
            if (
                math.hypot(
                    nodes[first][0] - nodes[second][0],
                    nodes[first][1] - nodes[second][1],
                )
                <= MAX_DISTANCE
            ):
                graph[first].append(second)
                graph[second].append(first)
    return graph


def write_graph(nodes, graph, output_path):
    """Write nodes and adjacency in the graph benchmark text format."""
    with open(output_path, "w") as output_file:
        for node_id, (x, y) in enumerate(nodes):
            neighbors = " ".join(map(str, graph[node_id]))
            output_file.write(f"{node_id} {x:.2f} {y:.2f} {DEFAULT_GAIN:.2f} {neighbors}\n")


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__.splitlines()[0] + " Its settings are constants in the module."
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=GENERATED_ROOT / OUTPUT_NAME,
        help="Topology file to write (default: %(default)s).",
    )
    parser.add_argument(
        "--seed", type=int, default=None, help="Seed for reproducible output (default: unseeded)."
    )
    args = parser.parse_args(argv)
    if args.seed is not None:
        random.seed(args.seed)
    nodes = generate_nodes()
    print(f"Generated {len(nodes)} nodes (Target: {NUM_NODES})")
    graph = build_adjacency(nodes)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    write_graph(nodes, graph, args.output)
    print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
