"""Generate a clustered spatial gain model for the graph benchmark."""

import argparse
import math
import random
from pathlib import Path

from open_apero_graph_benchmark.paths import GENERATED_ROOT

OUTPUT_NAME = "gain_model.txt"


def generate_gain_regions(num_clusters, x_min, x_max, y_min, y_max, gain_max):
    """Generate a list of non-overlapping circular gain-model regions.

    Args:
        num_clusters: Number of gain regions to place.
        x_min, x_max: Horizontal bounds of the placement area.
        y_min, y_max: Vertical bounds of the placement area.
        gain_max: Maximum gain value for any region.

    Returns:
        A list of [x_center, y_center, radius, min_gain, max_gain] lists.
    """
    gain_regions = []

    max_radius = min((x_max - x_min) / 6, (y_max - y_min) / 6)
    min_radius = max((x_max - x_min) / 12, (y_max - y_min) / 12)

    for _ in range(num_clusters):
        while True:
            x = random.uniform(x_min, x_max)
            y = random.uniform(y_min, y_max)
            radius = random.uniform(min_radius, max_radius)

            valid = True
            for existing in gain_regions:
                x_old, y_old, r_old, *_ = existing
                distance = math.sqrt((x - x_old) ** 2 + (y - y_old) ** 2)

                if distance < (radius + r_old):
                    valid = False
                    break

            if valid:
                break

        min_gain = 0
        max_gain = gain_max

        gain_regions.append([x, y, radius, min_gain, max_gain])

    return gain_regions


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__.splitlines()[0] + " Its settings are constants in main()."
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=GENERATED_ROOT / OUTPUT_NAME,
        help="Gain model file to write (default: %(default)s).",
    )
    parser.add_argument(
        "--seed", type=int, default=None, help="Seed for reproducible output (default: unseeded)."
    )
    args = parser.parse_args(argv)
    if args.seed is not None:
        random.seed(args.seed)
    num_clusters = 8
    x_min, x_max = 0, 25
    y_min, y_max = 0, 25
    gain_max = 100

    gain_regions = generate_gain_regions(num_clusters, x_min, x_max, y_min, y_max, gain_max)
    gain_regions.append([0, 0, 0, 0, 0])

    for region in gain_regions:
        print(" ".join(map(str, region)))

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with open(args.output, "w") as output_file:
        for region in gain_regions:
            output_file.write(" ".join(map(str, region)) + "\n")
    print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
