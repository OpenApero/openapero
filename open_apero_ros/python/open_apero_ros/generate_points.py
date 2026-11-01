"""Generate and visualize random collectible points for PointsEvaluator.

Usage:
    rosrun open_apero_ros generate_points.py --help
    rosrun open_apero_ros generate_points.py --num-points 5000

Writes to the data root ($OPEN_APERO_DATA_ROOT, else ~/open_apero_data) by default; copy the
result over the tracked data/points_3d.csv once you are happy with it.
"""

import argparse
import random
from pathlib import Path

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (registers "3d" before Matplotlib 3.2)

from open_apero_analysis import data_root

DEFAULT_OUTPUT = data_root() / "points_3d_temp.csv"


def generate_points(count, bounds, gain_range, rng=random):
    """Return ``count`` random ``(x, y, z, gain)`` point records.

    Args:
        bounds: ``((x_min, x_max), (y_min, y_max), (z_min, z_max))``.
        gain_range: inclusive ``(min, max)`` integer gain range.
    """
    (x_min, x_max), (y_min, y_max), (z_min, z_max) = bounds
    return [
        (
            rng.uniform(x_min, x_max),
            rng.uniform(y_min, y_max),
            rng.uniform(z_min, z_max),
            rng.randint(*gain_range),
        )
        for _ in range(count)
    ]


def write_points(points, output_file, collection_radius):
    """Write points in the CSV format consumed by PointsEvaluator."""
    Path(output_file).parent.mkdir(parents=True, exist_ok=True)
    with open(output_file, "w") as output:
        for x, y, z, gain in points:
            output.write(f"{x:.2f},{y:.2f},{z:.2f},{collection_radius:.2f},{gain}\n")


def plot_points(points):
    """Display the generated point cloud for a visual sanity check."""
    fig = plt.figure(figsize=(10, 10))
    axis = fig.add_subplot(111, projection="3d")
    xs, ys, zs, _gains = zip(*points)
    axis.scatter(xs, ys, zs, c="b", marker="o", alpha=0.6)
    axis.set_xlabel("X")
    axis.set_ylabel("Y")
    axis.set_zlabel("Z")
    axis.set_title("3D Points")
    plt.show()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--num-points",
        type=int,
        default=20000,
        help="How many points to generate (default: 20000).",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help="CSV to write (default: %s)." % DEFAULT_OUTPUT,
    )
    parser.add_argument(
        "--x-range",
        type=float,
        nargs=2,
        metavar=("MIN", "MAX"),
        default=(-50.0, 50.0),
        help="X bounds (default: -50 50).",
    )
    parser.add_argument(
        "--y-range",
        type=float,
        nargs=2,
        metavar=("MIN", "MAX"),
        default=(-50.0, 50.0),
        help="Y bounds (default: -50 50).",
    )
    parser.add_argument(
        "--z-range",
        type=float,
        nargs=2,
        metavar=("MIN", "MAX"),
        default=(-5.0, 5.0),
        help="Z bounds (default: -5 5).",
    )
    parser.add_argument(
        "--gain-range",
        type=int,
        nargs=2,
        metavar=("MIN", "MAX"),
        default=(1, 10),
        help="Inclusive gain range (default: 1 10).",
    )
    parser.add_argument(
        "--collection-radius",
        type=float,
        default=1.0,
        help="Radius written for every point (default: 1.0).",
    )
    parser.add_argument(
        "--seed", type=int, default=None, help="Seed for reproducible output (default: unseeded)."
    )
    parser.add_argument(
        "--no-show", action="store_true", help="Skip the 3D preview, e.g. when running headless."
    )
    args = parser.parse_args(argv)

    if args.num_points <= 0:
        parser.error("--num-points must be positive")
    for name in ("x_range", "y_range", "z_range", "gain_range"):
        low, high = getattr(args, name)
        if low > high:
            parser.error("--%s needs MIN <= MAX, got %s %s" % (name.replace("_", "-"), low, high))
    if args.collection_radius <= 0:
        parser.error("--collection-radius must be positive")

    rng = random.Random(args.seed) if args.seed is not None else random
    points = generate_points(
        args.num_points, (args.x_range, args.y_range, args.z_range), tuple(args.gain_range), rng=rng
    )
    write_points(points, args.output, args.collection_radius)
    print("Wrote %d points to %s" % (len(points), args.output))
    if not args.no_show:
        plot_points(points)


if __name__ == "__main__":
    main()
