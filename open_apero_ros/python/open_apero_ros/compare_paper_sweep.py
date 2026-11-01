"""Compare a paper_sweep.py data root against the paper's active-perception table.

Reads every ``<data_root>/<task>/HSSD/<scene>/<planner>/<stamp>/performance_log.csv``
and reports, per task x scene x planner, the gain at the point whose cost is nearest the
run's cost budget, beside the paper's mean and ground truth. The budget is the run's
``episode_summary.yaml`` ``cost_budget`` when it records a finite one, else the protocol's
budget for the scene (``open_apero_ros.paper_protocol``). The points are the log's rows and,
when the run has an ``episode_summary.yaml``, the episode's end (its ``total_cost`` and
planning gain), which no row records.

No unit conversion: ``planning.RealAccumulatedGain`` is already point gain, m^2 or m^3.

Stdlib only, apart from the stdlib open_apero_ros modules. Works with partial data -- an
empty cell prints '-'.

  compare_paper_sweep.py --data-root DIR [--output report.md]
"""

import argparse
import csv
import math
import os
import statistics
import sys

from open_apero_ros.episode_summary import read_episode_summary
from open_apero_ros.paper_protocol import (
    LARGE_BUDGET,
    LARGE_SCENES,
    PLANNERS,
    SMALL_BUDGET,
    SMALL_SCENES,
)
from open_apero_ros.paper_sweep import HelpFormatter

GAIN = "planning.RealAccumulatedGain"

TASK_LABEL = {
    "points": "point collection [gain]",
    "recon": "surface reconstruction [m^2]",
    "explore": "volumetric exploration [m^3]",
}

# Qu et al., IJRR 2026, Figs. 15-16 (legends): scene -> (ground truth, {planner: mean final gain}).
# FUEL, also in the paper, is omitted: it is not part of this release.
PAPER = {
    "points": {
        "102344280": (
            1028,
            {"nbs": 675, "dbs": 422, "spt": 560, "tsp": 213, "rrtstar": 361, "rrt": 402},
        ),
        "102344049": (
            1183,
            {"nbs": 644, "dbs": 377, "spt": 540, "tsp": 237, "rrtstar": 351, "rrt": 363},
        ),
        "102816009": (
            1713,
            {"nbs": 674, "dbs": 569, "spt": 556, "tsp": 179, "rrtstar": 389, "rrt": 428},
        ),
        "103997445": (
            1873,
            {"nbs": 740, "dbs": 578, "spt": 574, "tsp": 247, "rrtstar": 416, "rrt": 427},
        ),
        "102816036": (
            1879,
            {"nbs": 712, "dbs": 498, "spt": 527, "tsp": 178, "rrtstar": 362, "rrt": 387},
        ),
        "102815835": (
            645,
            {"nbs": 315, "dbs": 225, "spt": 275, "tsp": 172, "rrtstar": 218, "rrt": 198},
        ),
        "102816852": (
            496,
            {"nbs": 349, "dbs": 280, "spt": 306, "tsp": 255, "rrtstar": 257, "rrt": 226},
        ),
        "102816216": (
            540,
            {"nbs": 256, "dbs": 194, "spt": 248, "tsp": 151, "rrtstar": 183, "rrt": 168},
        ),
    },
    "recon": {
        "102344280": (
            1355,
            {"nbs": 898, "dbs": 818, "spt": 779, "tsp": 757, "rrtstar": 669, "rrt": 694},
        ),
        "102344049": (
            1375,
            {"nbs": 547, "dbs": 670, "spt": 645, "tsp": 652, "rrtstar": 610, "rrt": 604},
        ),
        "102816009": (
            2552,
            {"nbs": 843, "dbs": 809, "spt": 666, "tsp": 817, "rrtstar": 654, "rrt": 643},
        ),
        "103997445": (
            2186,
            {"nbs": 798, "dbs": 856, "spt": 771, "tsp": 636, "rrtstar": 644, "rrt": 672},
        ),
        "102816036": (
            3026,
            {"nbs": 830, "dbs": 981, "spt": 763, "tsp": 916, "rrtstar": 696, "rrt": 706},
        ),
        "102815835": (
            932,
            {"nbs": 511, "dbs": 518, "spt": 430, "tsp": 494, "rrtstar": 383, "rrt": 403},
        ),
        "102816852": (
            725,
            {"nbs": 527, "dbs": 483, "spt": 386, "tsp": 519, "rrtstar": 398, "rrt": 413},
        ),
        "102816216": (
            811,
            {"nbs": 471, "dbs": 444, "spt": 323, "tsp": 540, "rrtstar": 401, "rrt": 380},
        ),
    },
    "explore": {
        "102344280": (
            802.4,
            {"nbs": 715, "dbs": 687, "spt": 680, "tsp": 629, "rrtstar": 681, "rrt": 657},
        ),
        "102344049": (
            776.6,
            {"nbs": 634, "dbs": 621, "spt": 620, "tsp": 503, "rrtstar": 606, "rrt": 599},
        ),
        "102816009": (
            1463.0,
            {"nbs": 932, "dbs": 900, "spt": 910, "tsp": 678, "rrtstar": 756, "rrt": 739},
        ),
        "103997445": (
            1606.7,
            {"nbs": 1156, "dbs": 1109, "spt": 1067, "tsp": 591, "rrtstar": 806, "rrt": 858},
        ),
        "102816036": (
            1656.0,
            {"nbs": 1011, "dbs": 940, "spt": 1010, "tsp": 546, "rrtstar": 782, "rrt": 741},
        ),
        "102815835": (
            469.6,
            {"nbs": 369, "dbs": 373, "spt": 366, "tsp": 365, "rrtstar": 365, "rrt": 337},
        ),
        "102816852": (
            376.2,
            {"nbs": 320, "dbs": 319, "spt": 294, "tsp": 292, "rrtstar": 313, "rrt": 310},
        ),
        "102816216": (
            429.0,
            {"nbs": 368, "dbs": 368, "spt": 358, "tsp": 361, "rrtstar": 357, "rrt": 342},
        ),
    },
}
# Mean final TotalCost (last performance_log.csv row) of the paper's 10 runs per cell.
# Values well below the budget are the paper's wall cap truncating that planner, which is
# protocol.
PAPER_COST = {
    "points": {
        "102344280": {
            "nbs": 599.6,
            "dbs": 599.6,
            "spt": 599.5,
            "tsp": 183.2,
            "rrtstar": 498.6,
            "rrt": 599.3,
        },
        "102344049": {
            "nbs": 599.7,
            "dbs": 599.6,
            "spt": 599.5,
            "tsp": 206.0,
            "rrtstar": 565.7,
            "rrt": 599.5,
        },
        "102816009": {
            "nbs": 599.7,
            "dbs": 599.7,
            "spt": 599.7,
            "tsp": 192.0,
            "rrtstar": 484.8,
            "rrt": 599.2,
        },
        "103997445": {
            "nbs": 599.6,
            "dbs": 599.6,
            "spt": 599.6,
            "tsp": 189.7,
            "rrtstar": 576.6,
            "rrt": 599.4,
        },
        "102816036": {
            "nbs": 599.5,
            "dbs": 599.6,
            "spt": 599.6,
            "tsp": 214.6,
            "rrtstar": 557.5,
            "rrt": 599.6,
        },
        "102815835": {
            "nbs": 299.8,
            "dbs": 299.8,
            "spt": 299.8,
            "tsp": 223.6,
            "rrtstar": 299.6,
            "rrt": 299.6,
        },
        "102816852": {
            "nbs": 299.8,
            "dbs": 299.8,
            "spt": 299.8,
            "tsp": 182.7,
            "rrtstar": 299.6,
            "rrt": 299.7,
        },
        "102816216": {
            "nbs": 299.8,
            "dbs": 299.8,
            "spt": 299.8,
            "tsp": 189.6,
            "rrtstar": 299.6,
            "rrt": 299.7,
        },
    },
    "recon": {
        "102344280": {
            "nbs": 599.7,
            "dbs": 599.6,
            "spt": 599.7,
            "tsp": 556.9,
            "rrtstar": 599.5,
            "rrt": 599.4,
        },
        "102344049": {
            "nbs": 599.6,
            "dbs": 599.6,
            "spt": 599.5,
            "tsp": 595.3,
            "rrtstar": 599.4,
            "rrt": 599.5,
        },
        "102816009": {
            "nbs": 599.7,
            "dbs": 599.6,
            "spt": 599.6,
            "tsp": 557.1,
            "rrtstar": 599.4,
            "rrt": 599.4,
        },
        "103997445": {
            "nbs": 599.6,
            "dbs": 599.6,
            "spt": 599.6,
            "tsp": 503.0,
            "rrtstar": 598.3,
            "rrt": 599.5,
        },
        "102816036": {
            "nbs": 599.6,
            "dbs": 599.7,
            "spt": 599.7,
            "tsp": 538.9,
            "rrtstar": 599.4,
            "rrt": 599.6,
        },
        "102815835": {
            "nbs": 299.8,
            "dbs": 299.8,
            "spt": 299.8,
            "tsp": 299.7,
            "rrtstar": 299.6,
            "rrt": 299.6,
        },
        "102816852": {
            "nbs": 299.7,
            "dbs": 299.9,
            "spt": 299.7,
            "tsp": 299.7,
            "rrtstar": 299.5,
            "rrt": 299.5,
        },
        "102816216": {
            "nbs": 299.8,
            "dbs": 299.8,
            "spt": 299.8,
            "tsp": 299.7,
            "rrtstar": 299.5,
            "rrt": 299.6,
        },
    },
    "explore": {
        "102344280": {
            "nbs": 599.6,
            "dbs": 599.7,
            "spt": 599.6,
            "tsp": 498.3,
            "rrtstar": 572.2,
            "rrt": 599.4,
        },
        "102344049": {
            "nbs": 599.6,
            "dbs": 599.6,
            "spt": 599.5,
            "tsp": 573.1,
            "rrtstar": 577.7,
            "rrt": 599.5,
        },
        "102816009": {
            "nbs": 599.7,
            "dbs": 599.6,
            "spt": 599.5,
            "tsp": 511.9,
            "rrtstar": 582.7,
            "rrt": 599.3,
        },
        "103997445": {
            "nbs": 599.7,
            "dbs": 599.7,
            "spt": 599.6,
            "tsp": 469.9,
            "rrtstar": 498.3,
            "rrt": 599.5,
        },
        "102816036": {
            "nbs": 599.7,
            "dbs": 599.5,
            "spt": 599.6,
            "tsp": 545.6,
            "rrtstar": 590.1,
            "rrt": 599.4,
        },
        "102815835": {
            "nbs": 299.8,
            "dbs": 299.9,
            "spt": 299.8,
            "tsp": 299.7,
            "rrtstar": 299.5,
            "rrt": 299.5,
        },
        "102816852": {
            "nbs": 299.6,
            "dbs": 299.8,
            "spt": 299.7,
            "tsp": 299.7,
            "rrtstar": 299.8,
            "rrt": 299.7,
        },
        "102816216": {
            "nbs": 299.9,
            "dbs": 299.7,
            "spt": 299.8,
            "tsp": 299.5,
            "rrtstar": 299.7,
            "rrt": 299.6,
        },
    },
}
SCENES = LARGE_SCENES + SMALL_SCENES


def budget_of(scene):
    """The protocol's cost budget [s] for the scene."""
    return float(LARGE_BUDGET if scene in LARGE_SCENES else SMALL_BUDGET)


def run_budget(run_dir, scene):
    """The run's own finite cost_budget from episode_summary.yaml, else the protocol's."""
    budget = (read_episode_summary(run_dir) or {}).get("cost_budget")
    if isinstance(budget, float) and math.isfinite(budget) and budget > 0.0:
        return budget
    return budget_of(scene)


def gain_at_budget(path, budget):
    """(gain, cost) of the point whose cost is nearest the budget, or None.

    The points are the log's rows (TotalCost) and, after them, the episode's end from the
    run's episode_summary.yaml when it has one.
    """
    with open(path, newline="") as handle:
        rows = [r for r in csv.DictReader(handle) if r.get(GAIN)]
    points = []
    for row in rows:
        try:
            points.append((float(row[GAIN]), float(row.get("TotalCost") or 0.0)))
        except ValueError:
            continue
    summary = read_episode_summary(os.path.dirname(path)) or {}
    end_gain = (summary.get("real_accumulated_gain") or {}).get(GAIN.split(".")[0])
    end_cost = summary.get("total_cost")
    if isinstance(end_gain, float) and isinstance(end_cost, float):
        points.append((end_gain, end_cost))
    best = None
    for gain, cost in points:
        if best is None or abs(cost - budget) < abs(best[1] - budget):
            best = (gain, cost)
    return best


def collect(data_root, task, scene, planner):
    folder = os.path.join(data_root, task, "HSSD", scene, planner)
    out = []
    if not os.path.isdir(folder):
        return out
    for stamp in sorted(os.listdir(folder)):
        path = os.path.join(folder, stamp, "performance_log.csv")
        if not os.path.isfile(path):
            continue
        try:
            found = gain_at_budget(path, run_budget(os.path.join(folder, stamp), scene))
        except (OSError, csv.Error) as exc:
            print("  [skip] %s: %s" % (path, exc), file=sys.stderr)
            continue
        # A run truncated by the paper's wall cap is a result, not a failure: the paper's
        # own means include them, so nothing is filtered on TotalCost here.
        if found:
            out.append(found)
    return out


def num(value, digits=1, width=7):
    if value is None or (isinstance(value, float) and math.isnan(value)):
        return "%*s" % (width, "-")
    return "%*.*f" % (width, digits, value)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=HelpFormatter)
    parser.add_argument(
        "--data-root",
        required=True,
        default=argparse.SUPPRESS,  # required: no default to show
        help="paper_sweep.py --data-root, holding <task>/HSSD/...",
    )
    parser.add_argument("--output", help="also write the report to this Markdown file")
    parser.add_argument(
        "--tasks", nargs="+", default=["points", "recon", "explore"], help="tasks to report"
    )
    parser.add_argument("--scenes", nargs="+", default=SCENES, help="HSSD scene ids to report")
    parser.add_argument("--planners", nargs="+", default=PLANNERS, help="planners to report")
    args = parser.parse_args(argv)

    lines = [
        "# open_apero vs the paper -- active perception",
        "",
        "Data root: `%s`" % args.data_root,
        "",
        "Gain is read at the point whose cost is nearest the run's cost budget "
        "(its `episode_summary.yaml` `cost_budget`, else %d s for large scenes and %d s for "
        "small ones): a `performance_log.csv` row, or the episode's "
        "end from `episode_summary.yaml`. Runs the wall cap truncated are kept: "
        "the paper's means include them too, which is why `cost` is far below the "
        "budget for TSP (every point-collection cell, most large-scene reconstruction and "
        "exploration cells) and for RRT* (large-scene point collection, most large-scene "
        "exploration cells). "
        "No unit conversion." % (LARGE_BUDGET, SMALL_BUDGET),
        "",
    ]

    normalized = {task: {p: [] for p in args.planners} for task in args.tasks}
    paper_normalized = {task: {p: [] for p in args.planners} for task in args.tasks}

    for task in args.tasks:
        lines += [
            "## %s" % TASK_LABEL.get(task, task),
            "",
            "| scene | GT | planner | n | mean | std | paper | ratio | "
            "norm (ours) | norm (paper) | cost (ours) | cost (paper) |",
            "|---|---|---|---|---|---|---|---|---|---|---|---|",
        ]
        for scene in args.scenes:
            ground_truth, paper_means = PAPER[task][scene]
            for planner in args.planners:
                runs = collect(args.data_root, task, scene, planner)
                gains = [r[0] for r in runs]
                n = len(gains)
                mean = statistics.mean(gains) if n else None
                std = statistics.stdev(gains) if n > 1 else (0.0 if n == 1 else None)
                paper_mean = paper_means[planner]
                ratio = mean / paper_mean if n else None
                norm = mean / ground_truth if n else None
                paper_norm = paper_mean / ground_truth
                if n:
                    normalized[task][planner].append(norm)
                paper_normalized[task][planner].append(paper_norm)
                cost = statistics.mean([r[1] for r in runs]) if n else None
                lines.append(
                    "| %s | %.1f | %s | %d | %s | %s | %s | %s | %s | %s | %s | %s |"
                    % (
                        scene,
                        ground_truth,
                        planner.upper(),
                        n,
                        num(mean).strip(),
                        num(std).strip(),
                        num(paper_mean).strip(),
                        num(ratio, 2).strip(),
                        num(norm, 3).strip(),
                        num(paper_norm, 3).strip(),
                        num(cost).strip(),
                        num(PAPER_COST[task][scene][planner]).strip(),
                    )
                )
        lines.append("")

    lines += [
        "## Mean normalized gain per planner",
        "",
        "| task | " + " | ".join(p.upper() for p in args.planners) + " |",
        "|---" * (len(args.planners) + 1) + "|",
    ]
    for task in args.tasks:
        ours = [
            "%s (%s)"
            % (
                (
                    num(statistics.mean(normalized[task][p]), 3).strip()
                    if normalized[task][p]
                    else "-"
                ),
                num(statistics.mean(paper_normalized[task][p]), 3).strip(),
            )
            for p in args.planners
        ]
        lines.append("| %s | %s |" % (task, " | ".join(ours)))
    lines += [
        "",
        "Cells are `ours (paper)`; ours averages only the scenes with data.",
        "",
        "## Ranking",
        "",
    ]
    for task in args.tasks:
        ours_rank = sorted(
            (p for p in args.planners if normalized[task][p]),
            key=lambda p: -statistics.mean(normalized[task][p]),
        )
        paper_rank = sorted(
            args.planners, key=lambda p: -statistics.mean(paper_normalized[task][p])
        )
        lines.append(
            "- **%s** ours: %s" % (task, " > ".join(p.upper() for p in ours_rank) or "(no data)")
        )
        lines.append("  paper: %s" % " > ".join(p.upper() for p in paper_rank))
    lines.append("")

    text = "\n".join(lines)
    print(text)
    if args.output:
        with open(args.output, "w") as handle:
            handle.write(text)
        print("Wrote %s" % os.path.abspath(args.output))
    return 0


if __name__ == "__main__":
    sys.exit(main())
