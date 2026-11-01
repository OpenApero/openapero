"""Performance-overview computation and rendering for ``evaluate_experiment.py``.

Kept separate from ``scripts/evaluate_experiment.py`` so the numbers can be
checked without a display: rendering writes to a path the caller chooses. The
CLI keeps the argument parsing, console output and the directory layout.

The stacked chart measures two things from opposite ends. The components in
:data:`STACK_BANDS` accumulate upward from zero, while the ROS band is measured
down from one; whatever gap is left between them is unattributed time, drawn as
the "Other" residual. That is why the residual needs no column of its own.

The layer columns read here carry the ``planning.`` prefix of the shipped planning-layer
instance name; a run whose planning layer has another name has no such columns.
"""

import collections

import numpy as np
from matplotlib import pyplot as plt

from open_apero_analysis import cpu_time

__all__ = [
    "PerformanceOverview",
    "STACK_BANDS",
    "compute_performance_overview",
    "render_performance_overview",
    "smoothed_cpu_usage",
]

# Bottom-to-top bands of the CPU-time chart: display label, source column and
# fill color. Order is load-bearing — it fixes both the stacking order and the
# legend order, so a single tuple keeps the two from drifting apart.
STACK_BANDS = (
    ("GenerateNode", "planning.GenerateNode", "#a1b400"),
    ("GenerateEdge", "planning.GenerateEdge", "#009000"),
    ("EvaluateNode", "planning.EvaluateNode", "#dc1000"),
    ("PlanPathTotal", "PlanPathTotal", "#ffb800"),
    ("GeneratorUpdate", "planning.GeneratorUpdate", "#ffff00"),
    ("StartNodeUpdate", "planning.StartNodeUpdate", "#8b4513"),
    ("EvaluatorUpdate", "planning.EvaluatorUpdate", "#00eebc"),
    ("Visualization", "Visualization", "#d800dd"),
)

RESIDUAL_LABEL = "Other"
RESIDUAL_COLOR = "#a3baff"
ROS_LABEL = "ROS"
ROS_COLOR = "#606060"
ROS_COLUMN = "RosCallbacks"

# Seconds of simulated time averaged into one CPU-usage sample.
CPU_AVERAGING_THRESHOLD = 2.0

PerformanceOverview = collections.namedtuple(
    "PerformanceOverview",
    [
        "step_time",  # x for the stacked chart (doubled for a step outline)
        "band_edges",  # cumulative upper edge per STACK_BANDS entry
        "ros_edge",  # lower edge of the ROS band, measured down from 1.0
        "shares",  # percent of accounted CPU time per STACK_BANDS entry
        "residual_share",  # percent unattributed to any band
        "ros_share",  # percent spent in ROS callbacks
        "cycle_time",  # x for the node-count and CPU-usage panels
        "total_nodes",
        "expansions",  # expansion rounds per cycle (planning.NumExpansions)
        "usage_time",  # x for the smoothed CPU-usage panel
        "total_cpu_usage",
        "planning_cpu_usage",
    ],
)


def smoothed_cpu_usage(replan_time, cpu_series, threshold=CPU_AVERAGING_THRESHOLD):
    """Average CPU series into buckets of at least ``threshold`` simulated seconds.

    Consecutive cycles are accumulated until their summed replan time reaches the
    threshold, then emitted as one sample of CPU seconds per simulated second.
    Short cycles would otherwise produce a spiky, unreadable trace.

    Args:
        replan_time: per-cycle simulated duration [s].
        cpu_series: sequence of equal-length per-cycle CPU-time arrays [s].

    Returns:
        ``(times, usages)`` where ``times`` are bucket end points and ``usages``
        holds one array per input series. Both are empty when there are fewer
        than two cycles, or when no bucket ever reaches the threshold.
    """
    replan_time = np.asarray(replan_time, dtype=float)
    series = [np.asarray(s, dtype=float) for s in cpu_series]

    times = np.array([])
    usages = [np.array([]) for _ in series]
    if replan_time.size == 0:
        return times, usages

    index = 0
    bucket_time = replan_time[0]
    elapsed = 0.0
    bucket_cpu = [s[0] for s in series]

    while index + 1 < replan_time.size:
        index += 1
        if bucket_time >= threshold:
            for j in range(len(series)):
                usages[j] = np.append(usages[j], bucket_cpu[j] / bucket_time)
            elapsed += bucket_time
            times = np.append(times, elapsed)
            bucket_time = replan_time[index]
            for j in range(len(series)):
                bucket_cpu[j] = series[j][index]
        else:
            bucket_time += replan_time[index]
            for j in range(len(series)):
                bucket_cpu[j] += series[j][index]

    return times, usages


def compute_performance_overview(columns, timer_columns):
    """Derive every series the performance overview draws.

    Args:
        columns: mapping of column name -> per-cycle values, as produced by
            :func:`open_apero_analysis.load_run`.
        timer_columns: columns whose row-wise sum is the accounted CPU time,
            from :func:`cpu_time.ros_cpu_timer_selection`.

    Returns:
        :class:`PerformanceOverview`.
    """
    replan_time = np.asarray(columns["ReplanTimeRos"], dtype=float)
    cycle_time = np.cumsum(replan_time)
    total = cpu_time.accounted_cpu_time_per_cycle(columns, timer_columns)

    band_values = [np.asarray(columns[column], dtype=float) for _, column, _ in STACK_BANDS]
    ros_values = np.asarray(columns[ROS_COLUMN], dtype=float)

    # Cumulative upper edges, so band i is drawn between edge i-1 and edge i.
    edges = []
    running = np.zeros_like(total)
    for values in band_values:
        running = running + np.divide(values, total)
        edges.append(running)
    ros_edge = 1.0 - np.divide(ros_values, total)

    grand_total = np.sum(total)
    shares = [np.sum(values) / grand_total * 100 for values in band_values]
    ros_share = np.sum(ros_values) / grand_total * 100

    # Subtract one share at a time rather than a summed total: floating-point
    # addition is not associative, so the order fixes the residual's rounding.
    residual_share = 100
    for share in shares:
        residual_share = residual_share - share
    residual_share = residual_share - ros_share

    # Double every sample so the filled areas read as a step function: each cycle
    # holds its value until the next one starts, rather than ramping between them.
    step_time = np.repeat(cycle_time, 2)
    step_time = np.concatenate((np.array([0]), step_time[:-1]))

    # The planning layer's own time: its root timers and its own planPath, not PlanPathTotal,
    # which also holds the other layers' searches.
    planning_cpu = (
        np.asarray(columns["planning.ExpandTime"], dtype=float)
        + np.asarray(columns["planning.UpdateTime"], dtype=float)
        + np.asarray(columns["planning.StartNodeUpdate"], dtype=float)
        + np.asarray(columns["planning.PlanPathTime"], dtype=float)
    )
    usage_time, usages = smoothed_cpu_usage(replan_time, [total, planning_cpu])

    return PerformanceOverview(
        step_time=step_time,
        band_edges=tuple(np.repeat(edge, 2) for edge in edges),
        ros_edge=np.repeat(ros_edge, 2),
        shares=tuple(shares),
        residual_share=residual_share,
        ros_share=ros_share,
        cycle_time=cycle_time,
        total_nodes=np.asarray(columns["planning.NumNodesAtSelection"], dtype=int),
        expansions=np.asarray(columns["planning.NumExpansions"], dtype=int),
        usage_time=usage_time,
        total_cpu_usage=usages[0],
        planning_cpu_usage=usages[1],
    )


def render_performance_overview(overview, save_path):
    """Draw the three-panel overview and write it to ``save_path``.

    Returns the figure, still current in pyplot's state, so a caller that wants
    to display it interactively can do so after its own logging.
    """
    figure = plt.figure()
    axes = [
        plt.subplot2grid((5, 1), (0, 0), rowspan=3),
        plt.subplot(5, 1, 4),
        plt.subplot(5, 1, 5),
    ]

    lower = np.zeros_like(overview.step_time)
    for (_, _, color), upper in zip(STACK_BANDS, overview.band_edges):
        axes[0].fill_between(overview.step_time, lower, upper, facecolor=color, alpha=0.5)
        lower = upper
    axes[0].fill_between(
        overview.step_time, lower, overview.ros_edge, facecolor=RESIDUAL_COLOR, alpha=0.5
    )
    axes[0].fill_between(overview.step_time, overview.ros_edge, 1, facecolor=ROS_COLOR, alpha=0.5)
    axes[0].set_xlim(left=0, right=overview.step_time[-1])
    axes[0].set_ylim(bottom=0, top=1)
    axes[0].set_title("Percentage of CPU Time Spent per Function")
    axes[0].set_ylabel("Percent [%]")

    axes[1].plot(overview.cycle_time, overview.total_nodes, "b-")
    axes[1].plot(overview.cycle_time, overview.expansions, "g-")
    axes[1].fill_between(
        overview.cycle_time, 0, overview.expansions, facecolor="#009000", alpha=0.3
    )
    axes[1].fill_between(
        overview.cycle_time,
        overview.expansions,
        overview.total_nodes,
        facecolor="#0000ff",
        alpha=0.3,
    )
    axes[1].set_xlim(left=0, right=overview.cycle_time[-1])
    axes[1].set_ylim(bottom=0)
    axes[1].set_title("Number of Nodes")
    axes[1].set_ylabel("Count [-]")
    axes[1].legend(["Total", "Expansions"], loc="upper left", fancybox=True)

    axes[2].plot(overview.usage_time, overview.total_cpu_usage, "k-")
    axes[2].plot(overview.usage_time, overview.planning_cpu_usage, linestyle="-", color="#5492E7")
    axes[2].plot(
        np.array([0, overview.usage_time[-1]]),
        np.array([1, 1]),
        linestyle="-",
        color="0.7",
        alpha=0.8,
    )
    axes[2].set_xlim(left=0, right=overview.usage_time[-1])
    axes[2].set_ylim(bottom=0)
    axes[2].set_ylabel("CPU Usage [cores]")
    axes[2].set_title("Planner Consumed CPU Time per Simulated Time")
    axes[2].set_xlabel("Simulated Time [s]")
    axes[2].legend(["Planner total", "Planning layer"], loc="upper left", fancybox=True)

    figure.set_size_inches(15, 15, forward=True)
    plt.tight_layout()

    box = axes[0].get_position()
    axes[0].set_position([box.x0, box.y0 + box.height * 0.16, box.width, box.height * 0.84])
    legend = [
        "({0:02.1f}%) {1}".format(share, label)
        for (label, _, _), share in zip(STACK_BANDS, overview.shares)
    ]
    legend.append("({0:02.1f}%) {1}".format(overview.residual_share, RESIDUAL_LABEL))
    legend.append("({0:02.1f}%) {1}".format(overview.ros_share, ROS_LABEL))
    axes[0].legend(legend, loc="upper center", bbox_to_anchor=(0.5, -0.04), ncol=6, fancybox=True)

    plt.savefig(save_path, dpi=300, format="png", bbox_inches="tight")
    return figure
