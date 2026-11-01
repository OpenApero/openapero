"""Re-run the paper's graph benchmark on the stored graph instances and check it reproduces.

The tool copies only each dataset's ``graph_N/graph.txt`` into a scratch tree, runs
``benchmark_node`` there over the full sweep with the scratch tree as its ``results_folder``, and
checks the fresh results against the dataset's ``reference.json`` (written by
``benchmark_reference.py``), which it never writes to. Matching digests of the deterministic
columns give ``REPRODUCED (exact)``; differing digests with every paper-figure final-gain range
cell equal within 1e-9 give ``REPRODUCED (figure-level)``; anything else, including a different
result-file count or a digest group on one side only, is ``MISMATCH``, listing the differences.

It prints the final-gain range tables of the reference and the rerun. A dataset without a
``reference.json`` is refused before anything runs. The regenerated results stay in the scratch
tree, which the range plot scripts accept as a dataset path.

Reproduction is promised on the tested toolchain, the Docker image built from
``docker/Dockerfile``: the planners break remaining ties by enumeration order, which another
compiler or standard library can change. Rebuild the package before trusting a mismatch -- a
``benchmark_node`` older than ``libopen_apero_core.so`` can also disagree through a stale vtable
rather than through planning.

Usage (with the catkin workspace sourced)::

    rosrun open_apero_graph_benchmark verify_benchmark_results.py <dataset> [<dataset> ...]
    rosrun open_apero_graph_benchmark verify_benchmark_results.py --all --parallel 4
    rosrun open_apero_graph_benchmark verify_benchmark_results.py --all --detach
    # Compare an existing scratch tree without rerunning:
    rosrun open_apero_graph_benchmark verify_benchmark_results.py <dataset> --skip-run

Exit status is 0 when every requested dataset reproduced, 1 otherwise.
"""

import argparse
import os
import re
import shutil
import signal
import socket
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from pathlib import Path

from open_apero_graph_benchmark.benchmark_reference import (
    REFERENCE_NAME,
    TIME_DIGITS,
    build_reference,
    compare_references,
    dataset_type,
    load_reference,
    nested_ranges,
    range_table,
)
from open_apero_graph_benchmark.paths import INPUT_ROOT, PACKAGE_NAME, PACKAGE_ROOT, RESULTS_ROOT

DEFAULT_WORK_DIR = RESULTS_ROOT / "verify"

# A detached dataset runs this module under ``python3 -m`` in a child process, which inherits the
# sourced workspace's PYTHONPATH; the hint printed after detaching names the installed tool.
MODULE_NAME = "open_apero_graph_benchmark.verify_benchmark_results"
TOOL_NAME = "verify_benchmark_results.py"

# run_benchmark.launch's perception_radius for a dataset that is not op_<R>m_. It spans every
# shipped ap_ extent (the 50x50 diagonal is 70.7), so the whole graph is visible.
AP_PERCEPTION_RADIUS = 100.0

# Seeds the planners' tie-break RNGs. 0 is run_benchmark.launch's default and what data/ was written
# with; a negative value is nondeterministic and cannot reproduce anything.
DEFAULT_RANDOM_SEED = 0

DEFAULT_MASTER_PORT_BASE = 11700

# run_benchmark.launch marks benchmark_node required, so roslaunch exits with it; the run is also
# over as soon as the node process is gone, whichever comes first and whatever the log level.
NODE_NAME = "benchmark_node"

POLL_SECONDS = 2.0

NODE_START_TIMEOUT_SECONDS = 300.0

SHUTDOWN_GRACE_SECONDS = 30.0

DATASET_EXTENT_PATTERN = re.compile(r"(\d+)x(\d+)")

OP_RADIUS_PATTERN = re.compile(r"^op_(\d+(?:\.\d+)?)m_")

COST_PATTERN = re.compile(r"_cost_(\d+(?:\.\d+)?)$")


def derive_perception_radius(dataset_name):
    """Return the perception radius a dataset's name calls for.

    ``op_<R>m_`` means exactly R. ``ap_`` means a-priori visibility; its name must encode a
    ``<W>x<H>`` extent, and the radius is run_benchmark.launch's ``AP_PERCEPTION_RADIUS``.
    """
    kind = dataset_type(dataset_name)
    if kind == "op":
        match = OP_RADIUS_PATTERN.match(dataset_name)
        if not match:
            raise ValueError(
                "online-perception dataset name must encode its radius as op_<R>m_: {}".format(
                    dataset_name
                )
            )
        return float(match.group(1))

    match = DATASET_EXTENT_PATTERN.search(dataset_name)
    if not match:
        raise ValueError(
            "a-priori dataset name must encode a <W>x<H> extent: {}".format(dataset_name)
        )
    return AP_PERCEPTION_RADIUS


def derive_cost_budget(dataset_name):
    """Return the cost budget encoded in a dataset folder name's ``_cost_<N>`` suffix."""
    match = COST_PATTERN.search(dataset_name)
    if not match:
        raise ValueError("dataset name must end in _cost_<budget>: {}".format(dataset_name))
    return float(match.group(1))


def derive_dataset_parameters(dataset_name):
    """Return ``(perception_radius, cost_budget)`` for a dataset folder name."""
    return derive_perception_radius(dataset_name), derive_cost_budget(dataset_name)


def resolve_dataset(dataset):
    """Resolve a dataset name below ``data/`` or an existing path to its source directory."""
    candidate = Path(dataset)
    if candidate.is_dir():
        return candidate.resolve()
    below_data = INPUT_ROOT / candidate
    if below_data.is_dir():
        return below_data.resolve()
    raise ValueError("no such dataset directory: {}".format(dataset))


def all_dataset_names():
    """Return every ``ap_``/``op_`` dataset folder name under the package data directory."""
    return sorted(
        entry.name
        for entry in INPUT_ROOT.iterdir()
        if entry.is_dir() and (entry.name.startswith("ap_") or entry.name.startswith("op_"))
    )


def graph_instance_folders(source_dataset):
    """Return the ``graph_*`` subfolders of a dataset that hold a ``graph.txt``."""
    return sorted(
        folder
        for folder in source_dataset.iterdir()
        if folder.is_dir() and (folder / "graph.txt").is_file()
    )


def require_reference(source_dataset):
    """Raise RuntimeError if the dataset has no ``reference.json`` to check a rerun against."""
    if not (source_dataset / REFERENCE_NAME).is_file():
        raise RuntimeError(
            "no {} under {}. Each dataset under data/ ships one; restore it (git checkout -- "
            "data/), or write one from result files with scripts/benchmark_reference.py.".format(
                REFERENCE_NAME, source_dataset
            )
        )


def create_scratch_dataset(source_dataset, scratch_dataset):
    """Refresh ``scratch_dataset`` so it holds each instance's ``graph.txt`` and nothing else.

    Results must not be inherited from a previous attempt, so any existing graph folder is
    removed first; benchmark_node writes its output back into this tree.
    """
    instances = graph_instance_folders(source_dataset)
    if not instances:
        raise ValueError("no graph_*/graph.txt under {}".format(source_dataset))

    scratch_dataset.mkdir(parents=True, exist_ok=True)
    for existing in scratch_dataset.iterdir():
        if existing.is_dir():
            shutil.rmtree(existing)

    for instance in instances:
        target = scratch_dataset / instance.name
        target.mkdir()
        shutil.copyfile(instance / "graph.txt", target / "graph.txt")
    return [instance.name for instance in instances]


def build_roslaunch_command(scratch_dataset, perception_radius, cost_budget, seed, log_level):
    """Return the roslaunch argv running the paper sweep over a scratch dataset."""
    return [
        "roslaunch",
        "open_apero_graph_benchmark",
        "run_benchmark.launch",
        "graph_type_folder:={}".format(scratch_dataset),
        "results_folder:={}".format(scratch_dataset),
        "perception_radius:={}".format(perception_radius),
        "cost_budget:={}".format(cost_budget),
        "replan_type:=try_every_option",
        "objective:=try_every_option",
        "random_seed:={}".format(seed),
        "log_level:={}".format(log_level),
        "visualize:=false",
    ]


def require_free_master_port(port):
    """Raise if something already listens on ``port``.

    roslaunch joins a master that is already running instead of starting its own, and a second
    benchmark_node registering under the same name makes that master shut the first one down.
    """
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.settimeout(0.2)
        if probe.connect_ex(("127.0.0.1", port)) == 0:
            raise RuntimeError(
                "port {} is already in use; pass another --master-port-base".format(port)
            )


def _child_node_pids(parent_pid):
    """Return the pids of ``benchmark_node`` processes whose parent is ``parent_pid``."""
    pids = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        try:
            with open("/proc/{}/comm".format(entry)) as comm_file:
                if comm_file.read().strip() != NODE_NAME:
                    continue
            with open("/proc/{}/stat".format(entry)) as stat_file:
                # The comm field is parenthesized and may itself contain spaces, so split on the
                # last ") " rather than on whitespace.
                after_comm = stat_file.read().rsplit(") ", 1)[1]
        except (OSError, IndexError):
            continue
        if int(after_comm.split()[1]) == parent_pid:
            pids.append(int(entry))
    return pids


def wait_for_node_then_stop(process):
    """Wait until the launched ``benchmark_node`` has exited, then shut roslaunch down.

    Precondition: ``process`` was started in its own session, so its pid is the process-group id.
    Returns roslaunch's exit status if it exits by itself (benchmark_node is required); otherwise
    interrupts it once the node is gone and returns 0.
    """
    deadline = time.time() + NODE_START_TIMEOUT_SECONDS
    started = False
    while True:
        if process.poll() is not None:
            return process.returncode
        if _child_node_pids(process.pid):
            started = True
        elif started:
            break
        elif time.time() > deadline:
            _stop_process_group(process)
            raise RuntimeError(
                "benchmark_node did not start within {:.0f}s".format(NODE_START_TIMEOUT_SECONDS)
            )
        time.sleep(POLL_SECONDS)

    _stop_process_group(process)
    return 0


def _stop_process_group(process):
    """Interrupt the launcher's whole process group, escalating if it does not go."""
    try:
        os.killpg(process.pid, signal.SIGINT)
    except (ProcessLookupError, PermissionError):
        return
    try:
        process.wait(timeout=SHUTDOWN_GRACE_SECONDS)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            pass
        process.wait()


def run_benchmark(scratch_dataset, perception_radius, cost_budget, seed, log_level, log_path, port):
    """Run one sweep under its own ROS master and return the elapsed wall-clock seconds."""
    command = build_roslaunch_command(
        scratch_dataset, perception_radius, cost_budget, seed, log_level
    )
    environment = dict(os.environ)
    environment["ROS_MASTER_URI"] = "http://localhost:{}".format(port)

    started = time.time()
    with open(log_path, "w") as log_file:
        process = subprocess.Popen(
            command,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            env=environment,
            cwd=str(PACKAGE_ROOT),
            start_new_session=True,
        )
        status = wait_for_node_then_stop(process)
    if status not in (0, -signal.SIGINT):
        raise RuntimeError("roslaunch exited with status {}; see {}".format(status, log_path))
    return time.time() - started


def largest_range_difference(baseline, candidate):
    """Largest absolute difference over the baseline's range cells; inf if one is missing."""
    largest = 0.0
    for key, per_algorithm in baseline.items():
        for algorithm, (minimum, maximum) in per_algorithm.items():
            if key not in candidate or algorithm not in candidate[key]:
                return float("inf")
            other_minimum, other_maximum = candidate[key][algorithm]
            largest = max(largest, abs(minimum - other_minimum), abs(maximum - other_maximum))
    return largest


REFERENCE_VERDICTS = {
    "exact": "REPRODUCED (exact)",
    "figure-level": "REPRODUCED (figure-level)",
    "mismatch": "MISMATCH",
}


def compare_with_reference(source_dataset, scratch_dataset):
    """Check a rerun's scratch tree against the source dataset's reference.json.

    Returns ``(verdict, failures, notes, compared, range_tables_text, max_range_difference)``.
    """
    reference = load_reference(source_dataset / REFERENCE_NAME)
    candidate = build_reference(scratch_dataset, with_ranges=False)
    tables = ""
    try:
        candidate["final_gain_ranges"] = range_table(scratch_dataset, "gain")
        candidate["computation_time_ranges"] = range_table(
            scratch_dataset, "computation_time", TIME_DIGITS
        )
    except ImportError as error:
        tables = "range tables unavailable: {}".format(error)
    except (OSError, ValueError, IndexError) as error:
        tables = "range tables failed: {}".format(error)

    kind, failures, notes = compare_references(reference, candidate)
    difference = float("nan")
    if reference.get("final_gain_ranges") is not None and candidate["final_gain_ranges"]:
        baseline = nested_ranges(reference["final_gain_ranges"])
        rerun = nested_ranges(candidate["final_gain_ranges"])
        tables = format_range_tables(baseline, rerun)
        difference = largest_range_difference(baseline, rerun)
    return REFERENCE_VERDICTS[kind], failures, notes, candidate["result_files"], tables, difference


def format_range_tables(baseline, candidate):
    """Render the two range tables side by side, one line per (replan, objective, algorithm)."""
    lines = [
        "{:<12} {:<14} {:<4} {:>12} {:>12} {:>12} {:>12} {:>10}".format(
            "replan",
            "objective",
            "algo",
            "paper_min",
            "paper_max",
            "rerun_min",
            "rerun_max",
            "max_abs_d",
        )
    ]
    for key in baseline:
        replan_type, objective = key
        for algorithm, (minimum, maximum) in baseline[key].items():
            other = candidate.get(key, {}).get(algorithm)
            if other is None:
                lines.append(
                    "{:<12} {:<14} {:<4} {:>12.3f} {:>12.3f} {:>12} {:>12} {:>10}".format(
                        replan_type, objective, algorithm, minimum, maximum, "-", "-", "MISSING"
                    )
                )
                continue
            difference = max(abs(minimum - other[0]), abs(maximum - other[1]))
            lines.append(
                "{:<12} {:<14} {:<4} {:>12.3f} {:>12.3f} {:>12.3f} {:>12.3f} {:>10.3e}".format(
                    replan_type,
                    objective,
                    algorithm,
                    minimum,
                    maximum,
                    other[0],
                    other[1],
                    difference,
                )
            )
    return "\n".join(lines)


def write_report(report_path, record):
    """Write one dataset's verdict, parameters, timings and differences as Markdown."""
    lines = [
        "# Benchmark reproduction: {}".format(record["dataset"]),
        "",
        "- verdict: **{}**".format(record["verdict"]),
        "- finished: {}".format(record["finished"]),
        "- source tree: `{}`".format(record["source"]),
        "- scratch tree: `{}`".format(record["scratch"]),
        "- perception_radius: {}".format(record["perception_radius"]),
        "- cost_budget: {}".format(record["cost_budget"]),
        "- random_seed: {}".format(record["seed"]),
        "- graph instances: {}".format(record["instances"]),
        "- result files compared: {}".format(record["compared"]),
        "- run wall-clock: {}".format(
            "skipped" if record["seconds"] is None else "{:.1f}s".format(record["seconds"])
        ),
        "",
        "## Command",
        "",
        "```",
        " ".join(record["command"]),
        "```",
        "",
        "## Differences",
        "",
    ]
    if record["failures"]:
        lines.append("```")
        lines.extend(record["failures"])
        lines.append("```")
    elif record["verdict"] == REFERENCE_VERDICTS["exact"]:
        lines.append("None: every digest matches {}.".format(REFERENCE_NAME))
    else:
        lines.append("No final-gain range cell differs; the digests in the notes do.")
    if record["notes"]:
        lines.extend(["", "## Notes (not failures)", "", "```"] + record["notes"] + ["```"])
    if record["range_tables"]:
        lines.extend(
            [
                "",
                "## Paper-figure final-gain ranges",
                "",
                "Maximum absolute difference: {:.6g}".format(record["range_difference"]),
                "",
                "```",
                record["range_tables"],
                "```",
            ]
        )
    lines.append("")
    report_path.write_text("\n".join(lines))


def verify_dataset(dataset, work_dir, options, port):
    """Run, compare and report one dataset, leaving a ``status`` file however it ends."""
    try:
        return _verify_dataset(dataset, work_dir, options, port)
    except Exception as error:  # noqa: BLE001 - the status file must record a crash too
        try:
            status_path = work_dir / resolve_dataset(dataset).name / "status"
            status_path.parent.mkdir(parents=True, exist_ok=True)
            status_path.write_text(
                "ERROR {} {}\n".format(datetime.now().isoformat(timespec="seconds"), error)
            )
        except OSError:
            pass
        raise


def _verify_dataset(dataset, work_dir, options, port):
    """Run, compare and report one dataset. Returns the record dict the report is built from."""
    source = resolve_dataset(dataset)
    name = source.name
    perception_radius, cost_budget = derive_dataset_parameters(name)
    if options.perception_radius is not None:
        perception_radius = options.perception_radius
    if options.cost_budget is not None:
        cost_budget = options.cost_budget

    dataset_work_dir = work_dir / name
    scratch = dataset_work_dir / name
    log_path = dataset_work_dir / "roslaunch.log"
    command = build_roslaunch_command(
        scratch, perception_radius, cost_budget, options.seed, options.log_level
    )

    print(
        "[{}] perception_radius={} cost_budget={} seed={} port={}".format(
            name, perception_radius, cost_budget, options.seed, port
        )
    )

    if options.dry_run:
        print("[{}] scratch tree: {}".format(name, scratch))
        print("[{}] {}".format(name, " ".join(command)))
        return None

    require_reference(source)
    dataset_work_dir.mkdir(parents=True, exist_ok=True)
    seconds = None
    if options.skip_run:
        if not scratch.is_dir():
            raise RuntimeError("no scratch tree to compare at {}".format(scratch))
        instances = [folder.name for folder in graph_instance_folders(scratch)]
    else:
        require_free_master_port(port)
        instances = create_scratch_dataset(source, scratch)
        seconds = run_benchmark(
            scratch, perception_radius, cost_budget, options.seed, options.log_level, log_path, port
        )

    verdict, failures, notes, compared, tables, range_difference = compare_with_reference(
        source, scratch
    )

    record = {
        "dataset": name,
        "source": source,
        "scratch": scratch,
        "perception_radius": perception_radius,
        "cost_budget": cost_budget,
        "seed": options.seed,
        "instances": len(instances),
        "compared": compared,
        "seconds": seconds,
        "command": command,
        "failures": failures,
        "notes": notes,
        "range_tables": tables,
        "range_difference": range_difference,
        "verdict": verdict,
        "finished": datetime.now().isoformat(timespec="seconds"),
    }
    write_report(dataset_work_dir / "report.md", record)
    (dataset_work_dir / "status").write_text(
        "{} {} files={} {}\n".format(
            record["verdict"],
            record["finished"],
            compared,
            "" if seconds is None else "seconds={:.1f}".format(seconds),
        )
    )
    return record


def print_record(record):
    """Print one dataset's verdict, its first few differences and its range tables."""
    print("\n=== {} : {} ===".format(record["dataset"], record["verdict"]))
    print("result files compared: {}".format(record["compared"]))
    if record["seconds"] is not None:
        print("run wall-clock: {:.1f}s".format(record["seconds"]))
    for failure in record["failures"][:20]:
        print("FAIL: {}".format(failure))
    if len(record["failures"]) > 20:
        print("... {} further difference(s); see report.md".format(len(record["failures"]) - 20))
    for note in record["notes"][:20]:
        print("NOTE: {}".format(note))
    if record["range_tables"]:
        print(record["range_tables"])
        print("max abs range difference: {:.6g}".format(record["range_difference"]))
    print("report: {}".format(record["scratch"].parent / "report.md"))
    print("results: {} (a dataset path for the range plot scripts)".format(record["scratch"]))


def build_detach_command(dataset, work_dir, options, port):
    """Return the argv that runs this tool for one dataset in a detached process."""
    command = [
        sys.executable,
        "-m",
        MODULE_NAME,
        str(dataset),
        "--work-dir",
        str(work_dir),
        "--master-port-base",
        str(port),
        "--seed",
        str(options.seed),
        "--log-level",
        options.log_level,
    ]
    if options.perception_radius is not None:
        command += ["--perception-radius", str(options.perception_radius)]
    if options.cost_budget is not None:
        command += ["--cost-budget", str(options.cost_budget)]
    return command


def detach_dataset(dataset, work_dir, options, port):
    """Spawn a detached copy of this tool for one dataset and return its pid."""
    require_reference(resolve_dataset(dataset))
    require_free_master_port(port)
    dataset_work_dir = work_dir / resolve_dataset(dataset).name
    dataset_work_dir.mkdir(parents=True, exist_ok=True)
    with open(dataset_work_dir / "verify.log", "w") as log_file:
        process = subprocess.Popen(
            build_detach_command(dataset, work_dir, options, port),
            stdout=log_file,
            stderr=subprocess.STDOUT,
            cwd=str(PACKAGE_ROOT),
            start_new_session=True,
        )
    return process.pid


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Re-run the graph benchmark on stored graph instances and check the fresh "
        "results against each dataset's reference.json."
    )
    parser.add_argument("datasets", nargs="*", help="Dataset names below data/, or paths.")
    parser.add_argument("--all", action="store_true", help="Verify every ap_/op_ dataset.")
    parser.add_argument(
        "--work-dir",
        type=Path,
        default=DEFAULT_WORK_DIR,
        help="Scratch root; each dataset gets <work-dir>/<dataset>/ (default: %(default)s).",
    )
    parser.add_argument(
        "--perception-radius",
        type=float,
        default=None,
        help="Override the radius derived from the dataset name.",
    )
    parser.add_argument(
        "--cost-budget",
        type=float,
        default=None,
        help="Override the cost budget derived from the dataset name.",
    )
    parser.add_argument(
        "--seed", type=int, default=DEFAULT_RANDOM_SEED, help="random_seed (default: %(default)s)."
    )
    parser.add_argument(
        "--log-level",
        default="info",
        choices=("debug", "info", "warn", "error", "fatal"),
        help="run_benchmark.launch log_level (default: %(default)s).",
    )
    parser.add_argument(
        "--parallel",
        type=int,
        default=1,
        help="Datasets to run concurrently (default: %(default)s).",
    )
    parser.add_argument(
        "--master-port-base",
        type=int,
        default=DEFAULT_MASTER_PORT_BASE,
        help="First ROS master port; each dataset gets its own (default: %(default)s).",
    )
    parser.add_argument("--dry-run", action="store_true", help="Print the commands and stop.")
    parser.add_argument(
        "--skip-run", action="store_true", help="Compare an existing scratch tree without running."
    )
    parser.add_argument(
        "--detach",
        action="store_true",
        help="Launch each dataset in its own detached process and return immediately.",
    )
    args = parser.parse_args(argv)

    datasets = list(args.datasets)
    if args.all:
        datasets = all_dataset_names()
    if not datasets:
        parser.error("name at least one dataset, or pass --all")
    if args.parallel < 1:
        parser.error("--parallel must be at least 1")
    if args.detach and args.skip_run:
        parser.error("--detach and --skip-run are mutually exclusive")

    work_dir = args.work_dir.resolve()
    work_dir.mkdir(parents=True, exist_ok=True)

    if args.detach:
        for index, dataset in enumerate(datasets):
            port = args.master_port_base + index
            name = resolve_dataset(dataset).name
            if args.dry_run:
                print(
                    "[{}] would detach: {}".format(
                        name, " ".join(build_detach_command(dataset, work_dir, args, port))
                    )
                )
                continue
            pid = detach_dataset(dataset, work_dir, args, port)
            print("{}: pid {}  log {}".format(name, pid, work_dir / name / "verify.log"))
        if args.dry_run:
            return 0
        print(
            "\nCheck later with:\n  rosrun {} {} {} --work-dir {} --skip-run".format(
                PACKAGE_NAME,
                TOOL_NAME,
                " ".join(resolve_dataset(dataset).name for dataset in datasets),
                work_dir,
            )
        )
        return 0

    records = []
    with ThreadPoolExecutor(max_workers=args.parallel) as pool:
        futures = [
            pool.submit(verify_dataset, dataset, work_dir, args, args.master_port_base + index)
            for index, dataset in enumerate(datasets)
        ]
        for dataset, future in zip(datasets, futures):
            try:
                record = future.result()
            except Exception as error:  # noqa: BLE001 - one dataset must not sink the rest
                print("ERROR: {}: {}".format(dataset, error))
                records.append(None)
                continue
            if record is not None:
                records.append(record)
                print_record(record)

    if args.dry_run:
        return 0

    failed = [
        record
        for record in records
        if record is None or not record["verdict"].startswith("REPRODUCED")
    ]
    print("\n{}/{} dataset(s) reproduced.".format(len(records) - len(failed), len(datasets)))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
