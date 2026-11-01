"""Run the active-perception paper sweep: scenes x planners x tasks x trials.

One roslaunch per run, N lanes in parallel, each lane owning its own rosmaster
port so the simulations never share a master. Jobs are pulled from a single
trial-major queue, so partial results already cover every cell.

Stdlib only, apart from open_apero_ros.episode_summary and open_apero_ros.paper_protocol (both
stdlib) -- it runs under the Habitat environment's interpreter and shells out to roslaunch.
Source the ROS/Habitat environment before starting it. Each lane sets its own ROS_MASTER_URI,
whatever the environment holds.

  paper_sweep.py --data-root DIR --trials 10 --parallel 3
  paper_sweep.py --data-root DIR --resume          # schedule only missing trials
"""

import argparse
import csv
import os
import queue
import re
import signal
import socket
import subprocess
import sys
import threading
import time
import traceback

from open_apero_ros.episode_summary import read_episode_summary
from open_apero_ros.paper_protocol import (
    LARGE_BUDGET,
    LARGE_SCENES,
    PAPER_WALL_CAPS,
    PLANNERS,
    SMALL_BUDGET,
    SMALL_SCENES,
    TASKS,
)

# scene set -> cost_budget [s of trajectory time]; main() applies --large-budget/--small-budget.
SCENE_BUDGET = {"large": LARGE_BUDGET, "small": SMALL_BUDGET}

# A run that spent its cost budget reaches at least this fraction of it; below that the
# wall cap cut it short, which is a valid paper-protocol outcome, not a failure.
COMPLETE_FRACTION = 0.98
EGL_FLAKE = re.compile(
    r"Unable to create windowless context|unable to find CUDA device|Simulation startup failed"
)

# Marks one run's processes. roslaunch starts the master and every node in a session of its
# own, out of reach of a process-group kill, but each inherits roslaunch's environment.
RUN_TAG = "OPEN_APERO_SWEEP_RUN"

RUNS_HEADER = [
    "trial",
    "scene",
    "task",
    "planner",
    "lane",
    "start",
    "end",
    "wall_s",
    "exit",
    "status",
    "cost_reached",
    "truncated",
    "run_dir",
    "log",
]

stop_dispatch = threading.Event()
io_lock = threading.Lock()


def scene_set(scene):
    return "large" if scene in LARGE_SCENES else "small"


def log_line(sweep_log, text):
    stamp = time.strftime("%Y-%m-%dT%H:%M:%S")
    with io_lock:
        with open(sweep_log, "a") as handle:
            handle.write("%s %s\n" % (stamp, text))
            handle.flush()
        print("%s %s" % (stamp, text), flush=True)


def append_run(runs_csv, row):
    """One O_APPEND write of a single short line is atomic across threads."""
    line = ",".join(str(x) for x in row) + "\n"
    with io_lock:
        with open(runs_csv, "a") as handle:
            handle.write(line)
            handle.flush()


def port_free(port):
    """Whether a new master could listen on `port`: rosmaster's own bind, without iproute2."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(("", port))
        except OSError:
            return False
    return True


def kill_tagged(tag):
    """SIGKILL every process whose environment holds RUN_TAG=`tag`. Returns how many there were."""
    needle = ("%s=%s" % (RUN_TAG, tag)).encode()
    killed = set()
    for _ in range(3):  # again, for children forked during the scan
        pids = []
        for entry in os.listdir("/proc"):
            if not entry.isdigit():
                continue
            try:
                with open(os.path.join("/proc", entry, "environ"), "rb") as handle:
                    if needle in handle.read().split(b"\0"):
                        pids.append(int(entry))
            except OSError:  # gone, or another user's
                continue
        if not pids:
            break
        for pid in pids:
            try:
                os.kill(pid, signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass
        killed.update(pids)
        time.sleep(0.2)
    return len(killed)


def read_final_cost(run_dir):
    """The run's spent budget, or None.

    episode_summary.yaml's total_cost when the run has one; otherwise, for a run that ended
    without writing it, the last TotalCost in performance_log.csv.
    """
    total_cost = (read_episode_summary(run_dir) or {}).get("total_cost")
    if isinstance(total_cost, float):
        return total_cost
    try:
        with open(os.path.join(run_dir, "performance_log.csv"), newline="") as handle:
            rows = [r for r in csv.DictReader(handle) if r.get("TotalCost")]
    except (OSError, csv.Error):
        return None
    if not rows:
        return None
    try:
        return float(rows[-1]["TotalCost"])
    except ValueError:
        return None


def run_dirs(data_root, task, scene, planner):
    folder = os.path.join(data_root, task, "HSSD", scene, planner)
    if not os.path.isdir(folder):
        return []
    return [
        os.path.join(folder, d)
        for d in sorted(os.listdir(folder))
        if os.path.isdir(os.path.join(folder, d))
    ]


def vetoed_run_dirs(runs_csv):
    """Run directories runs.csv marks failed; never counted as complete."""
    out = set()
    try:
        with open(runs_csv, newline="") as handle:
            for row in csv.DictReader(handle):
                if row.get("status") == "failed" and row.get("run_dir"):
                    out.add(row["run_dir"])
    except (OSError, csv.Error):
        pass
    return out


def count_complete(data_root, task, scene, planner, veto):
    """Complete = a run directory with a readable final cost, truncated by the cap or not."""
    n = 0
    for run_dir in run_dirs(data_root, task, scene, planner):
        if run_dir in veto:
            continue
        if read_final_cost(run_dir) is not None:
            n += 1
    return n


def newest_run_dir(data_root, task, scene, planner, since):
    """The run directory created by the launch that started at `since`."""
    best = None
    for run_dir in run_dirs(data_root, task, scene, planner):
        try:
            mtime = os.path.getmtime(run_dir)
        except OSError:
            continue
        if mtime >= since - 5 and (best is None or mtime > best[0]):
            best = (mtime, run_dir)
    return best[1] if best else ""


def cap_of(scene, task, override):
    return override if override else PAPER_WALL_CAPS[(scene, task)]


def build_jobs(args):
    """Trial-major queue, interleaving scene/task/planner so cells fill evenly."""
    jobs = []
    for trial in range(1, args.trials + 1):
        cell = 0
        for task in args.tasks:
            for scene in args.scenes:
                for planner in args.planners:
                    jobs.append(
                        {
                            "trial": trial,
                            "scene": scene,
                            "task": task,
                            "planner": planner,
                            "budget": SCENE_BUDGET[scene_set(scene)],
                            "cap": cap_of(scene, task, args.cap_override),
                            "order": cell,
                        }
                    )
                    cell += 1
    # Rotate each trial's block so lanes do not start on the same slow planner.
    out = []
    per_trial = len(args.tasks) * len(args.scenes) * len(args.planners)
    for trial in range(args.trials):
        block = jobs[trial * per_trial : (trial + 1) * per_trial]
        shift = (trial * 7) % max(len(block), 1)
        out.extend(block[shift:] + block[:shift])
    return out


def drop_done(jobs, args, runs_csv):
    """--resume: keep only the trials a cell is still missing."""
    veto = vetoed_run_dirs(runs_csv)
    done = {}
    for task in args.tasks:
        for scene in args.scenes:
            for planner in args.planners:
                done[(task, scene, planner)] = count_complete(
                    args.data_root, task, scene, planner, veto
                )
    counted = sum(done.values())
    kept = []
    for job in jobs:
        key = (job["task"], job["scene"], job["planner"])
        if done[key] > 0:
            done[key] -= 1
        else:
            kept.append(job)
    return kept, counted


def launch(job, lane, port, args, log_path):
    """One roslaunch under `timeout`, in its own session.

    Returns (rc, cmd, number of the run's processes still alive after it, now killed).
    """
    cmd = [
        "timeout",
        "--signal=INT",
        "--kill-after=60",
        str(job["cap"] * 60 + 90),
        "roslaunch",
        "open_apero_ros",
        "run_experiment_sim.launch",
        "planner:=%s" % job["planner"],
        "task:=%s" % job["task"],
        "cost_budget:=%d" % job["budget"],
        "scene_id:=%s" % job["scene"],
        "time_limit:=%d" % job["cap"],
        "speedup_factor:=%d" % args.speedup_factor,
        "visualize:=false",
        "log_level:=warn",
        "scene_dataset:=HSSD",
        "data_root:=%s" % os.path.join(args.data_root, job["task"]),
    ]
    if args.dry_run:
        print(
            "lane%d ROS_MASTER_URI=http://localhost:%d %s" % (lane, port, " ".join(cmd)), flush=True
        )
        return 0, cmd, 0

    tag = "%d.%d.%d" % (os.getpid(), lane, time.time_ns())
    env = dict(os.environ, ROS_MASTER_URI="http://localhost:%d" % port)
    env[RUN_TAG] = tag
    with open(log_path, "w") as log:
        # Own session: a Ctrl-C of the sweep stops dispatching but leaves the running lanes alone.
        proc = subprocess.Popen(
            cmd, stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True
        )
        rc = proc.wait()
    # A roslaunch killed at the cap leaves its master and nodes behind; the master would keep
    # the lane's port and hand the next run its parameters.
    return rc, cmd, kill_tagged(tag)


def wait_for_port(port, sweep_log, lane, limit=180):
    """Wait up to `limit` seconds for the lane's master port to be free. False if it stays busy.

    A run must not launch onto a master already there: it would join it, with its parameters.
    """
    deadline = time.time() + limit
    while not port_free(port):
        if time.time() >= deadline:
            log_line(
                sweep_log,
                "lane%d port %d still busy after %ds: the lane stops, its job goes back to the "
                "queue" % (lane, port, limit),
            )
            return False
        time.sleep(5)
    return True


def worker(lane, port, jobs, args, runs_csv, sweep_log, stats):
    while not stop_dispatch.is_set():
        try:
            job = jobs.get_nowait()
        except queue.Empty:
            return
        tag = "trial%d %s/%s/%s" % (job["trial"], job["scene"], job["task"], job["planner"])
        for attempt in (1, 2):
            if not wait_for_port(port, sweep_log, lane):
                jobs.put(job)
                return
            name = "t%d_%s_%s_%s%s" % (
                job["trial"],
                job["scene"],
                job["task"],
                job["planner"],
                "_retry" if attempt == 2 else "",
            )
            log_path = os.path.join(args.data_root, "logs", name + ".log")
            start = time.time()
            log_line(
                sweep_log,
                "lane%d START %s (budget %d, cap %dmin)" % (lane, tag, job["budget"], job["cap"]),
            )
            rc, _, leftovers = launch(job, lane, port, args, log_path)
            end = time.time()
            if leftovers:
                log_line(
                    sweep_log, "lane%d killed %d processes %s left behind" % (lane, leftovers, tag)
                )
            run_dir = newest_run_dir(
                args.data_root, job["task"], job["scene"], job["planner"], start
            )
            cost = read_final_cost(run_dir) if run_dir else None
            # The wall cap is part of the paper protocol, so a cap-terminated run is a
            # result: complete but truncated. Only a broken run counts as failed.
            status = "complete" if (rc == 0 and cost is not None) else "failed"
            truncated = bool(cost is not None and cost < COMPLETE_FRACTION * job["budget"])
            append_run(
                runs_csv,
                [
                    job["trial"],
                    job["scene"],
                    job["task"],
                    job["planner"],
                    lane,
                    int(start),
                    int(end),
                    int(end - start),
                    rc,
                    status,
                    "%.1f" % cost if cost is not None else "",
                    int(truncated),
                    run_dir,
                    os.path.basename(log_path),
                ],
            )
            log_line(
                sweep_log,
                "lane%d END   %s rc=%d %s%s cost=%s wall=%ds"
                % (
                    lane,
                    tag,
                    rc,
                    status,
                    " truncated" if truncated else "",
                    "%.1f" % cost if cost is not None else "-",
                    int(end - start),
                ),
            )
            with io_lock:
                stats.setdefault(scene_set(job["scene"]), []).append(end - start)
                stats["done"] = stats.get("done", 0) + 1
            if status != "failed" or attempt == 2:
                break
            if not egl_flake(log_path):
                break
            log_line(sweep_log, "lane%d EGL startup flake on %s, retrying once" % (lane, tag))
            time.sleep(10)
        time.sleep(5)


def guarded_worker(lane, port, jobs, args, runs_csv, sweep_log, stats):
    """worker(), with a crash in sweep.log and the sweep's exit status, not only on stderr."""
    try:
        worker(lane, port, jobs, args, runs_csv, sweep_log, stats)
    except Exception:
        log_line(
            sweep_log,
            "lane%d CRASHED, its current job unrecorded (--resume schedules it again):\n%s"
            % (lane, traceback.format_exc().rstrip()),
        )
        with io_lock:
            stats["crashed"] = stats.get("crashed", 0) + 1


def egl_flake(log_path):
    try:
        with open(log_path, errors="replace") as handle:
            return bool(EGL_FLAKE.search(handle.read()))
    except OSError:
        return False


def eta_reporter(total, jobs, stats, sweep_log, lanes):
    while not stop_dispatch.is_set():
        time.sleep(300)
        with io_lock:
            done = stats.get("done", 0)
            means = {}
            for key in ("large", "small"):
                samples = stats.get(key, [])
                if samples:
                    means[key] = sum(samples) / len(samples)
        if not means or done == 0:
            continue
        fallback = sum(means.values()) / len(means)
        remaining = sum(means.get(scene_set(j["scene"]), fallback) for j in list(jobs.queue))
        hours = remaining / max(lanes, 1) / 3600.0
        log_line(
            sweep_log,
            "ETA %d/%d done, %.1f h left (mean wall %s)"
            % (done, total, hours, " ".join("%s=%.0fs" % kv for kv in sorted(means.items()))),
        )


class HelpFormatter(argparse.ArgumentDefaultsHelpFormatter, argparse.RawDescriptionHelpFormatter):
    """Show every default and keep the docstring's layout."""


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=HelpFormatter)
    # The data root every tool shares, spelled out here because this script imports no package
    # beyond the standard library (open_apero_analysis.paths is its definition).
    default_data_root = os.environ.get("OPEN_APERO_DATA_ROOT") or "~/open_apero_data"
    parser.add_argument(
        "--data-root",
        default=os.path.join(os.path.expanduser(default_data_root), "paper_sweep"),
        help="sweep root: runs in <data-root>/<task>/HSSD/<scene>/<planner>/<stamp>/, "
        "plus runs.csv, sweep.log and logs/",
    )
    parser.add_argument(
        "--scenes",
        nargs="+",
        default=LARGE_SCENES + SMALL_SCENES,
        choices=LARGE_SCENES + SMALL_SCENES,
        help="HSSD scene ids; large scenes get --large-budget, the others --small-budget",
    )
    parser.add_argument(
        "--planners",
        nargs="+",
        default=PLANNERS,
        help="planners, i.e. config/planners/<planner>.yaml",
    )
    parser.add_argument(
        "--tasks",
        nargs="+",
        default=TASKS,
        choices=TASKS,
        help="tasks: explore, recon and/or points",
    )
    parser.add_argument(
        "--trials", type=int, default=10, help="runs per (task, scene, planner) cell"
    )
    parser.add_argument(
        "--parallel", type=int, default=3, help="parallel roslaunch lanes, one master each"
    )
    parser.add_argument(
        "--speedup-factor", type=int, default=3, help="speedup_factor of the dummy controller"
    )
    parser.add_argument(
        "--master-port-base", type=int, default=11411, help="master port of lane 0; lane i uses +i"
    )
    parser.add_argument(
        "--large-budget", type=int, default=LARGE_BUDGET, help="cost_budget of the large scenes [s]"
    )
    parser.add_argument(
        "--small-budget", type=int, default=SMALL_BUDGET, help="cost_budget of the small scenes [s]"
    )
    parser.add_argument(
        "--cap-override",
        type=int,
        default=0,
        help="experiments only: one wall cap [min] for every cell, "
        "in place of the paper's per-cell PAPER_WALL_CAPS; 0 keeps those",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print the first two commands per lane and the queue size, run nothing",
    )
    parser.add_argument(
        "--resume",
        action="store_true",
        help="schedule only the trials each cell still misses; runs.csv's failed runs do not count",
    )
    args = parser.parse_args(argv)

    SCENE_BUDGET["large"] = args.large_budget
    SCENE_BUDGET["small"] = args.small_budget

    os.makedirs(os.path.join(args.data_root, "logs"), exist_ok=True)
    runs_csv = os.path.join(args.data_root, "runs.csv")
    sweep_log = os.path.join(args.data_root, "sweep.log")
    if not os.path.exists(runs_csv):
        with open(runs_csv, "w") as handle:
            handle.write(",".join(RUNS_HEADER) + "\n")

    jobs = build_jobs(args)
    total_planned = len(jobs)
    already = 0
    if args.resume:
        jobs, already = drop_done(jobs, args, runs_csv)
    log_line(
        sweep_log,
        "SWEEP START pid=%d lanes=%d ports=%d..%d jobs=%d "
        "(planned %d, already complete %d) speedup=%d caps=%s"
        % (
            os.getpid(),
            args.parallel,
            args.master_port_base,
            args.master_port_base + args.parallel - 1,
            len(jobs),
            total_planned,
            already,
            args.speedup_factor,
            "override %d min" % args.cap_override if args.cap_override else "paper per-cell",
        ),
    )

    if args.dry_run:
        for index, job in enumerate(jobs[: args.parallel * 2]):
            lane = index % args.parallel
            launch(job, lane, args.master_port_base + lane, args, "")
        print(
            "\n%d jobs queued (showing first %d)" % (len(jobs), min(len(jobs), args.parallel * 2))
        )
        return 0

    work = queue.Queue()
    for job in jobs:
        work.put(job)

    def handle_signal(signum, _frame):
        log_line(sweep_log, "signal %d: no new jobs, waiting for running lanes" % signum)
        stop_dispatch.set()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    stats = {}
    threads = []
    reporter = threading.Thread(
        target=eta_reporter, daemon=True, args=(len(jobs), work, stats, sweep_log, args.parallel)
    )
    reporter.start()
    for lane in range(args.parallel):
        thread = threading.Thread(
            target=guarded_worker,
            args=(lane, args.master_port_base + lane, work, args, runs_csv, sweep_log, stats),
        )
        thread.start()
        threads.append(thread)
        if not work.empty():
            time.sleep(20)  # stagger lane startup so three Habitat contexts do not race
    for thread in threads:
        thread.join()
    stop_dispatch.set()

    left, crashed = work.qsize(), stats.get("crashed", 0)
    log_line(
        sweep_log,
        "SWEEP END %d runs executed%s%s"
        % (
            stats.get("done", 0),
            ", %d jobs not run" % left if left else "",
            ", %d lanes crashed" % crashed if crashed else "",
        ),
    )
    return 1 if left or crashed else 0


if __name__ == "__main__":
    sys.exit(main())
