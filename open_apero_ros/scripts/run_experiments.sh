#!/usr/bin/env bash

# Sweep driver: one gnome-terminal per (scene, planner) run, so it needs a desktop
# session with gnome-terminal and util-linux `script` (checked at startup).
#
# Every knob below is overridable from the environment:
#
#   PLANNERS="nbs dbs" TRIALS=3 ./run_experiments.sh
#   SCENE_SET=small ./run_experiments.sh
#   TASK=points COST_BUDGET=300 ./run_experiments.sh
#
# Runs land in $DATA_ROOT/<scene_dataset>/<scene>/<planner>/<stamp>/, or with TASK set in
# $DATA_ROOT/<task>/<scene_dataset>/..., the layout paper_sweep.py writes.
#
# Each run lives in a terminal that closes as soon as it succeeds, so the driver
# keeps its own record under $DATA_ROOT/sweep_logs/<start stamp>/:
#
#   sweep.log            this driver's narration, timestamped
#   runs.csv             one row per run: status, exit code, run directory
#   trial<NN>_<scene>_<planner>.log   that run's console output

# No -e: a failed run or lookup must be recorded, not end the sweep.
set -uo pipefail

# Runtime configuration
PLAN_LIMIT=${PLAN_LIMIT:-6}
MAX_SECONDS=$((PLAN_LIMIT * 60 + 30))
TRIALS=${TRIALS:-15}
# Master of every run, exported to it as ROS_MASTER_URI; the caller's ROS_MASTER_URI is not used.
SWEEP_MASTER_URI=${SWEEP_MASTER_URI:-http://localhost:11411}

# Where the runs and this sweep's logs land. Both are forwarded to the launch
# file so the driver's record and the data cannot disagree about where a run
# went; the defaults match run_experiment_sim.launch: the data root every Open
# Apero tool shares, $OPEN_APERO_DATA_ROOT, else ~/open_apero_data.
DATA_ROOT=${DATA_ROOT:-${OPEN_APERO_DATA_ROOT:-$HOME/open_apero_data}}
SCENE_DATASET=${SCENE_DATASET:-HSSD}

# Active-perception task (explore, recon or points) and cost budget [s]. An unset
# TASK keeps config/tasks/_common.yaml (exploration); an unset COST_BUDGET keeps
# config/planners/_common.yaml's cost_budget.
TASK=${TASK:-}
COST_BUDGET=${COST_BUDGET:-}
# The launch file's data_root: one folder per task, so tasks never share a run folder.
RUN_ROOT=$DATA_ROOT${TASK:+/$TASK}

# all (default) keeps every run's console log; failed discards the logs of runs
# that finished cleanly.
KEEP_RUN_LOGS=${KEEP_RUN_LOGS:-all}

# Workspace `src/` directory (NOT the workspace root) -- the terminals below do
# `cd "$FOLDER" && source ../devel/setup.bash`: the nearest directory at or above this
# script whose parent holds devel/setup.bash.
# Set OPEN_APERO_WS to override; it must also point at the workspace's src/ dir.
find_workspace_src() {
    local dir
    dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    while [ "$dir" != "/" ]; do
        if [ -f "$(dirname "$dir")/devel/setup.bash" ]; then
            printf '%s\n' "$dir"
            return 0
        fi
        dir="$(dirname "$dir")"
    done
    return 1
}

if [ -n "${OPEN_APERO_WS:-}" ]; then
    FOLDER="$OPEN_APERO_WS"
elif ! FOLDER="$(find_workspace_src)"; then
    echo "error: could not find a catkin workspace above $(dirname "${BASH_SOURCE[0]}")." >&2
    echo "       Build the workspace first, or set OPEN_APERO_WS to its src/ directory." >&2
    exit 1
fi

# Preflight, because each of these would otherwise fail identically in every
# child terminal while the driver kept sleeping through the whole sweep.
if [ ! -f "$FOLDER/../devel/setup.bash" ]; then
    echo "error: no built workspace: '$FOLDER/../devel/setup.bash' does not exist." >&2
    exit 1
fi
for tool in gnome-terminal script; do
    if ! command -v "$tool" >/dev/null 2>&1; then
        echo "error: '$tool' not found; every run would fail without producing output." >&2
        exit 1
    fi
done
case "$KEEP_RUN_LOGS" in
    all | failed) ;;
    *) echo "error: KEEP_RUN_LOGS must be all or failed (got '$KEEP_RUN_LOGS')." >&2; exit 1 ;;
esac
case "$TASK" in
    '' | explore | recon | points) ;;
    *) echo "error: TASK must be explore, recon or points (got '$TASK')." >&2; exit 1 ;;
esac

# Space-separated override, e.g. PLANNERS="nbs dbs". The planners and scene sets match
# python/open_apero_ros/paper_protocol.py.
read -r -a PLANNERS <<< "${PLANNERS:-nbs dbs spt tsp rrtstar rrt}"

LARGE_SCENE_IDS=("102344280" "102344049" "102816009" "103997445" "102816036")
SMALL_SCENE_IDS=("102815835" "102816852" "102816216")
ALL_SCENE_IDS=("${LARGE_SCENE_IDS[@]}" "${SMALL_SCENE_IDS[@]}")

# One of: large (default), small, all. Or set SCENES directly.
case "${SCENE_SET:-large}" in
    large) DEFAULT_SCENE_IDS=("${LARGE_SCENE_IDS[@]}") ;;
    small) DEFAULT_SCENE_IDS=("${SMALL_SCENE_IDS[@]}") ;;
    all)   DEFAULT_SCENE_IDS=("${ALL_SCENE_IDS[@]}") ;;
    *)     echo "error: SCENE_SET must be large, small or all (got '$SCENE_SET')." >&2; exit 1 ;;
esac
read -r -a SCENE_IDS <<< "${SCENES:-${DEFAULT_SCENE_IDS[*]}}"

# Sweep log directory
SWEEP_START=$(date '+%s')
SWEEP_DIR="$DATA_ROOT/sweep_logs/$(date -d "@$SWEEP_START" '+%Y%m%d_%H%M%S')"
if ! mkdir -p "$SWEEP_DIR"; then
    echo "error: could not create the sweep log directory '$SWEEP_DIR'." >&2
    exit 1
fi
SWEEP_LOG="$SWEEP_DIR/sweep.log"
RUNS_CSV="$SWEEP_DIR/runs.csv"
printf 'trial,scene,planner,start,wall_s,status,exit,run_dir,log\n' > "$RUNS_CSV"

log() {
    printf '[%s] %s\n' "$(date '+%F %T')" "$*" | tee -a "$SWEEP_LOG"
}

format_duration() {
    printf '%dh%02dm%02ds' $(($1 / 3600)) $(($1 % 3600 / 60)) $(($1 % 60))
}

# Stamped run directories are named by the mission coordinator a moment after
# launch, so "newest name at or after this run's start stamp" identifies the
# directory this run produced without confusing it with an earlier sweep's.
run_directory_since() {
    local parent=$1 since=$2 name
    name=$(find "$parent" -mindepth 1 -maxdepth 1 -type d -printf '%f\n' 2>/dev/null |
        awk -v since="$since" '$0 >= since' | sort | tail -1)
    [ -n "$name" ] && printf '%s\n' "$parent/$name"
}

run_in_terminal() {
    # `script` puts the run on a pty, so rosconsole and the C++ nodes stay
    # line-buffered and colored as if watched directly while every line is also
    # captured. `timeout` sits inside the command so the cap still signals
    # roslaunch, which shuts its nodes down; capping `script` would orphan them
    # instead. `script -c` re-parses its string through a shell, so the command
    # arrives pre-quoted (printf %q) and the child composes no shell text.
    # shellcheck disable=SC2016  # expanded by the child shell, from its arguments
    gnome-terminal -- bash -c '
        cmd=$1 log=$2 workspace=$3 master=$4
        export ROS_MASTER_URI="$master"
        if cd "$workspace" && source ../devel/setup.bash; then
            script -q -e -f -c "$cmd" "$log"
            status=$?
        else
            status=1
            echo "error: cannot use workspace $workspace" | tee "$log" >&2
        fi
        printf "%s\n" "$status" > "$log.status"
        if [ "$status" -ne 0 ] && [ "$status" -ne 124 ]; then
            printf "\nExited with status %s. Log: %s\nPress Enter to close." "$status" "$log"
            read -r
        fi
    ' _ "$1" "$2" "$FOLDER" "$SWEEP_MASTER_URI"
}

TOTAL_RUNS=$((TRIALS * ${#SCENE_IDS[@]} * ${#PLANNERS[@]}))
RUN_BUDGET=$((MAX_SECONDS + 10))  # hard cap plus the settle time between runs

declare -i run_index=0 num_ok=0 num_timeout=0 num_failed=0 num_no_status=0
FAILURES=()
RUN_PENDING=  # set while a run has been launched but not yet accounted for

# Classify the run that was launched last and add it to the log and the table.
# Its terminal is detached, so the exit status comes back through a file; a
# missing one means the run outlived the budget the driver gave it.
record_run() {
    local exit_code status run_dir remaining seconds=$RUN_BUDGET csv_log=$RUN_LOG
    exit_code=$(cat "$RUN_LOG.status" 2>/dev/null)
    case "$exit_code" in
        0)   status=ok;        num_ok+=1 ;;
        124) status=timeout;   num_timeout+=1 ;;
        '')  status=no-status; num_no_status+=1 ;;
        *)   status=failed;    num_failed+=1 ;;
    esac
    if [ -f "$RUN_LOG.status" ]; then
        seconds=$(($(stat -c '%Y' "$RUN_LOG.status") - RUN_START_EPOCH))
        rm -f "$RUN_LOG.status"  # runs.csv is the record from here on
    fi

    run_dir=$(run_directory_since "$RUN_ROOT/$SCENE_DATASET/$SCENE_ID/$PLANNER" "$RUN_START_STAMP")
    remaining=$((TOTAL_RUNS - run_index))
    log "[$run_index/$TOTAL_RUNS][$SCENE_ID][$PLANNER] $status in $(format_duration "$seconds")\
${run_dir:+, data $run_dir}, $remaining left, eta $(date -d "+$((remaining * RUN_BUDGET)) seconds" '+%m-%d %H:%M')"
    if [ -z "$run_dir" ]; then
        log "  warning: no run directory appeared under $RUN_ROOT/$SCENE_DATASET/$SCENE_ID/$PLANNER;"
        log "           the mission coordinator never got as far as creating one."
    fi
    if [ "$status" != ok ]; then
        FAILURES+=("$status (exit ${exit_code:-none}) trial $i, $SCENE_ID, $PLANNER: $RUN_LOG")
    elif [ "$KEEP_RUN_LOGS" = failed ]; then
        rm -f "$RUN_LOG"
        csv_log=
    fi

    printf '%d,%s,%s,%s,%d,%s,%s,%s,%s\n' "$i" "$SCENE_ID" "$PLANNER" "$RUN_START_STAMP" \
        "$seconds" "$status" "${exit_code:-}" "$run_dir" "$csv_log" >> "$RUNS_CSV"
    RUN_PENDING=
}

print_summary() {
    log "Sweep summary after $(format_duration $(($(date '+%s') - SWEEP_START))): \
$num_ok ok, $num_timeout timed out, $num_failed failed, $num_no_status without status, \
of $run_index run(s) started ($TOTAL_RUNS planned)."
    local failure
    for failure in "${FAILURES[@]}"; do
        log "  $failure"
    done
    log "Sweep log: $SWEEP_LOG"
    log "Run table: $RUNS_CSV"
}

# Report what was reached instead of losing the tally when a sweep is cut short.
# The interrupt does not reach the detached terminals, so a run that is still
# going survives the driver -- account for it before summarizing either way.
on_interrupt() {
    echo
    log "Interrupted by the user; a run still in flight keeps going in its own terminal."
    [ -n "$RUN_PENDING" ] && record_run
    print_summary
    exit 130
}
trap on_interrupt INT

log "Sweeping ${#PLANNERS[@]} planner(s) x ${#SCENE_IDS[@]} scene(s) x $TRIALS trial(s) = $TOTAL_RUNS run(s)."
log "  planners:   ${PLANNERS[*]}"
log "  scenes:     ${SCENE_IDS[*]}"
log "  per run:    $PLAN_LIMIT min planning, ${MAX_SECONDS}s hard cap, ${RUN_BUDGET}s of driver time"
log "  workspace:  $FOLDER"
log "  data root:  $RUN_ROOT/$SCENE_DATASET"
log "  sweep logs: $SWEEP_DIR (KEEP_RUN_LOGS=$KEEP_RUN_LOGS)"
log "  ros master: $SWEEP_MASTER_URI"
log "Estimated finish: $(date -d "+$((TOTAL_RUNS * RUN_BUDGET)) seconds" '+%F %T') \
(in $(format_duration $((TOTAL_RUNS * RUN_BUDGET))))."

for i in $(seq 1 "$TRIALS"); do
    log ">>> Starting trial $i/$TRIALS..."

    for SCENE_ID in "${SCENE_IDS[@]}"; do
        for PLANNER in "${PLANNERS[@]}"; do
            run_index+=1
            RUN_LOG=$(printf '%s/trial%02d_%s_%s.log' "$SWEEP_DIR" "$i" "$SCENE_ID" "$PLANNER")
            RUN_START_STAMP=$(date '+%Y%m%d_%H%M%S')
            RUN_START_EPOCH=$(date '+%s')

            CMD_PLAN=(timeout "$MAX_SECONDS" roslaunch open_apero_ros run_experiment_sim.launch
                "planner:=$PLANNER" "scene_id:=$SCENE_ID" "time_limit:=$PLAN_LIMIT"
                "scene_dataset:=$SCENE_DATASET" "data_root:=$RUN_ROOT")
            [ -n "$TASK" ] && CMD_PLAN+=("task:=$TASK")
            [ -n "$COST_BUDGET" ] && CMD_PLAN+=("cost_budget:=$COST_BUDGET")
            CMD_TEXT=$(printf '%q ' "${CMD_PLAN[@]}")

            log "[$run_index/$TOTAL_RUNS][$SCENE_ID][$PLANNER] running, log $RUN_LOG"
            log "  cmd: $CMD_TEXT"
            RUN_PENDING=1
            run_in_terminal "$CMD_TEXT" "$RUN_LOG"
            sleep "$RUN_BUDGET"  # Wait for completion
            record_run
        done
    done
done

log "All trials complete."
print_summary
