#!/usr/bin/env bash
#
# DESCRIPTION:
#   Trajectory wrappers around the container. FILE is a CSV under
#   trajectories/ (RR_TRAJ_DIR), given as a path or a name relative to it; it
#   is seen inside the container at /traj/<name>. For fleet, FILE is the fleet
#   params file (YAML) under trajectories/.
#
# USAGE:
#   rr traj check FILE [traj check options]      offline, no ROS graph (--net=none)
#   rr traj run   FILE [--name N] [key:=value ...] [--fly]
#   rr traj fleet FILE.yaml [--name N] [--dry-run]
#
#   check: validates FILE against every offline limit and prints the summary;
#          add --setpoints for every setpoint (see `traj check --help`).
#   run:   runs traj_player in the FOREGROUND (Ctrl-C = abort). DRY RUN unless
#          --fly is given: without it the player prints the plan and every
#          setpoint, shadows the airborne gates and publishes nothing.
#          key:=value pairs become player parameters (frame:=relative,
#          interp:=pchip, pad_z0:=0.063, ...). Exit code = the player's
#          (0 DONE, 1 refused / aborted, 2 bad parameters).
#   fleet: one traj_player per vehicle in FILE.yaml (fleet.launch.py), in the
#          FOREGROUND. FLIES BY DEFAULT (--dry-run for the shadow run); a
#          vehicle absent from the graph is warned about and waited for while
#          the others fly. Per-vehicle parameters live in the yaml. ros2 launch does not propagate the
#          players' exit codes: read each node's DONE / ABORT lines.
#
# EXAMPLES:
#   ./rr traj check square_1m.csv --setpoints
#   ./rr traj run square_1m.csv                       # dry run: always first
#   ./rr traj fleet fleet.yaml --dry-run              # shadow run, every vehicle
#   RR_NET_MODE=lan ./rr traj run square_1m.csv --name traj --fly
#   ./rr traj fleet fleet.yaml --name fleet        # FLIES every vehicle in the yaml

source setup.env

sub="${1:-}"; shift || true
file="${1:-}"; shift || true
if [ -z "$sub" ] || [ -z "$file" ] || { [ "$sub" != check ] && [ "$sub" != run ] && [ "$sub" != fleet ]; }; then
    sed -n '2,/^$/p' "$0" | sed 's/^#//'
    exit 2
fi

traj_dir="$(cd "$RR_TRAJ_DIR" && pwd)" || exit 2
if [ -f "$file" ]; then
    abs="$(cd "$(dirname "$file")" && pwd)/$(basename "$file")"
elif [ -f "$traj_dir/$file" ]; then
    abs="$traj_dir/$file"
else
    echo "rr traj: no such CSV: $file (looked as given and under $traj_dir)" >&2
    exit 2
fi
case "$abs" in
    "$traj_dir"/*) rel="${abs#"$traj_dir"/}" ;;
    *) echo "rr traj: $abs is outside $traj_dir, which is the only directory mounted into the container (/traj). Copy it there or set RR_TRAJ_DIR." >&2; exit 2 ;;
esac

if [ "$sub" = check ]; then
    quoted=""
    for a in "$@"; do quoted+=" $(printf '%q' "$a")"; done
    exec docker run --rm --net=none --user "${RR_USER_UID}:${RR_USER_GID}" \
        -v "$(pwd)/ros2_ws/src/crazyflies_traj:/src:ro" -v "$traj_dir:/traj:ro" \
        -e PYTHONPATH=/src -e PYTHONDONTWRITEBYTECODE=1 \
        "${RR_IMAGE_NAME}:${RR_IMAGE_TAG}" \
        bash -c "python3 -m crazyflies_traj.cli check $(printf '%q' "/traj/$rel")$quoted"
fi

if [ "$sub" = fleet ]; then
    name="$RR_IMAGE_NAME"
    fly=true                                  # the fleet is the real thing by default
    while [[ "$#" -gt 0 ]]; do
        case $1 in
            --dry-run) fly=false; shift ;;
            --name) name="$2"; shift 2 ;;
            *) echo "rr traj fleet: unknown argument $1 (per-vehicle parameters live in the yaml)" >&2; exit 2 ;;
        esac
    done
    cmd="ros2 launch crazyflies_traj fleet.launch.py $(printf '%q' "config:=/traj/$rel") fly:=$fly"
    if [ "$fly" = true ]; then
        echo "=================================================================="
        echo " FLY: every vehicle in $rel takes command authority on its own"
        echo " /<ns>/cmd_* once airborne and stable (domain ${RR_ROS_DOMAIN_ID}, ${RR_NET_MODE})."
        echo " Preconditions: preflight GO, every drone hovering, e-stop shell ready."
        echo " Ctrl-C aborts every player (streams stop at once: land IMMEDIATELY after)."
        echo "=================================================================="
    else
        echo "DRY RUN (--dry-run): nothing will be published by any vehicle."
    fi
    echo "rr traj runs, inside the container:"; echo "  \$ $cmd"
exec bash ./scripts/run.bash --name "$name" --fg --cmd "$cmd"
fi

# ---- run ------------------------------------------------------------------
name="$RR_IMAGE_NAME"
fly=false
params=()
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --fly) fly=true; shift ;;
        --name) name="$2"; shift 2 ;;
        *:=*)
            case "$1" in
                fly:=*|csv_path:=*) echo "rr traj run: set fly with --fly and the CSV as FILE, not $1" >&2; exit 2 ;;
            esac
            params+=("$1"); shift ;;
        *) echo "rr traj run: unknown argument $1" >&2; exit 2 ;;
    esac
done

cmd="ros2 run crazyflies_traj traj_player --ros-args -p $(printf '%q' "csv_path:=/traj/$rel") -p fly:=$fly"
for kv in "${params[@]}"; do cmd+=" -p $(printf '%q' "$kv")"; done

if [ "$fly" = true ]; then
    echo "=================================================================="
    echo " FLY: traj_player will take command authority on /${RR_CF_NAME}/cmd_*"
    echo " once the vehicle is airborne and stable (domain ${RR_ROS_DOMAIN_ID}, ${RR_NET_MODE})."
    echo " Preconditions: preflight GO, drone hovering, e-stop shell ready."
    echo " Ctrl-C aborts (stream stops at once: land IMMEDIATELY after)."
    echo "=================================================================="
else
    echo "DRY RUN (no --fly): nothing will be published."
fi
echo "rr traj runs, inside the container:"; echo "  \$ $cmd"
exec bash ./scripts/run.bash --name "$name" --fg --cmd "$cmd"
