#!/usr/bin/env bash
#
# Offline rehearsal of traj_player against a MOCK plant, inside the throwaway
# rr container (host entry: ./rr rehearse). Every scenario starts a fresh
# mock_plant.py (records every command it receives), runs the real
# traj_player executable, and judges the recording with check_recording.py.
#
#   run_rehearsal.sh [scenario ...]     default: all
#
# REFUSES ROS_DOMAIN_ID 0 (the ROS default, which other ROS 2 setups on the same
# network may share) and 66 (the lab plant), and any non-loopback DDS profile: a
# rehearsal must never be able to see, or command, a real graph.

set -u

refuse() { echo "run_rehearsal: REFUSED: $*" >&2; exit 3; }
case "${ROS_DOMAIN_ID:-}" in
    ''|0|66) refuse "ROS_DOMAIN_ID='${ROS_DOMAIN_ID:-}' (0 and 66 are live graphs; use e.g. 133)" ;;
    *[!0-9]*) refuse "ROS_DOMAIN_ID='${ROS_DOMAIN_ID}' is not a number" ;;
esac
case "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" in
    *loopback.xml) ;;
    *) refuse "FASTRTPS_DEFAULT_PROFILES_FILE='${FASTRTPS_DEFAULT_PROFILES_FILE:-}' is not a loopback profile" ;;
esac

set +u   # the ROS setup scripts read unset variables
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source /ros2_ws/install/setup.bash
set -u

R=/ros2_ws/src/crazyflies_traj/test/rehearsal
T=/traj
PLAYER=/ros2_ws/install/crazyflies_traj/lib/crazyflies_traj/traj_player
OUT=/ros2_ws/rehearsal_out
rm -rf "$OUT"; mkdir -p "$OUT"

pass=0; fail=0; results=()
note() { results+=("$1"); case "$1" in PASS*) pass=$((pass + 1)) ;; *) fail=$((fail + 1)) ;; esac; echo "$1"; }

cmd_count() { local c; c=$(grep -c '"ev": "cmd"' "$1" 2>/dev/null); echo "${c:-0}"; }

wait_cmds() {  # wait_cmds REC N TIMEOUT: until the mock has N commands (1 = timed out)
    local i=0
    while [ "$(cmd_count "$1")" -lt "$2" ]; do
        [ $i -ge $(($3 * 20)) ] && { echo "  timed out waiting for $2 commands" >&2; return 1; }
        sleep 0.05; i=$((i + 1))
    done
}

# scenario NAME "MOCK ARGS" CSV "PLAYER PARAMS" ACTION
scenario() {
    local name=$1 mock_args=$2 csv=$3 params=$4 action=${5:-none}
    local rec="$OUT/$name.jsonl" log="$OUT/$name.log" ipid="" p
    echo "---- $name"
    # shellcheck disable=SC2086
    python3 "$R/mock_plant.py" --record "$rec" --duration 45 $mock_args > "$OUT/$name.mock.log" 2>&1 &
    local mpid=$!
    if [ "$action" = intruder_before ]; then
        python3 "$R/intruder.py" --record "$rec" --hold 45 > /dev/null 2>&1 &
        ipid=$!
        sleep 2
    fi
    local pargs=(--ros-args -p "csv_path:=$csv" -p wait_timeout:=20.0)
    for p in $params; do pargs+=(-p "$p"); done
    "$PLAYER" "${pargs[@]}" > "$log" 2>&1 &
    local ppid=$!
    case "$action" in
        sigint)
            wait_cmds "$rec" 60 20
            python3 -c 'import json,time; print(json.dumps({"ev": "sigint", "mono": round(time.monotonic(), 4)}))' >> "$rec"
            kill -INT "$ppid" ;;
        intruder_mid)
            wait_cmds "$rec" 60 20
            python3 "$R/intruder.py" --record "$rec" --hold 45 > /dev/null 2>&1 &
            ipid=$! ;;
    esac
    local k=0
    while kill -0 "$ppid" 2>/dev/null && [ $k -lt 1200 ]; do sleep 0.05; k=$((k + 1)); done
    if kill -0 "$ppid" 2>/dev/null; then
        echo "  player still running after 60 s: killing" >&2
        kill -KILL "$ppid"
    fi
    wait "$ppid"; local rc=$?
    sleep 0.5                                    # let the mock record the tail
    [ -n "$ipid" ] && kill -INT "$ipid" 2>/dev/null
    kill -INT "$mpid" 2>/dev/null; wait "$mpid" 2>/dev/null
    [ -n "$ipid" ] && wait "$ipid" 2>/dev/null
    note "$(python3 "$R/check_recording.py" "$name" "$rec" "$rc" "$log")"
}

SELECTED="$*"
want() { local s; for s in $SELECTED; do [ "$s" = "$1" ] && return 0; done; return 1; }
run() { if [ -z "$SELECTED" ] || want "$1"; then scenario "$@"; fi; }

echo "rehearsal on ROS_DOMAIN_ID=${ROS_DOMAIN_ID}, profile ${FASTRTPS_DEFAULT_PROFILES_FILE}"
if [ -z "$SELECTED" ]; then
    note "$(python3 "$R/check_interfaces.py" | tail -1)"
    note "$(python3 "$R/check_interfaces.py" --self-test | tail -1)"
fi

SHORT=$T/rehearsal/short_line.csv
LONG=$T/rehearsal/long_line.csv
run dry_run               ""                        "$SHORT" ""
run fly_short_line        ""                        "$SHORT" "fly:=true"
run fly_position_cmd      ""                        "$SHORT" "fly:=true command:=position"
run fly_relative          "--x0 0.5 --y0 -0.5"      "$SHORT" "fly:=true frame:=relative"
run abort_lock            "--lock-at 1.5"           "$LONG"  "fly:=true"
run abort_tumble          "--tumble-at 1.5"         "$LONG"  "fly:=true"
run abort_odom_stale      "--odom-stop-at 1.5"      "$LONG"  "fly:=true"
run abort_tracking        "--drift-at 1.5"          "$LONG"  "fly:=true"
run abort_sigint          ""                        "$LONG"  "fly:=true" sigint
run abort_intruder        ""                        "$LONG"  "fly:=true" intruder_mid
run refuse_intruder       ""                        "$SHORT" "fly:=true" intruder_before
run fly_approach          ""                        "$T/rehearsal/start_far.csv" "fly:=true"
run refuse_never_airborne "--takeoff-at -1"         "$SHORT" "fly:=true wait_timeout:=5.0"
run refuse_bad_csv        ""                        "$T/bad/speed.csv" "fly:=true"

# Seeded cross-check: the checker must go RED on real recordings judged against
# the wrong expectations (a flight is not a dry run, and vice versa).
if [ -z "$SELECTED" ]; then
    for pair in "fly_short_line:dry_run" "dry_run:fly_short_line" "abort_lock:fly_short_line" \
                "fly_approach:fly_short_line"; do
        rec=${pair%%:*}; as=${pair##*:}
        rc=0; [ "$rec" = fly_short_line ] || [ "$rec" = dry_run ] || [ "$rec" = fly_approach ] || rc=1
        if python3 "$R/check_recording.py" "$as" "$OUT/$rec.jsonl" "$rc" "$OUT/$rec.log" > /dev/null; then
            note "FAIL seeded: $rec judged as $as passed (the checker is blind)"
        else
            note "PASS seeded: $rec judged as $as goes RED"
        fi
    done
fi

echo "================ rehearsal: $pass PASS, $fail FAIL"
[ "$fail" -eq 0 ]
