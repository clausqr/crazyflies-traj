#!/usr/bin/env bash
#
# DESCRIPTION:
# Run the lean crazyflies-traj container (a DDS client of the lab plant). No USB, no GPU, no display: this image only runs traj_player and its
# tests. ros2_ws is bind-mounted at /ros2_ws, trajectories/ read-only at /traj,
# the local dds/ profile directory read-only at /ros2_ws/config. An
# interactive shell (no --cmd) prints /ros2_ws/motd.sh: domain, net mode,
# profile, and the dry-run commands to start from.
#
# USAGE:
#   rr run [--name <container>] [--cmd '<command>' [--fg]] [--print]
#
# OPTIONS:
#   --name <container>  container name (default: crazyflies-traj)
#   --cmd '<command>'   run <command> detached (bash -lc) instead of a shell
#   --fg                with --cmd: run it in the FOREGROUND (TTY when there is
#                       one), so Ctrl-C reaches it as SIGINT and the exit code is
#                       the command's
#   --print             print the docker command and exit (used by the tests)
#
# EXAMPLES (dry run first, always):
#   ./rr run --cmd 'ros2 launch crazyflies_traj traj_player.launch.py csv_path:=/traj/square_1m.csv'
#   RR_NET_MODE=lan ./rr run --name traj --cmd 'ros2 run crazyflies_traj traj_player --ros-args -p csv_path:=/traj/square_1m.csv'

source setup.env

container_name="$RR_IMAGE_NAME"
rr_cmd=""
print_only=0
foreground=0
while [[ "$#" -gt 0 ]]; do
    case $1 in
    --name) container_name="$2"; shift 2 ;;
    --cmd) rr_cmd="$2"; shift 2 ;;
    --print) print_only=1; shift ;;
    --fg) foreground=1; shift ;;
    --help) sed -n '2,/^$/p' "$0" | sed 's/^#//'; exit 0 ;;
    *) echo "Unknown parameter passed: $1" >&2; exit 2 ;;
    esac
done

refuse() { echo "rr run: REFUSED: $*" >&2; exit 3; }

# ---- guards (each witnessed by tests/test_rr_harness.py) -------------------
[ "${RR_USB_ENABLED:-0}" = "0" ] || refuse "RR_USB_ENABLED=${RR_USB_ENABLED}: this image never touches USB (no Crazyradio, no cflib). Radio work belongs to the groundstation."
case "${RR_ROS_DOMAIN_ID}" in
    ''|*[!0-9]*) refuse "RR_ROS_DOMAIN_ID='${RR_ROS_DOMAIN_ID}' is not a number" ;;
esac
[ "${RR_ROS_DOMAIN_ID}" -le 232 ] || refuse "RR_ROS_DOMAIN_ID=${RR_ROS_DOMAIN_ID} > 232 (DDS port overflow)"

dds_dir="${RR_DDS_DIR}"
case "${RR_NET_MODE}" in
    loopback) fast_file=fastdds_loopback.xml; cyc_file=cyclonedds_loopback.xml ;;
    lan)      fast_file=fastdds_client.xml;   cyc_file=cyclonedds_client.xml ;;
    *) refuse "RR_NET_MODE='${RR_NET_MODE}' (loopback | lan)" ;;
esac
if [ ! -f "${dds_dir}/${fast_file}" ]; then
    refuse "DDS profile ${dds_dir}/${fast_file} not found. Download it from the groundstation (README, \"DDS profiles\"): curl -fu ${RR_DDS_LOGIN} -o ${dds_dir}/${fast_file} --create-dirs ${RR_DDS_URL}/${fast_file}; curl -fu ${RR_DDS_LOGIN} -o ${dds_dir}/${cyc_file} ${RR_DDS_URL}/${cyc_file}"
fi
if grep -q '__OWN_IP__' "${dds_dir}/${fast_file}"; then
    refuse "${dds_dir}/${fast_file} still contains __OWN_IP__. Replace it with this machine's IP: sed -i \"s/__OWN_IP__/\$(hostname -I | awk '{print \$1}')/g\" ${dds_dir}/${fast_file}"
fi
dds_dir="$(cd "$dds_dir" && pwd)"
[ -d "${RR_TRAJ_DIR}" ] || refuse "trajectory dir ${RR_TRAJ_DIR} not found"

# ---- flags ------------------------------------------------------------------
args=(--rm --init --net=host
      --user "${RR_USER_UID}:${RR_USER_GID}"
      -e "ROS_DOMAIN_ID=${RR_ROS_DOMAIN_ID}"
      -e "CYCLONEDDS_URI=/ros2_ws/config/${cyc_file}"
      -e "FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/config/${fast_file}"
      -e "RR_CF_NAME=${RR_CF_NAME}"
      -e "RR_NET_MODE=${RR_NET_MODE}"
      -v "$(pwd)/ros2_ws:/ros2_ws"
      -v "${dds_dir}:/ros2_ws/config:ro"
      -v "${RR_TRAJ_DIR}:/traj:ro")
[ "${RR_ROS_LOCALHOST_ONLY:-0}" = "1" ] && args+=(-e ROS_LOCALHOST_ONLY=1)
if [ -n "${RR_VOLUMES:-}" ]; then
    IFS=',' read -ra _vols <<< "$RR_VOLUMES"
    for v in "${_vols[@]}"; do [ -n "$v" ] && args+=(-v "$v"); done
fi
if [ -n "${RR_ENV:-}" ]; then
    IFS=',' read -ra _envs <<< "$RR_ENV"
    for e in "${_envs[@]}"; do e="$(echo "$e" | xargs)"; [ -n "$e" ] && args+=(-e "$e"); done
fi
args+=(--name "$container_name")

hook='[ -f /ros2_ws/on_run.sh ] && source /ros2_ws/on_run.sh; '
if [ -n "$rr_cmd" ] && [ "$foreground" = 1 ]; then
    # Foreground: --init (tini) forwards Ctrl-C / TERM to the command.
    if [ -t 0 ]; then args=(-it -e RR_CMD "${args[@]}"); else args=(-i -e RR_CMD "${args[@]}"); fi
    startup="${hook}exec bash -lc \"\$RR_CMD\""
elif [ -n "$rr_cmd" ]; then
    # Detached; the command travels in the env (not argv) so quotes survive.
    args=(-d -e RR_CMD "${args[@]}")
    startup="${hook}exec bash -lc \"\$RR_CMD\""
else
    args=(-it "${args[@]}")
    startup="${hook}source /ros2_ws/motd.sh; exec bash"
fi

if [ "$print_only" = 1 ]; then
    printf 'docker run'; printf ' %q' "${args[@]}" "${RR_IMAGE_NAME}:${RR_IMAGE_TAG}" bash -c "$startup"; printf '\n'
    exit 0
fi

echo "Running ${RR_IMAGE_NAME}:${RR_IMAGE_TAG} as ${container_name}: ROS_DOMAIN_ID=${RR_ROS_DOMAIN_ID} RR_NET_MODE=${RR_NET_MODE} DDS profile ${dds_dir}/${fast_file}"
docker rm -f "$container_name" >/dev/null 2>&1 || true
export RR_CMD="$rr_cmd"
docker run "${args[@]}" "${RR_IMAGE_NAME}:${RR_IMAGE_TAG}" bash -c "$startup"
rc=$?
if [ -n "$rr_cmd" ] && [ "$foreground" = 0 ] && [ $rc -eq 0 ]; then
    echo "Container '${container_name}' started (detached): $rr_cmd"
    echo "Logs: docker logs -f ${container_name}   Stop: ./rr stop --name ${container_name}"
fi
exit $rc
