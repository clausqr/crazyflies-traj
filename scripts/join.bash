#!/usr/bin/env bash
#
# DESCRIPTION:
#   Open a shell (or run --cmd) in a running crazyflies-traj container.
#
# USAGE:
#   rr join [--name <container_name>] [--cmd '<command>']

source setup.env

container_name="$RR_IMAGE_NAME"
join_cmd=""
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --name) container_name="$2"; shift 2 ;;
        --cmd) join_cmd="$2"; shift 2 ;;
        --help) echo "Usage: $0 [--name <container_name>] [--cmd '<command>']"; exit 0 ;;
        *) echo "Unknown parameter passed: $1" >&2; exit 2 ;;
    esac
done

if [[ $(docker inspect -f '{{.State.Running}}' "$container_name" 2>/dev/null) != "true" ]]; then
    echo "Container $container_name is not running" >&2
    exit 1
fi
env_src='[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash; '
if [ -n "$join_cmd" ]; then
    exec docker exec -it -e RR_JOIN_CMD="$join_cmd" "$container_name" \
        bash --login -c "${env_src}exec bash -c \"\$RR_JOIN_CMD\""
fi
exec docker exec -it "$container_name" bash --login -c "${env_src}source /ros2_ws/motd.sh; exec bash"
