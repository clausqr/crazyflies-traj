#!/usr/bin/env bash
#
# DESCRIPTION:
#   Stop a running crazyflies-traj container (SIGTERM first).
#
# USAGE:
#   rr stop [--name <container_name>]

source setup.env

container_name="$RR_IMAGE_NAME"
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --name) container_name="$2"; shift 2 ;;
        --help) echo "Usage: $0 [--name <container_name>]"; exit 0 ;;
        *) echo "Unknown parameter passed: $1" >&2; exit 2 ;;
    esac
done

echo "Stopping ${container_name}..."
if docker stop "${container_name}"; then
    echo "${container_name} stopped OK"
else
    status=$?
    echo "Failed to stop ${container_name} (docker stop exit $status)" >&2
    exit $status
fi
