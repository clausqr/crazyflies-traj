#!/usr/bin/env bash
#
# DESCRIPTION:
#   Offline rehearsal: run traj_player against a MOCK plant in a throwaway
#   container (--rm) on a NON-lab ROS domain over the loopback DDS profile.
#   Witnesses: nothing published before airborne, the exact setpoints after,
#   every abort path, refusal of a second command publisher.
#
# USAGE:
#   rr rehearse [scenario ...]        (default: all; domain 133 unless overridden)
#   RR_ROS_DOMAIN_ID=140 ./rr rehearse abort_lock
#
# Refuses ROS_DOMAIN_ID 0 and 66 and RR_NET_MODE other than loopback.

export RR_ROS_DOMAIN_ID="${RR_ROS_DOMAIN_ID:-133}"
export RR_NET_MODE="${RR_NET_MODE:-loopback}"      # setup.env defaults to lan; a rehearsal never does
source setup.env

refuse() { echo "rr rehearse: REFUSED: $*" >&2; exit 3; }
case "$RR_ROS_DOMAIN_ID" in
    0|66) refuse "RR_ROS_DOMAIN_ID=$RR_ROS_DOMAIN_ID is a live graph (0: ROS default domain, 66: the lab plant)" ;;
esac
[ "$RR_NET_MODE" = loopback ] || refuse "RR_NET_MODE=$RR_NET_MODE: a rehearsal runs on loopback only"

name=crazyflies-traj-rehearsal
args=""
for s in "$@"; do args+=" $(printf '%q' "$s")"; done
exec bash ./scripts/run.bash --name "$name" --fg \
    --cmd "bash /ros2_ws/src/crazyflies_traj/test/rehearsal/run_rehearsal.sh$args"
