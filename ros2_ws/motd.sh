#!/bin/bash
#
# Printed by the interactive shells of `./rr run` and `./rr join` (sourced; the
# values are the container's own environment, set by scripts/run.bash).

echo "crazyflies-traj shell: ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-?}  RR_NET_MODE=${RR_NET_MODE:-?}  DDS profile ${FASTRTPS_DEFAULT_PROFILES_FILE:-?}"
echo "  vehicle RR_CF_NAME=${RR_CF_NAME:-?}; trajectories read-only at /traj; workspace /ros2_ws (sourced)"
echo "  ros2 topic list                                                                  # what the graph shows"
echo "  ros2 run crazyflies_traj traj_player --ros-args -p csv_path:=/traj/square_1m.csv   # DRY RUN, one vehicle"
echo "  ros2 launch crazyflies_traj fleet.launch.py config:=/traj/fleet.yaml               # FLIES every vehicle in the yaml (fly:=false = dry run)"
echo "  single vehicle: -p fly:=true after its dry run; on any exit while airborne, land at once"
