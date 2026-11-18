#!/usr/bin/env bash
#
# Runs once after `rr build` (scripts/build.bash, throwaway container, no
# network): colcon-build the workspace so the first `rr run` starts at once.
set -e
cd /ros2_ws
colcon build --symlink-install --base-paths src
