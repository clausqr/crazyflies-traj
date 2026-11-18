#!/bin/bash
#
# Sourced at container start (scripts/run.bash). Builds the workspace when it is
# missing or stale, then sources it.
#
# Stale = the install tree lacks something src/ has that --symlink-install does
# NOT pick up live: a console_scripts entry (setup.py) or a launch file. The
# Python package directory itself is one symlink (build/crazyflies_traj/
# crazyflies_traj -> src/...), so new AND edited module files are live.

_traj_ws_stale() {
    local pkg=/ros2_ws/src/crazyflies_traj inst=/ros2_ws/install/crazyflies_traj s f
    [ -f /ros2_ws/install/setup.bash ] || return 0
    [ -d "$inst" ] || return 0
    for s in $(sed -n "s/^[[:space:]]*'\([A-Za-z_][A-Za-z0-9_]*\) = .*/\1/p" "$pkg/setup.py"); do
        [ -e "$inst/lib/crazyflies_traj/$s" ] || return 0
    done
    for f in "$pkg"/launch/*.launch.py; do
        [ -e "$inst/share/crazyflies_traj/launch/$(basename "$f")" ] || return 0
    done
    return 1
}

if _traj_ws_stale; then
    echo "on_run.sh: building /ros2_ws (colcon, symlink-install)..."
    (cd /ros2_ws && colcon build --symlink-install --base-paths src) || echo "on_run.sh: colcon build FAILED" >&2
fi
# shellcheck disable=SC1091
[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash
