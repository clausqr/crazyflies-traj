#!/usr/bin/env python3
"""
Mutation witness for the guards: break each guard in a scratch copy of this repo
and require the host test suite to go RED. A guard whose mutation stays GREEN is
not witnessed (house rule: every guard ships a seeded-violation test that was
seen failing).

    python3 tools/mutation_witness.py        # exit 0 = every mutation caught

Each row is (name, file, exact anchor text, replacement). An anchor that no
longer matches exactly once counts as a failure too, so the list cannot rot
silently when the code moves.
"""
import os
import shutil
import subprocess
import sys
import tempfile

SRC = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG = 'ros2_ws/src/crazyflies_traj/crazyflies_traj/'
REH = 'ros2_ws/src/crazyflies_traj/test/rehearsal/'

M = [
    ('non_finite check', PKG + 'trajectory.py', "            if not math.isfinite(v):", "            if False:"),
    ('monotone t', PKG + 'trajectory.py', "        if not t1 > t0:", "        if False:"),
    ('duplicate row', PKG + 'trajectory.py', "        if t1 == t0 and p1 == p0:", "        if False:"),
    ('too few rows', PKG + 'trajectory.py', "    if len(rows) < 2:", "    if len(rows) < 1:"),
    ('t starts at 0', PKG + 'trajectory.py', "        if not normalize_t:", "        if False:"),
    ('geofence xy', PKG + 'trajectory.py', "    if not (limits.x_min - 1e-9 <= x", "    if False and not (limits.x_min - 1e-9 <= x"),
    ('floor margin', PKG + 'trajectory.py', "    if z_rel < limits.z_floor - 1e-9:", "    if False:"),
    ('geofence z_max', PKG + 'trajectory.py', "    if z_rel > limits.z_max + 1e-9:", "    if False:"),
    ('speed limit', PKG + 'trajectory.py', "        if s > limits.v_max + 1e-9:", "        if False:"),
    ('accel limit', PKG + 'trajectory.py', "        if a > limits.a_max + 1e-9:", "        if False:"),
    ('max duration', PKG + 'trajectory.py', "    if traj.duration > limits.max_duration:", "    if False:"),
    ('start continuity', PKG + 'trajectory.py', "    if not d <= tol:", "    if False:"),
    ('default x_max loosened', PKG + 'trajectory.py', "    x_max: float = 1.75", "    x_max: float = 1.85"),
    ('default x_min loosened', PKG + 'trajectory.py', "    x_min: float = -1.75", "    x_min: float = -1.85"),
    ('default y_max loosened', PKG + 'trajectory.py', "    y_max: float = 1.75", "    y_max: float = 1.85"),
    ('default y_min loosened', PKG + 'trajectory.py', "    y_min: float = -1.75", "    y_min: float = -1.85"),
    ('default z_max loosened', PKG + 'trajectory.py', "    z_max: float = 2.00", "    z_max: float = 2.05"),
    ('default v_max loosened', PKG + 'trajectory.py', "    v_max: float = 2.50", "    v_max: float = 2.60"),
    ('default a_max loosened', PKG + 'trajectory.py', "    a_max: float = 4.00", "    a_max: float = 4.20"),
    ('pchip monotone slopes', PKG + 'sampler.py', "        if d[k - 1] * d[k] <= 0.0:", "        if False:"),
    ('pchip speed check', PKG + 'sampler.py', "            if v > limits.v_max + 1e-9:", "            if False:"),
    ('yaw speed floor', PKG + 'sampler.py', "math.hypot(v[0], v[1]) >= spec.yaw_min_speed", "True"),
    ('yaw slew', PKG + 'sampler.py', "yaws[-1] + min(max(wrap_pi(held - yaws[-1]), -step), step)", "yaws[-1] + wrap_pi(held - yaws[-1])"),
    ('yaw_rate refusal', PKG + 'plan.py', "    if demand > limits.yaw_rate_max_dps + 1e-9:", "    if False:"),
    ('default yaw_rate_max loosened', PKG + 'trajectory.py', "    yaw_rate_max_dps: float = 90.0", "    yaw_rate_max_dps: float = 100.0"),
    ('takeover yaw from odom', PKG + 'plan.py', "        return (measured if self.yaw.yaw == 'path' and measured is not None else y1), y1", "        return y1, y1"),
    ('relative anchor geofence', PKG + 'plan.py', "    check_geofence(traj, limits, offset=(offset[0], offset[1]))", "    pass"),
    ('required bits', PKG + 'player_core.py', "REQUIRED_BITS = IS_ARMED | CAN_FLY | IS_FLYING", "REQUIRED_BITS = IS_ARMED | CAN_FLY"),
    ('fatal bits', PKG + 'player_core.py', "FATAL_BITS = IS_TUMBLED | IS_LOCKED | IS_CRASHED", "FATAL_BITS = IS_TUMBLED | IS_LOCKED"),
    ('height hysteresis', PKG + 'player_core.py', "        if self.airborne and h < g.h_exit:", "        if self.airborne and h < g.h_enter:"),
    ('anchor tol', PKG + 'player_core.py', "            if d > g.anchor_tol:", "            if False:"),
    ('stable speed', PKG + 'player_core.py', "        if speed > g.v_stable:", "        if False:"),
    ('stable time', PKG + 'player_core.py', "            if inp.now - self._t_stable0 < self.g.stable_s:", "            if False:"),
    ('foreign pub before', PKG + 'player_core.py',
     "            if inp.foreign_publishers > 0:\n                self._enter(out, Phase.ABORT, f'refusing: {inp.foreign_publishers} other '",
     "            if False:\n                self._enter(out, Phase.ABORT, f'refusing: {inp.foreign_publishers} other '"),
    ('foreign pub in flight', PKG + 'player_core.py', "        if inp.foreign_publishers > 0:\n            return (", "        if False:\n            return ("),
    ('odom stale in flight', PKG + 'player_core.py',
     "        if not self._fresh(inp.odom, now, g.odom_timeout):\n            return f'odom stale",
     "        if False:\n            return f'odom stale"),
    ('tracking error', PKG + 'player_core.py', "            if err > g.track_tol:", "            if False:"),
    ('default track_tol loosened', PKG + 'player_core.py', "    track_tol: float = 0.60", "    track_tol: float = 0.70"),
    ('release on abort', PKG + 'player_core.py', "            out.release = not self.shadow and self._t_auth0 is not None", "            out.release = False"),
    ('shadow authority', PKG + 'player_core.py', "            out.take_authority = not self.shadow", "            out.take_authority = True"),
    ('server match', PKG + 'player_core.py', "            if not self.shadow and not inp.server_matched:", "            if False:"),
    ('usb guard', 'scripts/run.bash', '[ "${RR_USB_ENABLED:-0}" = "0" ] || refuse', 'true || refuse'),
    ('profile guard', 'scripts/run.bash', 'if [ ! -f "${dds_dir}/${fast_file}" ]; then', 'if false; then'),
    ('own-ip marker guard', 'scripts/run.bash', 'if grep -q \'__OWN_IP__\' "${dds_dir}/${fast_file}"; then', 'if false; then'),
    ('rehearse domain guard', 'scripts/rehearse.bash', '    0|66) refuse', '    999) refuse'),
    ('in-container domain guard', REH + 'run_rehearsal.sh', "    ''|0|66) refuse", "    999) refuse"),
    ('launch fly default', 'ros2_ws/src/crazyflies_traj/launch/traj_player.launch.py', "('fly', 'false',", "('fly', 'true',"),
    ('node fly default', PKG + 'traj_player_node.py', "p('fly', False)", "p('fly', True)"),
    ('traj run fly kv', 'scripts/traj.bash', 'fly:=*|csv_path:=*) echo', 'nomatch) echo'),
    ('checker early cmd', REH + 'check_recording.py', "            if early:", "            if False:"),
    ('checker stop_after', REH + 'check_recording.py', "            if late:", "            if False:"),
]


def main():
    tmp = tempfile.mkdtemp(prefix='traj-mut-')
    work = os.path.join(tmp, 'repo')
    missed = 0
    try:
        for name, path, a, b in M:
            shutil.rmtree(work, ignore_errors=True)
            shutil.copytree(SRC, work, ignore=shutil.ignore_patterns(
                '.git', 'build', 'install', 'log', 'rehearsal_out', '__pycache__'))
            p = os.path.join(work, path)
            with open(p) as f:
                s = f.read()
            if s.count(a) != 1:
                print(f'STALE {name}: anchor found {s.count(a)} times in {path}')
                missed += 1
                continue
            with open(p, 'w') as f:
                f.write(s.replace(a, b))
            r = subprocess.run([sys.executable, '-m', 'pytest', '-q', '-x', '-p',
                                'no:cacheprovider'], cwd=work, capture_output=True, text=True)
            first = next((ln for ln in r.stdout.splitlines() if ln.startswith('FAILED')), '')
            red = r.returncode != 0
            print(f'{"RED  " if red else "GREEN"} {name}: {first[:110]}')
            missed += 0 if red else 1
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print(f'{len(M) - missed}/{len(M)} mutations caught')
    return 1 if missed else 0


if __name__ == '__main__':
    sys.exit(main())
