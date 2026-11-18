"""
The player state machine on plain data: nothing before airborne-and-stable, the
exact setpoints after, and every abort path (seeded RED each).
"""

import math

import pytest

from crazyflies_traj.plan import approach_plan, bind
from crazyflies_traj.player_core import (AUTO_ARM, CAN_FLY, Gates, Inputs, IS_ARMED,
                                         IS_CRASHED, IS_FLYING, IS_LOCKED, IS_TUMBLED, Phase,
                                         PlayerCore)
from crazyflies_traj.sampler import YawSpec
from crazyflies_traj.trajectory import Limits, norm, parse_csv, TrajError

Z0 = 0.05
HOVER = (0.0, 0.0, Z0 + 0.5)
GROUND_BITS = IS_ARMED | AUTO_ARM | CAN_FLY               # 14: auto-armed, on the pad
FLY_BITS = GROUND_BITS | IS_FLYING                        # 30: motors above idle
TRAJ = parse_csv('x,y,z,t\n0,0,0.5,0\n0.4,0,0.5,2\n0.4,0,0.5,2.5\n0,0,0.5,4.5\n')
DT = 0.02


def make(shadow=False, frame='absolute', traj=TRAJ, approach=True, sync=False, yaw=YawSpec(),
         **gates):
    g = Gates(**gates)

    def bind_fn(anchor_xy, z0):
        return bind(traj, Limits(), frame, 'linear', z0, anchor_xy, yaw, 1.0 / DT)

    def approach_fn(pos, first, z0, yaw):
        return approach_plan(pos, first, z0, Limits(), g.approach_v, yaw)
    return PlayerCore(bind_fn, g, start_tol=0.15, shadow=shadow, sync=sync,
                      approach_fn=approach_fn if approach else None)


class World:
    """A scripted plant: fresh streams by default, overridable per tick."""

    def __init__(self):
        self.now = 100.0
        self.bits = FLY_BITS
        self.pos = HOVER
        self.speed = 0.0
        self.yaw = None              # odom yaw (rad)
        self.z0 = Z0
        self.mocap = None            # None -> same as odom
        self.foreign = 0
        self.matched = True
        self.abort = None
        self.fleet_ready = True
        self.fleet_waiting = ''
        self.status_age = 0.1
        self.odom_age = 0.0
        self.poses_age = 0.0

    def inputs(self):
        mocap = self.mocap if self.mocap is not None else self.pos
        return Inputs(now=self.now,
                      status=(self.now - self.status_age, self.bits),
                      odom=(self.now - self.odom_age, self.pos, self.speed), yaw=self.yaw,
                      poses=(self.now - self.poses_age, mocap),
                      z0=self.z0, foreign_publishers=self.foreign,
                      server_matched=self.matched, abort=self.abort,
                      fleet_ready=self.fleet_ready, fleet_waiting=self.fleet_waiting)


def run(core, w, seconds, follow=True, until=None):
    """Tick for ``seconds``; the vehicle follows each setpoint exactly when ``follow``."""
    outs = []
    for _ in range(int(round(seconds / DT))):
        out = core.step(w.inputs())
        outs.append(out)
        if follow and out.setpoint is not None:
            w.pos = out.setpoint.pos
        w.now += DT
        if until is not None and core.phase == until:
            break
    return outs


def published(outs):
    return [o.setpoint for o in outs if o.setpoint is not None]


def fly_to(core, w, phase):
    outs = run(core, w, 30.0, until=phase)
    assert core.phase == phase, (core.phase, core.reason)
    return outs


# ---- happy path ------------------------------------------------------------

def test_full_flight_publishes_exactly_the_plan_then_holds_then_done():
    core, w = make(), World()
    outs = run(core, w, 12.0)
    phases = [p for o in outs for p, _ in o.entered]
    assert phases == [Phase.ARMED_READY, Phase.FLYING, Phase.HOLD, Phase.DONE], core.reason
    auth = [i for i, o in enumerate(outs) if o.take_authority]
    first_pub = next(i for i, o in enumerate(outs) if o.setpoint is not None)
    assert len(auth) == 1 and auth[0] < first_pub
    # nothing before stable_s of airborne-and-stable (1.0 s = 50 ticks)
    assert first_pub >= 50
    sps = published(outs)
    assert sps[0].pos == pytest.approx((0.0, 0.0, 0.55))
    assert sps[0].vel == (0.0, 0.0, 0.0)            # t = 0 is the hover point
    assert sps[1].vel == pytest.approx((0.2, 0.0, 0.0))
    # 4.5 s of trajectory at 50 Hz, then 3 s hold of the last point
    fly = [s for s in sps if s.vel != (0.0, 0.0, 0.0) or s.t < 4.5]
    assert 224 <= len(fly) <= 227
    hold = sps[len(fly):]
    assert 149 <= len(hold) <= 151
    assert all(s.pos == pytest.approx((0.0, 0.0, 0.55)) and s.vel == (0.0, 0.0, 0.0)
               for s in hold)
    done = [o for o in outs if any(p == Phase.DONE for p, _ in o.entered)][0]
    assert done.release and done.setpoint is None
    assert all(o.setpoint is None for o in outs[outs.index(done):])


def test_nothing_is_published_while_waiting_even_for_ever():
    core, w = make(wait_timeout=1000.0), World()
    w.pos = (0.0, 0.0, Z0)            # on the pad
    w.bits = GROUND_BITS
    outs = run(core, w, 60.0)
    assert core.phase == Phase.WAIT_AIRBORNE
    assert not any(o.setpoint or o.take_authority for o in outs)
    assert 'IS_FLYING' in outs[-1].waiting_on


def test_wait_timeout_refuses_without_authority():
    core, w = make(wait_timeout=5.0), World()
    w.z0 = None
    outs = run(core, w, 10.0)
    assert core.phase == Phase.ABORT and 'within 5 s' in core.reason
    assert not any(o.setpoint or o.take_authority or o.release for o in outs)


# ---- the airborne gates, one seeded RED each --------------------------------

@pytest.mark.parametrize('field,value,why', [
    ('z0', None, 'pad z0 unknown'),
    ('status_age', 5.0, 'no fresh /<ns>/status'),
    ('bits', GROUND_BITS, 'IS_FLYING'),
    ('bits', FLY_BITS & ~IS_ARMED, 'IS_ARMED'),
    ('bits', FLY_BITS & ~CAN_FLY, 'CAN_FLY'),
    ('odom_age', 1.0, 'no fresh odom'),
    ('pos', (0.0, 0.0, Z0 + 0.20), 'height'),
    ('poses_age', 1.0, 'no fresh /poses'),
    ('mocap', (0.2, 0.0, Z0 + 0.5), 'EKF not anchored'),
    ('speed', 0.4, 'speed'),
])
def test_each_airborne_gate_blocks(field, value, why):
    core, w = make(wait_timeout=1000.0), World()
    setattr(w, field, value)
    outs = run(core, w, 5.0)
    assert core.phase == Phase.WAIT_AIRBORNE
    assert not any(o.setpoint or o.take_authority for o in outs)
    assert why in outs[-1].waiting_on


def test_require_poses_false_skips_mocap():
    core, w = make(require_poses=False), World()
    w.poses_age = 99.0
    fly_to(core, w, Phase.FLYING)


@pytest.mark.parametrize('bit', [IS_LOCKED, IS_TUMBLED, IS_CRASHED])
def test_fatal_bits_refuse_before_any_command(bit):
    core, w = make(), World()
    w.bits = FLY_BITS | bit
    outs = run(core, w, 2.0)
    assert core.phase == Phase.ABORT and 'refusing' in core.reason
    assert not any(o.setpoint or o.take_authority or o.release for o in outs)


def test_height_hysteresis():
    core, w = make(wait_timeout=1000.0), World()
    w.pos = (0.0, 0.0, Z0 + 0.20)                  # below h_enter 0.25
    run(core, w, 0.5)
    assert not core.airborne
    w.pos = (0.0, 0.0, Z0 + 0.26)
    run(core, w, 0.1)
    assert core.airborne
    w.pos = (0.0, 0.0, Z0 + 0.20)                  # inside the band: stays airborne
    run(core, w, 0.1)
    assert core.airborne
    w.pos = (0.0, 0.0, Z0 + 0.14)                  # below h_exit 0.15
    run(core, w, 0.1)
    assert not core.airborne


def test_stability_timer_resets_on_a_disturbance():
    core, w = make(), World()
    run(core, w, 0.8)                              # 0.8 s stable
    w.speed = 0.5
    run(core, w, 0.04)                             # one gust
    w.speed = 0.0
    outs = run(core, w, 0.9)                       # 0.9 s < 1.0 s since the reset
    assert core.phase == Phase.WAIT_AIRBORNE and not any(o.take_authority for o in outs)
    run(core, w, 0.2)
    assert core.phase in (Phase.ARMED_READY, Phase.FLYING)


def test_bad_gates_refused():
    with pytest.raises(TrajError):
        make(h_exit=0.3, h_enter=0.25)
    with pytest.raises(TrajError):
        make(track_tol=0.0)


# ---- trigger-time refusals ---------------------------------------------------

def test_start_far_refused_without_authority():
    # no approach_fn: a far start is refused, as before the approach existed
    core, w = make(approach=False), World()
    w.pos = (1.0, 1.0, Z0 + 0.5)
    outs = run(core, w, 3.0)
    assert core.phase == Phase.ABORT and 'start_far' in core.reason
    assert not any(o.setpoint or o.take_authority for o in outs)


def test_far_start_is_approached_then_streamed_without_a_gap():
    core, w = make(), World()
    w.pos = (1.0, 1.0, Z0 + 0.5)                   # 1.41 m from the first setpoint
    outs = run(core, w, 18.0)
    phases = [p for o in outs for p, _ in o.entered]
    assert phases == [Phase.ARMED_READY, Phase.APPROACH, Phase.FLYING, Phase.HOLD,
                      Phase.DONE], core.reason
    sps = published(outs)
    assert sps[0].pos == pytest.approx((1.0, 1.0, 0.55)) and sps[0].vel == (0.0, 0.0, 0.0)
    assert all(norm(s.vel) <= 0.30 + 1e-9 for s in sps)        # approach_v, then 0.2 m/s
    first_pub = next(i for i, o in enumerate(outs) if o.setpoint is not None)
    fly = next(i for i, o in enumerate(outs) if (Phase.FLYING, ) == tuple(p for p, _ in o.entered))
    done = next(i for i, o in enumerate(outs) if any(p == Phase.DONE for p, _ in o.entered))
    assert outs[fly].setpoint.pos == pytest.approx((0.0, 0.0, 0.55))
    assert all(o.setpoint is not None for o in outs[first_pub:done])   # never a pause


def test_relative_frame_anchors_at_the_airborne_position():
    core, w = make(frame='relative'), World()
    w.pos = w.mocap = (1.0, 1.0, Z0 + 0.5)
    outs = run(core, w, 2.0, until=Phase.FLYING)
    assert core.phase == Phase.FLYING
    assert published(outs)[0].pos == pytest.approx((1.0, 1.0, 0.55))


def test_relative_geofence_at_the_anchor_refused():
    core, w = make(frame='relative'), World()
    w.pos = (1.4, 0.0, Z0 + 0.5)                   # +0.4 m in x leaves the 1.75 box
    outs = run(core, w, 3.0)
    assert core.phase == Phase.ABORT and 'geofence' in core.reason
    assert not any(o.setpoint or o.take_authority for o in outs)


def test_foreign_publisher_refused_before_authority():
    core, w = make(), World()
    w.foreign = 1                                  # e.g. another plant client is up
    outs = run(core, w, 3.0)
    assert core.phase == Phase.ABORT and 'single command authority' in core.reason
    assert not any(o.setpoint or o.take_authority for o in outs)


def test_server_never_matches_refuses_and_releases():
    core, w = make(), World()
    w.matched = False
    outs = run(core, w, 6.0)
    assert core.phase == Phase.ABORT and 'never matched' in core.reason
    assert any(o.take_authority for o in outs)
    assert not published(outs)
    assert outs[[i for i, o in enumerate(outs) if o.entered][-1]].release


# ---- in-flight aborts: stop publishing on the same tick --------------------

def _abort_mid_flight(mutate, expect):
    core, w = make(), World()
    fly_to(core, w, Phase.FLYING)
    run(core, w, 1.0)
    mutate(w)
    outs = run(core, w, 2.0)
    ab = [o for o in outs if any(p == Phase.ABORT for p, _ in o.entered)]
    assert ab, core.reason
    assert expect in core.reason
    assert ab[0].release and ab[0].setpoint is None
    assert not published(outs[outs.index(ab[0]):])
    return core


@pytest.mark.parametrize('mutate,expect', [
    (lambda w: setattr(w, 'bits', FLY_BITS | IS_LOCKED), 'IS_LOCKED'),
    (lambda w: setattr(w, 'bits', FLY_BITS | IS_TUMBLED), 'IS_TUMBLED'),
    (lambda w: setattr(w, 'bits', FLY_BITS | IS_CRASHED), 'IS_CRASHED'),
    (lambda w: setattr(w, 'bits', GROUND_BITS), 'IS_FLYING cleared'),
    (lambda w: setattr(w, 'status_age', 3.5), '/<ns>/status stale'),
    (lambda w: setattr(w, 'odom_age', 0.3), 'odom stale'),
    (lambda w: setattr(w, 'poses_age', 0.3), '/poses stale'),
    (lambda w: setattr(w, 'foreign', 1), 'single command authority'),
    (lambda w: setattr(w, 'abort', 'SIGINT'), 'operator abort: SIGINT'),
])
def test_in_flight_aborts(mutate, expect):
    _abort_mid_flight(mutate, expect)


def _out_and_back(dx):
    return parse_csv(f'x,y,z,t\n0,0,0.5,0\n{dx},0,0.5,3\n{dx},0,0.5,3.5\n0,0,0.5,6.5\n')


def test_tracking_error_aborts():
    # track_tol 0.60 m: the vehicle stays put while the setpoint runs 0.65 m away.
    core, w = make(traj=_out_and_back(0.65)), World()
    fly_to(core, w, Phase.FLYING)
    outs = run(core, w, 4.0, follow=False)
    assert core.phase == Phase.ABORT and 'tracking error' in core.reason
    ab = [o for o in outs if o.entered][-1]
    assert ab.release and not published(outs[outs.index(ab):])


def test_tracking_error_inside_track_tol_does_not_abort():
    # 0.55 m away at the worst, under track_tol 0.60: the flight completes.
    core, w = make(traj=_out_and_back(0.55)), World()
    fly_to(core, w, Phase.FLYING)
    run(core, w, 12.0, follow=False)
    assert core.phase == Phase.DONE, core.reason


def test_abort_during_hold():
    core, w = make(), World()
    fly_to(core, w, Phase.HOLD)
    w.bits = FLY_BITS | IS_LOCKED
    outs = run(core, w, 0.1)
    assert core.phase == Phase.ABORT and not published(outs)


def test_terminal_phases_are_sticky_and_silent():
    core, w = make(), World()
    w.abort = 'test'
    run(core, w, 0.1)
    w.abort = None
    outs = run(core, w, 5.0)
    assert core.phase == Phase.ABORT
    assert not any(o.setpoint or o.take_authority or o.release or o.entered for o in outs)


# ---- the dry run (shadow) never takes authority ----------------------------

def test_shadow_runs_the_gates_but_never_takes_authority():
    core, w = make(shadow=True), World()
    w.matched = False                              # no publisher, so never matched
    outs = run(core, w, 12.0, follow=False)       # nothing flies: no tracking check
    phases = [p for o in outs for p, _ in o.entered]
    assert phases == [Phase.ARMED_READY, Phase.FLYING, Phase.HOLD, Phase.DONE]
    assert not any(o.take_authority or o.release for o in outs)
    assert published(outs)                         # the would-send setpoints, for the log


def test_shadow_still_refuses_a_foreign_publisher():
    core, w = make(shadow=True), World()
    w.foreign = 1
    run(core, w, 3.0)
    assert core.phase == Phase.ABORT


# ---- the fleet barrier -------------------------------------------------------

def test_sync_holds_at_the_first_setpoint_until_the_fleet_is_ready():
    core, w = make(sync=True), World()
    w.fleet_ready, w.fleet_waiting = False, 'cf2'
    outs = fly_to(core, w, Phase.READY)
    assert outs[-1].ready                                  # latch /<ns>/traj_ready once
    hold = run(core, w, 2.0)
    assert core.phase == Phase.READY and 'cf2' in hold[-1].waiting_on
    assert all(o.setpoint is not None and o.setpoint.pos == pytest.approx((0.0, 0.0, 0.55))
               and o.setpoint.vel == (0.0, 0.0, 0.0) for o in hold)   # streaming the hold
    assert not any(o.ready for o in hold)
    w.fleet_ready = True
    outs = run(core, w, 0.1)
    assert core.phase == Phase.FLYING
    assert any(p == Phase.FLYING and 'fleet ready' in why for o in outs for p, why in o.entered)
    assert outs[0].setpoint is not None                    # no gap on the hand-over tick
    run(core, w, 12.0)
    assert core.phase == Phase.DONE


def test_sync_timeout_goes_without_the_fleet():
    core, w = make(sync=True, sync_timeout=1.0), World()
    w.fleet_ready = False
    fly_to(core, w, Phase.READY)
    outs = run(core, w, 1.2)
    assert core.phase == Phase.FLYING
    assert any(p == Phase.FLYING and 'sync_timeout' in why for o in outs for p, why in o.entered)


# ---- yaw: path ------------------------------------------------------------------

PATH = YawSpec(yaw='path')


@pytest.mark.parametrize('start_deg', [90.0, 25.0])
def test_path_yaw_turns_from_the_measured_yaw_at_the_rate_limit(start_deg):
    # At the first setpoint, nose start_deg off the first segment's heading (+X, 0 deg):
    # the approach is a pure turn from the odom yaw (no step) at
    # yaw_rate_max_dps, and FLYING starts aligned.
    core, w = make(yaw=PATH), World()
    w.yaw = math.radians(start_deg)
    outs = run(core, w, 3.0, until=Phase.FLYING)
    phases = [p for o in outs for p, _ in o.entered]
    assert phases == [Phase.ARMED_READY, Phase.APPROACH, Phase.FLYING], core.reason
    assert core.approach.traj.duration == pytest.approx(start_deg / 90.0)
    sps = published(outs)
    yaws = [math.degrees(s.yaw) for s in sps]
    assert yaws[0] == pytest.approx(start_deg)                     # starts where the nose is
    assert all(b <= a + 1e-9 for a, b in zip(yaws, yaws[1:]))      # turns one way
    assert all(abs(b - a) <= 90.0 * DT + 1e-6 for a, b in zip(yaws, yaws[1:]))
    assert math.degrees(sps[0].yaw_rate) == pytest.approx(-90.0)
    assert yaws[-1] == pytest.approx(0.0, abs=1e-9)                # FLYING, aligned


def test_fixed_yaw_ignores_the_measured_yaw():
    # yaw: fixed (default) keeps today's behaviour: no turn, yaw 0 from the first setpoint
    core, w = make(), World()
    w.yaw = math.radians(90.0)
    outs = run(core, w, 2.0, until=Phase.FLYING)
    assert [p for o in outs for p, _ in o.entered] == [Phase.ARMED_READY, Phase.FLYING]
    assert all(s.yaw == 0.0 and s.yaw_rate == 0.0 for s in published(outs))


def test_path_yaw_holds_the_last_heading_at_the_end():
    core, w = make(yaw=PATH), World()
    w.yaw = 0.0
    outs = run(core, w, 12.0)
    assert core.phase == Phase.DONE, core.reason
    hold = [o.setpoint for o in outs if o.phase == Phase.HOLD and o.setpoint is not None]
    assert hold and all(abs(math.degrees(s.yaw)) == pytest.approx(180.0) for s in hold)
    assert all(s.yaw_rate == 0.0 for s in hold)
