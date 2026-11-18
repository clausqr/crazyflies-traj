#!/usr/bin/env python3
"""
Interface drift guard: the apt message packages in the image must carry exactly
the fields and constants traj_player uses, as read from the crazyswarm2 pin the
plant builds (crazyflies-gs ros2_ws/src/crazyswarm2 @ f1e0995):
crazyflie_interfaces/msg/{FullState,Position,Status}.msg and
motion_capture_tracking_interfaces/msg/{NamedPose,NamedPoseArray}.msg.

    check_interfaces.py            exit 0 = match, 1 = drift (one line per field)
    check_interfaces.py --self-test   seeds a drift and requires the check to fail
"""

import sys

EXPECTED = {
    'FullState': {'header': 'std_msgs/Header', 'pose': 'geometry_msgs/Pose',
                  'twist': 'geometry_msgs/Twist', 'acc': 'geometry_msgs/Vector3'},
    'Position': {'header': 'std_msgs/Header', 'x': 'float', 'y': 'float', 'z': 'float',
                 'yaw': 'float'},
    'Status': {'header': 'std_msgs/Header', 'supervisor_info': 'uint16',
               'battery_voltage': 'float', 'pm_state': 'uint8', 'rssi': 'uint8',
               'num_rx_broadcast': 'uint32', 'num_tx_broadcast': 'uint32',
               'num_rx_unicast': 'uint32', 'num_tx_unicast': 'uint32',
               'latency_unicast': 'uint16'},
    'NamedPoseArray': {'header': 'std_msgs/Header',
                       'poses': 'sequence<motion_capture_tracking_interfaces/NamedPose>'},
    'NamedPose': {'name': 'string', 'pose': 'geometry_msgs/Pose'},
}
STATUS_BITS = {'SUPERVISOR_INFO_CAN_BE_ARMED': 1, 'SUPERVISOR_INFO_IS_ARMED': 2,
               'SUPERVISOR_INFO_AUTO_ARM': 4, 'SUPERVISOR_INFO_CAN_FLY': 8,
               'SUPERVISOR_INFO_IS_FLYING': 16, 'SUPERVISOR_INFO_IS_TUMBLED': 32,
               'SUPERVISOR_INFO_IS_LOCKED': 64}


def actual():
    from crazyflie_interfaces.msg import FullState, Position, Status
    from motion_capture_tracking_interfaces.msg import NamedPose, NamedPoseArray
    types = {m.__name__: m.get_fields_and_field_types()
             for m in (FullState, Position, Status, NamedPoseArray, NamedPose)}
    bits = {k: getattr(Status, k) for k in STATUS_BITS if hasattr(Status, k)}
    return types, bits


def diff(types, bits, expected=EXPECTED, expected_bits=STATUS_BITS):
    out = []
    for name, fields in expected.items():
        got = types.get(name, {})
        if got != fields:
            out.append(f'{name}: expected {fields}, got {got}')
    for k, v in expected_bits.items():
        if bits.get(k) != v:
            out.append(f'Status.{k}: expected {v}, got {bits.get(k)}')
    return out


def main(argv):
    types, bits = actual()
    if '--self-test' in argv:
        seeded = dict(types)
        seeded['Position'] = {k: v for k, v in types['Position'].items() if k != 'yaw'}
        bad = diff(seeded, dict(bits, SUPERVISOR_INFO_IS_LOCKED=128))
        ok = len(bad) == 2
        print(('PASS' if ok else 'FAIL') + f' check_interfaces self-test: seeded drift -> '
              f'{len(bad)} finding(s): {bad}')
        return 0 if ok else 1
    bad = diff(types, bits)
    for line in bad:
        print(f'FAIL interface drift: {line}')
    if not bad:
        print('PASS interfaces: FullState, Position, Status (+ bits), NamedPose(Array) match '
              'crazyswarm2 f1e0995')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
