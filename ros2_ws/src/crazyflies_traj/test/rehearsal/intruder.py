#!/usr/bin/env python3
"""
TEST-ONLY second command path for the rehearsal: creates a publisher on
``/<ns>/cmd_position`` and NEVER publishes on it. The publisher's mere presence
in the graph is what traj_player must detect (single command authority, the
readiness gate D3 counts publishers, not messages).

    intruder.py --record REC.jsonl [--ns cf1] [--hold 30]

Appends one ``intruder`` JSON line (with ``mono``) when the publisher exists.
"""

import argparse
import json
import time

from crazyflie_interfaces.msg import Position
import rclpy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ns', default='cf1')
    ap.add_argument('--record', required=True)
    ap.add_argument('--hold', type=float, default=30.0)
    a = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node('intruder')
    node.create_publisher(Position, f'/{a.ns}/cmd_position', 10)
    with open(a.record, 'a', buffering=1) as f:
        f.write(json.dumps({'ev': 'intruder', 'mono': round(time.monotonic(), 4)}) + '\n')
    t_end = time.monotonic() + a.hold
    try:
        while time.monotonic() < t_end:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
