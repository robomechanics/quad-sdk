#!/usr/bin/env python3
"""Drive the Go2 stock controller at a constant body velocity for a fixed time.

Replaces the joystick for repeatable straight-line trials: publishes the sport
API "Move" request (api_id 1008, same as unitree_sdk2 SportClient::Move) on
/api/sport/request at RATE Hz for DURATION s, then sends "StopMove" (1003).
Ctrl-C at any point also sends StopMove.

Runs inside the recorder image (needs unitree_api), via go2_sport_move.sh.

  go2_sport_move.py [--vx 0.5] [--vy 0] [--wz 0] [--duration 4] [--countdown 3]
"""
import argparse
import json
import sys
import time

import rclpy
from unitree_api.msg import Request

API_STOPMOVE = 1003
API_MOVE = 1008
RATE = 20.0          # Hz; Move must be refreshed or the controller stops
VX_MAX, VY_MAX, WZ_MAX = 1.0, 0.5, 1.0   # sanity caps for lab trials


def make_request(api_id, param=''):
    msg = Request()
    msg.header.identity.id = time.monotonic_ns()
    msg.header.identity.api_id = api_id
    msg.parameter = param
    return msg


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--vx', type=float, default=0.5, help='forward m/s (0.5)')
    ap.add_argument('--vy', type=float, default=0.0, help='left m/s (0)')
    ap.add_argument('--wz', type=float, default=0.0, help='yaw rad/s (0)')
    ap.add_argument('--duration', type=float, default=4.0, help='walk time s (4)')
    ap.add_argument('--countdown', type=float, default=3.0, help='delay before moving s (3)')
    a = ap.parse_args()

    if abs(a.vx) > VX_MAX or abs(a.vy) > VY_MAX or abs(a.wz) > WZ_MAX:
        sys.exit(f'refusing: |vx|<={VX_MAX}, |vy|<={VY_MAX}, |wz|<={WZ_MAX}')
    if not 0 < a.duration <= 30:
        sys.exit('refusing: duration must be in (0, 30] s')

    rclpy.init()
    node = rclpy.create_node('go2_sport_move')
    pub = node.create_publisher(Request, '/api/sport/request', 10)
    move = json.dumps({'x': a.vx, 'y': a.vy, 'z': a.wz})

    def stop():
        for _ in range(5):
            pub.publish(make_request(API_STOPMOVE))
            time.sleep(0.05)

    try:
        # Give DDS discovery time to match the robot's subscriber, else the
        # first commands are dropped.
        t_end = time.monotonic() + max(a.countdown, 1.0)
        while time.monotonic() < t_end:
            n = pub.get_subscription_count()
            print(f'\rstarting in {t_end - time.monotonic():3.1f} s '
                  f'(subscribers on /api/sport/request: {n})', end='', flush=True)
            time.sleep(0.1)
        print()
        if pub.get_subscription_count() == 0:
            sys.exit('no subscriber on /api/sport/request; is the Go2 in sport mode '
                     'and is this on the 192.168.123.x interface?')

        print(f'MOVE vx={a.vx} vy={a.vy} wz={a.wz} for {a.duration} s  (Ctrl-C = stop)')
        t0 = time.monotonic()
        k = 0
        while (t := time.monotonic() - t0) < a.duration:
            pub.publish(make_request(API_MOVE, move))
            k += 1
            time.sleep(max(0.0, t0 + k / RATE - time.monotonic()))
        print(f'sent {k} Move requests over {t:.2f} s')
    except KeyboardInterrupt:
        print('\ninterrupted')
    finally:
        stop()
        print('StopMove sent')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
