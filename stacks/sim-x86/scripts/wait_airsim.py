#!/usr/bin/env python3
"""Wait for the simulation RPC server before starting one-shot ROS clients."""
import time
import socket
import argparse
import rospy

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--tcp-only', action='store_true')
parser.add_argument('--host')
parser.add_argument('--port', type=int)
args = parser.parse_args()
if not args.tcp_only:
    import airsim

if (args.host is None) != (args.port is None):
    parser.error('--host and --port must be supplied together')
if args.host is None:
    rospy.init_node('wait_airsim', anonymous=True, disable_signals=True)
    host = rospy.get_param('/system/sim/airsim_ip')
    port = int(rospy.get_param('/system/sim/airsim_port'))
else:
    host, port = args.host, args.port
deadline = time.monotonic() + 60
last_error = None
while time.monotonic() < deadline:
    try:
        with socket.create_connection((host, port), timeout=1):
            pass
        if args.tcp_only:
            break
        client = airsim.MultirotorClient(ip=host, port=port, timeout_value=2)
        if client.ping():
            print('AirSim RPC ready at {}:{}'.format(host, port), flush=True)
            break
    except Exception as exc:
        last_error = str(exc)
    time.sleep(0.5)
else:
    raise SystemExit('AirSim RPC unavailable at {}:{}: {}'.format(host, port, last_error))
