"""Stand-in for the physical leader: streams joint readings over UDP, to
test the --joints-udp path of run_sim.py without the real arm.

    python run_sim.py --joints-udp 9500     # in one terminal
    python send_test_joints.py              # in another

Sends the same motion as --joints-demo. Your real reader should send the
same message: {"joints": [q1, ..., q7]}, radians, leader model convention.
"""

import argparse
import json
import socket
import time

from leader import DemoJoints


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9500)
    parser.add_argument("--rate", type=float, default=50.0, help="readings per second")
    args = parser.parse_args()

    motion = DemoJoints()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"sending to {args.host}:{args.port} at {args.rate:.0f} Hz (Ctrl+C to stop)")
    try:
        while True:
            message = {"joints": motion.latest().tolist()}
            sock.sendto(json.dumps(message).encode(), (args.host, args.port))
            time.sleep(1.0 / args.rate)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
