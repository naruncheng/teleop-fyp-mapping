"""Stand-in leader: streams a circle of poses over UDP, to test the
--udp path of run_sim.py without a real leader arm.

    python run_sim.py --udp 9400        # in one terminal
    python send_test_pose.py            # in another

The circle is centred on the KUKA's home tool pose (expressed in the
leader's frame through the KUKA's default mapping), with orientation held
fixed, so it's only guaranteed reachable with run_sim.py's default
--kuka-scale and --kuka-origin.
"""

import argparse
import json
import socket
import time

import mujoco
import numpy as np

from follower import Follower
from scene import build_model


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9400)
    parser.add_argument("--rate", type=float, default=50.0, help="poses per second")
    parser.add_argument("--radius", type=float, default=0.1,
                        help="circle radius at the KUKA, metres (default: %(default)s)")
    args = parser.parse_args()

    model = build_model(followers=("kuka",))
    kuka = Follower(model, mujoco.MjData(model), "kuka")
    center, rotation = kuka.world_to_leader(*kuka.tool_pose())
    radius = args.radius / kuka.scale
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"sending to {args.host}:{args.port} at {args.rate:.0f} Hz (Ctrl+C to stop)")

    start = time.time()
    try:
        while True:
            t = time.time() - start
            position = center + radius * np.array([0.0, np.cos(t), np.sin(t)])
            message = {"position": position.tolist(), "rotation": rotation.tolist()}
            sock.sendto(json.dumps(message).encode(), (args.host, args.port))
            time.sleep(1.0 / args.rate)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
