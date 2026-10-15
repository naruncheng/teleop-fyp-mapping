"""Open the MuJoCo viewer with one or more follower robots tracking a leader.

Leader arm in the scene, driven by its joint readings (FK done here):

    python run_sim.py --joints-demo          # leader sways, driven from code
    python run_sim.py --joints-opcua opc.tcp://<pi>:4840   # real mini-arm, via miniarm_v2.py
    python run_sim.py --joints-udp 9500      # leader follows joint readings sent over UDP

Leader that does its own FK and sends a pose (no leader drawn):

    python run_sim.py                        # no leader: followers hold their home poses
    python run_sim.py --demo                 # target sweeps around, driven from code
    python run_sim.py --udp 9400             # follow poses sent as JSON over UDP

Pick the followers with --followers (default: kuka):

    python run_sim.py --joints-demo --followers kuka scara

Each follower's arrow is its target pose (where the leader says its tool
should be); its tool chases it.
"""

import argparse
import logging
import time

import mujoco
import mujoco.viewer
import numpy as np

from follower import Follower
from leader import DemoJoints, LeaderArm, UdpJointSource
from pose_sources import DemoPose, UdpPoseSource
from scene import DEFAULT_LEADER_OFFSET, FOLLOWERS, build_model

# The target only updates at the leader's rate and IK is the expensive part,
# so solve every Nth physics step rather than every one (500 Hz physics ->
# 50 Hz IK).
IK_INTERVAL = 10


def make_pose_source(args, follower):
    if args.demo:
        # Centre the demo on the follower's home tool pose, expressed back in
        # the leader's frame so it goes through the same mapping a real
        # leader does.
        center_pos, center_rot = follower.world_to_leader(*follower.tool_pose())
        return DemoPose(center_pos, center_rot, size=1.0 / follower.scale)
    if args.udp is not None:
        logging.info("listening for poses on UDP :%d", args.udp)
        return UdpPoseSource(args.udp)
    return None


def make_joint_source(args):
    if args.joints_demo:
        return DemoJoints()
    if args.joints_opcua is not None:
        from opcua_source import OpcUaJointSource  # imported late so asyncua is optional
        logging.info("reading leader joints from %s (%s convention)",
                     args.joints_opcua, args.opcua_convention)
        return OpcUaJointSource(args.joints_opcua, convention=args.opcua_convention)
    if args.joints_udp is not None:
        logging.info("listening for leader joint readings on UDP :%d", args.joints_udp)
        return UdpJointSource(args.joints_udp)
    return None


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--joints-demo", action="store_true",
                        help="show the leader arm, swaying its joints from code")
    source.add_argument("--joints-opcua", metavar="URL",
                        help="show the leader arm, driven by miniarm_v2.py's OPC UA server, "
                             "e.g. opc.tcp://raspberrypi.local:4840")
    source.add_argument("--joints-udp", type=int, metavar="PORT",
                        help="show the leader arm, driven by joint readings sent to this port")
    source.add_argument("--demo", action="store_true",
                        help="animate the target pose from code (no leader arm)")
    source.add_argument("--udp", type=int, metavar="PORT",
                        help="follow poses sent as JSON datagrams to this port (no leader arm)")
    parser.add_argument("--followers", nargs="+", choices=sorted(FOLLOWERS), default=["kuka"],
                        help="which robots follow the leader (default: %(default)s)")
    parser.add_argument("--opcua-convention", choices=("kuka", "model"), default="kuka",
                        help="joint convention of the OPC UA readings: 'kuka' = miniarm_v2.py's "
                             "KUKA-ready angles (converted to the leader model's), 'model' = "
                             "already the leader model's (default: %(default)s)")
    parser.add_argument("--leader-offset", type=float, nargs=3, metavar=("X", "Y", "Z"),
                        default=DEFAULT_LEADER_OFFSET,
                        help="where the leader arm is drawn (default: %(default)s)")
    for name, spec in FOLLOWERS.items():
        parser.add_argument(f"--{name}-scale", type=float, nargs="+", metavar="S",
                            help=f"{name}: scale on leader position, one value or X Y Z "
                                 f"(default: {' '.join(f'{s:g}' for s in spec.scale)})")
        parser.add_argument(f"--{name}-origin", type=float, nargs=3, metavar=("X", "Y", "Z"),
                            help=f"{name}: where the leader's base maps to, relative to the "
                                 f"{name}'s base (default: {' '.join(f'{o:g}' for o in spec.origin)})")
    parser.add_argument("--position-only", action="store_true",
                        help="ignore orientation, match position only")
    parser.add_argument("--rot-weight", type=float, default=1.0,
                        help="full-orientation followers: how much orientation matters vs position "
                             "where both can't be met; 1.0 favours orientation, ~0.02 favours "
                             "position (default: %(default)s)")
    args = parser.parse_args()

    for name in FOLLOWERS:
        scale = getattr(args, f"{name}_scale")
        if scale is not None and len(scale) not in (1, 3):
            parser.error(f"--{name}-scale takes 1 value or 3 (X Y Z), got {len(scale)}")
    return args


def main():
    args = parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")

    with_leader = (args.joints_demo or args.joints_opcua is not None
                   or args.joints_udp is not None)
    model = build_model(followers=args.followers,
                        leader_offset=args.leader_offset if with_leader else None)
    data = mujoco.MjData(model)

    followers = [Follower(model, data, name,
                          scale=getattr(args, f"{name}_scale"),
                          origin=getattr(args, f"{name}_origin"),
                          rot_weight=args.rot_weight, position_only=args.position_only)
                 for name in args.followers]
    for f in followers:
        logging.info("%s: scale %s, origin %s, orientation %s", f.name,
                     np.round(f.scale, 3).tolist(), np.round(f.origin, 3).tolist(),
                     "ignored" if f.position_only else f.orientation)

    leader = LeaderArm(model, data) if with_leader else None
    joint_source = make_joint_source(args)
    pose_source = make_pose_source(args, followers[0])

    step_count = 0
    last_report = 0.0
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            # Shadows, reflections and haze off by default; still toggleable
            # in the viewer's Rendering panel.
            with viewer.lock():
                viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
                viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0
                viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_HAZE] = 0
            while viewer.is_running():
                step_start = time.time()

                if leader is not None:
                    joints = joint_source.latest()
                    if joints is not None:
                        leader.set_joints(joints)
                    leader.apply()  # every step: the leader has no motors to hold it

                if step_count % IK_INTERVAL == 0:
                    if leader is not None:
                        # Until the first reading arrives the leader is just
                        # standing at zero; don't send the followers there.
                        pose = leader.pose() if joint_source.latest() is not None else None
                    else:
                        pose = pose_source.latest() if pose_source is not None else None

                    report = time.time() - last_report >= 1.0  # measure, don't eyeball
                    for f in followers:
                        if pose is not None:
                            f.set_target(pose)
                        ik_error, rot_error = f.track()
                        if report:
                            # Two different numbers: how well IK solved, versus
                            # how far the arm actually is right now (servos need
                            # time to catch up, so the second lags while moving).
                            rot_note = ("" if rot_error is None
                                        else " | orientation: %.2f deg" % np.degrees(rot_error))
                            logging.info("%s: IK solve error: %.2f mm%s | actual tool gap: %.1f mm",
                                         f.name, ik_error * 1000, rot_note, f.tool_gap() * 1000)
                    if report:
                        last_report = time.time()

                mujoco.mj_step(model, data)  # advance physics by one timestep
                viewer.sync()                # push the new state to the window
                step_count += 1

                # Sleep off whatever's left of the timestep so it runs in real time.
                remaining = model.opt.timestep - (time.time() - step_start)
                if remaining > 0:
                    time.sleep(remaining)
    finally:
        for src in (joint_source, pose_source):
            if src is not None:
                src.stop()


if __name__ == "__main__":
    main()
