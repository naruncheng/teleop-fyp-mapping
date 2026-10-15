"""Builds the MuJoCo model: the empty world, one or more follower robots,
and optionally the leader arm.

Every robot is a separate model merged in here rather than in XML: MuJoCo
attaches one model to another, prefixing everything from the child to avoid
name clashes. So the KUKA's joints become kuka_joint1..7, the SCARA's
scara_joint1..4, the leader's leader_joint1..7.
"""

import pathlib
from dataclasses import dataclass

import mujoco
import numpy as np

HERE = pathlib.Path(__file__).parent
WORLD_PATH = HERE / "world.xml"
LEADER_PATH = HERE / "models" / "leader_arm" / "urdf" / "miniarm.xml"


@dataclass(frozen=True)
class FollowerSpec:
    """A follower robot and how the leader's motion maps onto it.

    Every follower model must name its base body "base", its tool site
    "attachment_site" (+Z pointing out of the flange), and its rest pose
    keyframe "home", and have one position servo per joint (any names: they
    are found by the follower's prefix once attached).
    """
    path: pathlib.Path
    position: tuple   # where it stands in the world
    color: tuple      # its target marker's RGBA
    # Default mapping, overridable from run_sim.py:
    scale: tuple      # multiplier on leader position, per axis (x, y, z)
    origin: tuple     # where the leader's base maps to, relative to this robot's base
    orientation: str  # "full": match the leader's tool orientation
                      # "yaw": tool points down; match only rotation about vertical
    # For "yaw": the joint that turns the tool about vertical without moving
    # the tool point (a SCARA's axis 4). Yaw is set on it directly rather
    # than searched for by IK, so it can never wind up against its limit.
    yaw_joint: str = None
    # For a parallelogram four-bar driving the elbow from the shoulder (the
    # Taipan): (shoulder, elbow, crank, rod) joint names. IK solves the
    # shoulder and the passive elbow; the crank motor is then sent
    # shoulder - elbow. The rod pivot is passive and always equals the elbow.
    four_bar: tuple = None


FOLLOWERS = {
    # 2x: the scale this project was built around. Measured on --joints-demo:
    # every IK solve within 10 mm / 5 deg.
    "kuka": FollowerSpec(
        path=HERE / "models" / "kuka_iiwa_14" / "iiwa14.xml",
        position=(0.0, -0.6, 0.0),
        color=(1.0, 0.85, 0.0, 1.0),
        scale=(2.0, 2.0, 2.0),
        origin=(0.0, 0.0, 0.0),
        orientation="full",
    ),
    # MuJoCo Menagerie UR5e, unchanged. 6 DOF, so full pose but no spare
    # joint. 850 mm reach vs the leader's 400 mm: 2x fits, and 1.6-2.0 all
    # measured the same on --joints-demo (every solve within 10 mm / 5 deg
    # once past the startup swing from its home pose).
    "ur5e": FollowerSpec(
        path=HERE / "models" / "universal_robots_ur5e" / "ur5e.xml",
        position=(0.0, 0.6, 0.0),
        color=(1.0, 0.45, 0.1, 1.0),
        scale=(2.0, 2.0, 2.0),
        origin=(0.0, 0.0, 0.0),
        orientation="full",
    ),
    # Nova rover's Taipan arm, built from its Onshape URDF by
    # models/taipan/build_taipan.py. 6 motors, but j3 drives the elbow
    # through a shoulder-mounted four-bar, so IK works on the elbow and the
    # crank follows. Reach ~1.17 m from the shoulder vs the leader's 0.40 m.
    # Stands on its base box (0.163 m below the "base" frame). Origin z puts
    # the leader's shoulder (0.12 m up) on the Taipan's (0.20 m up).
    "taipan": FollowerSpec(
        path=HERE / "models" / "taipan" / "taipan.xml",
        position=(0.0, 1.8, 0.163),
        color=(0.85, 0.2, 0.5, 1.0),
        scale=(2.8, 2.8, 2.8),
        origin=(0.0, 0.0, 0.201 - 2.8 * 0.12),
        orientation="full",
        four_bar=("j2", "l1_l2_pivot", "j3", "l1_p_upper"),
    ),
    # A SCARA can't tilt its tool, and its vertical travel is only 180 mm
    # against a 550 mm horizontal reach, so horizontal and vertical motion
    # are scaled separately. Horizontal: the leader's 0.40 m max reach x 1.35
    # = 0.54 m, inside the SCARA's 0.55 m. Vertical: the leader's tool height
    # (5-95th percentile 0.04-0.45 m above its base) x 0.4 + 0.03 m lands in
    # the SCARA's 0.04-0.22 m stroke.
    "scara": FollowerSpec(
        path=HERE / "models" / "scara" / "scara.xml",
        position=(0.0, 3.0, 0.0),
        color=(0.1, 0.85, 0.95, 1.0),
        scale=(1.35, 1.35, 0.4),
        origin=(0.0, 0.0, 0.03),
        orientation="yaw",
        yaw_joint="joint4",
    ),
}

LEADER_PREFIX = "leader_"
LEADER_BASE_BODY = "leader_base"  # world-aligned frame at the leader's base; poses are relative to this
LEADER_TOOL_SITE = "leader_tool"

# Where the leader stands: the world origin, between the KUKA (y = -0.6) and
# the UR5e (y = +0.6), then the Taipan (y = +1.8) and the SCARA at the end (y = +3.0).
DEFAULT_LEADER_OFFSET = (0.0, 0.0, 0.0)

# The leader's CAD frame is turned 90 degrees about Z relative to the
# KUKA's: its pitch joints (2, 4, 6) rotate about its X axis, the KUKA's
# about Y. Mounting it at -90 degrees lines the two up, so that at matching
# joint angles (up to sign, see JOINT_SIGNS) both tools have the identical
# orientation in their own base frames -- verified to <1e-5 for each joint
# individually and at the KUKA's home pose.
LEADER_YAW_DEG = -90.0

# Leader joint angle * sign = the KUKA joint angle that bends the same way.
# Not used by the FK+IK pipeline (which works in Cartesian space and doesn't
# care); used to convert miniarm_v2.py's KUKA-convention readings.
JOINT_SIGNS = np.array([-1.0, 1.0, -1.0, -1.0, -1.0, 1.0, -1.0])

# Tool point on the leader's last link: on the joint 7 axis, at the top face
# of the Link7_Base disc (mesh spans z = 0..0.011 in that frame), i.e. the
# leader's equivalent of the KUKA flange (whose attachment_site is 45 mm
# past joint 7). Any point further out along the last link would make a
# small wrist rotation of the leader demand a large tool translation of the
# KUKA once scaled up.
LEADER_TOOL_POS = (0.0, 0.0, 0.011)


def build_model(followers=("kuka",), leader_offset=None):
    """Compile the scene with the named followers (keys of FOLLOWERS), each
    standing at its FollowerSpec.position. With leader_offset (x, y, z), the
    leader arm is added standing there too."""
    spec = mujoco.MjSpec.from_file(str(WORLD_PATH))
    for name in followers:
        _add_follower(spec, name, FOLLOWERS[name])
    if leader_offset is not None:
        _add_leader(spec, leader_offset)
    return spec.compile()


def _add_follower(spec, name, follower):
    frame = spec.worldbody.add_frame(pos=list(follower.position))
    spec.attach(mujoco.MjSpec.from_file(str(follower.path)), prefix=f"{name}_", frame=frame)
    _add_target_marker(spec, f"{name}_target", follower.color)


def _add_target_marker(spec, name, rgba):
    """Where the leader says this follower's tool should be.

    mocap: physics never moves it; code sets its pose each frame via
    data.mocap_pos / data.mocap_quat. Every geom has contype/conaffinity 0,
    so it's visual-only and passes through the robots without pushing them.
    Local +Z is the pointing direction (the arrow), matching the tool sites.
    """
    body = spec.worldbody.add_body(name=name, mocap=True, pos=[0.5, 0.0, 0.5])

    def geom(**kwargs):
        body.add_geom(contype=0, conaffinity=0, **kwargs)

    sphere, cylinder, mesh = (mujoco.mjtGeom.mjGEOM_SPHERE, mujoco.mjtGeom.mjGEOM_CYLINDER,
                              mujoco.mjtGeom.mjGEOM_MESH)
    geom(type=sphere, size=[0.012, 0, 0], rgba=[1, 1, 1, 1])
    # pointer: shaft along +Z, capped by the arrowhead
    geom(type=cylinder, fromto=[0, 0, 0, 0, 0, 0.12], size=[0.006, 0, 0], rgba=list(rgba))
    geom(type=mesh, meshname="arrowhead", pos=[0, 0, 0.12], rgba=list(rgba))
    # orientation triad: X red, Y green, Z blue (thin, semi-transparent so the
    # arrow stays the thing your eye lands on)
    for end, color in (([0.07, 0, 0], [1, 0.1, 0.1, 0.8]),
                       ([0, 0.07, 0], [0.1, 1, 0.1, 0.8]),
                       ([0, 0, 0.07], [0.2, 0.4, 1, 0.8])):
        geom(type=cylinder, fromto=[0, 0, 0] + end, size=[0.003, 0, 0], rgba=color)


def _add_leader(spec, offset):
    base = spec.worldbody.add_body(name=LEADER_BASE_BODY, pos=list(offset))
    yaw = np.zeros(4)
    mujoco.mju_axisAngle2Quat(yaw, np.array([0.0, 0.0, 1.0]), np.deg2rad(LEADER_YAW_DEG))
    mount = base.add_site(name="leader_mount", quat=yaw)

    leader = mujoco.MjSpec.from_file(str(LEADER_PATH))
    spec.attach(leader, prefix=LEADER_PREFIX, site=mount)
    spec.body(LEADER_PREFIX + "link7_base").add_site(name=LEADER_TOOL_SITE, pos=list(LEADER_TOOL_POS))

    # The leader is driven straight from joint readings, not by physics:
    # LeaderArm overwrites its joint angles every step. Gravity compensation
    # stops it sagging in between (its exported masses are tiny, so without
    # this it would swing freely).
    for body in spec.bodies:
        if body.name.startswith(LEADER_PREFIX):
            body.gravcomp = 1.0
