"""The simulated leader arm: shows the physical leader's joint readings 1:1
and turns them into a tool pose with forward kinematics.

FK is done by MuJoCo on the leader's own model, so the pose handed to the
KUKA is exactly the pose of the leader you see on screen.
"""

import mujoco
import numpy as np

from pose_sources import UdpJsonListener
from scene import LEADER_BASE_BODY, LEADER_PREFIX, LEADER_TOOL_SITE

NUM_JOINTS = 7


class LeaderArm:
    def __init__(self, model, data):
        self.model = model
        self.data = data
        joints = [model.joint(f"{LEADER_PREFIX}joint{i + 1}").id for i in range(NUM_JOINTS)]
        self._qpos_adr = model.jnt_qposadr[joints]
        self._dof_adr = model.jnt_dofadr[joints]
        self.lower = model.jnt_range[joints, 0].copy()
        self.upper = model.jnt_range[joints, 1].copy()
        self._base = model.body(LEADER_BASE_BODY).id
        self._tool = model.site(LEADER_TOOL_SITE).id

        # FK on a scratch MjData, so it never depends on (or disturbs) the
        # live simulation's state.
        self._fk_data = mujoco.MjData(model)
        self._joints = np.zeros(NUM_JOINTS)

    @property
    def joints(self):
        return self._joints.copy()

    def set_joints(self, joints):
        """Take a new reading (7 angles, radians, in the leader model's
        convention). Out-of-range readings are clipped to the joint limits."""
        joints = np.asarray(joints, dtype=float)
        if joints.shape != (NUM_JOINTS,):
            raise ValueError(f"expected {NUM_JOINTS} joint angles, got shape {joints.shape}")
        self._joints = np.clip(joints, self.lower, self.upper)

    def apply(self):
        """Pin the simulated leader to the latest reading. Call before every
        physics step: the leader has no motors, so this is what moves it."""
        self.data.qpos[self._qpos_adr] = self._joints
        self.data.qvel[self._dof_adr] = 0.0

    def pose(self):
        """Tool pose for the latest reading, relative to the leader's base:
        (position (3,), rotation (3, 3)). Same contract as PoseSource."""
        d = self._fk_data
        d.qpos[self._qpos_adr] = self._joints
        mujoco.mj_kinematics(self.model, d)

        base_pos = d.xpos[self._base]
        base_rot = d.xmat[self._base].reshape(3, 3)
        tool_rot = d.site_xmat[self._tool].reshape(3, 3)
        return (base_rot.T @ (d.site_xpos[self._tool] - base_pos),
                base_rot.T @ tool_rot)


class JointSource:
    """Interface every source of leader joint readings implements."""

    def latest(self):
        """Most recent 7 joint angles (radians), or None if nothing yet."""
        raise NotImplementedError

    def stop(self):
        """Release any threads/sockets. Safe to call more than once."""


class DemoJoints(JointSource):
    """Every joint swaying slowly around a bent working pose, at different
    rates so the motion doesn't repeat quickly. Stands in for a physical
    leader."""

    # A bent working pose, picked by measurement at 2x scale: every IK solve
    # within 10 mm / 5 deg, and the KUKA never touching the floor. Lower
    # (e.g. the KUKA's home pose) drives the KUKA into the floor; more
    # upright misses up to 17% of solves near the KUKA's shoulder.
    CENTER = np.array([0.0, 0.6, 0.0, 1.3, 0.0, 0.6, 0.0])
    AMPLITUDE = np.array([0.6, 0.25, 0.3, 0.3, 0.4, 0.4, 0.6])
    RATE = np.array([0.31, 0.43, 0.37, 0.53, 0.61, 0.47, 0.71])  # rad/s

    def __init__(self, clock=None):
        import time
        self._clock = clock or time.time
        self._start = self._clock()

    def latest(self):
        t = self._clock() - self._start
        return self.CENTER + self.AMPLITUDE * np.sin(self.RATE * t)


class UdpJointSource(UdpJsonListener, JointSource):
    """Joint readings sent as JSON over UDP, one per datagram:

        {"joints": [q1, q2, q3, q4, q5, q6, q7]}

    in radians, in the leader model's convention: all zeros = arm straight
    up, and positive = the direction the model turns that joint.
    """

    @staticmethod
    def parse(message):
        import json
        joints = np.asarray(json.loads(message)["joints"], dtype=float)
        if joints.shape != (NUM_JOINTS,):
            raise ValueError(f"expected {NUM_JOINTS} joint angles, got shape {joints.shape}")
        return joints
