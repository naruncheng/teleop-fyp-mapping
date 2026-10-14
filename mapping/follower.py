"""A simulated follower robot that tracks an end-effector pose.

Hand it a pose from any leader -- (position, rotation matrix) in the
leader's own base frame -- and it maps that into its own workspace (offset +
per-axis scale, plus an orientation projection for robots that can't tilt
their tool), solves IK for the joint angles that reach it, and drives its
position servos there.

The same class drives every robot in scene.FOLLOWERS; several can share one
scene and follow the same leader at once. Knows nothing about where the pose
came from; see pose_sources.py and leader.py for that.
"""

import mujoco
import numpy as np

from ik import DifferentialIK
from scene import FOLLOWERS

ORIENTATIONS = ("full", "yaw")

# The only tool orientations a SCARA can reach: pointing straight down,
# turned some angle psi about vertical. At psi = 0 the tool's x axis lies
# along world x and its z axis (out of the flange) points down.
TOOL_DOWN = np.diag([1.0, -1.0, -1.0])

# Below this, the leader's tool is pointing nearly straight up, where every
# rotation about vertical is (almost) equally close: the yaw is meaningless,
# so the last good one is kept rather than letting it spin.
_YAW_DEGENERATE = 0.1


def _wrap(angle):
    """Angle wrapped into [-pi, pi)."""
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def mat_to_quat(rotation):
    """3x3 rotation matrix -> MuJoCo quaternion [w, x, y, z].

    MuJoCo quaternions are scalar FIRST. (SciPy and PyBullet use
    [x, y, z, w]; mixing the two up gives wrong rotations.)
    """
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, np.asarray(rotation, dtype=float).flatten())
    return quat


def project_to_yaw(rotation, fallback_yaw=0.0):
    """The tool-down orientation closest to `rotation`.

    Returns (Rz(psi) @ TOOL_DOWN, psi), with psi chosen to minimise the
    Frobenius distance to `rotation`: maximising trace(Rz(psi)^T M) with
    M = rotation @ TOOL_DOWN^T gives psi = atan2(M10 - M01, M00 + M11).
    In other words: keep the leader's twist about vertical, drop its tilt.
    """
    M = np.asarray(rotation, dtype=float) @ TOOL_DOWN.T
    s, c = M[1, 0] - M[0, 1], M[0, 0] + M[1, 1]
    psi = fallback_yaw if np.hypot(s, c) < _YAW_DEGENERATE else float(np.arctan2(s, c))
    rz = np.array([[np.cos(psi), -np.sin(psi), 0.0],
                   [np.sin(psi), np.cos(psi), 0.0],
                   [0.0, 0.0, 1.0]])
    return rz @ TOOL_DOWN, psi


class Follower:
    def __init__(self, model, data, name, scale=None, origin=None, orientation=None,
                 rot_weight=1.0, position_only=False):
        """
        model, data: the compiled scene (scene.build_model) containing this
            follower, and its live state. Followers in the same scene share
            them; each only ever writes its own joints and actuators.
        name: a key of scene.FOLLOWERS, e.g. "kuka" or "scara".
        scale: multiplier on leader position -- one number, or per axis
            (x, y, z). Defaults to the FollowerSpec's.
        origin: where the leader's base maps to, relative to this robot's
            base (metres). Defaults to the FollowerSpec's.
        orientation: "full" to match the leader's tool orientation, "yaw" to
            point the tool down and match only rotation about vertical.
            Defaults to the FollowerSpec's.
        rot_weight: how much a radian of orientation error matters vs a
            metre of position error, where both can't be met.
        position_only: ignore orientation and match position only.
        """
        spec = FOLLOWERS[name]
        self.name = name
        self.model = model
        self.data = data
        self.scale = np.broadcast_to(
            np.asarray(spec.scale if scale is None else scale, dtype=float), (3,)).copy()
        self.origin = np.asarray(spec.origin if origin is None else origin, dtype=float)
        self.orientation = spec.orientation if orientation is None else orientation
        if self.orientation not in ORIENTATIONS:
            raise ValueError(f"orientation must be one of {ORIENTATIONS}, got {self.orientation!r}")
        self.rot_weight = rot_weight
        self.position_only = position_only

        prefix = f"{name}_"
        # IK is solved over the whole scene, but the tool's Jacobian is zero
        # for every joint not on this robot, so solves never move them.
        self.solver = DifferentialIK(model, prefix + "attachment_site")
        self.site_id = self.solver.site_id

        # This robot's actuators, each a position servo on one joint. Where
        # that joint's angle lives in qpos (so an IK solution can be handed
        # over as servo targets), and where its force lives (for the
        # gravity term in track()).
        self._acts = np.array([i for i in range(model.nu)
                               if model.actuator(i).name.startswith(prefix)])
        joints = model.actuator_trnid[self._acts, 0]
        self._qpos_adr = model.jnt_qposadr[joints]
        self._dof_adr = model.jnt_dofadr[joints]
        self._servo_kp = model.actuator_gainprm[self._acts, 0].copy()

        # Put the robot in its "home" pose. Its keyframe covers the whole
        # scene, so only this robot's joints and servo targets are copied.
        key = model.key(prefix + "home").id
        data.qpos[self._qpos_adr] = model.key_qpos[key, self._qpos_adr]
        data.ctrl[self._acts] = model.key_ctrl[key, self._acts]
        mujoco.mj_forward(model, data)

        self.base_pos = data.xpos[model.body(prefix + "base").id].copy()

        if self.orientation == "yaw":
            if spec.yaw_joint is None:
                raise ValueError(f"{name}: orientation 'yaw' needs FollowerSpec.yaw_joint")
            yaw_joint = model.joint(prefix + spec.yaw_joint).id
            self._yaw_adr = model.jnt_qposadr[yaw_joint]
            self._yaw_range = model.jnt_range[yaw_joint].copy()
            # FK on a scratch MjData, to read the tool's yaw for an IK solution.
            self._fk = mujoco.MjData(model)
            # Which way the tool turns when the yaw joint turns (+1 or -1),
            # measured rather than assumed from the model's axis conventions.
            q = data.qpos.copy()
            yaw0 = self._yaw_at(q)
            q[self._yaw_adr] += 0.1
            self._yaw_sign = np.sign(_wrap(self._yaw_at(q) - yaw0))

        # A body's index among mocap bodies (not its body id) is what indexes
        # data.mocap_pos / data.mocap_quat.
        self._target = model.body_mocapid[model.body(prefix + "target").id]

        # Start the target on the tool, so nothing moves until a pose arrives.
        pos, rot = self.tool_pose()
        if self.orientation == "yaw":
            rot, self._yaw = project_to_yaw(rot)
        else:
            self._yaw = 0.0
        data.mocap_pos[self._target] = pos
        data.mocap_quat[self._target] = mat_to_quat(rot)

    def tool_pose(self):
        """Where the tool is now, in the world: (position, rotation)."""
        return (self.data.site_xpos[self.site_id].copy(),
                self.data.site_xmat[self.site_id].reshape(3, 3).copy())

    def leader_to_world(self, pose):
        """Leader pose (position, rotation in its base frame) -> world
        (position, quaternion) for this robot's tool.

        Position is offset and scaled per axis. Orientation is copied 1:1
        ("full") or reduced to its twist about vertical ("yaw")."""
        position, rotation = pose
        world_pos = (self.base_pos + self.origin
                     + self.scale * np.asarray(position, dtype=float))
        if self.orientation == "yaw":
            rotation, self._yaw = project_to_yaw(rotation, self._yaw)
        return world_pos, mat_to_quat(rotation)

    def world_to_leader(self, position, rotation):
        """Inverse of leader_to_world's position mapping: the leader pose
        that would put this robot's tool at this world pose."""
        return ((np.asarray(position) - self.base_pos - self.origin) / self.scale,
                np.asarray(rotation))

    def set_target(self, pose):
        """Point the robot at a new leader pose. Takes effect on the next track()."""
        pos, quat = self.leader_to_world(pose)
        self.data.mocap_pos[self._target] = pos
        self.data.mocap_quat[self._target] = quat

    def track(self):
        """Solve IK for the current target and hand the result to the servos.

        Returns (position_error_m, orientation_error_rad), where the latter
        is None in position-only mode. These are how well IK solved, not
        where the arm is right now -- see tool_gap() for that.
        """
        target_pos = self.data.mocap_pos[self._target]

        # Seed from where the arm actually is, so each solve is a small
        # correction rather than a fresh search.
        if self.position_only:
            q, pos_err = self.solver.solve_position(self.data.qpos, target_pos)
            rot_err = None
        elif self.orientation == "yaw":
            # Position by IK, then yaw by setting the yaw joint directly.
            # Leaving yaw to IK let the yaw joint ratchet into its limit,
            # after which IK traded hundreds of mm of position for the yaw
            # it could no longer reach. The yaw joint doesn't move the tool
            # point, so the position solve leaves it alone.
            q, pos_err = self.solver.solve_position(self.data.qpos, target_pos)
            q, rot_err = self._apply_yaw(q)
        else:
            q, pos_err, rot_err = self.solver.solve_pose(
                self.data.qpos, target_pos, self.data.mocap_quat[self._target],
                rot_weight=self.rot_weight)

        # These are position servos: force = kp * (target - actual), so
        # holding the arm's weight requires sitting permanently off-target
        # (measured as a 14mm droop on the KUKA in the original project).
        # Adding the force gravity needs (qfrc_bias) up front means the servo
        # no longer has to drift to generate it.
        self.data.ctrl[self._acts] = (q[self._qpos_adr]
                                      + self.data.qfrc_bias[self._dof_adr] / self._servo_kp)
        return pos_err, rot_err

    def _yaw_at(self, q):
        """Tool yaw (rotation about vertical, radians) at joint angles q."""
        self._fk.qpos[:] = q
        mujoco.mj_kinematics(self.model, self._fk)
        return project_to_yaw(self._fk.site_xmat[self.site_id].reshape(3, 3))[1]

    def _apply_yaw(self, q):
        """Set the yaw joint in q so the tool's yaw matches the target's.

        Any turn by a multiple of 360 degrees gives the same yaw, so of the
        candidates within the joint's limits, take the one nearest its
        current angle: the joint never winds up, and never takes the long
        way round. Returns (q, remaining_yaw_error_rad)."""
        error = _wrap(self._yaw - self._yaw_at(q))
        current = q[self._yaw_adr]
        lower, upper = self._yaw_range
        candidates = current + self._yaw_sign * error + 2.0 * np.pi * np.arange(-2, 3)
        legal = candidates[(candidates >= lower) & (candidates <= upper)]
        if legal.size:
            q[self._yaw_adr] = legal[np.argmin(np.abs(legal - current))]
        else:  # joint range under 360 degrees: get as close as the limits allow
            q[self._yaw_adr] = np.clip(candidates[2], lower, upper)
        return q, abs(_wrap(self._yaw - self._yaw_at(q)))

    def tool_gap(self):
        """Distance (m) from the tool to the target right now. Lags the IK
        error while moving, since the servos need time to catch up."""
        return float(np.linalg.norm(
            self.data.site_xpos[self.site_id] - self.data.mocap_pos[self._target]))
