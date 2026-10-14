"""Damped least-squares inverse kinematics for the KUKA in the MuJoCo scene.

MuJoCo has no built-in IK, so this is ours: iterate "look at the gap
between the tool and the target, ask the Jacobian which joint nudge
shrinks it, take that step, repeat".

Solving happens on a scratch MjData so the live simulation isn't
disturbed -- the caller gets joint angles back and decides what to do
with them (here: hand them to the position servos as targets).
"""

import mujoco
import numpy as np


class DifferentialIK:
    def __init__(self, model, site_name, damping=1e-2, max_iters=20,
                 tol=1e-4, max_joint_step=0.1):
        self.model = model
        self.scratch = mujoco.MjData(model)
        self.site_id = model.site(site_name).id

        # Jacobian buffers: 3 rows (x/y/z) x nv columns (one per joint).
        # jacp is for position, jacr for rotation (unused until pass 2).
        self.jacp = np.zeros((3, model.nv))
        self.jacr = np.zeros((3, model.nv))

        self.damping = damping
        self.max_iters = max_iters
        self.tol = tol
        self.max_joint_step = max_joint_step

        # Joint limits, so a solution can't ask for a pose the arm can't hold.
        self.lower = model.jnt_range[:, 0].copy()
        self.upper = model.jnt_range[:, 1].copy()

    def _dls_step(self, J, error):
        """dq = J^T (J J^T + lambda^2 I)^-1 error"""
        lam2 = self.damping ** 2
        return J.T @ np.linalg.solve(J @ J.T + lam2 * np.eye(J.shape[0]), error)

    def _step_respecting_limits(self, q, J, error):
        """A step that doesn't rely on joints that are pinned at a limit.

        Simply clipping the result afterwards makes things worse: the
        Jacobian assumed the pinned joint would contribute, so the others
        move to compensate for help that never comes, and the error can
        grow. Instead, drop those joints from the solve and re-solve with
        the ones that can still move.
        """
        free = np.ones(self.model.nv, dtype=bool)
        dq = np.zeros(self.model.nv)
        for _ in range(self.model.nv):
            J_free = J.copy()
            J_free[:, ~free] = 0.0
            dq = self._dls_step(J_free, error)
            dq[~free] = 0.0

            pinned = (((q >= self.upper - 1e-9) & (dq > 0)) |
                      ((q <= self.lower + 1e-9) & (dq < 0)))
            if not pinned.any():
                break
            free &= ~pinned
            if not free.any():
                return np.zeros(self.model.nv)
        return dq

    def solve_position(self, qpos_seed, target_pos):
        """Joint angles that put the tool at target_pos, starting the search
        from qpos_seed. Returns (qpos, final_position_error_in_metres)."""
        d = self.scratch
        d.qpos[:] = qpos_seed

        for _ in range(self.max_iters):
            # Update where everything is, then ask for the Jacobian there.
            mujoco.mj_kinematics(self.model, d)
            mujoco.mj_comPos(self.model, d)

            error = target_pos - d.site_xpos[self.site_id]
            if np.linalg.norm(error) < self.tol:
                break

            mujoco.mj_jacSite(self.model, d, self.jacp, self.jacr, self.site_id)
            dq = self._step_respecting_limits(d.qpos, self.jacp, error)

            # Cap the step so a big jump in the target can't fling the arm.
            biggest = np.abs(dq).max()
            if biggest > self.max_joint_step:
                dq *= self.max_joint_step / biggest

            d.qpos[:] = np.clip(d.qpos + dq, self.lower, self.upper)

        mujoco.mj_kinematics(self.model, d)
        final_error = np.linalg.norm(target_pos - d.site_xpos[self.site_id])
        return d.qpos.copy(), final_error

    def _orientation_error(self, current_mat, target_quat):
        """Rotation vector (axis * angle, in world frame) taking the tool's
        current orientation to the target one."""
        current_quat = np.zeros(4)
        mujoco.mju_mat2Quat(current_quat, current_mat)
        mujoco.mju_negQuat(current_quat, current_quat)      # inverse (unit quat)
        delta = np.zeros(4)
        mujoco.mju_mulQuat(delta, target_quat, current_quat)
        error = np.zeros(3)
        mujoco.mju_quat2Vel(error, delta, 1.0)
        return error

    def solve_pose(self, qpos_seed, target_pos, target_quat, rot_weight=1.0):
        """Joint angles putting the tool at target_pos AND target_quat.

        Same idea as solve_position, but the error is 6 numbers (3 position,
        3 rotation) and the Jacobian stacks jacp on top of jacr.
        rot_weight trades the two off: metres and radians aren't the same
        units, so this says how much a radian of twist matters relative to
        a metre of offset.

        Returns (qpos, position_error_m, orientation_error_rad).
        """
        d = self.scratch
        d.qpos[:] = qpos_seed

        for _ in range(self.max_iters):
            mujoco.mj_kinematics(self.model, d)
            mujoco.mj_comPos(self.model, d)

            pos_error = target_pos - d.site_xpos[self.site_id]
            rot_error = self._orientation_error(d.site_xmat[self.site_id], target_quat)
            if np.linalg.norm(pos_error) < self.tol and np.linalg.norm(rot_error) < self.tol:
                break

            mujoco.mj_jacSite(self.model, d, self.jacp, self.jacr, self.site_id)
            J = np.vstack([self.jacp, rot_weight * self.jacr])
            error = np.concatenate([pos_error, rot_weight * rot_error])
            dq = self._step_respecting_limits(d.qpos, J, error)

            biggest = np.abs(dq).max()
            if biggest > self.max_joint_step:
                dq *= self.max_joint_step / biggest

            d.qpos[:] = np.clip(d.qpos + dq, self.lower, self.upper)

        mujoco.mj_kinematics(self.model, d)
        pos_error = np.linalg.norm(target_pos - d.site_xpos[self.site_id])
        rot_error = np.linalg.norm(
            self._orientation_error(d.site_xmat[self.site_id], target_quat))
        return d.qpos.copy(), pos_error, rot_error
