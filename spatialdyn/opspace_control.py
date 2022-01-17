from typing import Optional, Tuple, Union

import ctrlutils  # type: ignore
from ctrlutils import eigen
import numpy as np
import spatialdyn as dyn


def opspace_control(
    ab: dyn.ArticulatedBody,
    pos: Optional[np.ndarray] = None,
    ori: Optional[Union[np.ndarray, eigen.Quaterniond]] = None,
    joint: Optional[np.ndarray] = None,
    pos_gains: Union[Tuple[float, float], np.ndarray] = (80.0, 18.0),
    ori_gains: Union[Tuple[float, float], np.ndarray] = (80.0, 18.0),
    joint_gains: Union[Tuple[float, float], np.ndarray] = (4.0, 1.0),
    task_pos: Optional[np.ndarray] = None,
    task_ori: Optional[np.ndarray] = None,
    max_pos_acceleration: Optional[float] = None,
    max_ori_acceleration: Optional[float] = None,
    pos_threshold: Optional[Tuple[float, float]] = None,
    ori_threshold: Optional[Union[Tuple[float, float],
                                  eigen.Quaterniond]] = None,
    gravity_comp: bool = True,
    integration_step: Optional[float] = None,
) -> Tuple[np.ndarray, bool]:
    """
    Computes command torques for controlling the robot to a given pose using
    Operational Space.

    If the robot doesn't have enough dof to achieve both the position and
    orientation (e.g. due to singularities), then it will track only the
    position. There are no safeties to prevent the robot from becoming unstable
    if the position is outside the robot's workspace. This should be handled by
    the user (e.g. by limiting the goal position to a workspace radius around
    robot's base).

    Args:
        ab: Articulated body with `ab.q` and `ab.dq` set to the current state.

        pos: Desired xyz position [3] in meters. If None, `pos` will be set to
            hold the end-effector's current position.

        ori: Desired orientation as an xyzw quaternion vector [4] or a [3 x 3]
            rotation matrix. If None, `ori` will be set to hold the end-effector's
            current orientation.

        joint: Desired joint configuration to be controlled in the nullspace of
            the pose task. Setting this prevents a robot arm with >6 dof from
            sagging due to gravity. A good configuration would be the default
            home configuration of the arm to minimize the chance of reaching
            a singularity. If None, `joint` will be set to hold the robot's
            current configuration.

        pos_gains: (`kp`, `kv`) gains for the position task as a [2] vector, or
            a [3 x 2] matrix for independent gains for each of the xyz axes of
            the task frame. Increasing kp will increase the force pulling the
            end-effector to the goal position, and increasing kv will increase
            velocity damping to prevent the end-effector from moving too quickly
            or oscillating. `kv = 2 * sqrt(kp)` results in a critically damped
            system assuming no friction. A good rule of thumb is to start off
            with `kv = kp / 4`, and then adjust `kv` depending on how much the
            end-effector oscillates or how slow it is to converge.

        ori_gains: (`kp`, `kv`) gains for the orientation task as a [2] vector,
            or a [3 x 2] matrix for independent gains for each of the xyz axes
            of the task frame. See `pos_gains` for more details.

        joint_gains: (`kp`, `kv`) gains for the orientation task as a [2]
            vector, or a [N x 2] matrix for independent gains for each of the N
            joints. See `pos_gains` for more details.

        task_pos: Optional xyz position offset of the task frame in the
            end-effector frame [3].

        task_ori: Optional orientation of the task frame in the end-effector
            frame as an xyzw quaternion [4] or a [3 x 3] matrix.

        max_pos_acceleration: Optional maximum acceleration due to the position
            error. The proportional error term in the PD control law
            `ddx = -kp * (x - x_des)` will be clipped to this value to prevent
            the robot from accelerating too quickly to the goal. No clipping
            will occur if `max_pos_acceleration <= 0`.

        max_ori_acceleration: Optional maximum acceleration due to the
            orientation error. See `max_pos_acceleration` for more details.

        pos_threshold: Optional convergence criteria (`x_thresh`,
            `dx_thresh`). If the distance to the goal is less than `x_thresh`
            and the end-effector's velocity is less than `dx_thresh`, then the
            position is considered to be converged, and this function will
            return True if `ori_threshold` is satisfied as well.

        ori_threshold: Optional convergence criteria (`ori_thresh`,
            `w_thresh`). See `pos_threshold` for more details.

        gravity_comp: Compensate for gravity.

        integration_step: Optional integration time step. If set to a positive
            number, this function will update `ab` with the expected position
            and velocity of the robot after applying the returned command
            torques `tau` for the given timsetep. This is helpful if the robot
            only supports position/velocity control, not torque control.

    Returns:
        2-tuple (`tau`, `converged`), where `tau` is an [N] array of torques (N
        is the dof of the given articulated body), and `converged` is a boolean
        that indicates whether the position and orientation convergence criteria
        are satisfied, if given.
    """
    # Parse function inputs.
    x_task_to_ee = _parse_opspace_control_task_pos(task_pos)
    quat_ee_to_task = _parse_opspace_control_task_ori(task_ori)
    x_des = _parse_opspace_control_pos(pos, ab, x_task_to_ee)
    quat_des = _parse_opspace_control_ori(ori, ab, quat_ee_to_task)
    q_des = _parse_opspace_control_joint(joint, ab)
    ddx_max = _parse_opspace_control_max_acc(max_pos_acceleration)
    dw_max = _parse_opspace_control_max_acc(max_ori_acceleration)
    if not isinstance(pos_gains, np.ndarray) or pos_gains.size == 2:
        kp_kv_pos = np.empty((3, 2))
        kp_kv_pos[:, 0] = pos_gains[0]
        kp_kv_pos[:, 1] = pos_gains[1]
    else:
        kp_kv_pos = pos_gains

    kp_kv_ori = ori_gains
    kp_kv_joint = joint_gains
    dt = integration_step if integration_step is not None else 0.

    return dyn.compute_opspace_control(ab, x_des, quat_des, q_des, kp_kv_pos, kp_kv_ori,
                                kp_kv_joint, x_task_to_ee, quat_ee_to_task, ddx_max, dw_max,
                                pos_threshold[0], pos_threshold[1], ori_threshold[0],
                                ori_threshold[1], gravity_comp, dt)

    # Compute Jacobian.
    J = dyn.jacobian(ab, offset=x_task_to_ee)
    J_v = J[:3]
    J_w = J[3:]
    if quat_ee_to_task is not None:
        # Rotate Jacobian to task frame.
        R_ee_to_task = quat_ee_to_task.matrix()
        J_v[...] = R_ee_to_task.dot(J_v)
        J_w[...] = R_ee_to_task.dot(J_w)

    # Compute position PD control.
    x = dyn.position(ab, offset=x_task_to_ee)
    dx = J_v.dot(ab.dq)
    ddx, x_err = ctrlutils.pd_control(x, x_des, dx, kp_kv_pos, ddx_max)

    # Get current orientation.
    quat = dyn.orientation(ab)
    if quat_ee_to_task is not None:
        # Rotate quaternion to task frame.
        quat = quat_ee_to_task * quat
    # Choose the quaternion closer to quat_des so the end-effector doesn't
    # rotate the far way around.
    quat = ctrlutils.near_quaternion(quat, quat_des)

    # Compute orientation PD control.
    w = J_w.dot(ab.dq)
    dw, w_err = ctrlutils.pd_control(quat, quat_des, w, kp_kv_ori, dw_max)

    # Compute opspace torques.
    N = np.eye(ab.dof)
    if dyn.opspace.is_singular(ab, J, svd_epsilon=0.01):
        # Do only position control if the arm is in a 6-dof singularity.
        tau_cmd = dyn.opspace.inverse_dynamics(
            ab, J_v, ddx, N, svd_epsilon=0.01)
    else:
        # Otherwise do combined position and orientation control.
        ddx_dw = np.concatenate((ddx, dw), axis=0)
        tau_cmd = dyn.opspace.inverse_dynamics(
            ab, J, ddx_dw, N, svd_epsilon=0.01
        )

    # Add joint task in nullspace.
    I = np.eye(ab.dof)
    ddq, q_err = ctrlutils.pd_control(ab.q, q_des, ab.dq, kp_kv_joint)
    tau_cmd += dyn.opspace.inverse_dynamics(ab, I, ddq, N)

    # Add gravity compensation.
    if gravity_comp:
        tau_cmd += dyn.gravity(ab)

    # Compute convergence.
    converged = is_converged(pos_threshold, x_err, dx) and is_converged(
        ori_threshold, w_err, w
    )

    # Apply command torques to update ab.q, ab.dq.
    if integration_step is not None:
        dyn.integrate(ab, tau_cmd, integration_step)

    return tau_cmd, converged


def is_converged(
    threshold: Optional[Tuple[float, float]],
    x_err: np.ndarray,
    dx: np.ndarray,
) -> bool:
    def less_than_norm(x: np.ndarray, norm: float) -> bool:
        return x.dot(x) < norm * norm
    if threshold is None:
        return True
    x_thresh, dx_thresh = threshold
    return less_than_norm(x_err, x_thresh) and less_than_norm(dx, dx_thresh)
    # return np.linalg.norm(x_err) < x_thresh and np.linalg.norm(dx) < dx_thresh


def _parse_opspace_control_task_pos(
    task_pos: Optional[np.ndarray],
) -> np.ndarray:
    # Get task position offset.
    return task_pos if task_pos is not None else np.zeros((3,))


def _parse_opspace_control_task_ori(
    task_ori: Optional[Union[np.ndarray, eigen.Quaterniond]],
) -> eigen.Quaterniond:
    # Convert task orientation in ee to rotation matrix from ee to task frame.
    if task_ori is None:
        return None
    elif isinstance(task_ori, eigen.Quaterniond):
        return task_ori.inverse()
    elif task_ori.size == 4:
        return eigen.Quaterniond(task_ori[3], task_ori[0], task_ori[1],
                                 task_ori[2]).inverse()
    else:
        return eigen.Quaterniond(task_ori.T)


def _parse_opspace_control_pos(
    pos: Optional[np.ndarray],
    ab: dyn.ArticulatedBody,
    x_task_to_ee: np.ndarray,
) -> np.ndarray:
    # Get desired position.
    return pos if pos is not None else dyn.position(ab, -1, x_task_to_ee)


def _parse_opspace_control_ori(
    ori: Optional[Union[np.ndarray, eigen.Quaterniond]],
    ab: dyn.ArticulatedBody,
    quat_ee_to_task: eigen.Quaterniond,
) -> eigen.Quaterniond:
    # Get desired orientation.
    if isinstance(ori, eigen.Quaterniond):
        return ori
    elif ori is None:
        quat_des = dyn.orientation(ab, -1)
        if quat_ee_to_task is not None:
            quat_des = quat_ee_to_task * quat_des
        return quat_des
    elif ori.size == 4:
        return eigen.Quaterniond(ori[3], ori[0], ori[1], ori[2])
    else:
        return eigen.Quaterniond(ori)


def _parse_opspace_control_joint(
    joint: Optional[np.ndarray],
    ab: dyn.ArticulatedBody,
) -> np.ndarray:
    # Get desired joint configuration.
    return joint if joint is not None else ab.q


def _parse_opspace_control_max_acc(max_acc: Optional[float]) -> float:
    # Convert error limit.
    return max_acc if max_acc is not None else 0.0
