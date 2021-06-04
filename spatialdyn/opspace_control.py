import typing

from ctrlutils import eigen
import numpy as np
import spatialdyn


def opspace_control(
    ab: spatialdyn.ArticulatedBody,
    pos: typing.Optional[np.ndarray] = None,
    ori: typing.Optional[typing.Union[np.ndarray, eigen.Quaterniond]] = None,
    joint: typing.Optional[np.ndarray] = None,
    pos_gains: typing.Union[typing.Tuple[float, float], np.ndarray] = (80.0, 18.0),
    ori_gains: typing.Union[typing.Tuple[float, float], np.ndarray] = (80.0, 18.0),
    joint_gains: typing.Union[typing.Tuple[float, float], np.ndarray] = (4.0, 1.0),
    task_pos: typing.Optional[np.ndarray] = None,
    task_ori: typing.Optional[np.ndarray] = None,
    max_pos_err: typing.Optional[float] = 1.0,
    max_ori_err: typing.Optional[float] = 1.0,
    pos_convergence: typing.Optional[typing.Tuple[float, float]] = None,
    ori_convergence: typing.Optional[typing.Tuple[float, float]] = None,
    gravity_comp: bool = True,
) -> typing.Tuple[np.ndarray, bool]:
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
            with `kv = kp / 5`, and then adjust `kv` depending on how much the
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

        max_pos_err: Optional maximum position error in meters. The distance
            between the current and goal position will be clipped to this value
            to prevent the robot from accelerating too quickly to the goal. No
            clipping will occur if `max_pos_err <= 0`.

        max_ori_err: Optional maximum orientation error in radians. See
            `max_pos_err` for more details.

        pos_convergence: Optional convergence criteria (`x_thresh`,
            `dx_thresh`). If the distance to the goal is less than `x_thresh`
            and the end-effector's velocity is less than `dx_thresh`, then the
            position is considered to be converged, and this function will
            return True if `ori_convergence` is satisfied as well.

        ori_convergence: Optional convergence criteria (`ori_thresh`,
            `w_thresh`). See `pos_convergence` for more details.

        gravity_comp: Compensate for gravity.

    Returns:
        2-tuple (tau, converged), where `tau` is an [N] array of torques (N is
        the dof of the given articulated body), and `converged` is a boolean
        that indicates whether the position and orientation convergence criteria
        are satisfied, if given.
    """
    import ctrlutils

    # Parse function inputs.
    x_task_to_ee = _parse_opspace_control_task_pos(task_pos)
    quat_ee_to_task = _parse_opspace_control_task_ori(task_ori)
    x_des = _parse_opspace_control_pos(pos, ab, x_task_to_ee)
    quat_des = _parse_opspace_control_ori(ori, ab, quat_ee_to_task)
    q_des = _parse_opspace_control_joint(joint, ab)
    x_err_max = _parse_opspace_control_max_err(max_pos_err)
    ori_err_max = _parse_opspace_control_max_err(max_ori_err)
    kp_kv_pos = pos_gains
    kp_kv_ori = ori_gains
    kp_kv_joint = joint_gains

    # Compute Jacobian.
    J = spatialdyn.jacobian(ab, offset=x_task_to_ee)
    J_v = J[:3]
    J_w = J[3:]
    if quat_ee_to_task is not None:
        # Rotate Jacobian to task frame.
        R_ee_to_task = quat_ee_to_task.matrix()
        J_v[...] = R_ee_to_task.dot(J_v)
        J_w[...] = R_ee_to_task.dot(J_w)

    # Compute position PD control.
    x = spatialdyn.position(ab, offset=x_task_to_ee)
    dx = J_v.dot(ab.dq)
    ddx, x_err = ctrlutils.pd_control(x, x_des, dx, kp_kv_pos, x_err_max)

    # Get current orientation.
    quat = spatialdyn.orientation(ab)
    if quat_ee_to_task is not None:
        # Rotate quaternion to task frame.
        quat = quat_ee_to_task * quat
    # Choose the quaternion closer to quat_des so the end-effector doesn't
    # rotate the far way around.
    quat = ctrlutils.near_quaternion(quat, quat_des)

    # Compute orientation PD control.
    w = J_w.dot(ab.dq)
    dw, w_err = ctrlutils.pd_control(quat, quat_des, w, kp_kv_ori, ori_err_max)

    # Compute opspace torques.
    N = np.eye(ab.dof)
    if spatialdyn.opspace.is_singular(ab, J, svd_epsilon=0.01):
        # Do only position control if the arm is in a 6-dof singularity.
        tau_cmd = spatialdyn.opspace.inverse_dynamics(ab, J_v, ddx, N, svd_epsilon=0.01)
    else:
        # Otherwise do combined position and orientation control.
        ddx_dw = np.hstack((ddx, dw))
        tau_cmd = spatialdyn.opspace.inverse_dynamics(
            ab, J, ddx_dw, N, svd_epsilon=0.01
        )

    # Add joint task in nullspace.
    I = np.eye(ab.dof)
    ddq, q_err = ctrlutils.pd_control(ab.q, q_des, ab.dq, kp_kv_joint)
    tau_cmd += spatialdyn.opspace.inverse_dynamics(ab, I, ddq, N)

    # Add gravity compensation.
    if gravity_comp:
        tau_cmd += spatialdyn.gravity(ab)

    # Compute convergence.
    converged = is_converged(pos_convergence, x_err, dx) and is_converged(
        ori_convergence, w_err, w
    )

    return tau_cmd, converged


def is_converged(
    convergence: typing.Optional[typing.Tuple[float, float]],
    x_err: np.ndarray,
    dx: np.ndarray,
) -> bool:
    if convergence is None:
        return True
    x_thresh, dx_thresh = convergence
    return np.linalg.norm(x_err) < x_thresh and np.linalg.norm(dx) < dx_thresh


def _parse_opspace_control_task_pos(
    task_pos: typing.Optional[np.ndarray],
) -> np.ndarray:
    # Get task position offset.
    return task_pos if task_pos is not None else np.zeros((3,))


def _parse_opspace_control_task_ori(
    task_ori: typing.Optional[eigen.Quaterniond],
) -> np.ndarray:
    # Convert task orientation in ee to rotation matrix from ee to task frame.
    if task_ori is None:
        return None
    elif type(task_ori) is eigen.Quaterniond:
        return task_ori.inverse()
    elif task_ori.shape == (4,):
        return eigen.Quaterniond(*task_ori).inverse()
    else:
        return eigen.Quaterniond(task_ori.T)


def _parse_opspace_control_pos(
    pos: typing.Optional[np.ndarray],
    ab: spatialdyn.ArticulatedBody,
    x_task_to_ee: np.ndarray,
) -> np.ndarray:
    # Get desired position.
    return pos if pos is not None else spatialdyn.position(ab, -1, x_task_to_ee)


def _parse_opspace_control_ori(
    ori: typing.Optional[np.ndarray],
    ab: spatialdyn.ArticulatedBody,
    quat_ee_to_task: eigen.Quaterniond,
) -> eigen.Quaterniond:
    # Get desired orientation.
    if ori is None:
        quat_des = spatialdyn.orientation(ab, -1)
        if quat_ee_to_task is not None:
            quat_des = quat_ee_to_task * quat_des
        return quat_des
    elif type(ori) is eigen.Quaterniond:
        return ori
    elif ori.shape == (4,):
        return eigen.Quaterniond(*ori)
    else:
        return eigen.Quaterniond(ori)


def _parse_opspace_control_joint(
    joint: typing.Optional[np.ndarray],
    ab: spatialdyn.ArticulatedBody,
) -> np.ndarray:
    # Get desired joint configuration.
    return joint if joint is not None else ab.q


def _parse_opspace_control_max_err(max_err: typing.Optional[float]) -> float:
    # Convert error limit.
    return max_err if max_err is not None else 0.0
