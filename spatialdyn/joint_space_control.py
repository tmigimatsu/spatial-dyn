from typing import Optional, Tuple, Union

import ctrlutils  # type: ignore
import numpy as np
import spatialdyn as dyn


def joint_space_control(
    ab: dyn.ArticulatedBody,
    joint: Optional[np.ndarray] = None,
    joint_gains: Union[Tuple[float, float], np.ndarray] = (25.0, 10.0),
    max_joint_acceleration: Optional[float] = None,
    joint_threshold: Optional[Tuple[float, float]] = None,
    gravity_comp: bool = True,
    integration_step: Optional[float] = None,
) -> Tuple[np.ndarray, bool]:
    """
    Computes command torques for controlling the robot to a given pose using
    joint space control.

    Args:
        ab: Articulated body with `ab.q` and `ab.dq` set to the current state.

        joint: Desired joint configuration. If None, `joint` will be set to hold
            the robot's current configuration.

        joint_gains: (`kp`, `kv`) gains for the orientation task as a [2]
            vector, or a [N x 2] matrix for independent gains for each of the N
            joints. Increasing kp will increase the force pulling the
            end-effector to the goal position, and increasing kv will increase
            velocity damping to prevent the end-effector from moving too quickly
            or oscillating. `kv = 2 * sqrt(kp)` results in a critically damped
            system assuming no friction. A good rule of thumb is to start off
            with `kv = kp / 4`, and then adjust `kv` depending on how much the
            end-effector oscillates or how slow it is to converge.

        max_joint_acceleration: Optional maximum acceleration due to the
            position error. The proportional error term in the PD control law
            `ddq = -kp * (q - q_des)` will be clipped to this value to prevent
            the robot from accelerating too quickly to the goal. No clipping
            will occur if `max_joint_acceleration <= 0`.

        joint_threshold: Optional convergence criteria (`q_thresh`,
            `dq_thresh`). If the distance to the goal is less than `q_thresh`
            and the end-effector's velocity is less than `dq_thresh`, then the
            position is considered to be converged, and this function will
            return True.

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
    q_des = _parse_joint_space_control_joint(joint, ab)
    ddq_max = _parse_joint_space_control_max_acc(max_joint_acceleration)
    kp_kv_joint = joint_gains

    # Compute PD control.
    ddq, q_err = ctrlutils.pd_control(ab.q, q_des, ab.dq, kp_kv_joint, ddq_max)
    tau_cmd = dyn.inverse_dynamics(
        ab, ddq, centrifugal_coriolis=False, gravity=gravity_comp
    )

    # Compute convergence.
    converged = is_converged(joint_threshold, q_err, ab.dq)

    # Apply command torques to update ab.q, ab.dq.
    if integration_step is not None:
        dyn.integrate(ab, tau_cmd, integration_step)

    return tau_cmd, converged


def is_converged(
    convergence: Optional[Tuple[float, float]],
    x_err: np.ndarray,
    dx: np.ndarray,
) -> bool:
    if convergence is None:
        return True
    x_thresh, dx_thresh = convergence
    return np.linalg.norm(x_err) < x_thresh and np.linalg.norm(dx) < dx_thresh


def _parse_joint_space_control_joint(
    joint: Optional[np.ndarray],
    ab: dyn.ArticulatedBody,
) -> np.ndarray:
    # Get desired joint configuration.
    return joint if joint is not None else ab.q


def _parse_joint_space_control_max_acc(max_acc: Optional[float]) -> float:
    # Convert error limit.
    return max_acc if max_acc is not None else 0.0
