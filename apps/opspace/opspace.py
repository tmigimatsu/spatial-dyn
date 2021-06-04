#!/usr/bin/env python

import signal

import ctrlutils
import numpy as np
import spatialdyn as dyn

g_runloop = True

# Signal handler
def signal_handler(sig, frame):
    global g_runloop
    g_runloop = False


PATH_URDF = "../../resources/franka_panda/franka_panda.urdf"

CONTROL_FREQ = 1000

# Default home configuration of the robot.
Q_HOME = np.array(
    [0.0, -np.pi / 6.0, 0.0, -5.0 / 6.0 * np.pi, 0.0, 2.0 / 3.0 * np.pi, 0.0]
)

# Position offset of the end-effector tip in the frame of the last link.
EE_OFFSET = np.array([0.0, 0.0, 0.107])

POS_THRESHOLD = (0.01, 0.01)
ORI_THRESHOLD = (0.01, 0.01)


def main():
    # Create signal handler
    global g_runloop
    signal.signal(signal.SIGINT, signal_handler)

    # Initialize the robot.
    ab = dyn.ArticulatedBody(dyn.urdf.load_model(PATH_URDF))
    ab.q = Q_HOME

    # Construct the goal position.
    x_des = dyn.position(ab, -1, EE_OFFSET) + np.array([0., 0.2, 0.])
    quat_des = dyn.orientation(ab)
    q_des = Q_HOME

    # Run control loop.
    timer = ctrlutils.Timer(CONTROL_FREQ)
    while g_runloop:
        # Wait for next control cycle.
        timer.sleep()

        # Compute control torques.
        tau_cmd, converged = dyn.opspace_control(
            ab,
            pos=x_des,
            ori=quat_des,
            joint=q_des,
            task_pos=EE_OFFSET,
            pos_threshold=POS_THRESHOLD,
            ori_threshold=ORI_THRESHOLD,
        )
        if converged:
            break

        # Apply command torques to update ab.q, ab.dq.
        dyn.integrate(ab, tau_cmd, timer.dt)

    # Print simulation stats
    print("Simulated {}s in {}s.".format(timer.time_sim(), timer.time_elapsed()))


if __name__ == "__main__":
    main()
