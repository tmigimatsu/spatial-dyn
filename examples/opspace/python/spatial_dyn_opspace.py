#!/usr/bin/env python

"""
spatial_dyn_opspace.py

Copyright 2019. All Rights Reserved.

Created: January 31, 2019
Authors: Toki Migimatsu
"""

import json
import os
import signal

import redis
import numpy as np

import ctrlutils
from ctrlutils.numpy import *
import spatialdyn
from spatialdyn import eigen

g_runloop = True

# Signal handler
def signal_handler(sig, frame):
    global g_runloop
    g_runloop = False

# Urdf
PATH_URDF = "../../../resources/kuka_iiwa/kuka_iiwa.urdf"

# Redis keys
KEY_PREFIX         = "spatial_dyn::"
KEY_MODELS_PREFIX  = KEY_PREFIX + "model::"
KEY_OBJECTS_PREFIX = KEY_PREFIX + "object::"

# GET keys
KEY_SENSOR_Q  = KEY_PREFIX + "sensor::q"
KEY_SENSOR_DQ = KEY_PREFIX + "sensor::dq"

# SET keys
KEY_CONTROL_TAU = KEY_PREFIX + "control::tau"
KEY_CONTROL_POS = KEY_PREFIX + "control::pos"
KEY_TRAJ_POS    = KEY_PREFIX + "trajectory::pos"
KEY_TRAJ_ORI    = KEY_PREFIX + "trajectory::ori"
KEY_MODEL       = KEY_PREFIX + "model"

# Webapp keys
NAME_APP            = "simulator"
KEY_WEB_RESOURCES   = "webapp::resources"
KEY_WEB_ARGS        = "webapp::" + NAME_APP + "::args"
KEY_WEB_INTERACTION = "webapp::" + NAME_APP + "::interaction"

# Controller gains
KEY_KP_KV_POS   = KEY_PREFIX + "control::kp_kv_pos"
KEY_KP_KV_ORI   = KEY_PREFIX + "control::kp_kv_ori"
KEY_KP_KV_JOINT = KEY_PREFIX + "control::kp_kv_joint"

# Controller parameters
EE_OFFSET       = np.array([0., 0., 0.03])
Q_HOME          = (np.pi / 180.) * np.array([90., -30., 0., 60., 0., -90., 0.])
KP_KV_POS       = np.array([40., 10.])
KP_KV_ORI       = np.array([40., 10.])
KP_KV_JOINT     = np.array([1., 2.])
TIMER_FREQ      = 1000.  # Max ~1400
GAIN_KEY_PRESS  = 0.1 / TIMER_FREQ
GAIN_CLICK_DRAG = 100.

def pd_control(x, x_des, dx, kp_kv):
    if type(x) is eigen.Quaterniond:
        return -kp_kv[0] * spatialdyn.opspace.orientation_error(x, x_des) - kp_kv[1] * dx
    return -kp_kv[0] * (x - x_des) - kp_kv[1] * dx

def main():

    # Create timer and Redis client
    redis_client = redis.Redis("127.0.0.1")
    redis_pipe   = redis_client.pipeline(transaction=False)
    timer = ctrlutils.Timer(TIMER_FREQ)

    # Load robot
    ab = spatialdyn.ArticulatedBody(spatialdyn.urdf.load_model(PATH_URDF))
    ab.q = Q_HOME

    # Initialize controller parameters
    q_des    = Q_HOME.copy()
    x_des    = spatialdyn.position(ab, -1, EE_OFFSET)
    quat_des = spatialdyn.orientation(ab)

    # Initialize Redis keys
    initialize_web_app(redis_pipe, ab)
    redis_pipe.set(KEY_SENSOR_Q, encode_matlab(ab.q))
    redis_pipe.set(KEY_KP_KV_POS, encode_matlab(KP_KV_POS))
    redis_pipe.set(KEY_KP_KV_ORI, encode_matlab(KP_KV_ORI))
    redis_pipe.set(KEY_KP_KV_JOINT, encode_matlab(KP_KV_JOINT))
    redis_pipe.execute()

    # Create signal handler
    global g_runloop
    signal.signal(signal.SIGINT, signal_handler)

    try:
        while g_runloop:
            # Wait for next loop
            timer.sleep()

            # Get Redis values
            redis_pipe.get(KEY_KP_KV_POS)
            redis_pipe.get(KEY_KP_KV_ORI)
            redis_pipe.get(KEY_KP_KV_JOINT)
            redis_pipe.get(KEY_WEB_INTERACTION)
            redis_results = redis_pipe.execute()
            kp_kv_pos, kp_kv_ori, kp_kv_joint = tuple(map(decode_matlab, redis_results[:3]))
            interaction = json.loads(redis_results[3].decode("utf-8"))

            # Compute Jacobian
            J = spatialdyn.jacobian(ab, offset=EE_OFFSET)

            # Compute position PD control
            x   = spatialdyn.position(ab, offset=EE_OFFSET)
            dx  = J[:3].dot(ab.dq)
            ddx = pd_control(x, x_des, dx, kp_kv_pos)

            # Compute orientation PD control
            quat = spatialdyn.opspace.near_quaternion(spatialdyn.orientation(ab), quat_des)
            w    = J[3:].dot(ab.dq)
            dw   = pd_control(quat, quat_des, w, kp_kv_ori)

            # Compute opspace torques
            N = np.eye(ab.dof)
            if spatialdyn.opspace.is_singular(ab, J, svd_epsilon=0.01):
                tau_cmd = spatialdyn.opspace.inverse_dynamics(ab, J[:3], ddx, N, svd_epsilon=0.01)
            else:
                ddx_dw  = np.hstack((ddx, dw))
                tau_cmd = spatialdyn.opspace.inverse_dynamics(ab, J, ddx_dw, N, svd_epsilon=0.01)

            # Add joint task in nullspace
            I   = np.eye(ab.dof)
            ddq = pd_control(ab.q, q_des, ab.dq, kp_kv_joint)
            tau_cmd += spatialdyn.opspace.inverse_dynamics(ab, I, ddq, N)

            # Add gravity compensation
            tau_cmd += spatialdyn.gravity(ab)

            # Parse interaction from web app
            f_ext = compute_external_forces(ab, interaction)
            adjust_position(interaction["key_down"], x_des)

            # Integrate
            spatialdyn.integrate(ab, tau_cmd, timer.dt, f_ext)

            # Set Redis values
            redis_pipe.set(KEY_CONTROL_TAU, encode_matlab(tau_cmd))
            redis_pipe.set(KEY_CONTROL_POS, encode_matlab(x_des))
            redis_pipe.set(KEY_SENSOR_Q, encode_matlab(ab.q))
            redis_pipe.set(KEY_SENSOR_DQ, encode_matlab(ab.dq))
            redis_pipe.set(KEY_TRAJ_POS, encode_matlab(x))
            redis_pipe.set(KEY_TRAJ_ORI, encode_matlab(quat.coeffs))
            redis_pipe.execute()

    except Exception as e:
        print(e)

    # Clear torques
    redis_client.set(KEY_CONTROL_TAU, encode_matlab(np.zeros((ab.dof,))))

    # Print simulation stats
    print("Simulated {}s in {}s.".format(timer.time_sim(), timer.time_elapsed()))

def initialize_web_app(redis_pipe, ab):
    # Register the urdf path so the server knows it's safe to fulfill requests for files in that directory.
    path_urdf = os.path.realpath(os.path.join(os.path.dirname(__file__), PATH_URDF))
    redis_pipe.hset(KEY_WEB_RESOURCES, NAME_APP, os.path.dirname(path_urdf))

    # Register key prefixes so the web app knows which models and objects to render.
    web_keys = {}
    web_keys["key_models_prefix"]  = KEY_MODELS_PREFIX
    web_keys["key_objects_prefix"] = KEY_OBJECTS_PREFIX
    redis_pipe.set(KEY_WEB_ARGS, json.dumps(web_keys))

    # Register the robot
    web_model = {}
    web_model["model"]    = json.loads(str(ab))
    web_model["key_q"]    = KEY_SENSOR_Q
    web_model["key_traj"] = KEY_TRAJ_POS
    redis_pipe.set(KEY_MODELS_PREFIX + ab.name, json.dumps(web_model))

    # Create a sphere marker for x_des
    web_object = {}
    x_des_marker = spatialdyn.Graphics("x_des_marker")
    x_des_marker.geometry.type = "sphere"
    x_des_marker.geometry.radius = 0.01
    web_object["graphics"] = [ json.loads(str(x_des_marker)) ]
    web_object["key_pos"]  = KEY_CONTROL_POS
    redis_pipe.set(KEY_OBJECTS_PREFIX + x_des_marker.name, json.dumps(web_object))

def compute_external_forces(ab, interaction):
    f_ext = {}

    # Check if the clicked object is the robot
    key_object = interaction["key_object"]
    if key_object != KEY_MODELS_PREFIX + ab.name:
        return f_ext

    # Extract the json fields
    idx_link = interaction["idx_link"]
    pos_mouse = np.array(interaction["pos_mouse_in_world"])
    pos_click = np.array(interaction["pos_click_in_link"])

    # Get the click position in world coordinates
    pos_click_in_world = spatialdyn.position(ab, idx_link, pos_click)

    # Set the click force
    f = GAIN_CLICK_DRAG * (pos_mouse - pos_click_in_world)
    f_click = np.hstack((f, np.zeros((3,))))

    # Translate the spatial force to the world frame
    f_ext[idx_link] = eigen.Translation3d(pos_click_in_world).fdot(f_click)

    return f_ext

def adjust_position(key, pos):
    if not key:
        return

    idx = 0
    sign = 1
    if key[0] == 'a':
        idx = 0
        sign = -1
    elif key[0] == 'd':
        idx = 0
        sign = 1
    elif key[0] == 'w':
        idx = 1
        sign = 1
    elif key[0] == 's':
        idx = 1
        sign = -1
    elif key[0] == 'e':
        idx = 2
        sign = 1
    elif key[0] == 'q':
        idx = 2
        sign = -1
    else:
        return

    pos[idx] += sign * GAIN_KEY_PRESS

if __name__ == "__main__":
    main()

