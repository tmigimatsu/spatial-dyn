#!/usr/bin/env python3

import argparse
import pathlib
from typing import Optional

import ctrlutils
from ctrlutils import eigen
import spatialdyn as dyn
import redisgl


def main(
    urdf: str,
    host: str,
    port: int,
    password: Optional[str],
    resource_path: Optional[str],
) -> None:
    ab = dyn.urdf.load_model(urdf, path_meshes="iiwa_description")

    redis = ctrlutils.RedisClient(host=host, port=port, password=password)
    redisgl.register_resource_path(
        redis,
        str(pathlib.Path(urdf).parent.absolute())
        if resource_path is None
        else resource_path,
    )
    model_keys = redisgl.ModelKeys("dyn")
    redisgl.register_model_keys(redis, model_keys)

    key_q = f"{ab.name}::sensor::q"
    redis.set_matrix(key_q, ab.q)
    robot_model = redisgl.RobotModel(articulated_body=ab, key_q=key_q)
    redisgl.register_robot(redis, model_keys, robot_model)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument(
        "--help",
        action="help",
        default=argparse.SUPPRESS,
        help="Show this help message and exit.",
    )
    parser.add_argument("urdf", help="Path to urdf.")
    parser.add_argument("-h", "--host", default="127.0.0.1", help="Redis host.")
    parser.add_argument("-p", "--port", type=int, default=6379, help="Redis port.")
    parser.add_argument("-a", "--password", help="Redis password.")
    parser.add_argument("--resource-path", help="Path to urdf meshes.")
    args = parser.parse_args()

    main(**vars(args))
