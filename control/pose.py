"""AUBO poses: base-frame metres and Rz(rz) @ Ry(ry) @ Rx(rx)."""

import numpy as np
from scipy.spatial.transform import Rotation


def vector6(value, name):
    vector = np.asarray(value, dtype=float)
    if vector.shape != (6,) or not np.isfinite(vector).all():
        raise ValueError(f"{name} must contain exactly six finite numbers")
    return vector


def pose_rotation(pose):
    # Extrinsic xyz with [rx, ry, rz] equals intrinsic ZYX with [rz, ry, rx].
    return Rotation.from_euler("xyz", pose[3:])


def pose_error(actual, target):
    actual = vector6(actual, "actual pose")
    target = vector6(target, "target pose")
    position = float(np.linalg.norm(actual[:3] - target[:3]))
    orientation = float((pose_rotation(target).inv() * pose_rotation(actual)).magnitude())
    return position, orientation
