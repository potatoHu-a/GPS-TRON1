#!/usr/bin/env python3
"""FAST-LIO FRD to ROS REP-103 FLU frame conversion helpers."""

import math


def _quat_multiply(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def normalize_quaternion(quaternion):
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm <= 1.0e-12:
        return 0.0, 0.0, 0.0, 1.0
    return tuple(value / norm for value in quaternion)


def convert_fastlio_frd_to_flu(position, orientation):
    """Change both world and body bases with S=diag(1,-1,-1).

    S is a proper 180-degree rotation about +X.  Applying P_flu = S P_frd
    S^-1 converts translation and orientation as one rigid frame convention
    change, instead of independently negating a yaw scalar.
    """
    x, y, z = position
    source = normalize_quaternion(orientation)
    frame_rotation = (1.0, 0.0, 0.0, 0.0)
    frame_inverse = (-1.0, 0.0, 0.0, 0.0)
    converted = _quat_multiply(
        _quat_multiply(frame_rotation, source), frame_inverse
    )
    return (x, -y, -z), normalize_quaternion(converted)


def quaternion_yaw(quaternion):
    x, y, z, w = quaternion
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def convert_fastlio_twist_frd_to_flu(linear, angular):
    """Rotate vectors by S=diag(1,-1,-1)."""
    lx, ly, lz = linear
    ax, ay, az = angular
    return (lx, -ly, -lz), (ax, -ay, -az)
