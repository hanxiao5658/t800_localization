#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Pure math helpers for level map <- raw FAST-LIO2 map coordinates.

R_map_raw maps vector coordinates expressed in raw to leveled map.
Only gravity fixes the two tilt degrees. +X in the leveled frame is the
horizontal projection of the original raw +X (conventional yaw gauge).
"""
import math
import numpy as np


def rotation_from_up(up):
    up = np.asarray(up, dtype=np.float64).reshape(3)
    if not np.all(np.isfinite(up)) or np.linalg.norm(up) < 1e-10:
        raise ValueError('invalid gravity-up vector')
    z = up / np.linalg.norm(up)
    x = np.array([1.0, 0.0, 0.0])
    x = x - np.dot(x, z) * z
    if np.linalg.norm(x) < 0.1:
        # Raw X almost vertical; preserve projected raw Y instead.
        x = np.array([0.0, 1.0, 0.0])
        x = x - np.dot(x, z) * z
    x = x / np.linalg.norm(x)
    y = np.cross(z, x)
    y /= np.linalg.norm(y)
    R = np.vstack((x, y, z))
    if np.linalg.det(R) < 0.9999:
        raise ValueError('invalid leveling rotation')
    return R


def rotation_from_manual(roll_deg, pitch_deg):
    """R_map_raw = R_y(pitch) R_x(roll). For exploratory adjustment only."""
    r = math.radians(float(roll_deg))
    p = math.radians(float(pitch_deg))
    cr, sr, cp, sp = math.cos(r), math.sin(r), math.cos(p), math.sin(p)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    return Ry @ Rx


def pose_matrix(position, quat_xyzw):
    from tf.transformations import quaternion_matrix
    q = np.asarray(quat_xyzw, dtype=float)
    if not np.all(np.isfinite(q)) or np.linalg.norm(q) < 1e-9:
        raise ValueError('invalid quaternion')
    M = quaternion_matrix(q / np.linalg.norm(q))
    M[:3, 3] = np.asarray(position, dtype=float)
    return M


def rpy_matrix(xyzrpy):
    from tf.transformations import euler_matrix
    v = np.asarray(xyzrpy, dtype=float)
    if v.size != 6 or not np.all(np.isfinite(v)):
        raise ValueError('initial pose must be six finite numbers')
    M = euler_matrix(v[3], v[4], v[5], axes='sxyz')
    M[:3, 3] = v[:3]
    return M


def horizontal_yaw(R):
    """Heading of body +X projected onto true leveled map XY plane."""
    x = np.asarray(R, dtype=float)[:3, 0]
    if math.hypot(x[0], x[1]) < 1e-5:
        return None
    return math.atan2(x[1], x[0])
