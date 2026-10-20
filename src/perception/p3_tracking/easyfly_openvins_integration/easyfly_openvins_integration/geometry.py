# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Rigid poses and OpenVINS error-state covariance, independent of ROS/TF."""

import math

import numpy as np


def vector(value, size):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError('invalid finite vector')
    return result.copy()


def quaternion(value):
    result = vector(value, 4)
    norm = np.linalg.norm(result)
    if abs(norm - 1.) > 1e-3:
        raise ValueError('quaternion must have unit norm')
    return result / norm


def multiply(a, b):
    """Hamilton multiplication, xyzw; a parent<-middle, b middle<-child."""
    a, b = quaternion(a), quaternion(b)
    return np.r_[a[3]*b[:3] + b[3]*a[:3] + np.cross(a[:3], b[:3]),
                 a[3]*b[3] - np.dot(a[:3], b[:3])]


def rotation(q):
    x, y, z, w = quaternion(q)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def skew(v):
    x, y, z = vector(v, 3)
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def covariance(value):
    result = np.asarray(value, dtype=float).reshape(6, 6)
    if not np.isfinite(result).all() or not np.allclose(
            result, result.T, atol=1e-8, rtol=1e-6):
        raise ValueError('invalid covariance')
    if np.linalg.eigvalsh(result).min() < -1e-9:
        raise ValueError('covariance must be positive semidefinite')
    return (result + result.T) / 2


def imu_to_base(position, orientation, raw_covariance, translation, orientation_ib):
    """T_GB = T_GI T_IB, with *upstream OpenVINS* covariance convention.

    OpenVINS publishes [dp_G, dtheta_I], a left JPL quaternion error (equivalently
    a right Hamilton error on its published q_GI). Convert both the lever arm
    and orientation error to fixed G axes. Extrinsic uncertainty is not supplied.
    """
    p, q = vector(position, 3), quaternion(orientation)
    r, t = rotation(q), vector(translation, 3)
    jacobian = np.zeros((6, 6))
    jacobian[:3, :3] = np.eye(3)
    jacobian[:3, 3:] = -r @ skew(t)
    jacobian[3:, 3:] = r
    result = jacobian @ covariance(raw_covariance) @ jacobian.T
    return p + r @ t, multiply(q, orientation_ib), (result + result.T) / 2


def interpolate(p0, q0, p1, q1, fraction):
    if not math.isfinite(fraction) or not 0 <= fraction <= 1:
        raise ValueError('extrapolation is forbidden')
    a, b = quaternion(q0), quaternion(q1)
    dot = float(np.dot(a, b))
    if dot < 0:
        b, dot = -b, -dot
    dot = min(1., dot)
    if dot > .9995:
        q = (1-fraction)*a + fraction*b
        q /= np.linalg.norm(q)
    else:
        theta = math.acos(dot)
        q = (math.sin((1-fraction)*theta)*a + math.sin(fraction*theta)*b) / math.sin(theta)
    return (1-fraction)*vector(p0, 3) + fraction*vector(p1, 3), q


def yaw(q):
    r = rotation(q)
    return math.atan2(r[1, 0], r[0, 0])


def yaw_quaternion(angle):
    return np.array([0., 0., math.sin(angle/2), math.cos(angle/2)])


def angle_between(a, b):
    return 2*math.acos(min(1., abs(float(np.dot(quaternion(a), quaternion(b))))))
