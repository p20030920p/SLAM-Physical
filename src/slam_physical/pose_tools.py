"""Planar timestamp matching, without fitting estimated tracks to truth."""

from __future__ import annotations

import math

import numpy as np

__all__ = ["yaw_of", "wrap", "matrix", "interpolate_pose"]


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def matrix(x, y, yaw, z=0.0):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array(
        [[c, -s, 0, x], [s, c, 0, y], [0, 0, 1, z], [0, 0, 0, 1]], dtype=float
    )


def interpolate_pose(samples, stamp, tolerance=0.12):
    if not samples:
        return None
    times = np.array([item[0] for item in samples])
    nearest = int(np.argmin(np.abs(times - stamp)))
    if abs(times[nearest] - stamp) > tolerance:
        return None
    right = int(np.searchsorted(times, stamp))
    if right == 0 or right == len(samples):
        return np.array(samples[nearest][1:]), float(abs(times[nearest] - stamp))
    left = right - 1
    if times[right] - times[left] > 2 * tolerance:
        return None
    alpha = (stamp - times[left]) / (times[right] - times[left])
    a, b = np.array(samples[left][1:]), np.array(samples[right][1:])
    result = a + alpha * (b - a)
    result[2] = wrap(a[2] + alpha * wrap(b[2] - a[2]))
    return result, float(min(stamp - times[left], times[right] - stamp))
