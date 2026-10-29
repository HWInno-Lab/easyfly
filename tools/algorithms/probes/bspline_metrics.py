#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Exact native B-spline polynomial conversion and conservative map certificates.

No fitting, SciPy binary dependency, trajectory retiming, or generated planner
output. Only consume recorded original knots, control points and degree.
"""
import base64
from functools import lru_cache
import math
import numpy as np


def polynomial_pieces(plan):
    degree = int(plan['order'])
    knots = np.asarray(plan['knots'], dtype=float)
    points = np.asarray(plan['position_points'], dtype=float)
    if (not 1 <= degree <= 7 or points.ndim != 2 or points.shape[1] != 3
            or len(knots) != len(points) + degree + 1 or not np.isfinite(points).all()
            or not np.isfinite(knots).all() or (np.diff(knots) < 0).any()):
        raise ValueError('invalid original B-spline')
    pieces = []
    origin = knots[degree]
    for interval in range(degree, len(points)):
        start, end = knots[interval:interval + 2]
        if end <= start:
            continue

        @lru_cache(None)
        def basis(index, order):
            if order == 0:
                return np.asarray([float(index == interval)])
            value = np.zeros(order + 1)
            denominator = knots[index + order] - knots[index]
            if denominator > 0:
                term = np.polynomial.polynomial.polymul(
                    basis(index, order - 1), [start - knots[index], 1.]) / denominator
                value[:len(term)] += term
            denominator = knots[index + order + 1] - knots[index + 1]
            if denominator > 0:
                term = np.polynomial.polynomial.polymul(
                    basis(index + 1, order - 1), [knots[index + order + 1] - start, -1.]) / denominator
                value[:len(term)] += term
            return value

        coefficients = np.zeros((3, degree + 1))
        for index, point in enumerate(points):
            coefficients += point[:, None] * basis(index, degree)[None, :]
        pieces.append(dict(start=float(start - origin), duration=float(end - start),
                           coefficients=coefficients[:, ::-1]))
    if not pieces:
        raise ValueError('zero native B-spline span')
    return pieces


def value(pieces, relative, derivative=0):
    relative = max(0., min(relative, pieces[-1]['start'] + pieces[-1]['duration']))
    for i, piece in enumerate(pieces):
        if relative <= piece['start'] + piece['duration'] or i == len(pieces) - 1:
            local = relative - piece['start']
            return np.array([np.polyval(np.polyder(c, derivative), local)
                             for c in piece['coefficients']])
    raise ValueError('empty polynomial')


def derivative_maximum(pieces, derivative):
    maximum = 0.
    for piece in pieces:
        squared = np.asarray([0.])
        for coefficients in piece['coefficients']:
            d = np.polyder(coefficients, derivative)
            squared = np.polyadd(squared, np.polymul(d, d))
        roots = np.roots(np.polyder(squared))
        times = [0., piece['duration']] + [r.real for r in roots
                 if abs(r.imag) < 1e-7 and 0 < r.real < piece['duration']]
        maximum = max(maximum, math.sqrt(max(0., max(float(np.polyval(squared, t)) for t in times))))
    return maximum


class MapCertificate:
    def __init__(self, message):
        if not message['valid'] or message['frame'] != 'odom' or message['inflation'] < .65:
            raise ValueError('invalid common map')
        self.size = np.asarray(message['size'], dtype=int)
        self.origin = np.asarray(message['origin'], dtype=float)
        self.resolution = float(message['resolution'])
        cells = np.frombuffer(base64.b64decode(message['data_b64']), dtype=np.int8)
        if len(cells) != math.prod(self.size) or not np.isin(cells, [-1, 0, 100]).all():
            raise ValueError('invalid map cells')
        self.cells = cells.reshape(tuple(self.size[::-1])).copy()
        # Same complete-cell height and XY inset rules as the SUPER owner.
        # This masks an owned evaluation copy; original evidence is untouched.
        for axis in range(3):
            low = self.origin[axis] + np.arange(self.size[axis]) * self.resolution
            high = low + self.resolution
            if axis == 2:
                blocked = (low < .5 - 1e-8) | (high > 2.5 + 1e-8)
            else:
                far = self.origin[axis] + self.size[axis] * self.resolution
                blocked = (low < self.origin[axis] + message['inflation']) | (high > far - message['inflation'])
            if axis == 0:
                self.cells[:, :, blocked] = 100
            elif axis == 1:
                self.cells[:, blocked, :] = 100
            else:
                self.cells[blocked, :, :] = 100

    def free_box(self, low, high):
        if not np.isfinite(low).all() or not np.isfinite(high).all():
            return False
        a = np.floor((low - 1e-9 - self.origin) / self.resolution).astype(int)
        b = np.floor((high + 1e-9 - self.origin) / self.resolution).astype(int)
        if (a < 0).any() or (b >= self.size).any() or (b < a).any():
            return False
        return bool((self.cells[a[2]:b[2]+1, a[1]:b[1]+1, a[0]:b[0]+1] == 0).all())

    def trajectory(self, pieces):
        nodes = [0]

        def clear(controls, depth):
            nodes[0] += 1
            if nodes[0] > 100000 or not np.isfinite(controls).all():
                return False
            if self.free_box(controls.min(axis=1), controls.max(axis=1)):
                return True
            if depth >= 18 or not self.free_box(controls[:, 0], controls[:, 0]) or not self.free_box(controls[:, -1], controls[:, -1]):
                return False
            work = controls.copy()
            left, right = np.zeros_like(work), np.zeros_like(work)
            left[:, 0], right[:, -1] = work[:, 0], work[:, -1]
            for level in range(1, work.shape[1]):
                work = .5 * (work[:, :-1] + work[:, 1:])
                left[:, level], right[:, -1-level] = work[:, 0], work[:, -1]
            return clear(left, depth + 1) and clear(right, depth + 1)

        for piece in pieces:
            c = piece['coefficients']
            degree = c.shape[1] - 1
            controls = np.zeros_like(c)
            for k in range(degree + 1):
                for j in range(k + 1):
                    controls[:, k] += c[:, degree-j] * piece['duration']**j * math.comb(k, j) / math.comb(degree, j)
            if not clear(controls, 0):
                return False
        return True


def jerk_energy(pieces):
    result = 0.
    for piece in pieces:
        for c in piece['coefficients']:
            j = np.polyder(c, 3)
            integral = np.polyint(np.polymul(j, j))
            result += float(np.polyval(integral, piece['duration']) - np.polyval(integral, 0.))
    return result
