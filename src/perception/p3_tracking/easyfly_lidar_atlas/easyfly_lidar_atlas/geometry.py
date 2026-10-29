# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Measured scan registration and anchored SE(3) graph optimization.

These geometric scores are admission evidence, not a global pose covariance.
All poses transform a keyframe's original IMU coordinates into an atlas frame.
"""
from dataclasses import dataclass
import math
import time

import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


class GeometryRejected(ValueError):
    pass


def transform(value):
    value = np.asarray(value, dtype=np.float64)
    if value.shape != (4, 4) or not np.isfinite(value).all() or \
            not np.allclose(value[3], [0, 0, 0, 1], atol=1e-9) or \
            not np.allclose(value[:3, :3].T @ value[:3, :3], np.eye(3), atol=1e-6) or \
            abs(np.linalg.det(value[:3, :3]) - 1) > 1e-6 or \
            np.max(np.abs(value[:3, 3])) > 10000:
        raise GeometryRejected('invalid metric rigid transform')
    return value.copy()


def apply(pose, points):
    return points @ pose[:3, :3].T + pose[:3, 3]


def inverse(pose):
    out = np.eye(4)
    out[:3, :3] = pose[:3, :3].T
    out[:3, 3] = -out[:3, :3] @ pose[:3, 3]
    return out


def pose_distance(a, b):
    relative = inverse(a) @ b
    return float(np.linalg.norm(relative[:3, 3])), float(
        Rotation.from_matrix(relative[:3, :3]).magnitude())


def points(value, *, maximum=200000):
    value = np.asarray(value, dtype=np.float64)
    if value.ndim != 2 or value.shape[1:] != (3,) or not 100 <= len(value) <= maximum or \
            not np.isfinite(value).all() or np.max(np.abs(value)) > 1000:
        raise GeometryRejected('invalid or unbounded scan')
    return value


def voxel_sample(value, leaf=.15, maximum=4096):
    value = points(value)
    if not .05 <= leaf <= .5 or not 200 <= maximum <= 4096:
        raise GeometryRejected('invalid geometric sampling budget')
    # Choose actual measurements, not generated centroids or filled surfaces.
    _, index = np.unique(np.floor(value / leaf).astype(np.int64), axis=0, return_index=True)
    if len(index) > maximum:
        index = index[np.linspace(0, len(index) - 1, maximum, dtype=np.int64)]
    if len(index) < 200:
        raise GeometryRejected('insufficient independent spatial samples')
    return value[index].copy()


def place_descriptor(scan):
    """Bounded polar geometry descriptor used only to propose ICP candidates."""
    scan = points(scan, maximum=4096)
    radius = np.linalg.norm(scan[:, :2], axis=1)
    good = (radius >= .6) & (radius < 24) & (abs(scan[:, 2]) < 8)
    if np.count_nonzero(good) < 200:
        raise GeometryRejected('insufficient place descriptor support')
    ring = np.minimum(19, (radius[good] / 1.2).astype(int))
    sector = (np.floor((np.arctan2(scan[good, 1], scan[good, 0]) + np.pi) /
                      (2 * np.pi) * 60).astype(int)) % 60
    # Occupancy plus measured vertical extent; empty cells stay empty.
    low = np.full((20, 60), np.inf)
    high = np.full((20, 60), -np.inf)
    np.minimum.at(low, (ring, sector), scan[good, 2])
    np.maximum.at(high, (ring, sector), scan[good, 2])
    occupied = np.isfinite(low)
    descriptor = np.zeros((20, 60))
    descriptor[occupied] = 1. + np.minimum(4., high[occupied] - low[occupied])
    return descriptor


def descriptor_proposals(query, references, maximum=6):
    if query.shape != (20, 60) or not 1 <= maximum <= 6:
        raise GeometryRejected('invalid descriptor proposal budget')
    scores = []
    for i, reference in enumerate(references):
        if reference.shape != query.shape or not np.isfinite(reference).all():
            raise GeometryRejected('malformed reference descriptor')
        denominator = np.linalg.norm(query) * np.linalg.norm(reference)
        if denominator <= 0:
            continue
        cyclic = [float(np.sum(np.roll(query, shift, axis=1) * reference) / denominator)
                  for shift in range(60)]
        shift = int(np.argmax(cyclic))
        scores.append((cyclic[shift], i, shift * (2 * np.pi / 60)))
    return sorted(scores, reverse=True)[:maximum]


@dataclass
class Registration:
    pose: np.ndarray
    rmse: float
    overlap: float
    point_plane_rmse: float
    information_eigenvalues: np.ndarray
    iterations: int
    heldout_samples: int


class RegistrationTarget:
    def __init__(self, cloud):
        self.cloud = voxel_sample(cloud)
        self.tree = cKDTree(self.cloud)
        _, indices = self.tree.query(self.cloud, k=20, workers=1)
        neighbors = self.cloud[indices]
        centered = neighbors - neighbors.mean(axis=1, keepdims=True)
        covariance = np.einsum('nki,nkj->nij', centered, centered) / 20
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        self.normals = eigenvectors[:, :, 0]
        self.planar = (eigenvalues[:, 0] < .08 * np.maximum(eigenvalues.sum(axis=1), 1e-12)) & \
            (eigenvalues[:, 1] > 1e-5)

    def register(self, scan, initial, *, deadline=None):
        source = voxel_sample(scan)
        current = transform(initial)
        deadline = time.monotonic() + 2 if deadline is None else deadline
        training, heldout = source[::2], source[1::2]
        # Independent heldout samples never contribute to the normal equations.
        iterations = 0
        for iterations in range(40):
            if time.monotonic() > deadline:
                raise GeometryRejected('registration compute deadline')
            moved = apply(current, training)
            distance, index = self.tree.query(moved, workers=1)
            radius = 1.2 if iterations < 8 else .5 if iterations < 16 else .3
            good = distance < radius
            if np.count_nonzero(good) < 100:
                raise GeometryRejected('insufficient scan overlap')
            if iterations < 8:
                a, b = moved[good], self.cloud[index[good]]
                ca, cb = a.mean(axis=0), b.mean(axis=0)
                u, _, vt = np.linalg.svd((a - ca).T @ (b - cb))
                rotation = vt.T @ np.diag([1., 1., np.linalg.det(vt.T @ u.T)]) @ u.T
                delta = np.eye(4)
                delta[:3, :3], delta[:3, 3] = rotation, cb - rotation @ ca
            else:
                good &= self.planar[index]
                p, n, target = moved[good], self.normals[index[good]], self.cloud[index[good]]
                if len(p) < 100:
                    raise GeometryRejected('insufficient planar constraints')
                residual = np.einsum('ij,ij->i', n, p - target)
                robust = np.minimum(1., .06 / np.maximum(abs(residual), 1e-12))
                jacobian = np.column_stack((n, np.cross(p, n)))
                information = jacobian.T @ (robust[:, None] * jacobian)
                spectrum = np.linalg.eigvalsh(information / len(p))
                if spectrum[0] <= 1e-4 or spectrum[-1] / spectrum[0] > 1e6:
                    raise GeometryRejected('geometrically unobservable registration')
                step = -np.linalg.solve(information, jacobian.T @ (robust * residual))
                if np.linalg.norm(step[:3]) > 1 or np.linalg.norm(step[3:]) > .3:
                    raise GeometryRejected('unstable registration increment')
                delta = np.eye(4)
                delta[:3, :3] = Rotation.from_rotvec(step[3:]).as_matrix()
                delta[:3, 3] = step[:3]
            current = transform(delta @ current)
            if iterations >= 16 and pose_distance(np.eye(4), delta)[0] < 1e-5 and \
                    pose_distance(np.eye(4), delta)[1] < 1e-5:
                break
        moved = apply(current, heldout)
        distance, index = self.tree.query(moved, workers=1)
        good = distance < .3
        overlap = np.count_nonzero(good) / len(heldout)
        if overlap < .6 or np.count_nonzero(good) < 100:
            raise GeometryRejected('heldout geometric overlap too small')
        rmse = float(np.sqrt(np.mean(distance[good] ** 2)))
        plane = self.planar[index] & good
        p, n = moved[plane], self.normals[index[plane]]
        residual = np.einsum('ij,ij->i', n, p - self.cloud[index[plane]])
        if len(p) < 100:
            raise GeometryRejected('heldout geometry unobservable')
        jacobian = np.column_stack((n, np.cross(p, n)))
        spectrum = np.linalg.eigvalsh(jacobian.T @ jacobian / len(p))
        plane_rmse = float(np.sqrt(np.mean(residual ** 2)))
        if rmse > .15 or plane_rmse > .06 or spectrum[0] <= 1e-4 or \
                spectrum[-1] / spectrum[0] > 1e6:
            raise GeometryRejected('heldout residual or observability rejected')
        return Registration(current, rmse, overlap, plane_rmse, spectrum, iterations + 1, len(heldout))


def unique_registration(candidates):
    if not candidates:
        raise GeometryRejected('no geometrically verified place')
    candidates = sorted(candidates, key=lambda result: result.rmse + .1 * (1 - result.overlap))
    best = candidates[0]
    for other in candidates[1:]:
        translation, rotation = pose_distance(best.pose, other.pose)
        if (translation > .5 or rotation > .15) and \
                other.rmse + .1 * (1 - other.overlap) < best.rmse + .1 * (1 - best.overlap) + .03:
            raise GeometryRejected('ambiguous geometrically distinct places')
    return best


@dataclass
class Edge:
    start: int
    end: int
    measurement: np.ndarray
    loop: bool = False


def optimize_graph(poses, edges, *, deadline=None):
    if not 2 <= len(poses) <= 256 or not len(poses) - 1 <= len(edges) <= 768:
        raise GeometryRejected('pose graph resource bounds')
    poses = [transform(pose) for pose in poses]
    deadline = time.monotonic() + 5 if deadline is None else deadline
    neighbors = [set() for _ in poses]
    for edge in edges:
        if not 0 <= edge.start < edge.end < len(poses):
            raise GeometryRejected('invalid graph edge')
        transform(edge.measurement)
        neighbors[edge.start].add(edge.end)
        neighbors[edge.end].add(edge.start)
    reached, pending = {0}, [0]
    while pending:
        for other in neighbors[pending.pop()] - reached:
            reached.add(other)
            pending.append(other)
    if len(reached) != len(poses):
        raise GeometryRejected('disconnected pose graph')

    def pack(pose):
        return np.r_[pose[:3, 3], Rotation.from_matrix(pose[:3, :3]).as_rotvec()]

    def unpack(values):
        result = [poses[0]]
        for value in values.reshape(-1, 6):
            pose = np.eye(4)
            pose[:3, :3] = Rotation.from_rotvec(value[3:]).as_matrix()
            pose[:3, 3] = value[:3]
            result.append(pose)
        return result

    def residual(values):
        if time.monotonic() > deadline:
            raise GeometryRejected('pose graph compute deadline')
        current = unpack(values)
        errors = []
        for edge in edges:
            error = inverse(edge.measurement) @ inverse(current[edge.start]) @ current[edge.end]
            # Explicit optimizer weights, never advertised as pose covariance.
            errors.extend(error[:3, 3] / .05)
            errors.extend(Rotation.from_matrix(error[:3, :3]).as_rotvec() / .03)
        return np.asarray(errors)

    initial = np.concatenate([pack(pose) for pose in poses[1:]])
    sparsity = lil_matrix((6 * len(edges), len(initial)), dtype=int)
    for i, edge in enumerate(edges):
        for vertex in [edge.start, edge.end]:
            if vertex:
                sparsity[6*i:6*i+6, 6*(vertex-1):6*vertex] = 1
    # Smooth robust curvature and column scaling avoid a nearly curvature-free
    # first step when the new closing edge carries the accumulated drift.
    result = least_squares(residual, initial, jac_sparsity=sparsity.tocsr(), loss='soft_l1',
                           x_scale='jac', f_scale=1., max_nfev=80,
                           ftol=1e-8, xtol=1e-8, gtol=1e-8)
    corrected = unpack(result.x)
    if not result.success or not np.isfinite(result.x).all() or \
            np.linalg.norm(residual(result.x)) > np.linalg.norm(residual(initial)) + 1e-7:
        raise GeometryRejected('pose graph failed to reduce measured residual')
    for before, after in zip(poses, corrected):
        translation, rotation = pose_distance(before, after)
        if translation > 2 or rotation > math.radians(20):
            raise GeometryRejected('graph correction outside declared local drift bounds')
    return corrected, dict(evaluations=result.nfev, cost=float(result.cost),
                          before_norm=float(np.linalg.norm(residual(initial))),
                          after_norm=float(np.linalg.norm(residual(result.x))))
