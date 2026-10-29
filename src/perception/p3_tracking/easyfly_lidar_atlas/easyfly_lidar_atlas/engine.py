# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Single-worker atlas state. Corrections never alter native local odometry."""
from dataclasses import replace
import time
import uuid

import numpy as np
from scipy.spatial.transform import Rotation

from .archive import Archive, Keyframe, load, save
from .geometry import (Edge, GeometryRejected, RegistrationTarget, apply,
                       descriptor_proposals, inverse, optimize_graph,
                       place_descriptor, pose_distance, transform,
                       unique_registration, voxel_sample)


class Engine:
    def __init__(self, backend, calibration_sha256, *, atlas_path=None):
        if backend not in ('fast_lio', 'fast_livo2') or len(calibration_sha256) != 64:
            raise GeometryRejected('invalid backend/calibration identity')
        self.backend, self.calibration = backend, calibration_sha256
        self.session = uuid.uuid4().hex
        self.archive = load(atlas_path, backend, calibration_sha256) if atlas_path else \
            Archive(backend, calibration_sha256, [], [])
        self.loaded_count = len(self.archive.keyframes)
        self.descriptors = [place_descriptor(key.scan) for key in self.archive.keyframes]
        self.correction = None if self.loaded_count else np.eye(4)
        self.last_key = None
        self.last_input_stamp = 0
        self.epoch = 0
        self.relocalizations = 0
        self.loops = 0
        self.last_loop_stamp = 0
        self.last_evidence = {}

    def _register(self, scan, descriptor, eligible):
        references = [self.descriptors[i] for i in eligible]
        proposals = descriptor_proposals(descriptor, references)
        accepted, evidence, rejected = [], [], []
        deadline = time.monotonic() + 3
        for score, ordinal, yaw in proposals:
            if time.monotonic() >= deadline:
                break
            index = eligible[ordinal]
            reference = self.archive.keyframes[index]
            rotation = np.eye(4)
            rotation[:3, :3] = Rotation.from_euler('z', yaw).as_matrix()
            try:
                # Register in the reference's measured IMU frame. Its atlas
                # pose only maps the geometric result into the shared frame.
                result = RegistrationTarget(reference.scan).register(
                    scan, rotation, deadline=deadline)
                result.pose = transform(reference.atlas_pose @ result.pose)
                accepted.append(result)
                evidence.append((result, index, dict(reference=index,
                    descriptor_score=score, rmse=result.rmse, overlap=result.overlap,
                    plane_rmse=result.point_plane_rmse, heldout=result.heldout_samples,
                    observability=result.information_eigenvalues.tolist())))
            except GeometryRejected as error:
                rejected.append(dict(reference=index, reason=str(error)))
                continue
        if not accepted:
            raise GeometryRejected('no geometrically verified place: ' +
                '; '.join(str(row['reference'])+':'+row['reason'] for row in rejected))
        best = unique_registration(accepted)
        return next((index, result, record) for result, index, record in evidence if result is best)

    def process(self, registered_cloud, native_pose, stamp_ns, source_sha256, *, scan_is_imu=False):
        """Consume an original scan and its same-source-time native IMU pose."""
        native_pose = transform(native_pose)
        if not isinstance(stamp_ns, int) or stamp_ns <= self.last_input_stamp or \
                len(source_sha256) != 64:
            raise GeometryRejected('regressing or invalid source identity')
        self.last_input_stamp = stamp_ns
        if self.last_key is not None:
            key = self.archive.keyframes[self.last_key]
            distance, angle = pose_distance(key.native_pose, native_pose)
            if distance < .75 and angle < np.deg2rad(15) and stamp_ns-key.stamp_ns < 5_000_000_000:
                return transform(self.correction @ native_pose), False
        if len(self.archive.keyframes) >= 256:
            raise GeometryRejected('atlas keyframe capacity reached; save and start a new session')
        scan = voxel_sample(registered_cloud if scan_is_imu else
                            apply(inverse(native_pose), registered_cloud))
        descriptor = place_descriptor(scan)
        keys, edges = list(self.archive.keyframes), list(self.archive.edges)
        current = len(keys)
        correction = self.correction
        record = dict(kind='native_odometry_keyframe', stamp_ns=stamp_ns)
        relocalized = correction is None
        if relocalized:
            reference, registration, geometric = self._register(scan, descriptor, list(range(current)))
            corrected = registration.pose
            correction = transform(corrected @ inverse(native_pose))
            edges.append(Edge(reference, current,
                              inverse(keys[reference].atlas_pose) @ corrected, True))
            record.update(kind='geometric_old_map_relocalization', **geometric)
        else:
            corrected = transform(correction @ native_pose)
            if self.last_key is not None:
                edges.append(Edge(self.last_key, current,
                                  inverse(keys[self.last_key].native_pose) @ native_pose))
        keys.append(Keyframe(scan, native_pose.copy(), corrected, stamp_ns, self.session, source_sha256))
        loop = False
        # Descriptor matches only propose a closure. A separate heldout ICP
        # check and the bounded measured pose graph must both succeed.
        if not relocalized and current >= 8 and stamp_ns-self.last_loop_stamp >= 5_000_000_000:
            eligible = [i for i, key in enumerate(keys[:-1]) if
                        key.session != self.session or
                        (current-i >= 8 and stamp_ns-key.stamp_ns >= 20_000_000_000)]
            if eligible:
                try:
                    reference, registration, geometric = self._register(scan, descriptor, eligible)
                    if pose_distance(corrected, registration.pose)[0] > 2 or \
                            pose_distance(corrected, registration.pose)[1] > np.deg2rad(20):
                        raise GeometryRejected('loop outside bounded drift domain')
                    candidate = edges + [Edge(reference, current,
                        inverse(keys[reference].atlas_pose) @ registration.pose, True)]
                    poses, optimization = optimize_graph([key.atlas_pose for key in keys], candidate)
                    keys = [replace(key, atlas_pose=pose) for key, pose in zip(keys, poses)]
                    edges = candidate
                    corrected = poses[-1]
                    correction = transform(corrected @ inverse(native_pose))
                    record.update(kind='geometric_loop_optimized', **geometric, optimization=optimization)
                    loop = True
                except GeometryRejected as error:
                    record['loop_rejected'] = str(error)
        # Commit only fully validated results. Input/native arrays remain owned
        # copies; graph optimization replaces atlas poses without rewriting them.
        self.archive = Archive(self.backend, self.calibration, keys, edges, self.archive.identity)
        self.descriptors.append(descriptor)
        self.last_key, self.correction = current, correction
        if relocalized:
            self.relocalizations += 1
        if loop:
            self.loops += 1
            self.last_loop_stamp = stamp_ns
        if loop or relocalized:
            self.epoch += 1
        self.last_evidence = record
        return transform(corrected), True

    def persist(self, path):
        if self.correction is None or len(self.archive.keyframes)-self.loaded_count < 2:
            raise GeometryRejected('at least two native session keyframes required before saving')
        return save(path, self.archive)

    def status(self):
        return dict(session=self.session, backend=self.backend, keyframes=len(self.archive.keyframes),
                    edges=len(self.archive.edges), loaded_keyframes=self.loaded_count,
                    old_map_identity=self.archive.identity,
                    localized=self.correction is not None, correction_epoch=self.epoch,
                    relocalizations=self.relocalizations, loops=self.loops,
                    evidence=self.last_evidence)
