"""Task policy on owned map copies; observed coverage never uses inflated masks."""
import hashlib
import math
import numpy as np


def geometry(msg):
    return (msg.size_x, msg.size_y, msg.size_z, msg.resolution,
            msg.origin.x, msg.origin.y, msg.origin.z)


def identity(msg):
    return (msg.session_id, msg.generation, msg.revision, msg.header.frame_id,
            msg.header.stamp.sec, msg.header.stamp.nanosec, *geometry(msg))


def validate_pair(raw, inflated):
    if identity(raw) != identity(inflated) or not raw.valid or not inflated.valid:
        raise ValueError('MAP_PAIR_IDENTITY')
    g = geometry(raw)
    if (raw.header.frame_id != 'odom' or not raw.session_id or
            not all(math.isfinite(x) for x in g) or
            min(g[:3]) < 3 or max(g[:3]) > 256 or np.prod(g[:3]) > 500000 or
            abs(g[3] - .2) > 1e-9 or raw.inflation_radius != 0. or
            not .65 <= inflated.inflation_radius <= 1.):
        raise ValueError('MAP_LAYOUT')
    a = np.asarray(raw.data, dtype=np.int8)
    b = np.asarray(inflated.data, dtype=np.int8)
    if (len(a) != np.prod(g[:3]) or len(b) != len(a) or
            not np.isin(a, [-1, 0, 100]).all() or not np.isin(b, [-1, 0, 100]).all() or
            ((b == 0) & (a != 0)).any()):
        raise ValueError('MAP_VALUES')
    return a, b


class Policy:
    def __init__(self, msg, low, high, forbidden):
        self.low = np.asarray(low, dtype=float)
        self.high = np.asarray(high, dtype=float)
        self.forbidden = np.asarray(forbidden, dtype=float).reshape((-1, 2, 3))
        self.geometry = geometry(msg)
        size = np.array(self.geometry[:3]); res = msg.resolution
        origin = np.array(self.geometry[4:])
        if (not np.isfinite(self.low).all() or not np.isfinite(self.high).all() or
                (self.high - self.low < .8).any() or
                (self.low < origin - 1e-8).any() or
                (self.high > origin + size * res + 1e-8).any() or
                self.low[2] < .5 or self.high[2] > 2.5 or
                not np.isfinite(self.forbidden).all() or
                (len(self.forbidden) and (self.forbidden[:, 0] >= self.forbidden[:, 1]).any())):
            raise ValueError('TASK_BOUNDS')
        z, y, x = np.indices((msg.size_z, msg.size_y, msg.size_x))
        self.centers = np.column_stack((x.ravel(), y.ravel(), z.ravel())) * res + origin + res/2
        # Whole cells must lie within the declared task; no boundary expansion.
        eligible = ((self.centers-res/2 >= self.low-1e-8) &
                    (self.centers+res/2 <= self.high+1e-8)).all(axis=1)
        navigable = eligible.copy()
        for low_box, high_box in self.forbidden:
            intersects = ((self.centers+res/2 > low_box) &
                          (self.centers-res/2 < high_box)).all(axis=1)
            eligible &= ~intersects
            clearance = ((self.centers+res/2 > low_box-.65) &
                         (self.centers-res/2 < high_box+.65)).all(axis=1)
            navigable &= ~clearance
        self.eligible = eligible
        self.navigable = navigable
        if eligible.sum() < 100:
            raise ValueError('TASK_VOLUME_TOO_SMALL')
        self.eligible_sha = hashlib.sha256(eligible.tobytes()).hexdigest()

    def apply(self, raw, inflated, reserve_unknown=False):
        a, b = validate_pair(raw, inflated)
        if geometry(raw) != self.geometry:
            raise ValueError('MAP_GEOMETRY_CHANGED')
        coverage = float(np.count_nonzero((a != -1) & self.eligible) / self.eligible.sum())
        a = a.copy(); b = b.copy()
        if reserve_unknown:
            # Reserve the complete physical map BEFORE applying the task fence:
            # outside-task unknown cells must not disappear behind its 100 mask.
            shape = (raw.size_z, raw.size_y, raw.size_x)
            mask = (b == -1).reshape(shape)
            extent = math.ceil(inflated.inflation_radius/raw.resolution)+1
            window = 2*extent+1
            for axis in range(3):
                padding = [(0, 0)]*3; padding[axis] = (extent, extent)
                padded = np.pad(mask, padding, constant_values=False)
                sums = np.cumsum(padded, axis=axis, dtype=np.int32)
                zero = list(sums.shape); zero[axis] = 1
                sums = np.concatenate((np.zeros(zero, dtype=np.int32), sums), axis=axis)
                first = [slice(None)]*3; last = first.copy()
                first[axis] = slice(None, -window); last[axis] = slice(window, None)
                mask = sums[tuple(last)]-sums[tuple(first)] > 0
            # New reservations are obstacles, not new source-unknown voxels:
            # the trajectory owner's independent unknown dilation remains once.
            b[mask.ravel() & (b == 0)] = 100
        a[~self.eligible] = 100
        b[~self.navigable] = 100
        return a, b, coverage

    def contains(self, point):
        p = np.asarray(point)
        if not np.isfinite(p).all() or (p < self.low).any() or (p >= self.high).any():
            return False
        return not any(((p >= lo-.65) & (p <= hi+.65)).all() for lo, hi in self.forbidden)
