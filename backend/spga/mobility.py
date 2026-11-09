from __future__ import annotations
import numpy as np


def _smoothstep01(x):
    x = np.clip(x, 0.0, 1.0)
    return x*x*(3.0 - 2.0*x)


def arc0_geometry_mobility(reference, primitive_ids, fixed_mask,
                           exclusion_radius=0.0, transition_radius=0.01):
    """Generic Arc0 geometry authority around fixed/contact anchors.

    Fixed Arc0 samples have zero geometry authority.  Neighboring free samples
    transition smoothly back to full authority, avoiding a force discontinuity
    immediately beside a fixed contact anchor.
    """
    ref = np.asarray(reference, float)
    ids = np.asarray(primitive_ids)
    fixed = np.asarray(fixed_mask, bool)
    if len(ref) != len(ids) or len(ref) != len(fixed):
        raise ValueError("reference/id/fixed length mismatch")
    mob = np.ones(len(ref), dtype=float)
    for pid in np.unique(ids):
        idx = np.flatnonzero(ids == pid)
        anchors = np.flatnonzero(fixed[idx])
        if len(anchors) == 0:
            continue
        s = np.zeros(len(idx), dtype=float)
        if len(idx) > 1:
            s[1:] = np.cumsum(np.linalg.norm(np.diff(ref[idx], axis=0), axis=1))
        sf = s[anchors]
        dist = np.min(np.abs(s[:, None] - sf[None, :]), axis=1)
        ex = float(exclusion_radius)
        tr = max(float(transition_radius), 1e-12)
        local = _smoothstep01((dist - ex) / tr)
        local[dist <= ex] = 0.0
        mob[idx] = np.minimum(mob[idx], local)
    mob[fixed] = 0.0
    return mob