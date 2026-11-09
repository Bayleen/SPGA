from __future__ import annotations
import numpy as np


def _spacing(reference):
    R = np.asarray(reference, float)
    if len(R) < 2:
        return np.zeros(0, dtype=float)
    ds = np.linalg.norm(np.diff(R, axis=0), axis=1)
    good = ds > 1e-12
    med = float(np.median(ds[good])) if np.any(good) else 1.0
    return np.maximum(ds / max(med, 1e-12), 1e-6)


def _rail_gradient(points, reference, lam_t, lam_b):
    """Reference-relative elastic deformation energy for one rail.

    The energy acts on the deformation field d = A - A_ref, not on absolute
    point coordinates.  A rigid translation of an entire segment therefore
    has zero ribbon energy.  First differences provide tension and second
    differences preserve the changing trend of the Tilt reference.
    """
    P = np.asarray(points, float)
    R = np.asarray(reference, float)
    grad = np.zeros_like(P)
    if len(P) < 2:
        return grad, 0.0, 0.0
    d = P - R
    rho = _spacing(R)
    inv = 1.0 / rho
    g = (d[1:] - d[:-1]) * inv[:, None]
    et = 0.5 * float(np.sum(g*g))
    gt = np.zeros_like(P)
    gt[:-1] -= g * inv[:, None]
    gt[1:] += g * inv[:, None]
    grad += float(lam_t) * gt

    eb = 0.0
    if len(P) >= 3:
        h = g[1:] - g[:-1]
        eb = 0.5 * float(np.sum(h*h))
        gb = np.zeros_like(P)
        inv_prev = inv[:-1]
        inv_next = inv[1:]
        gb[:-2] += h * inv_prev[:, None]
        gb[1:-1] -= h * (inv_prev + inv_next)[:, None]
        gb[2:] += h * inv_next[:, None]
        grad += float(lam_b) * gb
    return grad, et, eb


def cut_segments(primitive_ids, fixed_mask):
    """Split structural coupling at fixed Arc0 anchors and primitive borders."""
    ids = np.asarray(primitive_ids)
    fixed = np.asarray(fixed_mask, bool)
    segments = []
    for pid in np.unique(ids):
        idx = np.flatnonzero(ids == pid)
        if len(idx) < 2:
            continue
        anchors = np.flatnonzero(fixed[idx])
        if len(anchors) == 0:
            segments.append(idx)
            continue
        bounds = [0] + anchors.tolist() + [len(idx)-1]
        clean = []
        for b in bounds:
            if not clean or b != clean[-1]:
                clean.append(b)
        for a, b in zip(clean[:-1], clean[1:]):
            seg = idx[a:b+1]
            if len(seg) >= 2:
                segments.append(seg)
    return segments


def ribbon_energy_gradient(a0, a1, r0, r1, primitive_ids, fixed_mask,
                           tension_gain, bending_gain,
                           arc0_weight=1.0, arc1_weight=1.0):
    """Gradient of the 2xN Dual-Arc elastic ribbon energy."""
    a0 = np.asarray(a0, float); a1 = np.asarray(a1, float)
    r0 = np.asarray(r0, float); r1 = np.asarray(r1, float)
    if not (a0.shape == a1.shape == r0.shape == r1.shape):
        raise ValueError("dual-arc shape mismatch")
    g0 = np.zeros_like(a0); g1 = np.zeros_like(a1)
    et0 = eb0 = et1 = eb1 = 0.0
    for seg in cut_segments(primitive_ids, fixed_mask):
        q0, t0, b0 = _rail_gradient(a0[seg], r0[seg], tension_gain, bending_gain)
        q1, t1, b1 = _rail_gradient(a1[seg], r1[seg], tension_gain, bending_gain)
        g0[seg] += float(arc0_weight) * q0
        g1[seg] += float(arc1_weight) * q1
        et0 += float(arc0_weight) * t0
        eb0 += float(arc0_weight) * b0
        et1 += float(arc1_weight) * t1
        eb1 += float(arc1_weight) * b1
    return g0, g1, {
        "arc0_tension": float(tension_gain) * et0,
        "arc0_bending": float(bending_gain) * eb0,
        "arc1_tension": float(tension_gain) * et1,
        "arc1_bending": float(bending_gain) * eb1,
        "total": float(tension_gain) * (et0 + et1) + float(bending_gain) * (eb0 + eb1),
    }