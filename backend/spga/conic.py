from __future__ import annotations
from dataclasses import dataclass
import math
import numpy as np


def _norm_rows(v):
    v = np.asarray(v, float)
    return v / np.maximum(np.linalg.norm(v, axis=1, keepdims=True), 1e-12)


@dataclass
class ConicReference:
    center: np.ndarray
    tangent: np.ndarray
    surface_normal: np.ndarray
    alpha_star_rad: np.ndarray
    beta_rad: float
    tilted_axis: np.ndarray


def build_conic_from_tilted(dual_tilt, surface, cfg: dict) -> ConicReference:
    """Build the pose cone from the tilted tool axis.

    The cone center reconstructs the tilted axis sample-wise. Thus the cone is
    a soft pose-compatibility prior around the original tilted skill, not a
    shape constraint on the A0 trajectory.
    """
    a0 = np.asarray(dual_tilt.arc0, float)
    u = _norm_rows(np.asarray(dual_tilt.arc1, float) - a0)
    n = _norm_rows(surface.normal_from_world_batch(a0))
    proj = u - np.sum(u * n, axis=1, keepdims=True) * n
    pnorm = np.linalg.norm(proj, axis=1)
    t = np.zeros_like(proj)
    good = pnorm > 1e-9
    t[good] = proj[good] / pnorm[good, None]
    if np.any(~good):
        seed = np.tile(np.array([0.0, 0.0, 1.0]), (len(a0), 1))
        seed = seed - np.sum(seed * n, axis=1, keepdims=True) * n
        t[~good] = _norm_rows(seed)[~good]

    normal_component = np.sum(u * n, axis=1)
    alpha = np.arctan2(normal_component, np.maximum(pnorm, 1e-12))
    c = _norm_rows(np.cos(alpha)[:, None] * t + np.sin(alpha)[:, None] * n)
    beta = math.radians(float(cfg["conic"]["half_angle_deg"]))
    return ConicReference(c, t, n, alpha, beta, u.copy())


def conic_velocity(a0, a1, reference: ConicReference, ell: float, cfg: dict):
    """Soft conic pose correction.

    Returns (v_A0, v_A1, theta, h, k). A0 receives zero cone velocity; A1 is
    moved so the virtual tool axis stays near the tilted cone preference.
    """
    u = _norm_rows(np.asarray(a1, float) - np.asarray(a0, float))
    c = np.asarray(reference.center, float)
    dot = np.clip(np.sum(u * c, axis=1), -1.0, 1.0)
    theta = np.arccos(dot)

    tangent = c - dot[:, None] * u
    sinth = np.sin(theta)
    fac = np.zeros_like(theta)
    good = np.abs(sinth) > 1e-9
    fac[good] = theta[good] / sinth[good]
    e = fac[:, None] * tangent

    h = dot - math.cos(reference.beta_rad)
    eps = float(cfg["conic"]["epsilon_c"])
    r = np.clip((eps - h) / max(2.0 * eps, 1e-12), 0.0, 1.0)
    smooth = 3.0 * r * r - 2.0 * r * r * r
    k = float(cfg["conic"]["k_in"]) + float(cfg["conic"]["k_out"]) * smooth
    v1 = float(ell) * k[:, None] * e
    v0 = np.zeros_like(v1)
    return v0, v1, theta, h, k