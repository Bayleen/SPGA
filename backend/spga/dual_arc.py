from __future__ import annotations
from dataclasses import dataclass
import numpy as np


@dataclass
class DualArcReference:
    """Two-point virtual tool representation used by SPGA.

    A0 = physical/task tip. Contact semantics live only on A0.
    A1 = auxiliary endpoint that defines the regulated tool axis.
         A1 is always treated as non-contact/flexible by representation.
    """
    arc0: np.ndarray
    arc1: np.ndarray
    contact_mask_arc0: np.ndarray
    tool_axis: np.ndarray
    ell: float


def _normalize_rows(v: np.ndarray) -> np.ndarray:
    v = np.asarray(v, float)
    return v / np.maximum(np.linalg.norm(v, axis=1, keepdims=True), 1e-12)


def from_pose_trajectory(positions, rotations, contact_mask, ell, regulated_axis_index=1) -> DualArcReference:
    """Convert an SE(3) trajectory into A0/A1.

    A0_i = p_i
    A1_i = A0_i + ell * (R_i e_axis)
    """
    P = np.asarray(positions, float)
    R = np.asarray(rotations, float)
    C = np.asarray(contact_mask, bool)
    if P.ndim != 2 or P.shape[1] != 3:
        raise ValueError("positions must have shape (N,3)")
    if R.shape != (len(P), 3, 3):
        raise ValueError("rotations must have shape (N,3,3)")
    if C.shape != (len(P),):
        raise ValueError("contact_mask must have shape (N,)")
    u = _normalize_rows(R[:, :, int(regulated_axis_index)])
    a0 = P.copy()
    a1 = a0 + float(ell) * u
    return DualArcReference(a0, a1, C.copy(), u, float(ell))