from __future__ import annotations
from dataclasses import dataclass
from collections import deque
import numpy as np
from .geometry import min_rotation
from .geometry_field import geometry_force, crossbar_active_contact_force
from .mobility import arc0_geometry_mobility
from .ribbon_energy import ribbon_energy_gradient
from .conic import conic_velocity, ConicReference


@dataclass
class WarpResult:
    positions: np.ndarray
    rotations: np.ndarray
    arc0: np.ndarray
    arc1: np.ndarray
    initial_arc0: np.ndarray
    initial_arc1: np.ndarray
    diagnostics: dict


def _norm_rows(v):
    v = np.asarray(v, float)
    return v / np.maximum(np.linalg.norm(v, axis=1, keepdims=True), 1e-12)


def _project_fixed_length_tangent(v0, v1, a0, a1, fixed_a0):
    """Project dual-arc generalized velocity onto the fixed-length tangent space.

    For a free dual arc, the first-order fixed-length condition is

        u^T (dA1 - dA0) = 0,

    where u=(A1-A0)/ell.  The Euclidean minimum-change projection removes the
    relative radial component symmetrically from the two carrier velocities.

    If A0 is a fixed/contact anchor, dA0=0 and only the radial component of A1
    is removed, so the represented tool can rotate about A0 but cannot extend
    or shorten.
    """
    V0 = np.asarray(v0, float).copy()
    V1 = np.asarray(v1, float).copy()
    A0 = np.asarray(a0, float)
    A1 = np.asarray(a1, float)
    fixed = np.asarray(fixed_a0, bool)
    u = _norm_rows(A1 - A0)

    free = ~fixed
    if np.any(free):
        radial_rel = np.sum((V1[free] - V0[free]) * u[free], axis=1)
        corr = 0.5 * radial_rel[:, None] * u[free]
        V0[free] += corr
        V1[free] -= corr

    if np.any(fixed):
        V0[fixed] = 0.0
        radial_v1 = np.sum(V1[fixed] * u[fixed], axis=1)
        V1[fixed] -= radial_v1[:, None] * u[fixed]

    return V0, V1


def _windowed_balance_metrics(force_window, state_window):
    """Return averaged-force and bounded-state diagnostics for active-set chatter.

    Active crossbar contacts are nonsmooth: the deepest active point can switch
    between neighboring samples/triangles while the state remains in a small
    neighborhood.  This helper detects that *numerical* bounded equilibrium.
    It does not change any SPGA force.
    """
    if len(force_window) == 0 or len(state_window) == 0:
        return np.inf, np.inf, np.inf

    f0_stack = np.stack([x[0] for x in force_window], axis=0)
    f1_stack = np.stack([x[1] for x in force_window], axis=0)
    avg0 = np.mean(f0_stack, axis=0)
    avg1 = np.mean(f1_stack, axis=0)
    avg_residual = max(
        float(np.max(np.linalg.norm(avg0, axis=1))),
        float(np.max(np.linalg.norm(avg1, axis=1))),
    )

    s0_stack = np.stack([x[0] for x in state_window], axis=0)
    s1_stack = np.stack([x[1] for x in state_window], axis=0)
    drift = max(
        float(np.max(np.linalg.norm(s0_stack[-1] - s0_stack[0], axis=1))),
        float(np.max(np.linalg.norm(s1_stack[-1] - s1_stack[0], axis=1))),
    )
    mean0 = np.mean(s0_stack, axis=0)
    mean1 = np.mean(s1_stack, axis=0)
    span = max(
        float(np.max(np.linalg.norm(s0_stack - mean0[None, :, :], axis=2))),
        float(np.max(np.linalg.norm(s1_stack - mean1[None, :, :], axis=2))),
    )
    return avg_residual, drift, span


def warp_with_conic(dual_tilt, tilted_ref, conic_ref: ConicReference,
                    surface, cfg: dict) -> WarpResult:
    """Dual-Arc SPGA with active crossbar contacts and force-balance stopping.

    Method modules remain the same three-way soft balance:
      1) geometry repulsion,
      2) reference-relative dual-rail ribbon elasticity,
      3) soft conic pose preference.

    This revision changes the *mechanics/solver* of the geometry representation:
      - local crossbar geometry regions become active contacts instead of a
        quadrature-averaged body force;
      - each active contact is transferred to A0/A1 by exact interpolation
        Jacobian weights (1-tau, tau);
      - the combined generalized velocity/force is projected onto the tangent
        space of ||A1-A0||=ell BEFORE integration;
      - convergence is based on the allowed generalized-force residual, not a
        small state step caused by step clipping.

    A final fixed-length retraction is still used after each finite Euler step,
    but only to remove second-order numerical drift.  It is not used to repair
    an unconstrained free-endpoint update.
    """
    a0 = np.asarray(dual_tilt.arc0, float).copy()
    a1 = np.asarray(dual_tilt.arc1, float).copy()
    r0 = a0.copy(); r1 = a1.copy()
    u_ref = np.asarray(dual_tilt.tool_axis, float).copy()
    fixed = np.asarray(dual_tilt.contact_mask_arc0, bool)
    primitive_ids = np.asarray(tilted_ref.trajectory.pass_ids)
    ell = float(dual_tilt.ell)

    anchors = np.asarray(tilted_ref.contact_anchors, float)
    if np.any(fixed & ~np.all(np.isfinite(anchors), axis=1)):
        raise ValueError("fixed/contact anchors must be finite")
    if np.any(fixed):
        a0[fixed] = anchors[fixed]

    rcfg = cfg["ribbon"]
    scfg = cfg["solver"]
    mobcfg = cfg["mobility"]
    mob0 = arc0_geometry_mobility(
        r0, primitive_ids, fixed,
        mobcfg["contact_exclusion_radius_m"],
        mobcfg["contact_transition_radius_m"],
    )
    mob1 = np.ones(len(a1), dtype=float)

    dt = float(scfg["dtau"])
    max_iter = int(scfg["max_iterations"])
    min_iter = int(scfg.get("minimum_iterations", 20))
    # Backward-compatible fallback: old state tolerance / dt corresponds to an
    # allowed generalized velocity residual with the same first-order scale.
    force_tol = float(scfg.get(
        "force_residual_tolerance",
        float(scfg.get("state_step_tolerance_m", 3e-6)) / max(dt, 1e-12),
    ))

    # Small numerical stopping extension for nonsmooth active-contact switching.
    # It changes only the convergence detector, never geometry/ribbon/cone forces.
    ocfg = scfg.get("oscillatory_balance", {})
    oscillatory_enabled = bool(ocfg.get("enabled", True))
    oscillatory_window = max(4, int(ocfg.get("window_iterations", 60)))
    oscillatory_min_iter = max(min_iter, int(ocfg.get("minimum_iterations", 200)))
    avg_force_tol = float(ocfg.get("average_force_tolerance", 4.0 * force_tol))
    state_drift_tol = float(ocfg.get("state_drift_tolerance_m", 5e-4))
    state_span_tol = float(ocfg.get("state_span_tolerance_m", 1e-3))
    force_window = deque(maxlen=oscillatory_window)
    state_window = deque(maxlen=oscillatory_window)

    history = []
    converged = False
    convergence_mode = "max_iterations"
    final_avg_residual = float("inf")
    final_window_drift = float("inf")
    final_window_span = float("inf")
    max_retraction_error = 0.0

    # The old adaptive escape multiplier is deliberately not part of this
    # formulation. Geometry authority is determined by the geometry field and
    # active-contact mechanics, not by an escalation gain.
    adaptive_escape_enabled = False
    escape_target = float(cfg.get("geometry_field", {}).get("adaptive_escape", {}).get(
        "target_penetration_m", 1e-4
    ))

    for it in range(max_iter):
        # Geometry interaction on the represented rigid crossbar.  The exact
        # fixed A0 contact is exempt, but every other point including A1 can
        # form an active local geometry contact.
        f0, f1, geo_contact = crossbar_active_contact_force(
            a0, a1, surface, cfg,
            mobility0=mob0, mobility1=mob1,
            distance_offset=0.0, contact_mask=fixed,
        )

        # Endpoint queries are diagnostics only; endpoint loads are already
        # included in the unified active-contact crossbar calculation.
        _, geo0 = geometry_force(a0, surface, cfg, mobility=mob0, distance_offset=0.0)
        _, geo1 = geometry_force(a1, surface, cfg, mobility=mob1, distance_offset=0.0)

        # Reference-relative elastic ribbon on both rails.
        g0, g1, erib = ribbon_energy_gradient(
            a0, a1, r0, r1, primitive_ids, fixed,
            rcfg["tension_gain"], rcfg["bending_gain"],
            rcfg.get("arc0_weight", 1.0), rcfg.get("arc1_weight", 1.0),
        )

        # Soft conic pose preference.
        v0c, v1c, theta, _, _ = conic_velocity(a0, a1, conic_ref, ell, cfg)

        # Unconstrained generalized force / pseudo-velocity balance.
        v0 = f0 - g0 + float(rcfg.get("cone_gain_A0", 0.0)) * v0c
        v1 = f1 - g1 + float(rcfg.get("cone_gain_A1", 1.0)) * v1c
        if np.any(fixed):
            v0[fixed] = 0.0

        # Enforce fixed tool length at the velocity/force level.  The projected
        # residual is the physically admissible imbalance on the manifold.
        v0_allowed, v1_allowed = _project_fixed_length_tangent(
            v0, v1, a0, a1, fixed
        )
        residual = max(
            float(np.max(np.linalg.norm(v0_allowed, axis=1))),
            float(np.max(np.linalg.norm(v1_allowed, axis=1))),
        )

        # Keep the full projected force VECTOR and state for a short window.
        # This lets alternating active-contact forces cancel in the average while
        # requiring the state itself to remain bounded (no hidden drift).
        force_window.append((v0_allowed.copy(), v1_allowed.copy()))
        state_window.append((a0.copy(), a1.copy()))
        avg_residual = float("inf")
        window_drift = float("inf")
        window_span = float("inf")
        if len(force_window) == oscillatory_window:
            avg_residual, window_drift, window_span = _windowed_balance_metrics(
                force_window, state_window
            )
            final_avg_residual = float(avg_residual)
            final_window_drift = float(window_drift)
            final_window_span = float(window_span)

        smooth_balance = (it >= min_iter and residual < force_tol)
        oscillatory_balance = (
            oscillatory_enabled
            and it >= oscillatory_min_iter
            and len(force_window) == oscillatory_window
            and avg_residual < avg_force_tol
            and window_drift < state_drift_tol
            and window_span < state_span_tol
        )

        if smooth_balance or oscillatory_balance:
            converged = True
            convergence_mode = (
                "instantaneous_force_balance" if smooth_balance
                else "bounded_oscillatory_force_balance"
            )
            history.append({
                "iter": int(it),
                "pseudo_dt": float(dt),
                "allowed_force_residual": float(residual),
                "window_average_force_residual": float(avg_residual),
                "window_state_drift_m": float(window_drift),
                "window_state_span_m": float(window_span),
                "raw_max_velocity": float(residual),
                "max_state_step_m": float(dt * residual),
                "pre_retraction_max_length_error_m": 0.0,
                "min_phi_A0_m": float(np.min(geo0["min_phi"])),
                "min_phi_A1_m": float(np.min(geo1["min_phi"])),
                "min_phi_tool_body_m": float(geo_contact["min_phi_global"]),
                "min_phi_full_tool_m": float(geo_contact["min_phi_global"]),
                "mean_geometry_force_A0": float(np.mean(np.linalg.norm(f0, axis=1))),
                "mean_geometry_force_A1": float(np.mean(np.linalg.norm(f1, axis=1))),
                "active_contact_regions": int(geo_contact["total_active_regions"]),
                "active_contact_sample_count": int(geo_contact["sample_count"]),
                "max_geometry_barrier": float(geo_contact["max_barrier_magnitude"]),
                "active_max_penetration_m": float(max(-geo_contact["min_phi_global"], 0.0)),
                "collision_escape_multiplier": 1.0,
                "ribbon_energy": float(erib["total"]),
                "mean_cone_angle_deg": float(np.degrees(np.mean(theta))),
                "max_cone_angle_deg": float(np.degrees(np.max(theta))),
            })
            break

        prev0 = a0.copy(); prev1 = a1.copy()
        a0 += dt * v0_allowed
        a1 += dt * v1_allowed
        if np.any(fixed):
            a0[fixed] = anchors[fixed]

        # Retraction to the exact fixed-length manifold. Since the update was
        # already tangent, this only corrects O(dt^2) drift.
        d = a1 - a0
        lengths = np.linalg.norm(d, axis=1)
        pre_retract_err = float(np.max(np.abs(lengths - ell)))
        max_retraction_error = max(max_retraction_error, pre_retract_err)
        bad = lengths < 1e-12
        d[bad] = u_ref[bad]
        a1 = a0 + ell * _norm_rows(d)

        state_step = max(
            float(np.max(np.linalg.norm(a0-prev0, axis=1))),
            float(np.max(np.linalg.norm(a1-prev1, axis=1))),
        )
        active_max_penetration = float(max(-geo_contact["min_phi_global"], 0.0))
        history.append({
            "iter": int(it),
            "pseudo_dt": float(dt),
            "allowed_force_residual": float(residual),
            "window_average_force_residual": float(avg_residual),
            "window_state_drift_m": float(window_drift),
            "window_state_span_m": float(window_span),
            # Backward-compatible name used by reporting code.
            "raw_max_velocity": float(residual),
            "max_state_step_m": float(state_step),
            "pre_retraction_max_length_error_m": float(pre_retract_err),
            "min_phi_A0_m": float(np.min(geo0["min_phi"])),
            "min_phi_A1_m": float(np.min(geo1["min_phi"])),
            "min_phi_tool_body_m": float(geo_contact["min_phi_global"]),
            "min_phi_full_tool_m": float(geo_contact["min_phi_global"]),
            "mean_geometry_force_A0": float(np.mean(np.linalg.norm(f0, axis=1))),
            "mean_geometry_force_A1": float(np.mean(np.linalg.norm(f1, axis=1))),
            "active_contact_regions": int(geo_contact["total_active_regions"]),
            "active_contact_sample_count": int(geo_contact["sample_count"]),
            "max_geometry_barrier": float(geo_contact["max_barrier_magnitude"]),
            "active_max_penetration_m": float(active_max_penetration),
            "collision_escape_multiplier": 1.0,
            "ribbon_energy": float(erib["total"]),
            "mean_cone_angle_deg": float(np.degrees(np.mean(theta))),
            "max_cone_angle_deg": float(np.degrees(np.max(theta))),
        })

    # Recompute final quantities at the final state, including the actual
    # projected force residual rather than reusing the previous iteration.
    f0, f1, geo_contact = crossbar_active_contact_force(
        a0, a1, surface, cfg, mobility0=mob0, mobility1=mob1,
        distance_offset=0.0, contact_mask=fixed,
    )
    _, geo0 = geometry_force(a0, surface, cfg, mobility=mob0, distance_offset=0.0)
    _, geo1 = geometry_force(a1, surface, cfg, mobility=mob1, distance_offset=0.0)
    g0, g1, erib = ribbon_energy_gradient(
        a0, a1, r0, r1, primitive_ids, fixed,
        rcfg["tension_gain"], rcfg["bending_gain"],
        rcfg.get("arc0_weight", 1.0), rcfg.get("arc1_weight", 1.0),
    )
    v0c, v1c, theta, _, _ = conic_velocity(a0, a1, conic_ref, ell, cfg)
    v0 = f0 - g0 + float(rcfg.get("cone_gain_A0", 0.0)) * v0c
    v1 = f1 - g1 + float(rcfg.get("cone_gain_A1", 1.0)) * v1c
    if np.any(fixed):
        v0[fixed] = 0.0
    v0_allowed, v1_allowed = _project_fixed_length_tangent(v0, v1, a0, a1, fixed)
    final_residual = max(
        float(np.max(np.linalg.norm(v0_allowed, axis=1))),
        float(np.max(np.linalg.norm(v1_allowed, axis=1))),
    )
    if final_residual < force_tol and len(history) >= min_iter:
        converged = True
        if convergence_mode == "max_iterations":
            convergence_mode = "instantaneous_force_balance"

    # Re-evaluate the final window metrics for diagnostics.
    if len(force_window) == oscillatory_window:
        final_avg_residual, final_window_drift, final_window_span = _windowed_balance_metrics(
            force_window, state_window
        )

    u_warp = _norm_rows(a1-a0)
    R_ref = np.asarray(tilted_ref.trajectory.rotations, float)
    R_warp = np.empty_like(R_ref)
    for i in range(len(R_warp)):
        R_warp[i] = min_rotation(u_ref[i], u_warp[i]) @ R_ref[i]

    fixed_error = (float(np.max(np.linalg.norm(a0[fixed]-anchors[fixed], axis=1)))
                   if np.any(fixed) else 0.0)
    free = ~fixed
    free_disp = np.linalg.norm(a0[free]-r0[free], axis=1) if np.any(free) else np.zeros(0)
    final_max_pen = float(max(-geo_contact["min_phi_global"], 0.0))
    final_state_step = float(history[-1]["max_state_step_m"]) if history else 0.0

    m3_enabled=bool(cfg.get('geometry_field',{}).get('multi_normal',{}).get('enabled',False))
    diagnostics = {
        "iterations": int(len(history)),
        "converged": bool(converged),
        "convergence_mode": str(convergence_mode),
        "force_residual_tolerance": float(force_tol),
        "final_allowed_force_residual": float(final_residual),
        "final_window_average_force_residual": float(final_avg_residual),
        "final_window_state_drift_m": float(final_window_drift),
        "final_window_state_span_m": float(final_window_span),
        "oscillatory_balance_enabled": bool(oscillatory_enabled),
        "oscillatory_balance_window": int(oscillatory_window),
        "oscillatory_average_force_tolerance": float(avg_force_tol),
        "oscillatory_state_drift_tolerance_m": float(state_drift_tol),
        "oscillatory_state_span_tolerance_m": float(state_span_tol),
        # Backward-compatible key used by existing reporting code.
        "final_raw_max_velocity": float(final_residual),
        "final_max_state_step_m": float(final_state_step),
        "adaptive_step_count": 0,
        "minimum_pseudo_dt": float(dt),
        "adaptive_escape_enabled": bool(adaptive_escape_enabled),
        "adaptive_escape_target_m": float(escape_target),
        "adaptive_escape_soft_depth_m": 0.0,
        "collision_escape_final_multiplier": 1.0,
        "collision_escape_updates": [],
        "collision_target_satisfied": bool(final_max_pen <= escape_target),
        "escape_gain_saturated": False,
        "fixed_A0_max_error_m": fixed_error,
        "free_A0_max_deformation_m": float(np.max(free_disp)) if len(free_disp) else 0.0,
        "final_phi_A0": np.asarray(geo0["min_phi"]),
        "final_phi_A1": np.asarray(geo1["min_phi"]),
        "final_surface_distance_A1": np.asarray(geo1["surface_distance"]),
        "final_phi_tool_body": np.asarray(geo_contact["min_phi_per_pose"]),
        "final_surface_distance_tool_body": np.asarray(geo_contact["min_surface_distance_per_pose"]),
        "final_min_phi_full_tool_m": float(geo_contact["min_phi_global"]),
        "active_contact_regions_final": int(geo_contact["total_active_regions"]),
        "active_contact_sample_count": int(geo_contact["sample_count"]),
        "active_contact_max_spacing_m": float(geo_contact["sample_max_spacing_m"]),
        "max_pre_retraction_length_error_m": float(max_retraction_error),
        "ribbon_energy": erib,
        "shape_energy_A0": {
            "memory_energy": 0.0,
            "tension_energy": float(erib["arc0_tension"]),
            "bending_energy": float(erib["arc0_bending"]),
            "weighted_memory_energy": 0.0,
            "weighted_tension_energy": float(erib["arc0_tension"]),
            "weighted_bending_energy": float(erib["arc0_bending"]),
            "weighted_total_shape_energy": float(erib["arc0_tension"] + erib["arc0_bending"]),
        },
        "method_parameters": {
            "architecture": ("M3 multi-normal distributed soft crossbar + dual-rail ribbon + soft cone + fixed-length tangent projection" if m3_enabled else "active crossbar contacts + dual-rail ribbon + soft cone + fixed-length tangent projection"),
            "crossbar_geometry_mode": ("M3 distributed smooth multi-normal quadrature" if m3_enabled else "one deepest active contact per connected local geometry region"),
            "crossbar_force_transfer": ("M3 Jacobian-transferred smooth bar quadrature" if m3_enabled else "Jacobian transpose: ((1-tau)f, tau f), no quadrature d_tau dilution"),
            "fixed_length_update": "project combined generalized force to tangent space, then finite-step retraction",
            "convergence": "instantaneous projected-force residual OR bounded oscillatory averaged-force balance",
            "soft_margin_m": float(cfg["geometry_field"]["soft_margin_m"]),
            "active_contact_distance_m": float(cfg["geometry_field"].get(
                "active_contact_distance_m", cfg["geometry_field"]["soft_margin_m"]
            )),
            "active_contact_max_spacing_m": float(cfg["geometry_field"].get("active_contact_max_spacing_m", 0.01)),
            "geometry_clearance_gain": float(cfg["geometry_field"]["clearance_gain"]),
            "geometry_barrier_gain": float(cfg["geometry_field"]["barrier_gain"]),
            "geometry_penetration_gain": float(cfg["geometry_field"]["penetration_gain"]),
            "crossbar_samples_min": int(cfg["geometry_field"]["crossbar_samples"]),
            "ribbon_tension_gain": float(rcfg["tension_gain"]),
            "ribbon_bending_gain": float(rcfg["bending_gain"]),
            "cone_gain_A1": float(rcfg.get("cone_gain_A1", 1.0)),
            "cone_half_angle_deg": float(cfg["conic"]["half_angle_deg"]),
            "dtau": float(dt),
            "force_residual_tolerance": float(force_tol),
        },
        "history": history,
    }
    return WarpResult(
        positions=a0.copy(), rotations=R_warp,
        arc0=a0.copy(), arc1=a1.copy(),
        initial_arc0=r0.copy(), initial_arc1=r1.copy(),
        diagnostics=diagnostics,
    )