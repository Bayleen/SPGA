from __future__ import annotations
import numpy as np


def _as_surfaces(surface_or_surfaces):
    if isinstance(surface_or_surfaces, (list, tuple)):
        return list(surface_or_surfaces)
    return [surface_or_surfaces]


def geometry_magnitude_components(phi, preferred_margin, influence_distance,
                                  clearance_gain, barrier_distance,
                                  barrier_gain, barrier_cap,
                                  penetration_gain):
    """Three-layer soft geometry response.

    The preferred margin is deliberately *not* a hard clearance. Loads from
    several surfaces are summed and may balance inside the nominal margin.

    Components
    ----------
    margin
        Gentle repulsion that fades to zero at ``preferred_margin``.
    barrier
        Rapidly increasing repulsion close to the collision boundary.  It is
        still a force, not a projection/constraint, and is capped for numerical
        stability.  This prevents the ribbon/cone equilibrium from sitting
        arbitrarily close to phi=0 when free space is available.
    penetration
        Additional quadratic load when phi < 0.
    """
    phi = np.asarray(phi, dtype=float)
    m = float(preferred_margin)
    s = max(float(influence_distance), 1e-9)

    # 1) Soft preferred margin.
    deficit = np.maximum(m - phi, 0.0)
    z = deficit / s
    smooth = np.where(z < 1.0, z*z*(3.0 - 2.0*z), 1.0 + (z - 1.0))
    margin = float(clearance_gain) * smooth

    # 2) Near-collision barrier.  It is zero at d_b and rises as phi -> 0.
    # For penetration we keep the barrier saturated and let the penetration
    # component add the stronger depth-dependent response.
    db = max(float(barrier_distance), 1e-9)
    eps = max(0.05 * db, 1e-6)
    safe_phi = np.maximum(phi, eps)
    barrier_raw = float(barrier_gain) * np.maximum((db / safe_phi)**2 - 1.0, 0.0)
    barrier = np.where(phi < db, barrier_raw, 0.0)
    barrier = np.minimum(barrier, float(barrier_cap))

    # 3) Penetration load.
    penetration_depth = np.maximum(-phi, 0.0)
    penetration = float(penetration_gain) * (penetration_depth / s) ** 2

    total = margin + barrier + penetration
    return {
        "margin": margin,
        "barrier": barrier,
        "penetration": penetration,
        "total": total,
    }


def _m3_source_field(surface, gc):
    """Create mesh quadrature once; leave M2's query_batch untouched."""
    field=getattr(surface, '_m3_field', None)
    if field is not None:return field
    from .multi_normal_field import MultiNormalField
    if not (hasattr(surface,'vertices') and hasattr(surface,'faces')):
        raise TypeError('M3 requires mesh surface vertices and oriented faces')
    face_normals=getattr(surface,'_full_face_normals',None)
    vertex_normals=getattr(surface,'_full_vertex_normals',None)
    field=MultiNormalField(surface.vertices,surface.faces,
        vertex_normals=vertex_normals,face_normals=face_normals,
        cfg=gc['multi_normal'])
    surface._m3_field=field
    return field


def _m3_geometry_force(points, surface_or_surfaces, cfg, mobility, distance_offset):
    """Sum all visible normal sources, preserving original M2 phi diagnostics."""
    P=np.asarray(points,float); gc=cfg['geometry_field']
    surfaces=_as_surfaces(surface_or_surfaces)
    force=np.zeros_like(P); min_phi=np.full(len(P),np.inf)
    total_source_count=np.zeros(len(P),int)
    total_source_magnitude=np.zeros(len(P))
    for surface in surfaces:
        phi,_,_,_=surface.query_batch(P,float(gc['sigma_n_m']),float(gc['normal_gate_width_m']))
        min_phi=np.minimum(min_phi,np.asarray(phi,float)-distance_offset)
        ff,diag=_m3_source_field(surface,gc).evaluate(P)
        force+=ff
        total_source_count+=diag['source_count']
        total_source_magnitude+=diag['source_sum_magnitude']
    if mobility is not None:
        M=np.asarray(mobility,float).reshape(-1)
        force*=M[:,None]
        total_source_magnitude*=M
    mag=np.linalg.norm(force,axis=1)
    # No discontinuous M2 barrier in this branch. These are diagnostic fields.
    return force,dict(min_phi=min_phi,surface_distance=min_phi+distance_offset,
        distance_offset=float(distance_offset),force_magnitude=mag,
        margin_magnitude=mag,barrier_magnitude=np.zeros(len(P)),
        penetration_magnitude=np.zeros(len(P)),
        m3_source_count=total_source_count,m3_sum_source_magnitude=total_source_magnitude)


def geometry_force(points, surface_or_surfaces, cfg: dict, mobility=None, distance_offset=0.0):
    """Sum task-agnostic geometry loads at arbitrary 3-D carrier points.

    Multiple surfaces are additive by construction. The soft margin remains a
    preference: opposing nearby surfaces can naturally reach a force balance.
    """
    P = np.asarray(points, float)
    if P.ndim != 2 or P.shape[1] != 3:
        raise ValueError("points must have shape (N,3)")
    gc = cfg["geometry_field"]
    if gc.get("multi_normal",{}).get("enabled",False):
        return _m3_geometry_force(P,surface_or_surfaces,cfg,mobility,distance_offset)
    surfaces = _as_surfaces(surface_or_surfaces)
    total = np.zeros_like(P)
    min_phi = np.full(len(P), np.inf, dtype=float)
    min_surface_phi = np.full(len(P), np.inf, dtype=float)
    comp = {
        "margin": np.zeros(len(P), float),
        "barrier": np.zeros(len(P), float),
        "penetration": np.zeros(len(P), float),
    }

    for surface in surfaces:
        phi_surface, q, nnear, nbar = surface.query_batch(
            P,
            float(gc["sigma_n_m"]),
            float(gc["normal_gate_width_m"]),
        )
        # Current method evaluates the represented centerline signed distance.
        # distance_offset is retained only for backward API compatibility and is
        # expected to be zero in the current solver.
        phi = phi_surface - float(distance_offset)
        c = geometry_magnitude_components(
            phi,
            gc["soft_margin_m"],
            gc["influence_distance_m"],
            gc["clearance_gain"],
            gc["barrier_distance_m"],
            gc["barrier_gain"],
            gc["barrier_cap"],
            gc["penetration_gain"],
        )
        total += c["total"][:, None] * nbar
        min_phi = np.minimum(min_phi, phi)
        min_surface_phi = np.minimum(min_surface_phi, phi_surface)
        for key in comp:
            comp[key] += c[key]

    if mobility is not None:
        M = np.asarray(mobility, float).reshape(-1)
        if len(M) != len(P):
            raise ValueError("mobility length mismatch")
        total *= M[:, None]
        for key in comp:
            comp[key] *= M

    return total, {
        "min_phi": min_phi,
        "surface_distance": min_surface_phi,
        "distance_offset": float(distance_offset),
        "force_magnitude": np.linalg.norm(total, axis=1),
        "margin_magnitude": comp["margin"],
        "barrier_magnitude": comp["barrier"],
        "penetration_magnitude": comp["penetration"],
    }


def crossbar_geometry_force(a0, a1, surface_or_surfaces, cfg: dict,
                            mobility0=None, mobility1=None, distance_offset=0.0,
                            contact_mask=None, crossbar_length=None,
                            contact_exclusion_length=0.0):
    """Finite-tool geometry load on the interior of every A0--A1 crossbar.

    The two endpoints are intentionally excluded here because they are already
    handled by ``geometry_force(A0)`` and ``geometry_force(A1)``.  Interior
    samples are queried in 3-D and each sample load is distributed back to the
    two carriers using the crossbar interpolation weights:

        x(tau) = (1-tau) A0 + tau A1
        F_A0  += (1-tau) F(x) d_tau
        F_A1  += tau     F(x) d_tau

    This lets tool-body collision rotate/translate the dual arc while keeping
    the method representation at only two carriers.  Because all surface loads
    are soft/additive, several nearby surfaces can still push simultaneously.
    """
    A0 = np.asarray(a0, float)
    A1 = np.asarray(a1, float)
    if A0.shape != A1.shape or A0.ndim != 2 or A0.shape[1] != 3:
        raise ValueError("a0/a1 must have matching shape (N,3)")

    gc = cfg["geometry_field"]
    ns = int(gc.get("crossbar_samples", 9))
    if ns < 3:
        raise ValueError("crossbar_samples must be >= 3")

    # Endpoints are evaluated separately, so use only interior samples.
    tau = np.linspace(0.0, 1.0, ns)[1:-1]
    N = len(A0)
    T = len(tau)
    P = ((1.0 - tau[None, :, None]) * A0[:, None, :]
         + tau[None, :, None] * A1[:, None, :])

    Fflat, dflat = geometry_force(
        P.reshape(-1, 3), surface_or_surfaces, cfg,
        distance_offset=distance_offset,
    )
    F = Fflat.reshape(N, T, 3)
    phi = np.asarray(dflat["min_phi"]).reshape(N, T)
    surface_phi = np.asarray(dflat["surface_distance"]).reshape(N, T)

    # Current SPGA semantics: only the exact intended A0 contact point is
    # exempt from unwanted-collision evaluation.  This function samples only
    # interior crossbar points, so every interior sample remains geometry-active.
    active = np.ones((N, T), dtype=bool)
    margin = np.asarray(dflat["margin_magnitude"]).reshape(N, T)
    barrier = np.asarray(dflat["barrier_magnitude"]).reshape(N, T)
    penetration = np.asarray(dflat["penetration_magnitude"]).reshape(N, T)

    # Arc-length quadrature in normalized crossbar coordinate.  This makes the
    # result stable when crossbar_samples changes instead of scaling with count.
    d_tau = 1.0 / float(ns - 1)
    w0 = (1.0 - tau)[None, :, None]
    w1 = tau[None, :, None]
    f0 = d_tau * np.sum(w0 * F, axis=1)
    f1 = d_tau * np.sum(w1 * F, axis=1)

    if mobility0 is not None:
        M0 = np.asarray(mobility0, float).reshape(-1)
        if len(M0) != N:
            raise ValueError("mobility0 length mismatch")
        f0 *= M0[:, None]
    if mobility1 is not None:
        M1 = np.asarray(mobility1, float).reshape(-1)
        if len(M1) != N:
            raise ValueError("mobility1 length mismatch")
        f1 *= M1[:, None]

    # Per-pose body diagnostics; endpoints are handled by the caller.
    active_phi = np.where(active, phi, np.inf)
    active_surface_phi = np.where(active, surface_phi, np.inf)
    return f0, f1, {
        "tau": tau.copy(),
        "sample_phi": phi,
        "sample_surface_distance": surface_phi,
        "active_sample_mask": active,
        "min_phi_per_pose": np.min(active_phi, axis=1),
        "min_phi_global": float(np.min(active_phi)),
        "min_surface_distance_per_pose": np.min(active_surface_phi, axis=1),
        "min_surface_distance_global": float(np.min(active_surface_phi)),
        "distance_offset": float(distance_offset),
        "mean_force_A0": float(np.mean(np.linalg.norm(f0, axis=1))),
        "mean_force_A1": float(np.mean(np.linalg.norm(f1, axis=1))),
        "max_margin_magnitude": float(np.max(margin)),
        "max_barrier_magnitude": float(np.max(barrier)),
        "max_penetration_magnitude": float(np.max(penetration)),
    }


def crossbar_active_contact_force(a0, a1, surface_or_surfaces, cfg: dict,
                                  mobility0=None, mobility1=None,
                                  distance_offset=0.0, contact_mask=None):
    """Active-contact geometry load on a fixed-length dual-arc crossbar.

    Unlike ``crossbar_geometry_force``, this does not integrate/average the
    geometry field over all quadrature points.  Instead it samples the full
    represented crossbar (including endpoints), finds each connected region
    that lies inside the configured geometry activation distance, and keeps the
    deepest sample of each region as one active geometry contact.

    Each active contact force is transferred to the two dual-arc carriers by
    the exact interpolation/Jacobian-transpose weights

        x(tau) = (1-tau) A0 + tau A1
        F_A0  += (1-tau) f(x)
        F_A1  += tau     f(x)

    There is deliberately no d_tau quadrature factor: one physical/local
    contact region contributes one contact load independent of sample count.
    Sample density only improves localization of the active contact.

    For intended contact poses, the exact A0 point (tau=0) is exempt from
    unwanted-collision response, while the rest of the crossbar remains active.
    """
    A0 = np.asarray(a0, float)
    A1 = np.asarray(a1, float)
    if A0.shape != A1.shape or A0.ndim != 2 or A0.shape[1] != 3:
        raise ValueError("a0/a1 must have matching shape (N,3)")

    gc = cfg["geometry_field"]
    lengths = np.linalg.norm(A1 - A0, axis=1)
    max_len = float(np.max(lengths)) if len(lengths) else 0.0
    base_ns = max(int(gc.get("crossbar_samples", 9)), 3)
    max_spacing = float(gc.get("active_contact_max_spacing_m", 0.01))
    if max_spacing > 0.0 and max_len > 0.0:
        spacing_ns = int(np.ceil(max_len / max_spacing)) + 1
    else:
        spacing_ns = base_ns
    ns = max(base_ns, spacing_ns)
    tau = np.linspace(0.0, 1.0, ns)

    N = len(A0)
    P = ((1.0 - tau[None, :, None]) * A0[:, None, :]
         + tau[None, :, None] * A1[:, None, :])
    Fflat, dflat = geometry_force(
        P.reshape(-1, 3), surface_or_surfaces, cfg,
        distance_offset=distance_offset,
    )
    F = Fflat.reshape(N, ns, 3)
    phi = np.asarray(dflat["min_phi"], float).reshape(N, ns)
    surface_phi = np.asarray(dflat["surface_distance"], float).reshape(N, ns)
    margin = np.asarray(dflat["margin_magnitude"], float).reshape(N, ns)
    barrier = np.asarray(dflat["barrier_magnitude"], float).reshape(N, ns)
    penetration = np.asarray(dflat["penetration_magnitude"], float).reshape(N, ns)

    contact = (np.zeros(N, dtype=bool) if contact_mask is None
               else np.asarray(contact_mask, bool).reshape(-1))
    if len(contact) != N:
        raise ValueError("contact_mask length mismatch")

    # Keep the original smooth geometry semantics: a region becomes active as
    # soon as it enters the soft-margin band.  This avoids an on/off jump at
    # phi=0 while still representing each local region by one contact instead
    # of a quadrature average.
    active_distance = float(gc.get("active_contact_distance_m", gc["soft_margin_m"]))
    candidate = phi < active_distance
    candidate[contact, 0] = False  # exact intended A0 contact is exempt

    if gc.get('multi_normal',{}).get('enabled',False):
        return _m3_soft_tool_transfer(A0,A1,tau,F,phi,surface_phi,contact,
            candidate,margin,barrier,penetration,mobility0,mobility1,active_distance,
            distance_offset)

    selected = np.zeros_like(candidate, dtype=bool)
    f0 = np.zeros_like(A0)
    f1 = np.zeros_like(A1)
    region_count = np.zeros(N, dtype=int)
    selected_tau = [[] for _ in range(N)]

    for i in range(N):
        idx = np.flatnonzero(candidate[i])
        if idx.size == 0:
            continue
        # Split into connected sampled regions.
        cuts = np.flatnonzero(np.diff(idx) > 1)
        starts = np.r_[0, cuts + 1]
        ends = np.r_[cuts + 1, idx.size]
        for s, e in zip(starts, ends):
            reg = idx[s:e]
            if reg.size == 0:
                continue
            # One active contact per local geometry region: choose its deepest
            # signed-distance sample, not an average over region width.
            j = int(reg[np.argmin(phi[i, reg])])
            selected[i, j] = True
            region_count[i] += 1
            selected_tau[i].append(float(tau[j]))
            fj = F[i, j]
            f0[i] += (1.0 - tau[j]) * fj
            f1[i] += tau[j] * fj

    if mobility0 is not None:
        M0 = np.asarray(mobility0, float).reshape(-1)
        if len(M0) != N:
            raise ValueError("mobility0 length mismatch")
        f0 *= M0[:, None]
    if mobility1 is not None:
        M1 = np.asarray(mobility1, float).reshape(-1)
        if len(M1) != N:
            raise ValueError("mobility1 length mismatch")
        f1 *= M1[:, None]

    # Diagnostics use all sampled points, while active loads use only selected
    # local contacts.  The exact fixed A0 sample stays exempt in per-pose minima.
    eval_mask = np.ones_like(phi, dtype=bool)
    eval_mask[contact, 0] = False
    active_phi = np.where(eval_mask, phi, np.inf)
    active_surface_phi = np.where(eval_mask, surface_phi, np.inf)
    selected_force_mag = np.linalg.norm(np.where(selected[:, :, None], F, 0.0), axis=2)

    return f0, f1, {
        "tau": tau.copy(),
        "sample_phi": phi,
        "sample_surface_distance": surface_phi,
        "candidate_mask": candidate,
        "selected_contact_mask": selected,
        "selected_tau_per_pose": selected_tau,
        "active_region_count_per_pose": region_count,
        "total_active_regions": int(np.sum(region_count)),
        "active_contact_distance_m": float(active_distance),
        "sample_count": int(ns),
        "sample_max_spacing_m": float(max_len / max(ns - 1, 1)),
        "min_phi_per_pose": np.min(active_phi, axis=1),
        "min_phi_global": float(np.min(active_phi)),
        "min_surface_distance_per_pose": np.min(active_surface_phi, axis=1),
        "min_surface_distance_global": float(np.min(active_surface_phi)),
        "distance_offset": float(distance_offset),
        "mean_force_A0": float(np.mean(np.linalg.norm(f0, axis=1))),
        "mean_force_A1": float(np.mean(np.linalg.norm(f1, axis=1))),
        "max_selected_contact_force": float(np.max(selected_force_mag)) if selected_force_mag.size else 0.0,
        "max_margin_magnitude": float(np.max(margin)),
        "max_barrier_magnitude": float(np.max(barrier)),
        "max_penetration_magnitude": float(np.max(penetration)),
    }


def _m3_soft_tool_transfer(A0,A1,tau,F,phi,surface_phi,contact,
                           candidate,margin,barrier,penetration,mobility0,
                           mobility1,active_distance,distance_offset):
    """M3 distributed soft contact: integrate EVERY point's continuous load.

    The M2 deepest-contact winner would suppress simultaneous weak contributions
    and introduces discontinuities when the winning point changes. For M3 only,
    integrate the vector force (already summed over all visible surface normals)
    over the finite bar, and Jacobian-transfer each contribution. The integral
    is normalized over tau so doubling tool samples does not double authority.
    Exclude only exact prescribed A0 contact points; retain all other samples.
    """
    N,ns,_=F.shape
    loads=F.copy()
    loads[contact,0,:]=0.0
    # Trapezoidal 0..1 quadrature; smooth as tool moves, independent of sampling count.
    weights=np.full(ns,1.0/(ns-1));weights[0]*=0.5;weights[-1]*=0.5
    f0=np.sum(weights[None,:,None]*(1-tau[None,:,None])*loads,axis=1)
    f1=np.sum(weights[None,:,None]*tau[None,:,None]*loads,axis=1)
    if mobility0 is not None:f0*=np.asarray(mobility0,float)[:,None]
    if mobility1 is not None:f1*=np.asarray(mobility1,float)[:,None]
    selected=np.linalg.norm(loads,axis=2)>1e-12
    count=np.array([(int(1+np.count_nonzero(np.diff(np.flatnonzero(row))>1))
                     if np.any(row) else 0) for row in selected],dtype=int)
    eval_mask=np.ones_like(phi,bool);eval_mask[contact,0]=False
    eval_phi=np.where(eval_mask,phi,np.inf)
    selected_tau=[tau[selected[i]].tolist() for i in range(N)]
    return f0,f1,dict(tau=tau.copy(),sample_phi=phi,sample_surface_distance=surface_phi,
       candidate_mask=selected,selected_contact_mask=selected,
       selected_tau_per_pose=selected_tau,active_region_count_per_pose=count,
       total_active_regions=int(np.sum(count)),
       active_contact_distance_m=float(active_distance),sample_count=int(ns),
       sample_max_spacing_m=float(np.max(np.linalg.norm(A1-A0,axis=1))/max(ns-1,1)),
       min_phi_per_pose=np.min(eval_phi,axis=1),min_phi_global=float(np.min(eval_phi)),
       min_surface_distance_per_pose=np.min(np.where(eval_mask,surface_phi,np.inf),axis=1),
       min_surface_distance_global=float(np.min(np.where(eval_mask,surface_phi,np.inf))),
       distance_offset=float(distance_offset),
       mean_force_A0=float(np.mean(np.linalg.norm(f0,axis=1))),
       mean_force_A1=float(np.mean(np.linalg.norm(f1,axis=1))),
       max_selected_contact_force=float(np.max(np.linalg.norm(loads,axis=2))) if loads.size else 0.,
       max_margin_magnitude=float(np.max(margin)),
       max_barrier_magnitude=float(np.max(barrier)),
       max_penetration_magnitude=float(np.max(penetration)),
       m3_contact_model='distributed_trapezoidal_soft_load',
       m3_active_force_sample_count=int(np.count_nonzero(selected)))