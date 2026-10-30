# SPGA Interactive Pose Warping Lab

An **interactive browser-only 3D demonstrator** built for GitHub Pages. Zero npm, backend, or CDN dependencies. All rendering and geometry calculation run in the browser (Canvas 2D software 3D renderer).

## Launch locally

From the project root:

```bash
python3 -m http.server 8000
```

Open `http://localhost:8000`.

## Publish a public GitHub Pages site

1. Create a **public** GitHub repository, e.g. `spga-web-lab`.
2. Upload **the contents** of this folder to the repository root (including `index.html`, `styles.css`, `src/`, `.nojekyll`).
3. Go to **Settings → Pages → Build and deployment → Deploy from a branch**.
4. Choose **main** and **/(root)**, then Save.
5. After publication, the URL is `https://USERNAME.github.io/spga-web-lab/`.

`index.html` contains only relative paths, so it works with repository Pages URLs. No extra server or build is needed.

## How to use

1. Pick **Hill** or **Saddle**, or click **Draw 2D profile** to sketch a single-valued 2D height profile and extrude it along Z. Move and rotate with the sliders or with the `Move/Rotate` canvas mode. Orbit with drag, zoom with scroll.
2. Set **Cut angle** and **Plane offset**; the visible vertical cutting plane intersects the terrain to create the **Guide trajectory**.
3. **Draw nominal** is a left-to-right 2D drawing: horizontal position is progression along Guide, and vertical position is transverse offset. The 2D stroke is projected onto the current 3D terrain to create A0 contact points.
4. Set **Tool length**, **Tilt angle** (tilt measured away from the local surface normal, toward the path tangent), and **Cone half-angle** (permitted rotation away from each tilted axis). Optional **Reverse tool direction**.
5. **Generate Tilted** → **Warp**. Play or scrub the pose slider; Compare shows the two rods simultaneously. Collision samples are marked red on any displayed rod that intersects the terrain.
6. **Export JSON** to save input parameters, curves, output A0/A1 poses, and demo metrics.

## Important research distinction

This is a **teaching / outreach / interaction prototype** and **NOT a numerical reproduction** of the locked `SPGA-M2-C0-Field-FIXED-2026-10-30` Python research solver. In particular:

| Browser demo | Archived M2 solver |
|---|---|
| Procedural heightfield from 2D extrusion or Hill/Saddle | General mesh geometry with closest-triangle projection |
| Smooth heightfield normal at surface | Area-weighted vertex normal barycentric interpolation on selected mesh triangle |
| Rod sampled at 24 interior points; approximate signed-height clearance | Discrete active contact regions based on nearest-mesh signed distance |
| Bounded angular search around tilted rod, favor clearance | Geometry/Conic/Ribbon pseudo-force equilibrium and Jacobian contact transfer |
| A0 fixed; rod length fixed | A0 anchor; fixed-length projection; conic/ribbon forces and convergence rules |

The demo preserves **A0** and **rod length** exactly; it searches for rod orientations within a maximum angle from the tilted pose and demonstrates collision avoidance. It is not the numerical SPGA force balance, does not preserve the full ribbon objective, and cannot guarantee full collision clearance or a globally smooth vector field. In particular, numerical values and success rates **must not** be used as paper experiments.

The procedure can be upgraded to real SPGA-M2 with a Python service/API running `spga/warping.py`, but GitHub Pages alone cannot execute the original Python dependencies server-side. An alternative is a full, tested JS/WASM reimplementation.

## Geometry and scope limits

- User-drawn surfaces are **2.5D height profiles** (single height per local X) extruded along Z, not arbitrary closed solids.
- Hill/Saddle are heightfields (single height per X,Z), not manifold self-intersecting meshes.
- Intersection and penetration are vertical-height based approximations; there is no continuous rod-vs-triangle collision or articulated robot checking.
- The `angle` for the browser tilt is a user-friendly visual parameter, not the full SPGA nominal skill/conic parameterization.
- 1 world unit is arbitrary; exported units are clearly identified and **not millimeters**.
- Users can run the public GitHub Pages site on their own computer without sharing drawings on a server; exported data is downloaded locally.

## Files

- `index.html` — responsive studio UI
- `styles.css` — layout and theme
- `src/geometry.js` — deterministic geometry & demonstration solver
- `src/app.js` — canvas 3D scene, drawing modal, sliders, animation & export
- `tests/geometry.test.mjs` — Node unit tests for geometry and pose constraints

License: MIT (see LICENSE).