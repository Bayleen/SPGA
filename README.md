# SPGA Interactive Web Lab — v2 (Guide / Nominal / Tilted separated)

Browser-only 3D educational demonstrator for skill-preserving pose placement and collision-aware pose correction. Publishable on **GitHub Pages** (no Python server needed).

## Critical semantics

- **Surface:** Hill, Saddle, or a hand-drawn 2D height profile extruded into a 3D heightfield.
- **Guide:** A surface-following curve from a vertical cutting plane; you can also hand-draw its lateral displacement. Guide is NOT the source skill.
- **Nominal skill:** A0 positions along a learned/source shape (`u,v,w`), independent of the surface, with per-pose `contact` flags. A1 derives from A0 plus a fixed-length tool axis, not another copy of the guide.
- **Tilted:** Each individual learned primitive is placed between two equal-arc-length guide anchors using **one global similarity transform per primitive** (rotation, translation, and one uniform scale to match guide chord length). Interior samples are NOT individually projected to the surface; the full shape is preserved up to that uniform scale. This matches the key *placement principle* of the SPGA Python code; it is not a complete SE(3) transfer implementation.
- **Warp:** Browser-only approximate collision-aware orientation search. Marked A0 contact anchors are unchanged. Non-contact A0 may receive a limited lift if the rod still collides; length and cone are respected. **Not the archived SPGA-M2 force-balance / ribbon solver**. Do not use as validated research results.

## Usage

1. Select a surface, move/rotate it (before generation).
2. Adjust cutting plane and click **Select cut guide**, or choose **Draw guide A0** to draw an offset on that surface.
3. Select **Lifted skill** (palpation/spiral) or **All-contact** (brushing).
4. Lifted skill: click **Draw nominal** to sketch a 2D A0 path (START/END contact), then **3D edit** to orbit and drag white interior points in 3D (vertical and lateral). Drag the background to orbit; the two green contact feet are locked. **3D Spiral** preset generates a true 3D spiral-on-half-circle rather than an x-sorted wave.
5. Optional: import a **CSV** containing canonical A0 positions; `examples/nominal_3d_spiral.csv` is downloadable directly from the page.
6. Choose how many repeats to tile along the guide; each interval is spaced equally in **guide arc length**.
7. Click **Generate Tilted**, then **Warp**; scrub/play and compare. Generated geometry is read-only; drag pans/orbits camera only. **Edit scene** unlocks and clears results.
8. In **All-contact**, the complete guide is A0 contact; **Draw A1 orientation** changes the rod's angle along guide progress and maintains its fixed length.

### CSV format

UTF-8 comma-separated, with header, 3–3000 rows:

```csv
u,v,w,contact
0,0,0,1
0.25,0.30,0.15,0
0.5,0.42,-0.12,0
0.75,0.30,0.15,0
1,0,0,1
```

- `u`: canonical displacement along the source skill's endpoint chord.
- `v`: height/lift in the nominal sketch plane.
- `w`: lateral 3D displacement (spiral and other non-planar skills).
- `contact`: `1` for START/END only in Lifted skill mode; only endpoint anchors are supported in this placement version. Invalid intermediary contact flags are rejected rather than silently ignored.
- Optional `ax,ay,az` gives a **per-pose unit tool axis in source u/v/w coordinates**, which is rotated during Tilted placement, not used as an A1 absolute point. Without it, a single tool tilt parameter controls the primitive.
- All numeric coordinates are arbitrary canonical **display units**, not meters. A0 is the source trajectory, not a surface-following guide. Contact endpoints are always fixed.
- Data is displayed **in row order**: repeated X, reversed segments, loops and coils are preserved. `u` need not be monotonic.
- The source endpoint chord must be nonzero. Shape similarity includes uniform scale when target anchor chord differs from the source chord.

## Engineering details

- Plain static ES modules: `src/geometry.js` contains geometry and browser solver, `src/app.js` contains editing/visualization.
- Surface uses a height-field rendering and collision sampling (not exact arbitrary 3D mesh collision).
- The stroke-to-skill conversion uses cumulative 3D arc length for resampling, not `x` sorting; freehand loops won't collapse into a wave.
- Camera/geometry edit is explicitly locked after generating Tilted; only camera view changes until **Edit scene** is chosen.
- Run `npm test` (Node 20+); no npm dependencies are needed.
- External researchers should use the archived Python SPGA-M2 for formal evaluations; this site is for interaction and pedagogy.

## GitHub Pages

For a public repository, select `Settings → Pages → Deploy from a branch → main → / (root)`.

## License

See `LICENSE`.


## v3 live tool / one-sided geometry (2026-10-30)

After **Generate Tilted**, surface/guide/nominal are frozen; tool length and surface-relative angle (0–180°) remain editable and recompute Tilted/previous Warp immediately. `0°=forward tangent`, `90°=outward normal`, `180°=backward tangent`. For imported CSV tool axes, the angle slider applies a common offset from its reference `25°`.

The right panel exposes **browser-demo weights** for geometry, conic, ribbon tension and bending and a cone half-angle; these coefficients are *not* the calibrated Python SPGA-M2 coefficients. Browser warp still uses independent candidate pose search, with predecessor-based ribbon-like scoring, and separate optional A0 lift for noncontact poses. It is **not** the full M2 dual-rail force-balance algorithm. Surface signed-height is a one-sided height-field approximation: negative points are treated as penetration and outward normals always point above the surface. No underside/second-sided normal is used.

The demo's penetration term increases quadratically with depth. It is stronger for deeper penetration, but cone and ribbon penalties mean complete removal is not mathematically guaranteed.