# SPGA Interactive Web Lab — v4 (Always-live Editor + Per-pose Conic)

Browser-only 3D educational demonstrator for skill-preserving pose placement and collision-aware pose correction. Publishable on **GitHub Pages** (no Python server needed).

## Critical semantics

- **Surface:** Hill, Saddle, or a hand-drawn 2D height profile extruded into a 3D heightfield.
- **Guide:** A surface-following curve from a vertical cutting plane; you can also hand-draw its lateral displacement. Guide is NOT the source skill.
- **Nominal skill:** A0 positions along a learned/source shape (`u,v,w`), independent of the surface, with per-pose `contact` flags. A1 derives from A0 plus a fixed-length tool axis, not another copy of the guide.
- **Tilted:** Each individual learned primitive is placed between two equal-arc-length guide anchors using **one global similarity transform per primitive** (rotation, translation, and one uniform scale to match guide chord length). Interior samples are NOT individually projected to the surface; the full shape is preserved up to that uniform scale. This matches the key *placement principle* of the SPGA Python code; it is not a complete SE(3) transfer implementation.
- **Warp:** Browser-only approximate collision-aware orientation search. Marked A0 contact anchors are unchanged. Non-contact A0 may receive a limited lift if the rod still collides; length and cone are respected. **Not the archived SPGA-M2 force-balance / ribbon solver**. Do not use as validated research results.

## Usage — v4: Always editable, no lock mode

1. Pick Hill / Saddle / drawn profile, change Surface X/Z/rotation or Cut Plane/Guide. Use **Orbit** for camera-only dragging; **Move** and **Rotate** explicitly manipulate the surface.
2. Draw a separate Nominal (2D or CSV 3D Spiral). Click **3D edit**: every repeated primitive displays **exactly three interior white handles**. Drag any handle to reshape that position for **all repeats**; contact endpoints remain on the Guide. **Brush curve tension** controls the C1 cubic handle deformation. The displayed white A0 path uses a smoother visual brush line; collision solving still uses the original discrete poses.
3. Click **Generate Tilted**, then **Warp**. **Nothing locks**: changing the surface, guide, repeat count, nominal handles or tool angle/length updates Tilted immediately, and an existing Warp recalculates after a short debounce (~125 ms plus solve time). The currently selected pose is preserved wherever possible.
4. Turn on **Show selected conic** and **Focus one pose**. Click a tool/trajectory point in the scene or use the playback slider to choose which pose's A0, tilted axis, cone half-angle and warped axis to inspect. Turn Focus off for all sampled rods.
5. Right-side **Geometry / Conic / Ribbon** sliders change the browser demo objective. Use Avoid collision / Preserve skill presets to find a clearly affected pose; the inspector shows changed-pose count, difference in axes, A1, and penetration. Slider plateaus are possible because search is discrete. These are **demo weights, not validated Python M2 gains**.
6. In **All-contact**, the full Guide is A0 contact; **Draw A1 orientation** alters tool direction along the guide.

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
- No stage-dependent lock: Orbit always changes camera only; Move/Rotate and 3D edit explicitly change geometry while generated results stay up to date.
- Run `npm test` (Node 20+); no npm dependencies are needed.
- External researchers should use the archived Python SPGA-M2 for formal evaluations; this site is for interaction and pedagogy.

## GitHub Pages

For a public repository, select `Settings → Pages → Deploy from a branch → main → / (root)`.

## License

See `LICENSE`.


## v3 historical note: tool and one-sided geometry (2026-10-30)

**Superseded by v4:** surface, guide, nominal, tool, and balance inputs are all editable at any stage. Tool length and surface-relative angle (0–180°) recompute Tilted/previous Warp after edits. `0°=forward tangent`, `90°=outward normal`, `180°=backward tangent`. For imported CSV tool axes, the angle slider applies a common offset from its reference `25°`.

The right panel exposes **browser-demo weights** for geometry, conic, ribbon tension and bending and a cone half-angle; these coefficients are *not* the calibrated Python SPGA-M2 coefficients. Browser warp still uses independent candidate pose search, with predecessor-based ribbon-like scoring, and separate optional A0 lift for noncontact poses. It is **not** the full M2 dual-rail force-balance algorithm. Surface signed-height is a one-sided height-field approximation: negative points are treated as penetration and outward normals always point above the surface. No underside/second-sided normal is used.

The demo's penetration term increases quadratically with depth. It is stronger for deeper penetration, but cone and ribbon penalties mean complete removal is not mathematically guaranteed.

## v4 notes — live editing, selected cone, three brush handles

- All source scene controls remain enabled after generating Tilted/Warp; only Orbit moves the camera.
- Three C1 interior brush handles per repeated primitive; shared source skill changes apply to each repeat.
- One selected pose's cone can be visualized at A0, with the selected tilted and warped tool.
- Right-side weights affect the browser candidate-search objective; at poses without penetration, many values can produce the same solution.
- Solver is illustrative and client-side; **do not mistake it for the exact M2 force-balance solver**.