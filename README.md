# SPGA — Interactive Pose Warping

A static, browser-based demonstration of **Skill-Preserving Geometric Adaptation (SPGA)**, ready for GitHub Pages. The browser uses a simplified visual solver, **not** the archived Python SPGA-M2 force-balance implementation.

## Beginner tutorial

Select **Beginner Tutorial** in the top bar, confirm **Start tutorial**, and follow the seven highlighted steps. The tutorial does not reset the scene and allows visitors to interact with each control. Use **Next**, **Back**, **Skip**, or **Escape**.

1. **Surface:** choose Hill or Saddle, or draw a 2D profile extruded into 3D. Orbit to view; Move / Rotate to edit the surface.
2. **Guide:** move/rotate the cutting plane and choose the yellow curve where the skill will be placed.
3. **Nominal:** draw or import a 2D/3D A0 skill; contact anchors are at the endpoints for lifted skills. The three white handles create a C¹-smooth curve. All-contact mode follows Guide as A0 and allows an A1 orientation sketch.
4. **Tilted:** place repeated nominal primitives along the Guide; the interior A0 points are not projected onto the target surface.
5. **Warp:** compute an outward-only geometric correction with approximate Conic and Ribbon trade-offs.
6. **Playback:** select poses, view a single cone, and compare Tilted with Warped rods.
7. **Force Balance:** adjust Geometry, cone half-angle, Conic restoring, Ribbon tension and Ribbon bending; the browser recomputes the example live.

## Main features

- In-browser 3D viewer, interactive surface transforms, 2D surface profile sketches, and Hill / Saddle presets.
- Independent Guide and Nominal paths; lifted-skill and all-contact modes.
- 3D spiral, half-circle, freehand Nominal and ordered CSV importing. See `examples/nominal_3d_spiral.csv`.
- Selectable poses with Conic visualization; live tool angle (0–180° relative to surface tangent) and length changes.
- Real-time demonstration balance sliders, trajectory playback, geometry diagnostics, and JSON export.
- No application backend: works on static hosting such as GitHub Pages.

## Run locally

Open via any local HTTP server (ES modules require HTTP, not `file://`):

```bash
python3 -m http.server 8000
```

Then visit `http://localhost:8000`. Run the geometry tests with `npm test` (Node.js required).

## Deploy to GitHub Pages

Use `Settings → Pages → Deploy from a branch`, choose `main` and `/(root)`. After deployment the site will be available at your repository's Pages URL.

## Research boundary

The browser weights and the simplified pose search demonstrate qualitative SPGA trade-offs, but they are **not** calibrated to the fixed research solver. Collision checks sample the rendered rod and are not a formal safety guarantee. For quantitative scientific results, run the archived Python SPGA-M2 implementation.

## Force visualizations (v5)

The right-hand **Force Balance** section presents three independent SVG diagrams that respond immediately to the sliders:

- **Geometry (Fᵍ)** shows an outward normal correction arrow. Increasing Geometry enlarges the arrow.
- **Conic restoring (Fᶜ)** shows a large cone centered on the Tilted tool axis. **Cone half-angle β** changes the cone aperture; **Conic restoring** lengthens the arrow that points toward its center axis.
- **Ribbon tension (Fʳ)** shows a semicircular ribbon becoming shallower as the tension slider increases, with larger outward pull arrows. This is a **visual metaphor**, not a claim that the SPGA Ribbon term geometrically flattens the nominal trajectory; the actual term penalizes changes between adjacent poses.

Illustrated arrows are proportional to the **browser control values**, not measured physical force. The independent tutorial's final step explains how to use them. These illustrations do not modify the frozen Python solver.