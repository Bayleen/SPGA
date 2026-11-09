# SPGA — Interactive Skill-Preserving Pose Warping

**Live website:** https://bayleen.github.io/SPGA/

SPGA is a public, browser-based interactive demonstration of **Skill-Preserving Geometric Adaptation** for surface manipulation. Build a surface, choose a Guide and Nominal Skill, and compare the transferred (Tilted) and adapted (Warped) tool poses.

The site runs the **original Python SPGA force-balance method**, including the multi-normal continuous geometry field, BVH/visibility, conic restoring, ribbon preservation, fixed-contact A0 anchors and fixed-length projection. It is **not** a hand-animated JavaScript collision demo.

## One-click example

Select **⚡ View instant SPGA example** at any time. This restores the public default configuration and shows the recorded result immediately:

| Setting | Published default |
| --- | --- |
| Surface | Hill |
| Guide cut angle | **60°** |
| Guide plane offset | **0.50** |
| Guide stroke width | **0.70** |
| Nominal mode | **All-contact**, 81 fixed A0 anchors |
| Tool length | 1.20 display units |
| Tool angle vs surface | 25° |
| Conic half-angle | **30°** |
| Geometry / Conic gains | 1.00 / 1.00 |
| Ribbon tension / bending gains | 0.60 / 0.35 |
| Preview budget | 20 solver iterations |

The default result is **not synthesized or visually guessed**. It was computed offline from the uploaded original Python SPGA research solver using exactly this mesh and nominal pose sequence. The website validates all 81 Tilted A0/A1 positions and the relevant parameters before accepting that recorded data. All A0 anchors are preserved.

**Scientific limitation:** the recorded example hit its **20-iteration limit** and **has not converged**. It is an interactive preview of the real solver, *not* an equilibrium claim, collision-clearance guarantee, or robot execution plan. The status and exported diagnostics preserve that distinction.

Any change to Surface, Guide, Nominal, tool size/orientation, cone, or force gains bypasses the recording and uses the genuine solver. The **Full Solve** option computes every pose (up to 3,000 iterations) and does not use the recorded preview.

## Use the website

1. Click **⚡ View instant SPGA example** to see a real recorded Warp immediately.
2. Edit the Surface → Guide → Nominal → Tool/Conic settings. Generate Tilted and click **Warp with SPGA** to compute your own case.
3. Inspect the selected pose/cone, use playback, change preservation-vs-clearance preferences, and export JSON.

The progress panel reports live preparation/loading stages, actual solver iteration counts (when computed), and convergence diagnostics. You can cancel or restart a running computation.

## Technical architecture

- `index.html`, `styles.css`, `src/app.js`: browser UI.
- `src/geometry.js`: surface editor and nominal placement.
- `src/spga-preset-cache.js`: versioned recorded default example, with an input-match guard.
- `src/real-m3.js`, `src/m3-browser-worker.js`: interface to the original Python solver through Pyodide.
- `backend/browser_bridge.py`: coordinates and diagnostics adapter.
- `backend/spga/`: unchanged core research solver.
- `backend/app.py`: optional native Python API for faster full solves.

Some internal research filenames retain their historical development names; the public-facing algorithm and product are named **SPGA**.

## Running locally

Start an HTTP server at repository root:

```bash
python3 -m http.server 8000
```

Then open http://localhost:8000 . The Python WebAssembly runtime is fetched from a public CDN on uncached custom runs. For native regression checks:

```bash
cd backend
python -m pip install -r requirements.txt pytest httpx
PYTHONPATH=. pytest -q tests/
```

GitHub Pages is static; it does not host a native Python server. Full browser computations may be slow. Python progress counts are **iterations within the maximum budget**, not percentage completion or predicted finish time.

## Responsible interpretation

Tool collisions shown in the viewer are sampled geometric diagnostics, not certified safety. An unconverged result is always identified as such. Changing parameters can trade clearance against shape similarity, which is a property of SPGA's soft force balance.
