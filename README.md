# SPGA — Research M3 Method + Interactive 3D Website

A real **SPGA M3** implementation and an interactive viewer. The production
website runs the **original Python solver**, not the legacy JavaScript visual
search. The repository intentionally includes only method code and small
regression tests. It **does not include** the 1,920-trial experimental suite,
bulk outputs, research CSV runs, ablation pipeline, or batch experiment tools.

## Public website

**https://bayleen.github.io/SPGA/**

Open the site, edit **Surface → Guide → Nominal**, click **Generate Tilted**,
select an M3 mode and press **Run SPGA M3**.

- **Fast M3 Preview (default):** Executes the genuine M3 force-balance solver
  on approximately 18 sampled poses (including every fixed A0 anchor),
  with up to 20 iterations and a 9×9 surface grid. Intermediate visual
  poses are interpolated **only for playback**. This is not a scientific
  penetration, force balance, or convergence assessment of the entire path.
- **Full Research M3:** All poses, 21×21 surface grid, up to 3,000
  iterations. **Every returned pose is solved by the original Python method.**
  In-browser execution may be very slow for long trajectories.

Both modes expose the actual solver's iteration count and convergence
diagnostics. Any max-iteration output is labelled **not converged**. The
viewer uses a 0.05 m per display-unit conversion, recorded in the JSON
export. This is not a certified robot safety analysis.

## Architecture (zero server configuration)

```
GitHub Pages website (HTML / CSS / JavaScript visualization)
   -> src/real-m3.js
   -> src/m3-browser-worker.js
   -> Pyodide (CPython + NumPy + SciPy compiled to WebAssembly)
   -> backend/browser_bridge.py
   -> ORIGINAL backend/spga/ Python M3 modules
      - geometry multi-normal field + original BVH/visibility computation
      - active-contact, dual arc, ribbon, conic
      - fixed-length tangent projection
      - projected force residual / bounded oscillatory stopping
   -> Warped A0/A1, convergence and diagnostics -> 3D scene
```

The browser loads Pyodide from a public CDN, so an internet connection is
required on the first load. Browsers do not support Numba's LLVM JIT;
the original M3 BVH Python ray-intersection function executes as-is without
the JIT decorator. This preserves method semantics but is substantially
slower than a native Python backend. No API token or cloud server is required.

An **optional native backend** is included for fast full runs; configure
`src/m3-config.js` to point to a running HTTPS service (and configure CORS).
In that case the same method is run on the server. GitHub Pages alone
cannot host a persistent native Python process.

## Local tests

```bash
cd backend
python -m pip install -r requirements.txt pytest httpx
PYTHONPATH=. pytest -q tests/
```

The browser adapter regression test compares original native M3 API results
to `browser_bridge.solve_json()` on identical mesh/poses, verifying A0/A1,
the resulting tool axes, iterations, and convergence status. The original
Python scientific solver remains unchanged.

To view locally from the **repository root**:

```bash
python -m http.server 8000
```

Open http://localhost:8000 and select **Run SPGA M3**. ES modules,
Web Workers and Python asset loading require HTTP(s), not `file://`.

## Research clarity

- `backend/spga/` — authentic M3 research method.
- `backend/app.py` — optional HTTP server for native computation.
- `backend/browser_bridge.py` — browser-compatible JSON adapter, same
  `MeshSurface`, configuration and `solve()` logic as the native API.
- `src/real-m3.js` — request preparation and transparent preview-mode
  interpolation for visualization; no force computation in JavaScript.
- `src/m3-browser-worker.js` — real Python runtime on GitHub Pages.
- `src/geometry.js` — guide/nominal placement and display geometry;
  historical `warpDemo` is not called by the web application's M3 path.

This is an interactive research explanation, not a substitute for the
original offline experiment suite or safety validation.
