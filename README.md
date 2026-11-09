# SPGA — real research method + interactive web viewer

This repository contains **the SPGA-M3 pose-warping research METHOD** and a browser visualization. The original M3 solver is Python, in `backend/spga/`, and the UI is a static GitHub Pages website.

**No main-experiment suites, 1,920-trial outputs, 40-path batch runners, or result archives are included.** The only test file is `backend/tests/test_real_m3_api.py`, a small method/API validation.

## Architecture

```
Website / GitHub Pages
  Surface mesh + Nominal / Tilted A0,A1 + parameters
    -> POST /api/warp
  Python API (backend/app.py)
    -> SPGA M3: BVH + visibility + multi-normal field
    -> dual-arc + conic + ribbon + fixed-length projection
    -> iterative force equilibrium
    -> Warped A0,A1 and convergence diagnostics
  Website
    -> real warped poses, pose inspector and playback
```

The browser **does not reproduce the solver**. `src/real-m3.js` calls the Python API. The old simplified JS implementation may remain in `src/geometry.js` for historical reference, but `src/app.js` on this branch does not call its `warpDemo` function.

## Run the method

Python 3.12:

```bash
cd backend
python -m pip install -r requirements.txt
uvicorn app:api --host 0.0.0.0 --port 8080
curl http://localhost:8080/api/health
```

or via Docker:

```bash
docker build -t spga-m3 ./backend
docker run --rm -p 8080:8080 -e PORT=8080 spga-m3
```

Run unit/API tests:

```bash
cd backend
python -m pip install pytest httpx
PYTHONPATH=. pytest -q tests/test_real_m3_api.py
```

## Make the live GitHub Pages viewer use M3

GitHub Pages is static and **cannot run Python**. Deploy `backend/` to a persistent HTTPS compute service such as Render, Cloud Run, or your own host, then set the service URL in `src/m3-config.js`:

```js
export const SPGA_M3_BACKEND_URL = 'https://YOUR-HTTPS-SERVICE.example.com';
```

Configure server CORS using `SPGA_CORS_ORIGINS`. The default permits `https://bayleen.github.io` and local development.

Until the backend is running, pressing Warp on this branch deliberately shows an error (never fake M3 output). GitHub Pages itself remains available for interactive visual editing.

## Scientific boundary

- Solver functions `build_conic_from_tilted` and `warp_with_conic` come from the uploaded 2026-11-02 M3 research code, with its multi-normal field.
- The website generates a custom triangulated scene (21×21 vertices) rather than the experimental Bunny mesh. Thus website metrics are **not** the paper's Bunny results.
- Scene scale is `1 display unit = 0.05 m`. Server force balance and length constraints are calculated in metres.
- Penetration numbers shown by the website are renderer-sampled indicators, not certified collision safety.
- The API reports actual convergence mode, iteration count and projected force residual; max-iteration cases are marked **not converged**.
- Interactive force diagrams are **schematic illustrations**, not logged per-iteration force magnitudes. The website can be extended with server diagnostics later.
- The public API needs deployment-side access control, request quotas and compute-budget safeguards if exposed broadly.

## Source layout

- `backend/spga/`: essential original M3 method modules.
- `backend/configs/milestone_m3.json`: original M3 method configuration.
- `backend/app.py`: mesh/pose conversion + async solver API.
- `backend/tests/`: small correctness checks (not scientific experiments).
- `src/app.js`, `src/real-m3.js`: viewer and bridge.
- `src/m3-config.js`: backend address.
