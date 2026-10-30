# v2 changes (2026-10-30)

- Fixed conceptual mismatch: guide and source nominal skill are separate data structures.
- Source A0 is a true lifted/non-contact trajectory between start/end contact feet; A1 obtained from pose and rigid rod.
- Tilted transfers whole primitive by a single Sim(3), with equally spaced guide arc intervals, and preserves pairwise shape up to global uniform scale.
- Added true 3D half-circle spiral, editable control points, ordered-arc-length freehand resampling (supports loops), CSV import with optional tool-axis columns.
- Added All-contact / Brushing mode: every guide A0 is contact, optional user-defined A1 tilt-angle profile.
- Proper guide interpolation endpoint surface projection and explicit contact mask propagation into demo Warp.
- Preserved lock-after-generation behavior, orbit, comparison and playback.
- The browser demo is still **not** the full research SPGA-M2 solver.