# UGV UI fix/improve backlog

**Process:** pick top open item → small CSS/HTML/JS or docs fix → re-shot with `--catalog` if UI → mark done.

**Workflows:** `.grok/workflows/ui-fix-improve.rhai` · `.grok/workflows/honesty-complete-64.rhai`  
**Screenshots:** `ui_shots/` via `scripts/screenshot_ui.py --catalog`  
**Sources of truth:** `docs/operator-guide.md`, `docs/ui-qa-report.md`, fork status in `README.md`  
**Last honesty pass:** 2026-08-23

## Done (UI / safety / ops chrome)

- [x] Global STOP + Seek-running pill + finite Seek defaults + Idle heartbeat (HB) rename  
- [x] Navbar short chips + landscape sticky row  
- [x] Empty still placeholders; Seek pano empty hint; taller pano  
- [x] Seek sticky actions (phone); body.seek-running log expand; config lock while running  
- [x] Seek mode-specific camera hints (`syncSeekCameraHint` a/b/c)  
- [x] Seek radio highlight via `:has(input:checked)`  
- [x] Chat → full `/ai` link; Chat/Seek live fail overlay + retry; single active MJPEG
- [x] Chat Grab still is the JPEG Send attaches (not a second live grab)  
- [x] Twin larger drawer; ops/twin body classes  
- [x] Playwright catalog + operator guide  
- [x] Catalog does not auto-flip Direct↔ROS2 (UART)  
- [x] ROS serial fallback + rosbridge autoheal (see README — not pure UI)  
- [x] Drive signs on `/api/status`; config `drive_linear_sign: -1` for this chassis  
- [x] Kill `ugv_bringup` when switching ROS 2 → Direct (`UGV_AUTOSTOP_BRINGUP`)  
- [x] Seek battery gate (known low V); nav-plan extracted to `seek_nav.py` + unit tests  
- [x] Seek hop/turn tables match this chassis (UI-Fast T:1 spins; punchy linear)  
- [x] Seek dry-run default + confirm_live + exclusive chassis while Seek/Track runs  
- [x] Detector FOUND bar 0.45 / two-view; rear ±135 not used as door jambs  
  *(superseded 2026-08-22: flat **0.85** every class, per-class sofa/chair minimums removed — see README)*
- [x] Seek radios match the loop: a = L/C/R heuristic (no LLM); b/c front-first forced JSON, sides only when blocked
- [x] Chat Grab still is the JPEG Send attaches on both Chat and `/ai` (`snapshot_data_url`), live grab only as fallback
- [x] Single USB RoArm handle: app.py uses `roarm_ctrl.current_roarm/get_roarm/shutdown_roarm`; hangar stop/start cannot leave a closed handle
- [x] Seek run does not persist Chat tool toggles (thread-local override, not `.ai_capabilities.json` writes)
- [x] 3D Twin libs vendored locally (`three.min.js`, `OrbitControls.js`, `roslib.min.js`) — page loads offline in ~2s, was ~12s CDN-blocked
- [x] `[hidden]` always wins over class `display:` rules (global CSS rule) — dead "Recalculate every N steps" interval row now actually hidden
- [x] Stale Seek copy fixed: pano empty hint, scene-nav tooltip, start log no longer claim triple-view / interval nav
- [x] Catalog captures `seek.dry_run` + `shell.loadout` (coverage 55/56; `shell.path_ros2` env-dependent by design)
- [x] Screenshot script survives slow/dead pages: `goto()` never raises, `/3d` gets a 45s budget

## Open (honest residual)

| Priority | Item | Notes |
|----------|------|--------|
| P2 | Chat live may still blank under multi-tab / heavy load | Fail overlay + exclusivity exist; not bulletproof |
| P3 | Catalog path chips env-dependent | By design — manual capture of the other path |
| Later | Dual drawers open; Chat/`/ai` unify; look-map; coach marks | Product polish, not blockers for pilot demos |

## Product backlog (not UI-only)

See **Remaining** in `README.md` (LLM reliability, odom/lidar, closed-loop turns, container boot, CI mocks, security defaults).
