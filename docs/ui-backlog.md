# UGV UI fix/improve backlog

**Process:** pick top open item → small CSS/HTML/JS or docs fix → re-shot with `--catalog` if UI → mark done.

**Workflows:** `.grok/workflows/ui-fix-improve.rhai` · `.grok/workflows/honesty-complete-64.rhai`  
**Screenshots:** `ui_shots/` via `scripts/screenshot_ui.py --catalog`  
**Sources of truth:** `docs/operator-guide.md`, `docs/ui-qa-report.md`, fork status in `README.md`  
**Last honesty pass:** 2026-08-13

## Done (UI / safety / ops chrome)

- [x] Global STOP + Seek-running pill + finite Seek defaults + Idle heartbeat (HB) rename  
- [x] Navbar short chips + landscape sticky row  
- [x] Empty still placeholders; Seek pano empty hint; taller pano  
- [x] Seek sticky actions (phone); body.seek-running log expand; config lock while running  
- [x] Seek mode-specific camera hints (`syncSeekCameraHint` a/b/c)  
- [x] Seek radio highlight via `:has(input:checked)`  
- [x] Chat → full `/ai` link; Chat/Seek live fail overlay + retry; single active MJPEG  
- [x] Twin larger drawer; ops/twin body classes  
- [x] Playwright catalog + operator guide  
- [x] Catalog does not auto-flip Direct↔ROS2 (UART)  
- [x] ROS serial fallback + rosbridge autoheal (see README — not pure UI)  
- [x] Drive signs on `/api/status`; config `drive_linear_sign: -1` for this chassis  
- [x] Kill `ugv_bringup` when switching ROS 2 → Direct (`UGV_AUTOSTOP_BRINGUP`)  
- [x] Seek battery gate (known low V); nav-plan extracted to `seek_nav.py` + unit tests  
- [x] Seek hop/turn tables match this chassis (UI-Fast T:1 spins; punchy linear for cable runner)  

## Open (honest residual)

| Priority | Item | Notes |
|----------|------|--------|
| P2 | Chat live may still blank under multi-tab / heavy load | Fail overlay + exclusivity exist; not bulletproof |
| P3 | README historical sections may still say “Motors” / old labels | Fork status + Drive safety updated; older upstream narrative below may lag |
| P3 | Catalog path chips env-dependent | By design — manual capture of the other path |
| Later | Dual drawers open; Chat/`/ai` unify; Twin CDN offline; look-map; coach marks | Product polish, not blockers for pilot demos |

## Product backlog (not UI-only)

See **Remaining** in `README.md` (LLM reliability, odom/lidar, closed-loop turns, container boot, CI mocks, security defaults).
