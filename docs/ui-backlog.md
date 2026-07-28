# UGV UI fix/improve backlog

**Process:** each iteration = pick top open P0/P1 → implement small CSS/HTML/JS change → re-shot key surfaces → mark done.

**Workflow:** `.grok/workflows/ui-fix-improve.rhai` (named run needs project folder trust; use `script_path` or run iterations manually).  
**Screenshots:** `ui_shots/` via `scripts/screenshot_ui.py`  
**Last batch:** 10 iterations 2026-07-28

## Done (prior)

- [x] Global STOP + Seek-running pill + finite Seek defaults + Idle heartbeat rename
- [x] Navbar short chips (RTSP/Direct/HB/WiFi/Log/Twin) + landscape sticky row
- [x] Empty still placeholders (Chat + AI Agent)
- [x] Seek pano empty-state hint
- [x] Playwright screenshot suite

## Iterations 1–10 (this run)

| # | Priority | Item | Status |
|---|----------|------|--------|
| 1 | P0 | Seek sticky action bar (Seek/Stop/Check) on phone / short height | **done** |
| 2 | P0 | `body.seek-running` expands Seek log; collapse log when idle on mobile | **done** |
| 3 | P1 | STOP visual hierarchy (white STOP, demote form Stop) | **done** |
| 4 | P1 | Twin drawer larger default size | **done** |
| 5 | P1 | Chat panel: link to full `/ai` power console | **done** |
| 6 | P1 | Seek panorama taller empty/active height | **done** |
| 7 | P1 | Ops/twin body classes for cascade (drawers still mutual-exclude) | **done** |
| 8 | P2 | AI Agent landscape: hide tip, tighter composer | **done** |
| 9 | P2 | Seek config locked while running | **done** |
| 10 | P2 | Sticky nav content scroll-padding | **done** |

## Later (next cycles)

- [ ] Allow ops log + twin both open with real cascade (drop mutual exclude)
- [ ] Unify Chat tab with `/ai` iframe embed
- [ ] Vendor Twin CDN assets for offline AP demos
- [ ] Seek look-map / exploration trail visualization
- [ ] First-run coach marks on Seek tab
- [ ] Sticky Seek/Stop also in navbar when running (pill already stops)
- [ ] Design tokens (single accent scale) across stock mint + fork blue
