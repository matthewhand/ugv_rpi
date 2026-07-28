# UGV UI fix/improve backlog

**Process:** each iteration = pick top open P0/P1 → implement small CSS/HTML/JS change → re-shot key surfaces → mark done.

**Workflows:** `.grok/workflows/ui-fix-improve.rhai` · `.grok/workflows/honesty-complete-64.rhai`  
**Screenshots:** `ui_shots/` via `scripts/screenshot_ui.py`  
**Sources of truth:** `docs/operator-guide.md`, `docs/ui-qa-report.md`  
**Last batch:** 10 iterations 2026-07-28 · honesty priorities synced same day

## Done (prior)
- [x] Global STOP + Seek-running pill + finite Seek defaults + Idle heartbeat rename
- [x] Navbar short chips (RTSP/Direct/HB/WiFi/Log/Twin) + landscape sticky row
- [x] Empty still placeholders (Chat + AI Agent) — no broken black boxes
- [x] Seek pano empty-state hint
- [x] Playwright screenshot suite
- [x] Seek radio highlight via `:has(input:checked)` (not static `--default` only) — honesty.radio_checked_css
- [x] Seek config locked while running (`setSeekControlsRunning` + body.seek-running) — honesty.seek_config_lock
- [x] `seek.running_chrome` labeled **Simulated** in operator-guide / catalog notes — honesty.running_chrome_label
- [x] Freq. stop → Idle heartbeat / HB chip labels (no “Freq. stop” residue in main chrome) — honesty.freq_stop_rename_residue
- [x] Catalog does not auto-flip Direct↔ROS2 (UART); guide notes manual capture — honesty.catalog_path_manual

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

## Open — honesty / guide-alignment (pick next)

From `docs/ui-qa-report.md` residual defects + honesty workflow priorities.  
**Order:** top open P0/P1 first.

| # | Priority | Item | Status | Notes |
|---|----------|------|--------|-------|
| 11 | **P1** | Chat live camera blank / missing MJPEG (`#chat-live-preview`) | **open** | honesty.chat_live_mjpeg — retry wired in modes.js; still often blank in shots; need honest fail state or reliable stream |
| 12 | **P2** | Seek camera hint mode-specific (a/b/c), not always LLM L/C/R copy | **open** | honesty.seek_mode_hint — `#seek-hint-camera` static in index.html |
| 13 | P3 | Catalog `shell.path_direct` when server is ROS2 | **deferred** | Env-dependent; manual flip only — do not thrash UART in automation |

## Later (product polish — after honesty P1/P2)

- [ ] Allow ops log + twin both open with real cascade (drop mutual exclude)
- [ ] Unify Chat tab with `/ai` iframe embed
- [ ] Vendor Twin CDN assets for offline AP demos
- [ ] Seek look-map / exploration trail visualization
- [ ] First-run coach marks on Seek tab
- [ ] Sticky Seek/Stop also in navbar when running (pill already stops)
- [ ] Design tokens (single accent scale) across stock mint + fork blue
