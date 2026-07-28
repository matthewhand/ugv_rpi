# UI catalog QA report (guide-aligned)

**Last catalog sample:** 2026-07-28  
**Docs honesty pass:** 2026-07-29  
**Guide:** `docs/operator-guide.md`  
**Tool:** `scripts/screenshot_ui.py --catalog`  
**Shots:** `ui_shots/` (gitignored)

## What this is / is not

| Claim | Truth |
|-------|--------|
| We have a feature catalog mapped to a guide | **Yes** — `guide_id` rows + `--catalog` |
| Every PNG is vision-QA’d every commit | **No** — sample inspection + coverage counts; re-run after big UI changes |
| Catalog flips all control modes automatically | **No** — does **not** thrash Direct↔ROS2, WiFi stop, or Seek start |

## Coverage (2026-07-28 catalog run)

| Metric | Value |
|--------|------:|
| Shots | ~91 (0 fail that run) |
| Guide IDs | 54 |
| Captured | ~53/54 |
| Typical miss | Other control path chip (env-dependent) |

**By design not automated:** real motor motion, confirmed ESP32 WiFi stop, Docker restart, LLM replies.

## Findings status (honest)

| Sev | Issue | Status as of 2026-07-29 |
|-----|--------|-------------------------|
| P1 | Seek radio green on non-selected card | **Fixed** (`:has(input:checked)`) |
| P1 | Chat blank live MJPEG | **Mitigated** (exclusivity, fail overlay, retry) — still watch multi-client load |
| P2 | Seek camera hint always LLM L/C/R | **Fixed** (`syncSeekCameraHint` per mode a/b/c) |
| P2 | Simulated Seek running vs phase idle | **Documented** — sim chrome only |
| P3 | Path chip catalog env-dependent | **By design** (manual other path) |

## Re-run

```bash
.tools/playwright-env/bin/python scripts/screenshot_ui.py --catalog \
  --base http://127.0.0.1:5000 --out ui_shots
```

After run: compare `manifest.json` `missing_guide_ids` and spot-check STOP / HB / Seek modes / Chat live against the guide.

## Related ops issues (not pure UI)

Documented in README: ROS2 + dead rosbridge caused **dropped serial** (PTZ); **serial fallback** + **autoheal** address that. Drive F/B polarity is `drive_linear_sign` — confirm via `/api/status` after app restart.
