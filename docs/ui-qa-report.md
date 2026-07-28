# UI catalog QA report (guide-aligned)

**Date:** 2026-07-28  
**Guide:** `docs/operator-guide.md`  
**Tool:** `scripts/screenshot_ui.py --catalog`  
**Shots:** `ui_shots/` (gitignored) · `manifest.json`

## Direct answer

| Question | Answer |
|----------|--------|
| Were we capturing **every** feature/toggle before? | **No.** Only ~11 page landings × 2 viewports. |
| Is there a user guide screenshots map to? | **Now yes:** `docs/operator-guide.md` with `guide_id` rows. |
| Do we inspect every shot vs the guide? | **This run:** catalog + sample multimodal inspection + coverage report. Full manual pass of all 91 PNGs is ongoing practice, not automated vision QA. |

## Coverage (final catalog run)

| Metric | Value |
|--------|------:|
| Shots written | **91** (0 fail) |
| Guide IDs | **54** |
| Captured | **53 / 54** |
| Missing | `shell.path_direct` (server was on **ROS2** at capture; catalog does not thrash path) |

**Path chips (`shell.path_direct` / `shell.path_ros2`):** catalog capture is **manual / env-dependent** — the suite records whichever path the server is on and **never** auto-flips Direct↔ROS2 (no UART thrash). Capture the other state by hand if needed.

**Intentionally not flipped in automation:** ROS2↔Direct toggle (UART thrash), ESP32 WiFi confirm, Seek start (motors), stack restart.

## What the catalog walks

- Modes Raw / Chat / Seek  
- Navbar chips: HB, RTSP, path, WiFi, Log, Twin, STOP, running pill (simulated)  
- Raw: speed, CV motion/faces, reaction capture, AUTODRIVE, OBJECTS, MP FACE, headlight, base light, PTZ self-test  
- Chat: empty still, attach off, grab still, `/ai` link  
- Seek: modes a/b/c, on-found none/tts, scene-nav off, limits, pano empty, running chrome  
- `/ai`, `/3d`, settings, photo, video  
- Desktop + phone landscape  

## Inspection findings (shot vs guide)

### Aligns with guide ✓

| guide_id | Observation |
|----------|-------------|
| `shell.stop` | Red **STOP** always present |
| `shell.hb_on` / chips | Compact RTSP/Direct|ROS2/HB/WiFi/Log/Twin |
| `seek.pano_empty` | “No scan yet…” placeholder, not broken image |
| `seek.running_chrome` | Pill **Seek running · 3/30** next to STOP; config locked (`is-locked` / disabled inputs) via sim lock path (**simulated**; phase may stay **idle**) |
| `chat.link_ai` | “Open full AI agent →” present |
| `chat.still_empty` | “No still yet — Grab still” (good) |
| `seek.limits` | Max steps 30 / timeout 300 |
| `seek.mode_a` | Radio **a** selected; scene-nav unchecked |

### Defects / misalignments found ✗

| Sev | Issue | Evidence | Guide expectation | Fix |
|-----|--------|----------|-------------------|-----|
| **P1** | Seek mode **b** stayed green-bordered when **a** selected | `desktop__seek_mode_a.png` | Selected card only | CSS: highlight via `:has(input:checked)` not static `--default` (**fixed** in this commit) |
| **P1** | Chat “Live camera” often **missing MJPEG** (only Grab still) | `desktop__chat_still_empty.png` ~42KB vs grab ~280KB | Live feed visible | Investigate `#chat-live-preview` not rendering / failed load; not fixed this pass |
| **P2** | Seek camera hint always describes LLM L/C/R cycle | `seek_mode_a` | Mode-specific hint | Dynamic hint (still open) |
| **P2** | Phase shows **idle** while pill says running | simulated chrome only | Real running would update phase | OK for sim; real run needs poll |
| **P3** | `path_direct`/`path_ros2` catalog is manual/env-dependent (no UART thrash) | missing `path_direct` when ROS2 live | Both path states in guide | Manual flip or mock for CI — by design, not a catalog bug |
| **P3** | Steady ON click previously hit restart banner | catalog log | Scoped steady selector | Improved selectors |

### Out of scope (by design)

- Live motor motion, real Seek start, confirmed WiFi stop, Docker restart  
- LLM reply content  

## How to re-run + inspect

```bash
# App on :5000
.tools/playwright-env/bin/python scripts/screenshot_ui.py --catalog \
  --base http://127.0.0.1:5000 --out ui_shots

# Coverage printed at end; open PNGs by guide_id from manifest.json
python3 -c "import json;m=json.load(open('ui_shots/manifest.json'));print(m['guide_ids_captured'],'/',m['guide_ids_total'], m['missing_guide_ids'])"
```

**Workflow for continuous QA:** pick guide_id → open matching PNG → check Expected UI column → file defects in `docs/ui-backlog.md`.

## Bottom line

We **now** have a guide + feature catalog (~full surface). We are **not** yet at “every screenshot auto-verified by vision model against guide,” but we **do** have:

1. Documented expected UI per control  
2. Automated capture of almost every toggle state  
3. Human/multimodal inspection of this run with concrete defects  

Highest residual product defect from this pass: **Chat live camera often blank** and **seek radio highlight** (highlight fixed).
