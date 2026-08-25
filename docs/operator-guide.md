# UGV Operator Guide (fork)

Visual QA source of truth for Playwright feature catalog.  
**Shot mapping:** each `guide_id` below is captured by `scripts/screenshot_ui.py --catalog`.

> Labels: **Idle heartbeat** (was “Freq. stop”). **Direct / ROS2** = motion path chips. **STOP** = global emergency stop.

---

## 1. Shell & global chrome

| guide_id | Feature | Expected UI |
|----------|---------|-------------|
| `shell.raw` | Mode Raw | Tab **Raw** active; stock video HUD + control grid |
| `shell.chat` | Mode Chat | Tab **Chat** active; live camera + vision chat; link **Open full AI agent →** |
| `shell.seek` | Mode Seek | Tab **Seek** active; live camera, log, mode radios a/b/c, finite max steps/timeout |
| `shell.stop` | Global STOP | Red **STOP** always in navbar (Raw/Chat/Seek) |
| `shell.rtsp_off` | RTSP chip OFF | Chip **RTSP** red/`is-off` |
| `shell.rtsp_on` | RTSP chip ON | Chip **RTSP** green/`is-on` (stream enabled server-side) |
| `shell.path_direct` | Motion path Direct | Chip **Direct** green/`is-on`. **Catalog is env-dependent:** captures the live path only; does **not** flip ROS2↔Direct (no UART thrash). Capture the other state manually. |
| `shell.path_ros2` | Motion path ROS2 | Chip **ROS2** blue accent (may show restart banner). Same rule: **manual/env-dependent** catalog capture — no auto-flip, no UART thrash. |
| `shell.hb_on` | Idle heartbeat ON | Chip **HB** green — safety default for manual |
| `shell.hb_off` | Idle heartbeat OFF | Chip **HB off** amber — for AI timed drives |
| `shell.wifi_chip` | ESP32 WiFi chip | Chip **WiFi** / **WiFi off** (confirm dialog before stop — do not confirm in QA) |
| `shell.ops_log` | App log open | Purple ops drawer bottom-right; level filter, follow, clear |
| `shell.twin_drawer` | 3D Twin open | Blue twin drawer; iframe of `/3d`; larger default size |
| `shell.navbar_phone` | Phone landscape chrome | Single sticky row; brand may hide; **STOP** visible |
| `shell.loadout` | Mode Loadout | Tab **Loadout** active; hangar bay with rover/beast stencils and attachment overlay |

**Operator notes**
- Prefer **Direct** for simple Seek demos and reliable PTZ.
- Leave **HB ON** for stick driving; turn **HB off** only when using AI timed drives.
- **STOP** zeros wheels, cancels Seek **and** Track, clears AI motion lock.
- **ROS 2:** needs rosbridge; if it dies, firmware autoheal retries and serial fallback keeps motion alive (see README). Do not expect silent perfection if Docker/`ugv_ros2` is broken.
- **Drive polarity:** after changing `drive_linear_sign`, **restart the Flask app** and check `GET /api/status` → `drive_linear_sign` (browser refresh alone does not reload yaml).
- `shell.path_direct` / `shell.path_ros2` catalog shots depend on server path at capture time; flip the chip yourself if you need the other PNG (catalog never thrash-flips UART).

---

## 2. Raw mode (stock teleop)

| guide_id | Feature | Expected UI |
|----------|---------|-------------|
| `raw.overview` | Full Raw panel | Video feed, D-pad, speed, CV sections, galleries |
| `raw.speed_mid` | Speed Middle selected | Middle button active styling |
| `raw.speed_fast` | Speed Fast selected | Fast active |
| `raw.steady_on` | PT Steady ON | Steady ON active |
| `raw.cv_motion` | Detection = Motion | Movtion button active |
| `raw.cv_faces` | Detection = Faces | Faces active |
| `raw.reaction_capture` | Reaction = Capture | Capture active |
| `raw.autodrive` | AUTODRIVE highlighted | AUTODRIVE pressed (line-follow only; needs floor line) |
| `raw.objects` | OBJECTS CV | OBJECTS active |
| `raw.mp_face` | MediaPipe face | MP FACE active |
| `raw.headlight_on` | Head light ON | ON active |
| `raw.base_light` | Base light ON | BASE ON active |
| `raw.ptz_selftest` | PTZ Self-Test block | Button **Run PTZ test** visible with hint |

**Operator notes**
- **AUTODRIVE** is OpenCV line-follow, not Seek/LLM nav.
- Stock typos (Movtion, baterry) may still appear in icons — known debt.

---

## 3. Chat mode

| guide_id | Feature | Expected UI |
|----------|---------|-------------|
| `chat.overview` | Chat panel | Live MJPEG + chat log + compose |
| `chat.still_empty` | No still yet | Placeholder “No still yet — Grab still” (not broken black box) |
| `chat.still_grabbed` | After Grab still | Thumbnail still visible; placeholder gone |
| `chat.attach_off` | Attach still unchecked | Checkbox off |
| `chat.link_ai` | Link to full agent | Visible **Open full AI agent →** / `/ai` |

**Operator notes**
- Chat is a thin client of `/api/ai/chat`. Tool capability toggles live on **`/ai`** only.
- Motion tools default off server-side; enable on `/ai` first.
- **Grab still** freezes the JPEG that **Send** attaches when the checkbox is on. If no still has been grabbed, Send captures a live frame.
- Voice Off / Browser / Robot needs `UGV_STT_URL` and `UGV_TTS_URL` in `.env`.

---

## 4. Seek mode

| guide_id | Feature | Expected UI |
|----------|---------|-------------|
| `seek.overview` | Default Seek (mode b) | Radio **b** selected; max steps 30; timeout 300; scene nav checked |
| `seek.mode_a` | Object Detector only | Radio **a**; scene-nav checkbox disabled/unchecked |
| `seek.mode_b` | Detector + LLM Nav | Radio **b** (default); scene nav enabled |
| `seek.mode_c` | LLM Detection & Vision | Radio **c**; free-text goal field shown |
| `seek.goal_select` | Detector goal class | Select shows VOC labels (e.g. person, chair) |
| `seek.onfound_none` | Upon found = none | Do nothing selected; TTS field hidden |
| `seek.onfound_tts` | Upon found = TTS | TTS phrase field visible |
| `seek.scenenav_off` | Scene nav unchecked (b/c) | Checkbox off (mode c: scan/found only, no drive) |
| `seek.limits` | Finite limits | Max steps + timeout inputs visible |
| `seek.dry_run` | Dry run default ON | Checkbox **Dry run (no drive)** checked |
| `seek.pano_empty` | Panorama empty | Hint “No scan yet…”; no broken image |
| `seek.actions` | Action bar | Seek / Stop / Check once (sticky on phone) |
| `seek.running_chrome` | Simulated running chrome | Catalog-only (no motors): pill **Seek running · 3/30**; `body.seek-running`; config card `is-locked` + disabled mode/goal/on-found/scene-nav/limits; Start/Check disabled. Live run also updates phase via poll. |

**Operator notes**
- Defaults are **finite** (30 steps / 300s). Set 0 only for unlimited.
- **Dry run (default ON):** camera sweep + decide only. No chassis. Uncheck and confirm to allow live drive. Battery gate does **not** apply in dry-run.
- **Battery gate (live only):** Seek will not start (and will halt if already running) when pack voltage is known and ≤ `UGV_BATTERY_LOW_V` (default 9.5 V). If `v` is missing or looks like raw ADC, Seek does **not** block. Override: `UGV_SEEK_BATTERY_GATE=0`.
- **Config is locked while Seek is running** (mode radios, goal, upon-found, scene nav, max steps, timeout). Card tooltip: “Config locked while Seek is running — Stop to edit”. Use **Stop** or navbar **STOP** to unlock and edit.
- Leaving Seek while running prompts stop confirm.
- Prefer mode **a** or **b** with a VOC class (`person`, `chair`, …) for detector demos; mode **c** for a free-text goal.
- **Mode a** is detector + L/C/R heuristic nav (no LLM). **Mode b** is detector found + front-first LLM hops. **Mode c** is LLM found + the same hops. Uncheck **LLM Scene Navigation** on b/c to scan without driving.
- **This chassis:** Seek turns are **UI-Fast T:1** spins (soft twist yaw does nothing). Soft linear stalls on thick floor transitions; hops are punchy (~0.22–0.28). short/medium/long ≈ 0.85s / 1.1s / 1.6s. Restart the Flask app after changing hop tables.
- **Indoor helpers (mode a):** drive *away* from walls when the planner is heuristic; after a doorway-like chute take another hop so the body is past the jambs. Cruise tilt ≈ −12°. PTZ from `/api/ptz` or Seek updates the Raw HUD needles (last command, not HW encoder).

---

## 4b. Track mode (experimental)

PTZ-only hunt: the camera sweeps; the chassis does not drive. A VOC class uses MobileNet-SSD and tries to centre the bbox. Any other goal string uses the vision LLM and locks the current pose (no bbox). Navbar **STOP** cancels Track. Experimental — not a walk-away tracker.

---

## 5. AI Agent (`/ai`)

| guide_id | Feature | Expected UI |
|----------|---------|-------------|
| `ai.overview` | Full agent page | Config sidebar, live camera, still, chat, tools |
| `ai.still_empty` | Empty still | Placeholder, not broken icon |
| `ai.config_panel` | Server config | Readonly base URL / model / key mask |
| `ai.tools_tree` | Capability chips | Motion tools off by default (dashed/off chips) |
| `ai.landscape` | Phone landscape | Config hidden until Config toggle; compact composer |

**Operator notes**
- Enable **stop_motors** before drive tools.
- Set main UI **HB off** for timed AI drives; use navbar **STOP** to halt.

---

## 6. 3D Twin

| guide_id | Feature | Expected UI |
|----------|---------|-------------|
| `twin.page` | Full page `/3d` | Hangar-true chassis + RoArm-M2 (Waveshare L1/L2/L3 FK) or PT gimbal; HUD joints |
| `twin.drawer` | In-page drawer | Open from Twin chip; same `/3d` model; Back/full tab link |
| `twin.hangar` | Raw RoArm overlay | Same twin as `/3d` (not a separate “workspace approx”) |

---

## 7. Settings / media

| guide_id | Feature | Expected UI |
|----------|---------|-------------|
| `settings.overview` | Servo init wizard | Steps 1–8; Set Pan ID / Release / Middle Set / Set Tilt ID |
| `photo.overview` | Photo gallery page | Photo list/grid |
| `video.overview` | Video files page | Video list |

**Operator notes**
- Settings page is **first-time pan-tilt servo ID** setup, not app preferences.

---

## 8. Out of scope for automated UI shots

- Actually driving motors / starting long Seek runs (safety)
- Confirming ESP32 WiFi stop (destructive session action)
- ROS stack restart (Docker)
- JupyterLab external link
- Live LLM replies (variable latency)
- Flipping **Direct ↔ ROS2** path during catalog (UART thrash) — `shell.path_direct` / `shell.path_ros2` are **manual / env-dependent**

These appear in the guide as text only; catalog captures **pre-confirm** UI where relevant.
