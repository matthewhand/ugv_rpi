# ugv_rpi (fork)

![GitHub last commit](https://img.shields.io/github/last-commit/matthewhand/ugv_rpi) ![GitHub](https://img.shields.io/github/license/matthewhand/ugv_rpi)

A **supervised-pilot** fork of [waveshareteam/ugv_rpi](https://github.com/waveshareteam/ugv_rpi) — the Raspberry Pi Flask web app for Waveshare UGV robots (WAVE ROVER, UGV Rover, UGV Beast, RaspRover, UGV01, UGV02).

The stock dashboard, Jupyter tutorials, and OpenCV toys are still here. This tree adds **Seek** (a supervised scan-and-hop loop), experimental **Track** (PTZ-only hunt), a **Chat / `/ai` agent**, **Direct ↔ ROS 2** UART routing, and operator chrome (global **STOP**, ops log, PTZ HUD).

**This is not a finished product.** Keep a hand on **STOP**.

| | |
|---|---|
| **This repo** | [matthewhand/ugv_rpi](https://github.com/matthewhand/ugv_rpi) |
| **Upstream** | [waveshareteam/ugv_rpi](https://github.com/waveshareteam/ugv_rpi) |
| **UI** | `http://<pi>:5000` — tabs **Raw · Chat · Seek · Track · Loadout** |
| **Full agent** | `http://<pi>:5000/ai` |
| **Operator guide** | [docs/operator-guide.md](docs/operator-guide.md) |

The original Waveshare README (install, hotspot, robot type) is unchanged **below**.

---

## What this fork adds

### Modes

| Tab | What it does |
|-----|----------------|
| **Raw** | Stock teleop: sticks, PTZ, OpenCV. **AUTODRIVE** is line-follow only, not Seek. |
| **Chat** | Thin vision chat against `/api/ai/chat`. Enable motion tools on `/ai` first. |
| **Seek** | Finite scan-and-hop loop: L/C/R stills, a referee, then a timed hop / turn / reverse. Optional TTS if the referee says found. |
| **Track** | Experimental. Camera hunts only — **wheels do not move**. Closed-list VOC labels use MobileNet-SSD and try to centre the bbox. Any other goal string uses the vision LLM and locks the current PTZ pose. |
| **Loadout** | Chassis + attachment profile (rover/beast × none/ptz/roarm2). Runtime `.loadout.json` via `/api/loadout`. Examples: [docs/CONFIG_PROFILES.md](docs/CONFIG_PROFILES.md). USB RoArm starts **only** when hangar attachment=`roarm2`; rover+PTZ keeps `roarm_started=false`. |

Navbar **STOP** zeros wheels, cancels Seek **and** Track, and clears the AI motion lock.

### Seek (scan, then move)

- **Modes:** `a` detector + L/C/R heuristic nav (no LLM) · `b` detector found + front-first LLM hops (default) · `c` LLM found + the same hops (free-text goals). Uncheck **LLM Scene Navigation** on b/c to scan without driving.
- **LLM nav (b/c):** each step one front JPEG with forced JSON (`clear_forward_little` / `clear_forward_lot` / `subject_in_scene`). Sides (±135° then taper) only if front is blocked; chassis turns only after the gimbal recentres. Optional **multi-image** checkbox sends all three views in one call. Timeout or schema reject → **heuristic fallback** (8s, no long retry). Detector FOUND is a flat **0.85** on every class.
- **This chassis:** `T:13` twist yaw does **not** turn the rover. Seek uses **UI-Fast `T:1` tank spins**. Soft linear (~0.12) stalls on thick floor transitions; hops are punchy (~0.22–0.28). Open-loop tables, timed on *this* rover 2026-08-13: short / medium / long ≈ 0.85s / 1.1s / 1.6s; turns ≈ 350 / 700 / 1100 ms. Restart Flask after changing hop tables. Another floor will need retuning.
- **Mode a indoor helpers:** cruise tilt ≈ −12°. Doorway-commit and turn-away-from-wall run on the **heuristic** planner only.
- **Battery gate:** live Seek refuses start / halt if pack V is known and ≤ `UGV_BATTERY_LOW_V` (default **9.5**). **Dry-run skips this gate.** Missing or ADC-looking `v` does **not** block. Override: `UGV_SEEK_BATTERY_GATE=0`.
- **Limits:** default 30 steps / 300 s (0 = unlimited). Config locks while running.
- **Dry run (default ON):** PTZ still sweeps; **wheels never move**. Logs say `WOULD drive`. Uncheck and confirm only for a live test. API live start requires `confirm_live=true` (the UI sends that after the confirm dialog). Process latch also drops T:1 / T:13 sticks while Seek or Track owns the chassis.
- **Obstacles** are vision heuristics (Canny / texture / bright walls) — **not lidar**. Exploration is a heading trail, not a map.

### PTZ HUD

Commanded pan/tilt from `/api/ptz`, `T:133`, Seek, Track, and Chat gimbal go through `_publish_ptz_aim`. `/api/status.ptz` and a Socket.IO `ptz_aim` event update the Raw needles and Seek compass. Hardware pan feedback on this kit often sticks near 0°, so the HUD is **last command + a settle timer**, not a closed-loop encoder. Hard-refresh the UI after deploying this.

### Chat / `/ai`

- OpenAI-compatible (`OPENAI_BASE_URL` + `OPENAI_MODEL`; key from `.env`).
- Chat completion budget defaults to **8192** tokens (`UGV_CHAT_MAX_TOKENS`) because **512 truncated tool JSON**. Seek nav forced-JSON calls wait 8s, then heuristic.
- **Grab still** is the JPEG that Send attaches (not a new live grab) when the checkbox is on. If nothing has been grabbed, Send captures live.
- Motion tools default **off**. Enable on `/ai`. Set navbar **HB off** for timed AI drives. Seek does **not** persist those tools on.
- Chat voice (Off / Browser / Robot): `UGV_STT_URL` / `UGV_TTS_URL`.

### Direct serial ↔ ROS 2

Navbar chips. **Direct** (default for Seek / PTZ): Flask owns UART. **ROS 2**: Flask releases UART for the chassis driver (`ugv_driver_min` preferred, else `ugv_bringup`; no `ugv_ws` required), sticks go via rosbridge (`ws://127.0.0.1:9090`). Leaving ROS 2 **stops the chassis driver** in the container (`UGV_AUTOSTOP_BRINGUP`, default on) then reclaims UART. If rosbridge dies, serial fallback + background autoheal (`UGV_ROS_AUTOHEAL`).

### Other chrome

- Seek / Track logs are a **fixed height** with a scrollbar.
- **Idle heartbeat (HB):** every 2s the UI re-sends the last wheel cmd (idle → stop). ON for sticks; OFF for timed AI drives.
- Drive polarity: `base_config.drive_linear_sign` (this tree **`-1`**). After a yaml change, **restart the app** and check `GET /api/status`.
- **Loadout** tab / `GET|POST /api/loadout`: chassis + attachment; live `camera_prefer` re-init; RoArm gated on `roarm2`. Examples `config.rover.yaml` / `config.beast.yaml` — live file stays `config.yaml`. [docs/CONFIG_PROFILES.md](docs/CONFIG_PROFILES.md).
- Ops log drawer, 3D Twin (box + CDN Three.js — not a digital twin), session-only ESP32 WiFi stop.
- Unit tests: `tests/test_seek_nav.py`, `tests/test_ai_track.py`, `tests/test_loadout.py`, `tests/test_dual_hw_gating.py` (planner / referee / hangar gates — not a live find-rate).

---

## Honest limits

- Seek is a **supervised loop**, not a product finder. With dry-run off it will scan and move. It will also miss the goal, call a bright wall “open”, or drive into furniture. Keep a hand on **STOP**.
- Live indoor runs on this tree have **not** shown a reliable “found” rate. The detector often stays quiet **or** fires junk (`aeroplane` / `train` on a window; `person` at ~0.3 may be real or a cushion). The LLM on the configured compatible backend has been **timing out**; we navigate on heuristics.
- Mixed OpenAI-compatible load balancers can be slow or empty. Seek nav waits **8s**, then heuristic. It does **not** sit through a second long retry.
- Hops and turns are **open-loop time tables**, not encoder / IMU closed loop.
- No map, no lidar, no true localization.
- Track is experimental and PTZ-only. Treat it as a hunt/centre sketch, not a tracker you can walk away from.
- ROS autoheal is best-effort `docker exec`, not a full container lifecycle. Wheel odom may still read zeros without encoder ticks.
- Secrets stay in **gitignored `.env`**. Do not commit keys or camera dumps.

### Still todo

- [ ] Reliable scene LLM under load; less heuristic fallback
- [ ] Localization / map; encoder odom; lidar when present
- [ ] Closed-loop turn / pan (not open-loop tables / synthetic pan)
- [ ] Always boot rosbridge + bringup with the ROS container
- [ ] Seek look-map UI; richer cancel mid-drive
- [ ] Security defaults (hotspot / Jupyter token)
- [ ] CI for control_mode / autostart; Chat ↔ `/ai` unification; Twin offline meshes

---

## Env (no secrets in git)

Copy to `ugv_rpi/.env` (already gitignored):

| Variable | Role |
|----------|------|
| `OPENAI_API_KEY` | Required for Chat / Seek LLM / Track unique goals |
| `OPENAI_BASE_URL` | Optional. Default official OpenAI. Set to your OpenAI-compatible `/v1` |
| `OPENAI_MODEL` | Optional. Default `gpt-4o-mini` |
| `UGV_CHAT_MAX_TOKENS` | Chat / agent completion budget (default **8192**, clamp 1024–32768) |
| `UGV_AI_TOKEN` | If set, `/api/ai/*` and `/api/snapshot` require it |
| `UGV_STT_URL` / `UGV_TTS_URL` | Optional Chat voice (empty = voice disabled) |
| `UGV_BATTERY_LOW_V` / `UGV_SEEK_BATTERY_GATE` | Seek pack-voltage gate |
| `UGV_AUTOSTOP_BRINGUP` / `UGV_ROS_AUTOHEAL` | ROS 2 exit / retry |

---

## Quick operator notes

| Control | Use for |
|---------|---------|
| **Raw** | Manual sticks, stock CV |
| **Chat / AI** | Vision chat; enable motion on `/ai`; **HB off** for timed drives |
| **Seek** | Scan-and-hop loop; **dry-run ON by default** (no drive). Prefer **Direct**; hand on **STOP**. Live mode skips start if pack V is known-low |
| **Track** | PTZ hunt only; chassis stays still |
| **Direct** | Flask owns UART — best for PTZ + Seek |
| **ROS 2** | Needs healthy rosbridge; autoheal + serial fallback |
| **HB** | ON = resend last wheel cmd every 2s. OFF for long AI timed drives |
| **Drive sign** | Restart app after config change; check `/api/status` → `drive_linear_sign` |

More detail: [docs/operator-guide.md](docs/operator-guide.md). Implementation notes immediately below. Stock Waveshare install / hotspot / robot-type docs follow that.

## Implementation notes (this fork)

### Serial port robustness (`base_ctrl.py`)

- `serial.Serial()` init wrapped in try/except — app boots when `/dev/serial0` is absent or locked (e.g. ROS 2 already owns it)
- `send_command` and `process_commands` guard against `self.ser = None`
- `feedback_data` / `on_data_received` return early if serial is None
- `ReadLine` returns empty bytes immediately when `s` is None

### ROS 2 control routing (`base_ctrl.py`, `app.py`, `templates/`)

Unified **Control: Direct serial | ROS 2 relay** (`POST /api/control_mode`, `GET /api/status`).

- **Direct:** Flask opens UART; chassis + PTZ JSON go to the ESP32.
- **ROS 2:** Flask **releases** UART for `ugv_bringup`; stick/AI motion prefer **rosbridge**.
- **Leaving ROS 2:** Flask **stops `ugv_bringup`** (container PID kill; `UGV_AUTOSTOP_BRINGUP=0` to skip) then **reclaims** UART. rosbridge is left running.
- If rosbridge is **down**: commands **fall back to serial** after reclaim (so PTZ is not left on “Dropped serial cmd”).
- While mode stays **ROS 2**, a **background autoheal** thread (`UGV_ROS_AUTOHEAL=1`, interval `UGV_ROS_AUTOHEAL_S` ≈15s) retries sidecars and re-releases UART when the bridge recovers.
- Env: `UGV_MOTOR_BYPASS=1` at process start (dev); `UGV_AUTOSTART_ROSBRIDGE` / `UGV_AUTOSTART_BRINGUP` / `UGV_AUTOSTOP_BRINGUP` (default on).

Gimbal kits: `base_config.module_type: 2` in `config.yaml`. When ROS 2 + rosbridge are up, UI `T:133` is published over rosbridge; otherwise serial fallback after reclaim.

### Drive safety

- Body-frame positive linear = **camera-forward** in software. Hardware polarity is `drive_linear_sign` / `drive_angular_sign` in `config.yaml` (or `UGV_DRIVE_LINEAR_SIGN` / `UGV_DRIVE_ANGULAR_SIGN`). **This tree defaults linear to `-1`**.
- AI chassis drives are **timed by default**; continuous motion needs an explicit flag. Motion tools must be enabled on `/ai` first.
- **Idle heartbeat (navbar HB):** every 2s the UI re-sends the last wheel command (idle = L0/R0). Leave **ON** for manual teleop. **HB off** for AI timed drives so the heartbeat does not cut them short. Navbar **STOP** always clears the lock and zeros chassis.

---

# Waveshare UGV Robots

This is a Raspberry Pi example for the [Waveshare](https://www.waveshare.com/) UGV robots: **WAVE ROVER**, **UGV Rover**, **UGV Beast**, **RaspRover**, **UGV01**, **UGV02**.

![](./media/UGV-Rover-details-23.jpg)

> Upstream README from [waveshareteam/ugv_rpi](https://github.com/waveshareteam/ugv_rpi). Clone this fork instead if you want Seek / Track / Chat: `git clone https://github.com/matthewhand/ugv_rpi.git`

## Basic Description
The Waveshare UGV robots utilize both an upper computer and a lower computer. This repository contains the program running on the upper computer, which is typically a Raspberry Pi in this setup.  

The program running on the lower computer is either named [ugv_base_ros](https://github.com/effectsmachine/ugv_base_ros.git) or [ugv_base_general](https://github.com/effectsmachine/ugv_base_general.git) depending on the type of robot driver being used.  

The upper computer communicates with the lower computer (the robot's driver based on ESP32) by sending JSON commands via GPIO UART. The host controller, which employs a Raspberry Pi, handles AI vision and strategy planning, while the sub-controller, utilizing an ESP32, manages motion control and sensor data processing. This setup ensures efficient collaboration and enhanced performance.

## Features
- Real-time video based on WebRTC
- Interactive tutorial based on JupyterLab
- Pan-tilt camera control
- Robotic arm control
- Cross-platform web application base on Flask
- Auto targeting (OpenCV)
- Object Recognition (OpenCV)
- Gesture Recognition (MediaPipe)
- Face detection (OpenCV & MediaPipe)
- Motion detection (OpenCV)
- Line tracking base on vision (OpenCV)
- Color Recognition (OpenCV)
- Multi-threaded CV processing
- Audio interactive
- Shortcut key control
- Photo taking
- Video Recording

## Quick Install
You need to install Raspberry Pi on your robot if you are using **WAVE ROVER**, **UGV01** or **UGV02**.  

This app is already installed in the SD card of **UGV Rover**, **UGV Beast** and **RaspRover**.  

You can use this tutorial to upgrade your robot's upper computer program.  

You can use this tutorial to install this program on a pure Raspberry Pi OS.  


### Download the repo from github

You can clone this repository from Waveshare's GitHub to your local machine.

    git clone https://github.com/waveshareteam/ugv_rpi.git
    
### Grant execution permission to the installation script
    cd ugv_rpi/
    sudo chmod +x setup.sh
    sudo chmod +x autorun.sh
### Install app (it'll take a while before finish)
    sudo ./setup.sh
### Autorun setup
    ./autorun.sh
### AccessPopup installation
    cd AccessPopup
    sudo chmod +x installconfig.sh
    sudo ./installconfig.sh
    *Input 1: Install AccessPopup
    *Press any key to exit
    *Input 9: Exit installconfig.sh
### Reboot Device
    sudo reboot

After powering on the robot, the Raspberry Pi will automatically establish a hotspot, and the LED screen will display a series of system initialization messages:  

![](./media/RaspRover-LED-screen.png)
- The first line `E` displays the IP address of the Ethernet port, which allows remote access to the Raspberry Pi. If it shows No Ethernet, it indicates that the Raspberry Pi is not connected to an Ethernet cable.
- The second line `W` indicates the robot's wireless mode. In Access Point (AP) mode, the robot automatically sets up a hotspot with the default IP address `192.168.50.5`. In Station (STA) mode, the Raspberry Pi connects to a known WiFi network and displays the IP address for remote access.
- The third line `F/J` specifies the Ethernet port numbers. Port `5000` provides access to the robot control Web UI, while port `8888` grants access to the JupyterLab interface.
- The fourth line `STA` indicates that the WiFi is in Station (STA) mode. The time value represents the duration of robot usage. The dBm value indicates the signal strength RSSI in STA mode.  


You can access the robot web app using a mobile phone or PC. Simply open your browser and enter `[IP]:5000` (for example, `192.168.10.50:5000`) in the URL bar to control the robot.  

To access JupyterLab, use `[IP]:8888` (for example, `192.168.10.50:8888`).  

If the robot is not connected to a known WiFi network, it will automatically set up a hotspot named "`AccessPopup`" with the password `1234567890`. You can then use a mobile phone or PC to connect to this hotspot. Once connected, open your browser and enter `192.168.50.5:5000` in the URL bar to control the robot.  

To ensure compatibility with various types of robots running on Raspberry Pi, we utilize a config.yaml file to specify the particular robot being used. You can configure the robot by entering the following command:

    s 22

In this command, the s directive denotes a robot-type setting. The first digit, `2`, signifies that the robot is a `UGV Rover`, with `1` representing `RaspRover` and `3` indicating `UGV Beast`. The second digit, also `2`, specifies the module as `Camera PT`, where `0` denotes `Nothing` and `1` signifies `RoArm-M2`.  

### Reboot Device
If the program fails to run and encounters errors related to v4l2.py during runtime, you need to delete v4l2.py from both the Python virtual environment and the user environment. This will allow the program to automatically use the system-wide v4l2.py.  

    cd ugv_rpi/  
    sudo rm ugv-env/lib/python3.11/site-packages/v4l2.py  
    sudo rm /home/[your_user_name]/.local/lib/python3.11/site-packages/v4l2.py  

Now you can restart the main program app.py.

# License
ugv_rpi for the Raspberry Pi: an open source robotics platform for the Raspberry Pi.
Copyright (C) 2024 [Waveshare](https://www.waveshare.com/)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/gpl-3.0.txt>.
