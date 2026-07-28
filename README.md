![GitHub top language](https://img.shields.io/github/languages/top/effectsmachine/ugv_rpi) ![GitHub language count](https://img.shields.io/github/languages/count/effectsmachine/ugv_rpi)
![GitHub code size in bytes](https://img.shields.io/github/languages/code-size/effectsmachine/ugv_rpi)
![GitHub repo size](https://img.shields.io/github/repo-size/effectsmachine/ugv_rpi) ![GitHub](https://img.shields.io/github/license/effectsmachine/ugv_rpi) ![GitHub last commit](https://img.shields.io/github/last-commit/effectsmachine/ugv_rpi)

# Waveshare UGV Robots
This is a Raspberry Pi example for the [Waveshare](https://www.waveshare.com/) UGV robots: **WAVE ROVER**, **UGV Rover**, **UGV Beast**, **RaspRover**, **UGV01**, **UGV02**.  

![](./media/UGV-Rover-details-23.jpg)

> **Fork:** [matthewhand/ugv_rpi](https://github.com/matthewhand/ugv_rpi) (also [effectsmachine/ugv_rpi](https://github.com/effectsmachine/ugv_rpi)) — based on [waveshareteam/ugv_rpi](https://github.com/waveshareteam/ugv_rpi).  
> The stock Waveshare app is still the base. This fork adds **Seek mode**, **ROS 2 control routing**, **ops tooling**, and UI polish. An honest status of that work is below.

## Fork modifications — status

### What we set out to improve
Make the stock dashboard usable for **autonomous “find a thing” pilots** (Seek), safer **AI/ROS motion**, clearer **operator feedback** (logs, pan aim, 3D twin), and less friction when switching **Direct serial ↔ ROS 2**.

### Honest assessment
This fork is a **supervised pilot**, not a finished product.

- **Seek** works for demos with a human on **STOP**, known VOC class (or LLM goal), and **Direct** control preferred. Scene nav often falls back to **camera heuristics** when the LLM is slow or fails. Obstacles are **vision-only** (Canny / texture / bright-wall heuristics — not lidar). Nav hops are **open-loop ms tables**, not calibrated yaw. Exploration is a **dead-reckoning heading trail**, not a map.
- **ROS 2** works when rosbridge (`:9090`) + `ugv_bringup` are healthy. If rosbridge dies after mode toggle, a **background autoheal** retries sidecars and **falls back to serial** so PTZ/drive are not left on “dropped serial cmd”. Autoheal is best-effort (Docker/`docker exec`); it is not a full container lifecycle manager. Wheel **odometry may still read zeros** without encoder ticks.
- **Drive polarity** is chassis-specific: `base_config.drive_linear_sign` (default on this tree **`-1`**) maps body “forward” to hardware. Confirm with `GET /api/status` → `drive_linear_sign` after restart. Wrong sign = UI forward drives camera-backward.
- **3D Twin** is a lightweight demo (box model, CDN Three.js when online) — not a high-fidelity digital twin.
- Secrets stay in **gitignored `.env`**. Do not commit camera dumps or keys.

### Achieved (exists in tree — pilot-grade unless noted)

- [x] **Seek** Raw/Chat/Seek shell: detector and/or LLM referee, TTS on found, finite default max steps/timeout (unlimited only if set to 0)  
- [x] **Triple-view pan** L/C/R ≈ ±135°; wait-for-pan with **blind settle** if HW pan feedback is stuck; three-panel stitch for LLM (not a true geometric 360°)  
- [x] **Nav tiers** short/medium/long — **open-loop** duration/speed tables (not measured calibration)  
- [x] **Safety bias** (image heuristics): avoid forward when centre looks “near”; prefer turn-to-open; reverse capped short  
- [x] **Corridor / blank-wall heuristics** (incl. bright painted walls) — still false open/closed under some lighting  
- [x] **Exploration trail** for LLM prompt (heading counters only)  
- [x] **Pan overlay** (`cam_aim`) with timed needle when HW pan sticks near 0°  
- [x] **Seek UI**: status, log, pano card, mode-specific camera hints, config lock while running, sticky actions on small screens  
- [x] **Global STOP** + Seek-running pill; leave-Seek confirm/auto-stop; keyboard teleop gated in Chat/Seek  
- [x] **Control: Direct ↔ ROS 2** chips; UART release/reclaim; **serial fallback** if rosbridge is down; **periodic rosbridge autoheal** when in ROS 2 (`UGV_ROS_AUTOHEAL`, default on)  
- [x] **Drive signs** in `config.yaml` / env; exposed on `/api/status`  
- [x] **AI agent** (`/ai`) + thinner Chat tab; motion caps default off; motion lock vs **Idle heartbeat** (navbar **HB**); Chat has fail/retry on live stream  
- [x] **Ops log** drawer; **3D Twin** popup drawer  
- [x] **Serial null guards**; session ESP32 WiFi stop (T:408, non-persistent)  
- [x] **TTS** hardened if `pyttsx3` missing  
- [x] **Operator guide** + Playwright `--catalog` screenshots (`docs/operator-guide.md`, `scripts/screenshot_ui.py`)  
- [x] **`.env` / capabilities / control_mode** gitignored  

### Remaining (todo)

- [ ] **Reliable scene LLM** under load; less heuristic fallback  
- [ ] **True localization** / map exploration (not heading-only trail)  
- [ ] **Encoder odom** + **lidar-aware** obstacles when hardware present  
- [ ] **Turn / pan closed-loop** feedback (not open-loop tables / synthetic pan)  
- [ ] **ROS exit**: kill bringup when switching to Direct (autoheal re-releases UART when bridge is up; clean stop on exit still open)  
- [ ] **Container boot**: always start rosbridge+bringup with `ugv_ros2`, not only on UI toggle / autoheal  
- [ ] **Seek product polish**: battery gate, look-map UI, richer cancel mid-drive  
- [ ] **Security defaults** (hotspot / Jupyter token)  
- [ ] **CI**: nav-plan safety tests, control_mode/autostart mocks  
- [ ] **Chat ↔ `/ai` unification**; Twin offline CDN / real meshes  
- [ ] **Docs**: expand operator-guide into a single printed runbook (guide exists; README historical sections below may lag labels)

### Quick operator notes

| Control | Use for |
|---------|---------|
| **Raw** | Manual sticks, stock CV (**AUTODRIVE** = line-follow only) |
| **Chat / AI** | Vision chat; enable motion tools on `/ai` first; set **HB off** for timed AI drives; use navbar **STOP** to halt |
| **Seek** | Find-goal loop; prefer Direct; hand on **STOP**; default 30 steps / 300 s |
| **Direct** | Flask owns UART — best default for PTZ + Seek demos |
| **ROS 2** | Needs healthy rosbridge; autoheal retries; serial fallback if bridge dies |
| **HB (Idle heartbeat)** | ON = re-send last wheel cmd every 2s (idle → stop). OFF for long AI timed drives |
| **Drive sign** | After config change, **restart app** and check `/api/status` → `drive_linear_sign` |

Detailed historical notes follow (some button names in older sections may still say “Motors” / “Freq. stop” — UI chrome uses **Control** / **HB** / **Idle heartbeat**).

## Changes from upstream

### Serial port robustness (`base_ctrl.py`)

- `serial.Serial()` init wrapped in try/except — app boots cleanly when `/dev/serial0` is absent or locked (e.g. ROS 2 already owns it)
- `send_command` and `process_commands` guard against `self.ser = None` — no crash when serial unavailable
- `feedback_data` and `on_data_received` return early / skip reset if serial is None
- `ReadLine` returns empty bytes immediately when `s` is None rather than blocking

### ROS 2 control routing (`base_ctrl.py`, `app.py`, `templates/`)

Unified **Control: Direct serial | ROS 2 relay** (navbar chips / `POST /api/control_mode`, `GET /api/status`).

- **Direct:** Flask opens UART; chassis + PTZ serial JSON go to the ESP32.  
- **ROS 2:** Flask **releases** UART for `ugv_bringup`; stick/AI motion prefer **rosbridge** (`ws://127.0.0.1:9090` by default).  
- If rosbridge is **down**: commands **fall back to serial** after reclaiming UART (so PTZ is not left on “Dropped serial cmd”).  
- While mode stays **ROS 2**, a **background autoheal** thread (`UGV_ROS_AUTOHEAL=1`, interval `UGV_ROS_AUTOHEAL_S` ≈15s) retries rosbridge/bringup and re-releases UART when the bridge recovers.  
- Legacy `enable_motor_control=False` still bypasses chassis-only on serial when the port is open; full ROS mode uses release + relay as above.  
- Env: `UGV_MOTOR_BYPASS=1` at process start (dev); `UGV_AUTOSTART_ROSBRIDGE` / `UGV_AUTOSTART_BRINGUP` (default on).

Gimbal kits: `base_config.module_type: 2` in `config.yaml`.

### Pan/tilt via ROS 2 (optional)

When control mode is ROS 2 and rosbridge is up, UI `T:133` is published over rosbridge (joint_states / PT controller topics → bringup → ESP32). If rosbridge is down, Flask uses **serial fallback** after reclaim (see above).

### AI vision agent (`/ai`)

OpenAI-compatible chat UI at `/ai` with optional live camera still attach.

- **Env:** `OPENAI_API_KEY` (required), optional `OPENAI_BASE_URL` and `OPENAI_MODEL` (defaults: official OpenAI + `gpt-4o-mini`)
- **Auth (optional):** when `UGV_AI_TOKEN` is set, `/api/ai/*` and `/api/snapshot` require that token
- **Capabilities:** toggle tree persisted in `.ai_capabilities.json` — motion tools default **off** for safety; enable in the AI UI before the LLM can drive
- **Tools:** telemetry, CV detections, snapshot metadata, and motion (direct serial or ROS 2 per `control_mode`)
- **Vision:** attach checkbox sends a live JPEG on that turn; chat history stays text-only (no image re-send)

### Drive safety / conventions

- Body-frame positive linear = **camera-forward** in software. Hardware polarity is applied once via `drive_linear_sign` / `drive_angular_sign` in `config.yaml` (or `UGV_DRIVE_LINEAR_SIGN` / `UGV_DRIVE_ANGULAR_SIGN`). **This tree defaults linear to `-1`** for the common Waveshare chassis wiring — verify after any change with `GET /api/status` and a short stick test (restart the app after editing yaml).
- AI chassis drives are **timed by default**; continuous motion requires an explicit continuous flag.
- Motion tools must be enabled in the AI capability UI before the LLM can call them.
- **Idle heartbeat (navbar HB):** every 2s the UI re-sends the last wheel command (idle = L0/R0 stop). Leave **ON** for manual teleop. Set **HB off** for AI timed drives so the heartbeat does not cut them short. The server **AI motion lock** ignores those idle zeros during timed/continuous AI moves; navbar **STOP** always clears the lock and zeros chassis.

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
