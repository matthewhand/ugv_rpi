![GitHub top language](https://img.shields.io/github/languages/top/effectsmachine/ugv_rpi) ![GitHub language count](https://img.shields.io/github/languages/count/effectsmachine/ugv_rpi)
![GitHub code size in bytes](https://img.shields.io/github/languages/code-size/effectsmachine/ugv_rpi)
![GitHub repo size](https://img.shields.io/github/repo-size/effectsmachine/ugv_rpi) ![GitHub](https://img.shields.io/github/license/effectsmachine/ugv_rpi) ![GitHub last commit](https://img.shields.io/github/last-commit/effectsmachine/ugv_rpi)

# Waveshare UGV Robots
This is a Raspberry Pi example for the [Waveshare](https://www.waveshare.com/) UGV robots: **WAVE ROVER**, **UGV Rover**, **UGV Beast**, **RaspRover**, **UGV01**, **UGV02**.  

![](./media/UGV-Rover-details-23.jpg)

> **Fork:** [matthewhand/ugv_rpi](https://github.com/matthewhand/ugv_rpi) (also [effectsmachine/ugv_rpi](https://github.com/effectsmachine/ugv_rpi)) — based on [waveshareteam/ugv_rpi](https://github.com/waveshareteam/ugv_rpi).  
> The stock Waveshare app is still the base. This fork adds **Seek mode**, **ROS 2 control routing**, **ops tooling**, and UI polish. An honest status of that work is below.

**Design authority for this Beast line:** [docs/BEAST_TECHNICAL_DESIGN.md](docs/BEAST_TECHNICAL_DESIGN.md) (TOC, diagrams, transfer plan). Older notes: [docs/CONFIG_PROFILES.md](docs/CONFIG_PROFILES.md), [docs/ROARM.md](docs/ROARM.md). The July merge checklist is historical: [docs/MERGE_PTZ_AND_ROARM_PLAN.md](docs/MERGE_PTZ_AND_ROARM_PLAN.md).

## Fork modifications — status

### What we set out to improve
Make the stock dashboard usable for **autonomous “find a thing” pilots** (Seek), safer **AI/ROS motion**, clearer **operator feedback** (logs, pan aim, 3D twin), and less friction when switching **Direct serial ↔ ROS 2**.

### Honest assessment
Seek is **usable as a pilot**, not a finished robot product. OpenAI-compatible scene nav often falls back to **vision heuristics** when the local LLM is slow or fails. Obstacle handling is **camera-only** (no lidar fusion on this stack by default). Recovery prefers **short turns** over long reverse after hard-wall lessons. Exploration uses a **dead-reckoning trail + heading**, not a real map. ROS 2 mode **does** drive chassis and PTZ when rosbridge + `ugv_bringup` are up (auto-started on mode toggle when Docker is available); wheel odometry may still report zeros if the ESP32 stream lacks encoder ticks. Secrets stay in **gitignored `.env`** — do not commit camera dumps or keys.

### Achieved

- [x] **Seek mode** in the multi-mode shell (Raw / Chat / Seek): goal text, detector and/or LLM referee, optional TTS on found  
- [x] **Triple-view pan** (L / centre / R ≈ ±135°) with wait-for-pan before shutter; near-360 stitch for LLM  
- [x] **Nav tiers** short/medium/long for drives and turns; Fast T:1 tank turns calibrated smaller to reduce overshoot  
- [x] **Safety / recovery**: forbids forward into “near” centre; turn-to-open preferred; reverse capped short (avoid rear walls)  
- [x] **Corridor bias**: longer hops when centre looks open; aim short turn toward emptier lane in FOV  
- [x] **Exploration trail**: record moves + rough heading; leave overused headings; inject trail into LLM prompt  
- [x] **Pan overlay** (`cam_aim` SoT): live needle with timed animation when HW pan feedback sticks near 0°  
- [x] **Seek UI**: live status, event log, panorama card, image reload/retry, OLED 3-line seek context where available  
- [x] **Control mode Direct ↔ ROS 2**: UART release/reclaim; body→hardware drive signs configurable  
- [x] **On ROS mode toggle**: auto-start **rosbridge** (`:9090`) and **ugv_bringup** in `ugv_ros2` when missing (env-gated)  
- [x] **AI agent** (`/ai`): OpenAI-compatible tools, capability tree, timed drives, motion lock vs UI “Freq. stop”  
- [x] **Ops log** pane: ring-buffer server log for control mode, WiFi session stop, AI motion, errors  
- [x] **3D Twin** as resizable in-page popup (iframe of `/3d`), not only a full-page tab  
- [x] **Serial robustness**: open/close guards when ROS owns UART; session ESP32 WiFi stop (non-persistent T:408)  
- [x] **TTS path** hardened when `pyttsx3` missing; on-found speech optional  
- [x] **`.env` / capabilities / control_mode** gitignored; `ai_proof/` removed and ignored (no camera dumps in git)
- [x] **Dual robot profiles** *(in this tree)*: PTZ Seek robot (`module_type: 2`) and Beast USB RoArm (`arm_config.transport: usb_serial`)  
- [x] **USB RoArm** *(in this tree)*: `roarm_ctrl`, Aim: RoArm overlay, default **travel_tuck**; Seek look-around skips PTZ pans on Beast  
- [x] **USB camera rediscovery** when V4L index jumps after re-enumerate  

### Dual robot (PTZ vs Beast RoArm)

| Profile | Config | Camera look-around | Arm |
|---------|--------|--------------------|-----|
| **PTZ / Seek** | `module_type: 2`, no USB arm transport | Gimbal pan (`T:133`) | N/A |
| **Beast** | `module_type: 1`, `arm_config.transport: usb_serial` | Base-camera + limited arm peek; **no** PTZ pan | USB CP2102, `travel_tuck` default |

Details: [docs/CONFIG_PROFILES.md](docs/CONFIG_PROFILES.md), [docs/ROARM.md](docs/ROARM.md), [docs/MERGE_PTZ_AND_ROARM_PLAN.md](docs/MERGE_PTZ_AND_ROARM_PLAN.md). Hardware notes for this Pi: `~/beast-image/CUSTOM_BUILD.md`.

**Git honesty (2026-07-29):** Dual-robot behaviour above describes the **local working tree on this Beast** (`git log -1`). It is **not** fully published to `origin/main` yet (local tip diverged from origin; partial PR [dual-robot-beast-land](https://github.com/matthewhand/ugv_rpi/pull/1) must not be treated as complete until `app.py` on that branch includes the USB Seek gate). Always check `git status` / `git rev-parse HEAD` on the robot you are running.

### Remaining (todo)


- [ ] **Reliable scene LLM**: stable JSON nav under load; less heuristic fallback; better latency on local Ollama  
- [ ] **True localization**: fuse `/odom` (wheel + lidar EKF) when lidar present; explore by map cells, not heading-only trail  
- [ ] **Encoder odom on this firmware**: confirm `odl`/`odr` (or equivalent) so `/odom_wheel` moves when the robot does  
- [ ] **Lidar-aware seek**: use `/scan` for near obstacles instead of centre-image Canny only  
- [ ] **Stuck / contact detection**: better than identical-frame + edge density (IMU jolt, motor current, bumper)  
- [ ] **Turn calibration**: per-surface yaw feedback (odom/IMU) instead of open-loop ms tables  
- [ ] **Pan feedback**: trustworthy HW pan degrees (or closed-loop aim) without synthetic estimate  
- [ ] **ROS mode exit**: stop/kill bringup automatically when switching back to Direct so UART reclaim is clean  
- [ ] **Container entrypoint**: start rosbridge + bringup with `ugv_ros2` boot, not only on UI toggle  
- [ ] **Seek product polish**: better goal labels UX, cancel mid-pan, map of where it looked, battery gate before long runs  
- [ ] **Security defaults**: change stock hotspot passwords; don’t leave Jupyter with empty token on shared LANs  
- [ ] **Tests / CI**: unit coverage for nav plan safety, exploration trail, control_mode autostart mocks  
- [ ] **Docs**: single operator runbook (Direct vs ROS, Seek modes, Freq. stop, capability toggles)  
- [ ] **Publish dual-robot tip**: rebase/merge with `origin/main`, finish remote `app.py`/UI cores, push (no force); do not merge incomplete PR #1

### Quick operator notes

| Mode | Use for |
|------|---------|
| **Raw** | Manual sticks, stock CV buttons (**AUTODRIVE** = line-follow only, needs a floor line) |
| **Chat / AI** | Conversational tools; enable motion caps first; Freq. stop OFF for timed drives |
| **Seek** | “Find *goal*” loops: scan → decide → drive → re-scan |
| **Control: Direct** | Flask owns UART (default for simple seek) |
| **Control: ROS 2** | Flask → rosbridge → bringup; needs Docker + `ugv_ros2` |

Detailed historical notes follow.

## Changes from upstream

### Serial port robustness (`base_ctrl.py`)

- `serial.Serial()` init wrapped in try/except — app boots cleanly when `/dev/serial0` is absent or locked (e.g. ROS 2 already owns it)
- `send_command` and `process_commands` guard against `self.ser = None` — no crash when serial unavailable
- `feedback_data` and `on_data_received` return early / skip reset if serial is None
- `ReadLine` returns empty bytes immediately when `s` is None rather than blocking

### ROS 2 motor bypass mode (`base_ctrl.py`, `app.py`, `templates/`)

`BaseController` gains an `enable_motor_control` flag (default `True`). When set to `False` (ROS 2 chassis mode), only **wheel/chassis** commands are dropped (`T:1` differential drive, `T:13` X/Z velocity). **Pan/tilt gimbal** commands (`T:133`, `T:141`) still go out so the web UI stick can aim the camera while ROS owns driving.

Toggled at runtime — no restart required:

- **Web UI:** "Motors: Direct ON / ROS 2 Bypass" button in the dashboard
- **API:** `POST /api/toggle_motors`, `GET /api/status`
- **Env:** `UGV_MOTOR_BYPASS=1` at process start

Gimbal kits should set `base_config.module_type: 2` in `config.yaml` (0=None, 1=ARM, 2=Gimbal). Lights and other non-chassis serial commands always pass through.

### Pan/tilt via ROS 2 (optional)

With `UGV_MOTION_BACKEND=ros2` and `UGV_PT_BACKEND=auto` (default), the web stick’s `T:133` commands are **routed over rosbridge** when it is up:

- publishes `sensor_msgs/JointState` on `/joint_states` (names `pt_base_link_to_pt_link1`, `pt_link1_to_pt_link2`) for **`ugv_bringup`** → ESP32  
- also publishes `Float64MultiArray` on `/pt_joint_position_controller/commands` (joy/vision/gazebo path)

If rosbridge is down, Flask **falls back to serial `T:133`**. Force with `UGV_PT_BACKEND=serial` or `ros2`.

### AI vision agent (`/ai`)

OpenAI-compatible chat UI at `/ai` with optional live camera still attach.

- **Env:** `OPENAI_API_KEY` (required), optional `OPENAI_BASE_URL` and `OPENAI_MODEL` (defaults: official OpenAI + `gpt-4o-mini`)
- **Auth (optional):** when `UGV_AI_TOKEN` is set, `/api/ai/*` and `/api/snapshot` require that token
- **Capabilities:** toggle tree persisted in `.ai_capabilities.json` — motion tools default **off** for safety; enable in the AI UI before the LLM can drive
- **Tools:** telemetry, CV detections, snapshot metadata, and motion (direct serial or ROS 2 per `control_mode`)
- **Vision:** attach checkbox sends a live JPEG on that turn; chat history stays text-only (no image re-send)

### USB RoArm + UI Aim mode (`roarm_ctrl.py`, `app.py`, `templates/`)

**Other robots (gimbal / stock arm on base UART):** leave `arm_config` out of `config.yaml`, or set `arm_config.transport: base_uart`. Original pan/tilt stick path (`T:133`) is unchanged when `module_type: 2` or **Aim: PT**.

**This Beast (USB-C RoArm on CP2102):** set in `config.yaml`:

```yaml
arm_config:
  transport: usb_serial   # routes T:144 / arm JSON to USB, not ttyAMA0
  ui_aim_default: roarm   # stick drives RoArm position
```

- Web UI button **Aim: RoArm / Aim: PT** toggles overlay target (`POST /api/ui_aim_mode`).
- **Aim: RoArm** — video pan/tilt overlay stick sends stock `T:144` E/Z/R; Flask maps to USB RoArm joints (`T:102`). Scroll wheel still adjusts reach (E).
- **Aim: PT** — original gimbal logic (`T:133`) for pan/tilt kits / dual-mode use.
- Status: `GET /api/status` includes `ui_aim_mode`, `arm_transport`, `roarm.{connected,port,last_joints}`.
- Env override for port: `ROARM_SERIAL=/dev/serial/by-id/...`

#### RoArm + ROS 2 (`control_mode=ros2`)

Arm USB is **independent** of chassis `ttyAMA0`. Default is **hybrid** (UI always works):

| Mode | Chassis | Aim: RoArm path |
|------|---------|-----------------|
| `control_mode=direct` | Flask UART | Flask → CP2102 `T:102` |
| `control_mode=ros2` (default hybrid) | ROS `ugv_driver_min` on `ttyAMA0` | Publish `/ugv/roarm/joint_command` **and** Flask USB write; mirror `/ugv/roarm/joint_states` |
| Exclusive driver | same | `UGV_ROARM_USB_OWNER=driver` → Flask only publishes command; driver owns USB |

**Topics** (`sensor_msgs/JointState`, names `roarm_base/shoulder/elbow/hand`):

- `/ugv/roarm/joint_command` — targets (UI / AI / `ros2 topic pub`)
- `/ugv/roarm/joint_states` — state mirror or driver feedback

**Start stack (chassis + rosbridge):**

```bash
cd ~/ugv_rpi/ros2 && docker compose up -d   # rosbridge :9090 + ugv_driver_min
```

**Optional exclusive RoArm driver** (do not run while Flask holds CP2102):

```bash
# Host bridge (no rclpy; uses rosbridge + ugv-env pyserial) — preferred on this Pi:
UGV_ROARM_USB_OWNER=driver   # set in Flask env / .env, restart app
~/ugv_rpi/ros2/start_roarm_driver.sh

# Or in-container rclpy driver:
ROARM_ENABLE_DRIVER=1 docker compose up -d   # in ~/ugv_rpi/ros2
```

**Verify:**

```bash
# offline unit tests
./ugv-env/bin/python tests/test_roarm_ros2.py
# live (rosbridge up)
ROARM_ROS2_LIVE=1 ./ugv-env/bin/python tests/test_roarm_ros2.py
# from ROS container:
docker exec ugv_ros2 bash -lc 'source /opt/ros/humble/setup.bash && ros2 topic echo /ugv/roarm/joint_states --once'
```

### Drive safety / conventions

- Positive `linear_x` / `T:13` X = forward (ROS unicycle). UI `T:1` positive L/R = forward on stock firmware.
- AI chassis drives are **timed by default**; continuous motion requires an explicit continuous flag.
- Motion tools must be enabled in the AI capability UI before the LLM can call them.
- **UI “Freq. stop” (2s heartbeat):** the main dashboard re-sends last wheel cmd every 2s (idle = `L=0/R=0` stop). That can cut AI timed drives short if a control tab is open. Toggle **Freq. stop: OFF** for AI sessions. The server also arms an **AI motion lock** during timed AI drives that ignores those zero heartbeats until the maneuver ends.

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
