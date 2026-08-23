# Config profiles (chassis + attachment)

One codebase, two chassis profiles:

| Profile | Drive | `base_config.main_type` | `robot_name` |
|---------|-------|-------------------------|--------------|
| **rover** | wheels | `2` | UGV Rover |
| **beast** | tracks | `3` | UGV Beast |

Attachments (Waveshare `base_config.module_type`):

| Attachment | `module_type` | Behaviour |
|------------|---------------|-----------|
| **none** | `0` | No turret / no arm |
| **ptz** | `2` | Gimbal / PT camera (rover default) |
| **roarm2** | `1` | USB RoArm-M2 path (`roarm_ctrl`) |

Default loadout if `.loadout.json` is missing: rover + ptz, lidar off, `camera_prefer: auto`.

## Runtime selection

Live software profile is **`.loadout.json`** (gitignored) in the app root.

- **UI:** `http://<pi>:5000` → tab **Loadout** (hangar bay)
- **API:** `GET /api/loadout` · `POST /api/loadout`

`POST` may send any of `base`, `attachment`, `use_lidar`, `camera_prefer`. Unknown keys are dropped; bad values fall back to the defaults in `loadout.py`.

Changing loadout overlays `main_type` / `module_type` / `use_lidar` / `camera_prefer` / `robot_name` onto the in-memory `base_config`. It does **not**:

- rewrite `config.yaml` on disk
- write `drive_linear_sign` or `drive_angular_sign`

It **does** (behind hangar gates):

- re-init the camera pipeline when `camera_prefer` / `base` changes (`cv_ctrl.apply_camera_prefer`)
- start USB RoArm drivers **only** when `attachment=roarm2`
- stop / keep RoArm off when attachment is `ptz` or `none` (`roarm_started=false`)

## Example YAML files vs live `config.yaml`

| File | Role |
|------|------|
| **`config.yaml`** | **Live** machine file. Do not replace rover’s with a Beast dump. |
| **`config.rover.yaml`** | **Example** wheeled + PTZ profile. |
| **`config.beast.yaml`** | **Example** tracked + RoArm-typed profile. |

- Do **not** copy `config.beast.yaml` over this rover’s live `config.yaml`.
- Do **not** copy Beast drive signs onto the rover.
- If you copy an example to a real robot, **measure** F/B and L/R before you keep the signs.

## Drive signs (per-machine)

`drive_linear_sign` and `drive_angular_sign` belong to the **machine wiring**, not to the software profile. Loadout never writes them.

| Machine | Typical linear | Notes |
|---------|----------------|-------|
| This rover tree | **`-1`** | Required so UI/AI “forward” matches camera-forward on *this* chassis. |
| Beast example | **`+1`** | **Example only** — measure; do not copy blindly. |

Body frame in software: `+linear` = camera-forward, `+angular` = CCW / left. Env overrides: `UGV_DRIVE_LINEAR_SIGN` / `UGV_DRIVE_ANGULAR_SIGN`.

## Lidar

`use_lidar` is optional and **defaults false**. Do not force lidar on.

## Camera

Gate is `loadout.camera_strategy()` (used from `cv_ctrl`):

| `camera_prefer` | Rover (`base=rover`) | Beast (`base=beast`) |
|-----------------|----------------------|----------------------|
| `auto` | CSI first | USB first |
| `csi` | CSI first | CSI first |
| `usb` | USB first | USB first |

Hangar SAVE applies the preference live (re-opens the camera). USB path rediscovers `/dev/videoN` after re-enumerate.

## RoArm

See [ROARM.md](ROARM.md). Hangar `attachment=roarm2` enables:

- `roarm_ctrl` USB serial (CP2102)
- T:144 E/Z/R → joint map
- named poses (`travel_tuck`, `scan_ready`, …)
- Seek look-around skips PTZ (`path=roarm_look`)

Rover + PTZ never opens the arm USB port.

## ROS chassis driver

Entering ROS mode prefers `ugv_driver_min` when available (Beast image / bind-mount). Falls back to `ugv_bringup` on rover images. Does **not** require `ugv_ws-ugv_ros2`. Direct mode still preloads rosbridge, stops the chassis driver, and keeps UART handoff / autoheal / serial fallback.
