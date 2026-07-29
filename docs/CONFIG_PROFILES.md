# Config profiles: Beast (RoArm) vs PTZ robot

One codebase, two robots. Select behaviour with `config.yaml` (or a copy per machine).

**Honesty:** profiles describe how **this tree** is meant to be configured. Deployed git tips can lag; verify with `git -C ~/ugv_rpi status` and the `arm_config` block actually on disk before assuming Seek/RoArm gating is live.

## Beast (USB RoArm, this Pi)

```yaml
base_config:
  main_type: 3          # UGV Beast
  module_type: 1        # Arm module (UI layout)
  robot_name: UGV Beast

arm_config:
  transport: usb_serial # CP2102 RoArm ESP32 — not base UART
  serial_port: ""       # or ROARM_SERIAL env / by-id auto
  baud: 115200
  ui_aim_default: roarm
  default_pose: travel_tuck  # lean-back + deep elbow fold (low CG)
```

**Effects:**

- Boot moves arm to **travel_tuck** (not stock inverted-L `T:100`).
- UI **Aim: RoArm** routes overlay stick → `T:144` → USB joints (`T:102`).
- Seek look-around **does not** PTZ-pan; limited arm base peek / skip + log.
- Camera: USB UVC with multi-index rediscovery (`cv_ctrl`).

## PTZ robot (gimbal camera)

```yaml
base_config:
  main_type: 2          # or as shipped
  module_type: 2        # Gimbal / PT
  robot_name: UGV Rover # example

# Either omit arm_config entirely, or:
arm_config:
  transport: base_uart  # no USB RoArm driver
  ui_aim_default: pt
```

**Effects:**

- Boot centers gimbal via serial / ROS PT path.
- Seek: full PTZ pan look-around, triple-view, HW pan wait.
- No open of CP2102; no `travel_tuck`.

## Shared

- Chassis drive signs: `base_config.drive_linear_sign` / `drive_angular_sign`
- Control mode: direct serial vs ROS 2 (`/api/control_mode`)
- Seek / Chat / Raw UI modes (`templates/modes.js`)

## Env overrides

| Env | Purpose |
|-----|---------|
| `ROARM_SERIAL` | USB RoArm device path |
| `ROARM_BAUD` | Baud (default 115200) |
| `UGV_ROARM_USB_OWNER` | `flask` (hybrid) or `driver` (exclusive ROS node) |
| `UGV_BATTERY_LOW_V` | Low-battery log threshold (default 9.5) |
