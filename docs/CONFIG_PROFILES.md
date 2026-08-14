# Config profiles (chassis + attachment)

One codebase, two chassis profiles:

| Profile | Drive | `base_config.main_type` | `robot_name` |
|---------|-------|-------------------------|--------------|
| **rover** | wheels | `2` | UGV Rover |
| **beast** | tracks | `3` | UGV Beast |

Attachments (Waveshare `base_config.module_type`):

| Attachment | `module_type` | On this tree |
|------------|---------------|--------------|
| **none** | `0` | Wired (no turret / no arm) |
| **ptz** | `2` | Wired (gimbal / PT camera) |
| **roarm2** | `1` | **Not wired.** Profile stencil only |

Default loadout if `.loadout.json` is missing: rover + ptz, lidar off, `camera_prefer: auto`.

This is **not** the Beast `roarm-usb` branch. Selecting Beast or RoArm here does **not** boot an arm, tuck it for travel, or switch the UI into Aim mode.

## Runtime selection

Live software profile is **`.loadout.json`** (gitignored) in the app root.

- **UI:** `http://<pi>:5000` → tab **Loadout**
- **API:** `GET /api/loadout` · `POST /api/loadout`

Templates hot-reload. The Flask **process** must be restarted (do not expect a browser refresh) before `/api/loadout` exists on a build that just landed this route.

`POST` may send any of `base`, `attachment`, `use_lidar`, `camera_prefer`. Unknown keys are dropped; bad values fall back to the defaults in `loadout.py`.

Changing loadout overlays `main_type` / `module_type` / `use_lidar` / `camera_prefer` / `robot_name` onto the in-memory `base_config`. It does **not**:

- rewrite `config.yaml` on disk
- write `drive_linear_sign` or `drive_angular_sign`
- write or enable `arm_config`
- import or start RoArm drivers

If `attachment` is `roarm2`, UI and API report:

```text
arm offline — not wired on this build yet
```

`roarm_started` stays `false`.

## Example YAML files vs live `config.yaml`

| File | Role |
|------|------|
| **`config.yaml`** | **Live** machine file. This rover tree. Do not replace it with a Beast dump. |
| **`config.rover.yaml`** | **Example** wheeled + PTZ profile. |
| **`config.beast.yaml`** | **Example** tracked + RoArm-*typed* profile. |

`config.rover.yaml` and `config.beast.yaml` are drop-in shaped copies of `config.yaml` (same `video` / `cv` / `cmd_config` / … blocks) so you can copy one onto a **new** machine. They are **examples**.

- Do **not** copy `config.beast.yaml` over this rover’s live `config.yaml`.
- Do **not** copy Beast drive signs, `main_type`, or a live `arm_config` onto the rover.
- If you copy an example to a real robot, **measure** F/B and L/R before you keep the signs.

## Drive signs (per-machine)

`drive_linear_sign` and `drive_angular_sign` belong to the **machine wiring**, not to the software profile. Loadout never writes them.

| Machine | Typical linear | Notes |
|---------|----------------|-------|
| This rover tree | **`-1`** | Required so UI/AI “forward” matches camera-forward on *this* chassis. |
| Beast example | **`+1`** | **Example only** — typical Beast wiring on the `roarm-usb` tree. Measure; do not copy blindly. |

Body frame in software: `+linear` = camera-forward, `+angular` = CCW / left. Linear also flips tank-turn left/right (both tracks/wheels negated). If F/B **and** L/R feel inverted, flip linear only (`±1`). Env overrides: `UGV_DRIVE_LINEAR_SIGN` / `UGV_DRIVE_ANGULAR_SIGN`. After a yaml change, **restart the app** and check `GET /api/status`.

## Lidar

`use_lidar` is optional and **defaults false**. Do not force lidar on. Seek obstacles are still vision heuristics, not a lidar map.

## Camera

Gate is `loadout.camera_strategy()` (used from `cv_ctrl`):

| `camera_prefer` | Rover (`base: rover`) | Beast (`base: beast`) |
|-----------------|------------------------|------------------------|
| `auto` (default) | **CSI first** | **USB UVC first** (multi-index rediscovery) |
| `csi` | CSI | CSI |
| `usb` | USB | USB |

`camera_prefer` may appear on `base_config` in the example YAMLs; runtime can also set it via `.loadout.json` / the Loadout tab.

## RoArm is a stencil, not a driver

Selecting `attachment=roarm2` only records `module_type: 1` and the offline banner. This consolidate tree does **not** start USB serial, ROS arm nodes, travel tuck, or Aim-mode overlay routing.

## Not ported (do not expect from Beast)

These exist on `beast/roarm-usb` and are **not** live behavior here:

- `roarm_ctrl.py`
- Aim mode (`ui_aim_default` / overlay stick → `T:144` → USB joints)
- `travel_tuck` (boot lean-back + elbow fold)
- `roarm_look` (Seek peek via arm base instead of PTZ)
- USB arm ROS (`UGV_ROARM_USB_OWNER`, exclusive driver node)
- default drive-sign changes (do not flip this rover to Beast `+1`)

A commented `arm_config` block may appear in `config.beast.yaml` so the old keys are visible. It is labeled **not wired on this consolidate build — do not enable**.
