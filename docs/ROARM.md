# USB RoArm (Beast)

Hardware: RoArm has its **own ESP32** on USB-C CP2102 — **not** base UART (`ttyAMA0`).

**Scope:** operator guide for the dual-robot tree on this Pi. Behaviour below matches **local** `ugv_rpi` when `arm_config.transport: usb_serial`. Confirm the running process was started from that tree (`git -C ~/ugv_rpi log -1`). Remote `origin/main` may lag until dual-robot publish finishes (see [MERGE_PTZ_AND_ROARM_PLAN.md](MERGE_PTZ_AND_ROARM_PLAN.md) end-of-day status).

## Default stance: `travel_tuck`

| Joint | Approx | Meaning |
|-------|--------|---------|
| base | 0 | centered yaw |
| shoulder | −0.62 | lean back over chassis |
| elbow | 0.88 | deep fold (low CG) |
| hand | 3.05 | neutral gripper |

Stock **home** is inverted L (`elbow ≈ 1.57`) — long reach; use only with clear space.  
**Do not** call firmware `T:100` as a “safe stow” near furniture.

```python
import roarm_ctrl
r = roarm_ctrl.get_roarm()
r.pose("travel_tuck")   # default drive stance
r.pose("scan_ready")    # slightly more open
r.pose("home")          # stock inverted L
```

JSON equivalent: `{"T":102,"base":0,"shoulder":-0.62,"elbow":0.88,"hand":3.05,"spd":0,"acc":10}`

## UI Aim mode

- **Aim: RoArm** — video overlay stick → `T:144` E/Z/R → USB joints  
- **Aim: PT** — original pan/tilt `T:133` (for gimbal robots / dual mode)

Toggle: navbar button or `POST /api/ui_aim_mode`.  
Hidden when `module_type != 1` and transport is not USB.

## Seek on Beast

Seek’s look-around on PTZ robots pans the gimbal.  
On Beast (`arm_config.transport: usb_serial`), `_seek_look_deg` **skips PTZ** and may do a **limited arm base peek**, then settles for photos (`path=roarm_look`).

## Camera

USB UVC may re-enumerate (`video0` ↔ `video1`).  
`cv_ctrl` rediscovers the first working index after failed reads.

## Config

See [CONFIG_PROFILES.md](CONFIG_PROFILES.md). Beast requires:

```yaml
arm_config:
  transport: usb_serial
  default_pose: travel_tuck
```

## Related

- `~/beast-image/CUSTOM_BUILD.md` — wiring  
- `~/beast-image/ROARM_CONTROL_PLAN.md` — phases  
- `docs/MERGE_PTZ_AND_ROARM_PLAN.md` — dual-robot merge checklist  
