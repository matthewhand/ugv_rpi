# Hailo-8L optional features (scaffold)

Frigate already uses the Hailo-8L as its **object detector** (YOLO 320x320) on `/dev/hailo0` (HailoRT ~4.21 matching Frigate 0.17). This fork adds a **separate** optional-feature path for:

| Feature | Stub module | Intended HEF (not in git) |
|---------|-------------|---------------------------|
| Pose | `hailo_features/pose.py` | `yolov8s_pose.hef` (hailo-rpi5-examples pose_estimation) |
| Hand / gesture | `hailo_features/hand.py` | e.g. `hand_landmark.hef` (community hand GST pipelines) |
| Face metrics / person cues | `hailo_features/face.py` | e.g. `retinaface_mobilenet_v1.hef` (retinaface GST) |

These are **not** Frigate detectors. They share the device with Frigate via HailoRT **SHARED** scheduling when both apps run.

## Loadout / config gate

Persisted in gitignored `.loadout.json`:

```json
{
  "base": "rover",
  "attachment": "ptz",
  "use_lidar": false,
  "camera_prefer": "auto",
  "hailo": {
    "enabled": "auto",
    "pose": true,
    "hand": true,
    "face": true,
    "device": "/dev/hailo0",
    "hef_dir": "models/hailo"
  }
}
```

| `hailo.enabled` | Behaviour |
|-----------------|-----------|
| `auto` (default) | Active when `device` exists (usually `/dev/hailo0`) |
| `true` | Force on (still needs HEFs + HailoRT) |
| `false` | Force off |

API: `POST /api/loadout` may include a nested `hailo` object (deep-merged).

Helpers: `loadout.hailo_wanted()`, `hailo_features.status_payload()`, `hailo_features.tick()`.

## HEF download (do not commit binaries)

Place HEFs under `models/hailo/` (gitignored `*.hef`):

```bash
mkdir -p models/hailo
# Use Hailo model zoo / rpi5-examples docs for URLs matching HailoRT 4.21:
#   yolov8s_pose.hef
#   hand_landmark.hef
#   retinaface_mobilenet_v1.hef
```

Community references:

- https://github.com/hailo-ai/hailo-rpi5-examples — pose_estimation
- Hailo model zoo / GStreamer face (retinaface) examples
- Frigate multi-app shared Hailo device (SHARED group)

**Never** commit HEF weights, camera passwords, MQTT passwords, or API keys.

## MQTT topic sketch (no secrets)

Default prefix `ugv/hailo` (override with env `UGV_HAILO_MQTT_PREFIX`):

| Key | Topic |
|-----|-------|
| status | `ugv/hailo/status` |
| pose | `ugv/hailo/pose` |
| hand | `ugv/hailo/hand` |
| gesture | `ugv/hailo/gesture` |
| face | `ugv/hailo/face` |

Broker URL / credentials belong in `.env` only (see `.env.example`). Publishing is not wired in this scaffold.

## What this PR does / does not

**Does:** loadout gate, auto-detect, stub modules, MQTT topic constants, docs, tests.

**Does not:** download HEFs, start a second GStreamer pipeline, restart Frigate, or authenticate OpenCode Space Bunny.