import serial
import sys
import math
import re
import cv2
import numpy as np
import mpl_toolkits
mpl_toolkits.__path__ = [p for p in mpl_toolkits.__path__ if 'dist-packages' not in p]

# import base_ctrl library
from base_ctrl import BaseController
import threading
import yaml, os
import base64
import urllib.error
import urllib.request

# Load local secrets / LLM config before reading env-driven flags
try:
    from dotenv import load_dotenv
    load_dotenv(os.path.join(os.path.dirname(os.path.realpath(__file__)), '.env'))
except Exception:
    pass

# raspberry pi version check.
def is_raspberry_pi5():
    with open('/proc/cpuinfo', 'r') as file:
        for line in file:
            if 'Model' in line:
                if 'Raspberry Pi 5' in line:
                    return True
                else:
                    return False

if is_raspberry_pi5():
    base = BaseController('/dev/ttyAMA0', 115200)
else:
    base = BaseController('/dev/serial0', 115200)

threading.Thread(target=lambda: base.breath_light(15), daemon=True).start()

# config file.
curpath = os.path.realpath(__file__)
thisPath = os.path.dirname(curpath)
with open(thisPath + '/config.yaml', 'r') as yaml_file:
    f = yaml.safe_load(yaml_file)

base.base_oled(0, f["base_config"]["robot_name"])
base.base_oled(1, f"sbc_version: {f['base_config']['sbc_version']}")
base.base_oled(2, f"{f['base_config']['main_type']}{f['base_config']['module_type']}")
base.base_oled(3, "Starting...")


# Import necessary modules
from flask import Flask, render_template, Response, request, jsonify, redirect, url_for, send_from_directory, send_file
from flask_socketio import SocketIO, emit
from werkzeug.utils import secure_filename
from aiortc import RTCPeerConnection, RTCSessionDescription
import hmac
import json
import uuid
import asyncio
import time
import logging
import cv_ctrl
import audio_ctrl
import os_info
import app_log
from app_log import app_log as olog

# Get system info
UPLOAD_FOLDER = thisPath + '/sounds/others'
si = os_info.SystemInfo()

# Create a Flask app instance
app = Flask(__name__)
# log = logging.getLogger('werkzeug')
# log.disabled = True
# manage_session=False: avoid Flask 3.1 crash
# "AttributeError: property 'session' of 'RequestContext' object has no setter"
# which otherwise breaks ALL Socket.IO handlers (sticks never reach the robot).
socketio = SocketIO(app, manage_session=False)

# Hot reload for UI iteration on :5000
#   UGV_HOT_RELOAD=1     → templates auto-reload + no browser cache (default for run_dev.sh)
#   UGV_RELOADER=1       → also restart process on *.py changes (heavier; re-opens serial)
#   FLASK_DEBUG=1        → same as UGV_HOT_RELOAD=1
_HOT_RELOAD = os.environ.get('UGV_HOT_RELOAD', '').lower() in ('1', 'true', 'yes') \
    or os.environ.get('FLASK_DEBUG', '').lower() in ('1', 'true', 'yes')
_USE_RELOADER = os.environ.get('UGV_RELOADER', '').lower() in ('1', 'true', 'yes')
if _HOT_RELOAD:
    app.config['TEMPLATES_AUTO_RELOAD'] = True
    app.config['SEND_FILE_MAX_AGE_DEFAULT'] = 0
    app.jinja_env.auto_reload = True

# ---------------------------------------------------------------------------
# Unified control routing: "direct" (ESP32 serial) vs "ros2" (rosbridge relay)
# All UI stick + AI motion tools go through get_control_mode() / route helpers.
# ---------------------------------------------------------------------------
_CONTROL_MODE_PATH = os.path.join(thisPath, '.control_mode.json')
_control_mode_lock = threading.Lock()


def _default_control_mode():
    env = (os.environ.get('UGV_CONTROL_MODE') or '').strip().lower()
    if env in ('direct', 'serial', 'raw'):
        return 'direct'
    if env in ('ros2', 'ros', 'relay'):
        return 'ros2'
    # Legacy: UGV_MOTOR_BYPASS=1 meant ROS-friendly chassis
    if os.environ.get('UGV_MOTOR_BYPASS', '').lower() in ('1', 'true', 'yes'):
        return 'ros2'
    return 'direct'


_control_mode = _default_control_mode()


def _load_control_mode():
    global _control_mode
    try:
        if os.path.isfile(_CONTROL_MODE_PATH):
            with open(_CONTROL_MODE_PATH, 'r') as fh:
                data = json.load(fh)
            m = (data.get('mode') or '').strip().lower()
            if m in ('direct', 'ros2'):
                _control_mode = m
    except Exception as e:
        print(f'[app.py] load control_mode: {e}')


def _save_control_mode():
    try:
        with open(_CONTROL_MODE_PATH, 'w') as fh:
            json.dump({'mode': _control_mode}, fh)
    except Exception as e:
        print(f'[app.py] save control_mode: {e}')


def get_control_mode():
    with _control_mode_lock:
        return _control_mode


def _rosbridge_reachable():
    """True when rosbridge websocket accepts connections (PTZ/drive ROS path)."""
    try:
        import ros_motion
        return bool(ros_motion.rosbridge_status().get('ok'))
    except Exception:
        return False


def _ensure_flask_serial(reason='fallback'):
    """Reclaim UART for Flask when ROS path is dead so PTZ/drive are not silent.

    Returns True if serial is open and owned by Flask after the call.
    """
    if base.serial_is_open() and not getattr(base, 'serial_released_for_ros', False):
        return True
    ok = bool(base.claim_serial_for_flask())
    if ok:
        # Allow chassis on serial while in fallback (ROS mode may have set this False)
        base.enable_motor_control = True
        olog.warn(
            'serial',
            f'Reclaimed UART for Flask ({reason}) — rosbridge unavailable; '
            'PTZ/chassis use serial until ROS path is healthy',
            reason=reason,
            serial_open=True,
            control_mode=get_control_mode(),
        )
    else:
        olog.error(
            'serial',
            f'UART reclaim failed ({reason}) — PTZ/drive may be dead until Direct mode or rosbridge',
            reason=reason,
            serial_open=False,
            control_mode=get_control_mode(),
        )
    return ok


def _drive_sign(name, default=1.0):
    """Read ±1 drive sign from env (preferred) or config.yaml base_config."""
    env_key = 'UGV_DRIVE_LINEAR_SIGN' if name == 'linear' else 'UGV_DRIVE_ANGULAR_SIGN'
    env = (os.environ.get(env_key) or '').strip()
    if env != '':
        try:
            return -1.0 if float(env) < 0 else 1.0
        except ValueError:
            pass
    try:
        cfg_key = 'drive_linear_sign' if name == 'linear' else 'drive_angular_sign'
        v = f.get('base_config', {}).get(cfg_key, default)
        return -1.0 if float(v) < 0 else 1.0
    except (TypeError, ValueError):
        return 1.0 if default >= 0 else -1.0


def body_to_hw_twist(linear_x, angular_z=0.0):
    """Map body-frame velocities (camera-forward +linear) to hardware twist.

    Single choke point for AI T:13, ROS cmd_vel, and any other twist API.
    """
    s_lin = _drive_sign('linear', 1.0)
    s_ang = _drive_sign('angular', 1.0)
    return float(linear_x) * s_lin, float(angular_z) * s_ang


def body_to_hw_diff(left, right):
    """Map body-frame differential wheel speeds to hardware T:1 L/R.

    Same linear sign as twist so stick-forward and AI-forward stay aligned.
    """
    s_lin = _drive_sign('linear', 1.0)
    return float(left) * s_lin, float(right) * s_lin


def set_control_mode(mode, *, source='api'):
    """Set 'direct' (Flask owns UART) or 'ros2' (release UART for ugv_bringup).

    ros2: close /dev/ttyAMA0 so ROS can open it; motion goes via rosbridge.
    direct: reclaim UART; motion JSON goes serial.
    """
    global _control_mode
    mode = (mode or '').strip().lower()
    if mode in ('serial', 'raw'):
        mode = 'direct'
    if mode in ('ros', 'relay'):
        mode = 'ros2'
    if mode not in ('direct', 'ros2'):
        raise ValueError("mode must be 'direct' or 'ros2'")
    with _control_mode_lock:
        prev = _control_mode
        _control_mode = mode
        # Legacy flag; full ROS mode also releases the port entirely.
        base.enable_motor_control = (mode == 'direct')

    # Port ownership: only one of Flask or ROS may hold the UART.
    serial_ok = True
    if mode == 'ros2':
        serial_ok = bool(base.release_serial_for_ros())
    else:
        # Leaving ROS: stop bringup first so it does not keep /dev/ttyAMA0
        if prev == 'ros2':
            try:
                fields_stop = _stop_ugv_bringup()
            except Exception as e:
                fields_stop = {'ok': False, 'detail': str(e)}
                olog.warn('ros_autostop', f'bringup stop failed: {e}', error=str(e)[:200])
        else:
            fields_stop = None
        serial_ok = bool(base.claim_serial_for_flask())

    _save_control_mode()
    fields = {
        'mode': mode,
        'prev_mode': prev,
        'uart_owner': 'flask' if mode == 'direct' else 'ros2',
        'serial_open': base.serial_is_open(),
        'serial_released_for_ros': bool(getattr(base, 'serial_released_for_ros', False)),
        'source': source,
        'serial_ok': serial_ok,
    }
    if mode == 'direct' and prev == 'ros2':
        fields['ros_autostop'] = fields_stop
    # Entering ROS mode: auto-start rosbridge (+ bringup) so UI PTZ/drive work
    if mode == 'ros2' and (prev != mode or source in ('api', 'ui_toggle', 'startup')):
        try:
            fields['ros_autostart'] = _ensure_ros2_sidecar_stack()
        except Exception as e:
            fields['ros_autostart'] = {'error': str(e)}
            olog.warn('ros_autostart', f'sidecar ensure failed: {e}', error=str(e)[:200])
    if mode == 'ros2':
        try:
            import ros_motion
            br = ros_motion.rosbridge_status()
            fields['rosbridge_ok'] = bool(br.get('ok'))
            if br.get('url'):
                fields['rosbridge_url'] = br.get('url')
            if br.get('error'):
                fields['rosbridge_error'] = br.get('error')
        except Exception as e:
            fields['rosbridge_ok'] = False
            fields['rosbridge_error'] = str(e)
        # Critical: if rosbridge is down after release, reclaim serial so PTZ is not dead.
        if not fields.get('rosbridge_ok'):
            reclaimed = _ensure_flask_serial(reason='ros2_without_rosbridge')
            fields['serial_fallback'] = True
            fields['serial_ok'] = reclaimed
            fields['serial_open'] = base.serial_is_open()
            fields['serial_released_for_ros'] = bool(
                getattr(base, 'serial_released_for_ros', False)
            )
            fields['uart_owner'] = 'flask_fallback' if reclaimed else 'none'
            fields['fallback_reason'] = (
                'rosbridge not reachable — reclaimed UART so pan/tilt and drive work via serial. '
                'Start rosbridge in ugv_ros2 or switch Control to Direct.'
            )
            olog.warn(
                'control_mode',
                fields['fallback_reason'],
                rosbridge_error=fields.get('rosbridge_error'),
                serial_open=fields['serial_open'],
            )
    if prev != mode:
        olog.info(
            'control_mode',
            f'Control mode → {mode} (UART owner: {fields["uart_owner"]}, '
            f'serial_open={fields["serial_open"]})',
            **fields,
        )
    # Stash last ensure result for API payload
    try:
        set_control_mode._last_fields = fields  # type: ignore[attr-defined]
    except Exception:
        pass
    return mode


_load_control_mode()
# Apply side effects for loaded mode (including UART ownership)
if get_control_mode() == 'ros2':
    base.enable_motor_control = False
    base.release_serial_for_ros()
    # Startup heal: do not leave ROS mode with closed UART and dead rosbridge
    if not _rosbridge_reachable():
        _ensure_flask_serial(reason='startup_ros2_no_rosbridge')
else:
    base.enable_motor_control = True
    base.claim_serial_for_flask()
olog.info(
    'startup',
    f'control_mode={get_control_mode()} '
    f'(uart_owner={"flask" if get_control_mode() == "direct" else "ros2"}, '
    f'serial_open={base.serial_is_open()}, '
    f'serial_released={getattr(base, "serial_released_for_ros", False)})',
    control_mode=get_control_mode(),
    uart_owner='flask' if get_control_mode() == 'direct' else 'ros2',
    serial_open=base.serial_is_open(),
    serial_released_for_ros=bool(getattr(base, 'serial_released_for_ros', False)),
    rosbridge_ok=_rosbridge_reachable() if get_control_mode() == 'ros2' else None,
)

# Set to keep track of RTCPeerConnection instances
active_pcs = {}

# Maximum number of active connections allowed
MAX_CONNECTIONS = 1

# Set to keep track of RTCPeerConnection instances
pcs = set()

# Camera funcs
cvf = cv_ctrl.OpencvFuncs(thisPath, base)

cmd_actions = {
    f['code']['zoom_x1']: lambda: cvf.scale_ctrl(1),
    f['code']['zoom_x2']: lambda: cvf.scale_ctrl(2),
    f['code']['zoom_x4']: lambda: cvf.scale_ctrl(4),

    f['code']['pic_cap']: cvf.picture_capture,
    f['code']['vid_sta']: lambda: cvf.video_record(True),
    f['code']['vid_end']: lambda: cvf.video_record(False),

    f['code']['cv_none']: lambda: cvf.set_cv_mode(f['code']['cv_none']),
    f['code']['cv_moti']: lambda: cvf.set_cv_mode(f['code']['cv_moti']),
    f['code']['cv_face']: lambda: cvf.set_cv_mode(f['code']['cv_face']),
    f['code']['cv_objs']: lambda: cvf.set_cv_mode(f['code']['cv_objs']),
    f['code']['cv_clor']: lambda: cvf.set_cv_mode(f['code']['cv_clor']),
    f['code']['mp_hand']: lambda: cvf.set_cv_mode(f['code']['mp_hand']),
    f['code']['cv_auto']: lambda: cvf.set_cv_mode(f['code']['cv_auto']),
    f['code']['mp_face']: lambda: cvf.set_cv_mode(f['code']['mp_face']),
    f['code']['mp_pose']: lambda: cvf.set_cv_mode(f['code']['mp_pose']),

    f['code']['re_none']: lambda: cvf.set_detection_reaction(f['code']['re_none']),
    f['code']['re_capt']: lambda: cvf.set_detection_reaction(f['code']['re_capt']),
    f['code']['re_reco']: lambda: cvf.set_detection_reaction(f['code']['re_reco']),

    f['code']['mc_lock']: lambda: cvf.set_movtion_lock(True),
    f['code']['mc_unlo']: lambda: cvf.set_movtion_lock(False),

    f['code']['led_off']: lambda: cvf.head_light_ctrl(0),
    f['code']['led_aut']: lambda: cvf.head_light_ctrl(1),
    f['code']['led_ton']: lambda: cvf.head_light_ctrl(2),

    f['code']['release']: lambda: base.bus_servo_torque_lock(255, 0),
    f['code']['s_panid']: lambda: base.bus_servo_id_set(255, 2),
    f['code']['s_tilid']: lambda: base.bus_servo_id_set(255, 1),
    f['code']['set_mid']: lambda: base.bus_servo_mid_set(255),

    f['code']['base_of']: lambda: base.lights_ctrl(0, base.head_light_status),
    f['code']['base_on']: lambda: base.lights_ctrl(255, base.head_light_status),
    f['code']['head_ct']: lambda: cvf.head_light_ctrl(3),
    f['code']['base_ct']: base.base_lights_ctrl
}

cmd_feedback_actions = [f['code']['cv_none'], f['code']['cv_moti'],
                        f['code']['cv_face'], f['code']['cv_objs'],
                        f['code']['cv_clor'], f['code']['mp_hand'],
                        f['code']['cv_auto'], f['code']['mp_face'],
                        f['code']['mp_pose'], f['code']['re_none'],
                        f['code']['re_capt'], f['code']['re_reco'],
                        f['code']['mc_lock'], f['code']['mc_unlo'],
                        f['code']['led_off'], f['code']['led_aut'],
                        f['code']['led_ton'], f['code']['base_of'],
                        f['code']['base_on'], f['code']['head_ct'],
                        f['code']['base_ct']
                        ]

# cv info process
def process_cv_info(cmd):
    if cmd[f['fb']['detect_type']] != f['code']['cv_none']:
        print(cmd[f['fb']['detect_type']])
        pass

# Function to generate video frames from the camera
def generate_frames():
    while True:
        frame = cvf.frame_process()
        # print(cvf.cv_info())
        try:
            yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n') 
        except Exception as e:
            print("An [generate_frames] error occurred:", e)






# Feature Toggles (Default OFF)
enable_rtsp_stream = False

@app.route('/api/status')
def api_status():
    mode = get_control_mode()
    with _ros_autoheal_lock:
        heal = dict(_ros_autoheal_state)
    if heal.get('last_tick_at') is not None:
        try:
            heal['last_tick_age_s'] = round(time.time() - float(heal['last_tick_at']), 1)
        except (TypeError, ValueError):
            pass
    br_ok = None
    if mode == 'ros2':
        br_ok = _rosbridge_reachable()
    return jsonify({
        'enable_rtsp_stream': enable_rtsp_stream,
        'enable_motor_control': base.enable_motor_control,
        'control_mode': mode,  # 'direct' | 'ros2'
        'control_mode_label': 'Direct serial' if mode == 'direct' else 'ROS 2 relay',
        'uart_owner': (
            'flask' if mode == 'direct'
            else ('flask_fallback' if base.serial_is_open() and not getattr(base, 'serial_released_for_ros', False) else 'ros2')
        ),
        'serial_open': base.serial_is_open() if hasattr(base, 'serial_is_open') else bool(getattr(base, 'ser', None)),
        'serial_released_for_ros': bool(getattr(base, 'serial_released_for_ros', False)),
        'esp32_wifi_stopped': bool(_esp32_wifi_session.get('stopped')),
        'rosbridge_ok': br_ok,
        'ros_autoheal': heal,
        # Effective signs used by body_to_hw_* (UI/AI forward → wheel cmd)
        'drive_linear_sign': _drive_sign('linear'),
        'drive_angular_sign': _drive_sign('angular'),
        'ptz': _ptz_aim_public(),
    })

@app.route('/api/toggle_rtsp', methods=['POST'])
def api_toggle_rtsp():
    global enable_rtsp_stream
    enable_rtsp_stream = not enable_rtsp_stream
    olog.info('rtsp_toggle', f'RTSP stream {"ON" if enable_rtsp_stream else "OFF"}',
              enable_rtsp_stream=enable_rtsp_stream)
    return jsonify({'success': True, 'enable_rtsp_stream': enable_rtsp_stream})

def _control_mode_payload(mode=None, *, mode_changed=False, prev_mode=None):
    """Shared status + restart guidance after control_mode changes."""
    mode = mode or get_control_mode()
    bridge = {}
    if mode == 'ros2':
        try:
            import ros_motion
            bridge = ros_motion.rosbridge_status()
        except Exception as e:
            bridge = {'ok': False, 'error': str(e)}
    advice = _uart_restart_advice(mode, prev_mode=prev_mode, mode_changed=mode_changed)
    # Soften restart banner when we already auto-started rosbridge successfully
    last = getattr(set_control_mode, '_last_fields', None) or {}
    auto = last.get('ros_autostart') or {}
    rb = auto.get('rosbridge') or {}
    if mode == 'ros2' and rb.get('ok'):
        advice['restart_required'] = False
        advice['restart_reason'] = (
            'ROS 2 mode active; rosbridge is up. '
            'Use Restart only if bringup failed to open the UART.'
        )
    payload = {
        'success': True,
        'control_mode': mode,
        'control_mode_label': 'Direct serial' if mode == 'direct' else 'ROS 2 relay',
        'enable_motor_control': base.enable_motor_control,
        'uart_owner': 'flask' if mode == 'direct' else 'ros2',
        'serial_open': base.serial_is_open() if hasattr(base, 'serial_is_open') else bool(getattr(base, 'ser', None)),
        'serial_released_for_ros': bool(getattr(base, 'serial_released_for_ros', False)),
        'rosbridge': bridge,
        **advice,
    }
    if auto:
        payload['ros_autostart'] = auto
    return payload


def _uart_restart_advice(mode, *, prev_mode=None, mode_changed=False):
    """Explain that peer stack may need restart to re-open /dev/ttyAMA0 exclusively."""
    container = (os.environ.get('UGV_ROS_CONTAINER') or 'ugv_ros2').strip() or 'ugv_ros2'
    allow = os.environ.get('UGV_ALLOW_DOCKER_RESTART', '1').lower() in ('1', 'true', 'yes')
    # Always recommend after a live mode change; mild hint on GET when ros2 + serial free
    if mode == 'ros2':
        reason = (
            'Flask released /dev/ttyAMA0. Restart the ROS container (or re-launch ugv_bringup) '
            'so it can open the UART exclusively for chassis + PTZ.'
        )
        label = f'Restart {container}'
        detail = (
            f'docker restart {container}  — then ensure bringup + rosbridge are running inside it. '
            'Device node is already mounted; restart is for process re-open, not remount.'
        )
    else:
        reason = (
            'Flask reclaimed /dev/ttyAMA0. If ROS/bringup still holds the port, restart or stop '
            f'{container} so Direct serial can work alone.'
        )
        label = f'Restart {container}'
        detail = (
            f'docker restart {container}  frees serial if bringup had it open. '
            'Flask does not need a restart for UART reclaim.'
        )
    return {
        'restart_required': bool(mode_changed) or (mode == 'ros2' and not base.serial_is_open()),
        'restart_reason': reason,
        'restart_detail': detail,
        'restart_button_label': label,
        'restart_target': container,
        'restart_allowed': allow,
        'mode_changed': bool(mode_changed),
        'prev_mode': prev_mode,
    }


def _docker_restart_container(name):
    """Restart a docker container by name. Returns (ok, message, detail)."""
    allow = os.environ.get('UGV_ALLOW_DOCKER_RESTART', '1').lower() in ('1', 'true', 'yes')
    if not allow:
        return False, 'Docker restart disabled (set UGV_ALLOW_DOCKER_RESTART=1)', {}
    name = (name or '').strip()
    # only allow simple container names
    if not name or not all(c.isalnum() or c in '-_' for c in name):
        return False, 'invalid container name', {}
    allowed = {
        (os.environ.get('UGV_ROS_CONTAINER') or 'ugv_ros2').strip(),
        'ugv_ros2',
        'ugv_rpi_ui',
    }
    if name not in allowed:
        return False, f'container {name!r} not in allow-list {sorted(allowed)}', {}
    import subprocess
    try:
        proc = subprocess.run(
            ['docker', 'restart', name],
            capture_output=True, text=True, timeout=120,
        )
        out = (proc.stdout or '').strip()
        err = (proc.stderr or '').strip()
        ok = proc.returncode == 0
        msg = out or err or (f'restart {name} exit {proc.returncode}')
        olog.log(
            'info' if ok else 'error',
            'stack_restart',
            f'docker restart {name}: {"ok" if ok else "failed"} — {msg[:200]}',
            container=name, returncode=proc.returncode, ok=ok,
        )
        return ok, msg, {'stdout': out, 'stderr': err, 'returncode': proc.returncode}
    except FileNotFoundError:
        return False, 'docker CLI not found on Flask host', {}
    except subprocess.TimeoutExpired:
        return False, f'docker restart {name} timed out', {}
    except Exception as e:
        return False, str(e), {}


def _ros_container_name():
    return (os.environ.get('UGV_ROS_CONTAINER') or 'ugv_ros2').strip() or 'ugv_ros2'


def _env_flag(name, default='1'):
    return os.environ.get(name, default).strip().lower() in ('1', 'true', 'yes', 'on')


def _docker_exec(script, *, detach=False, timeout=45):
    """Run bash -lc script inside the ROS container. Returns (ok, stdout, stderr, code)."""
    import subprocess
    container = _ros_container_name()
    cmd = ['docker', 'exec']
    if detach:
        cmd.append('-d')
    cmd.extend([container, 'bash', '-lc', script])
    try:
        proc = subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout,
        )
        return (
            proc.returncode == 0,
            (proc.stdout or '').strip(),
            (proc.stderr or '').strip(),
            proc.returncode,
        )
    except FileNotFoundError:
        return False, '', 'docker CLI not found', -1
    except subprocess.TimeoutExpired:
        return False, '', f'docker exec timed out ({timeout}s)', -1
    except Exception as e:
        return False, '', str(e), -1


def _rosbridge_is_up():
    try:
        import ros_motion
        return bool(ros_motion.rosbridge_status().get('ok'))
    except Exception:
        return False


def _ensure_rosbridge_running():
    """Start rosbridge in ugv_ros2 if not already accepting connections.

    Env: UGV_AUTOSTART_ROSBRIDGE=1 (default on).
    """
    out = {
        'wanted': True,
        'already_up': False,
        'started': False,
        'ok': False,
        'detail': '',
    }
    if not _env_flag('UGV_AUTOSTART_ROSBRIDGE', '1'):
        out['wanted'] = False
        out['detail'] = 'UGV_AUTOSTART_ROSBRIDGE disabled'
        out['ok'] = _rosbridge_is_up()
        return out

    if _rosbridge_is_up():
        out['already_up'] = True
        out['ok'] = True
        out['detail'] = 'rosbridge already accepting connections'
        return out

    # Process check — bracket trick avoids matching the grep itself
    _ok, so, _se, _ = _docker_exec(
        "ps aux | grep '[r]osbridge_websocket' || true",
        detach=False, timeout=15,
    )
    has_proc = bool((so or '').strip())

    if not has_proc:
        # docker exec -d so the launch survives after the exec returns
        start_script = r'''
source /opt/ros/humble/setup.bash
source /home/ws/ugv_ws/install/setup.bash 2>/dev/null || true
mkdir -p /tmp/ugv_ros_logs
exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090 \
  >> /tmp/ugv_ros_logs/rosbridge.log 2>&1
'''
        ok_st, so2, se2, code = _docker_exec(start_script, detach=True, timeout=20)
        # detach: docker returns immediately; non-zero rare
        out['started'] = True
        out['detail'] = so2 or se2 or ('detach_ok' if ok_st else f'exit {code}')
        olog.info(
            'ros_autostart',
            f'rosbridge launch issued (docker exec -d): {out["detail"][:160]}',
            component='rosbridge', started=True, ok=ok_st,
        )

    # Wait for WebSocket accept (launch can take a few seconds)
    for i in range(40):
        if _rosbridge_is_up():
            out['ok'] = True
            out['detail'] = (
                'rosbridge up (already running)' if has_proc and not out['started']
                else 'rosbridge started and accepting'
            )
            return out
        time.sleep(0.25)

    # One more process peek for diagnostics
    _ok2, so2b, _, _ = _docker_exec(
        "ps aux | grep '[r]osbridge_websocket' || true; "
        "tail -15 /tmp/ugv_ros_logs/rosbridge.log 2>/dev/null || true",
        detach=False, timeout=15,
    )
    out['ok'] = False
    out['detail'] = (
        f'rosbridge not accepting on :9090 after wait. '
        f'proc={(so2b or "")[:300]}'
    )
    olog.warn('ros_autostart', out['detail'][:240], component='rosbridge', ok=False)
    return out


def _ensure_ugv_bringup_running():
    """Start ugv_bringup in container so /cmd_vel + /joint_states reach ESP32.

    Env: UGV_AUTOSTART_BRINGUP=1 (default on). Requires UART released (ros2 mode).
    """
    out = {
        'wanted': True,
        'already_up': False,
        'started': False,
        'ok': False,
        'detail': '',
    }
    if not _env_flag('UGV_AUTOSTART_BRINGUP', '1'):
        out['wanted'] = False
        out['detail'] = 'UGV_AUTOSTART_BRINGUP disabled'
        return out

    ok_chk, so, se, _ = _docker_exec(
        "pgrep -af '/lib/ugv_bringup/ugv_bringup|ugv_bringup ugv_bringup' || true",
        detach=False, timeout=15,
    )
    # pgrep output may include the pgrep line itself in some shells — look for real binary
    has_proc = bool(so and 'ugv_bringup' in so and 'pgrep' not in so.splitlines()[-1] if so else False)
    if so:
        for line in so.splitlines():
            if 'ugv_bringup' in line and 'pgrep' not in line and 'bash -lc' not in line:
                has_proc = True
                break
        else:
            # any line with the installed executable path
            has_proc = any(
                '/ugv_bringup/ugv_bringup' in line or line.strip().endswith('ugv_bringup')
                for line in so.splitlines()
                if 'pgrep' not in line
            )

    if has_proc:
        out['already_up'] = True
        out['ok'] = True
        out['detail'] = 'ugv_bringup already running'
        return out

    port = (os.environ.get('UGV_SERIAL_PORT') or '/dev/ttyAMA0').strip()
    start_script = f'''
set -e
source /opt/ros/humble/setup.bash
source /home/ws/ugv_ws/install/setup.bash 2>/dev/null || true
mkdir -p /tmp/ugv_ros_logs
nohup ros2 run ugv_bringup ugv_bringup --ros-args \
  -p serial_port:={port} -p baud_rate:=115200 \
  > /tmp/ugv_ros_logs/bringup.log 2>&1 &
sleep 0.4
# confirm process
if pgrep -f '/lib/ugv_bringup/ugv_bringup' >/dev/null 2>&1; then
  echo bringup_ok
  exit 0
fi
# looser match
if pgrep -af ugv_bringup | grep -v pgrep | grep -q ugv_bringup; then
  echo bringup_ok_loose
  exit 0
fi
echo bringup_start_uncertain
tail -5 /tmp/ugv_ros_logs/bringup.log 2>/dev/null || true
exit 0
'''
    ok_st, so2, se2, code = _docker_exec(start_script, detach=False, timeout=40)
    out['started'] = 'bringup_ok' in (so2 or '')
    out['ok'] = out['started'] or ('bringup_ok' in (so2 or ''))
    out['detail'] = so2 or se2 or f'exit {code}'
    if out['ok']:
        olog.info('ros_autostart', 'ugv_bringup started in container',
                  component='bringup', started=True)
    else:
        # still mark started attempt — process may be up without our match
        out['started'] = bool(ok_st)
        out['ok'] = bool(ok_st)
        olog.info(
            'ros_autostart',
            f'ugv_bringup start attempted: {out["detail"][:200]}',
            component='bringup', ok=out['ok'],
        )
    return out


def _stop_ugv_bringup():
    """Stop ugv_bringup in the ROS container so Flask can reclaim UART.

    Called when leaving ROS 2 for Direct. Does not stop rosbridge.
    Env: UGV_AUTOSTOP_BRINGUP=1 (default on). Kill is by PID inside the
    container (not host `pkill -f`) so the docker-exec wrapper is not matched.
    """
    out = {
        'wanted': True,
        'stopped': False,
        'already_down': False,
        'ok': False,
        'detail': '',
        'pids': [],
    }
    if not _env_flag('UGV_AUTOSTOP_BRINGUP', '1'):
        out['wanted'] = False
        out['detail'] = 'UGV_AUTOSTOP_BRINGUP disabled'
        return out

    try:
        from ros_motion import parse_ugv_bringup_pids
    except Exception as e:
        out['detail'] = f'parse helper import failed: {e}'
        olog.warn('ros_autostop', out['detail'], component='bringup')
        return out

    _ok_chk, so, se, _ = _docker_exec(
        "ps -eo pid,args 2>/dev/null | grep -F ugv_bringup || true",
        detach=False, timeout=15,
    )
    pids = parse_ugv_bringup_pids(so)
    out['pids'] = pids
    if not pids:
        out['already_down'] = True
        out['ok'] = True
        if se and ('docker CLI not found' in se or 'No such container' in se
                   or 'Cannot connect' in se):
            out['detail'] = f'bringup stop skipped: {se[:160]}'
        else:
            out['detail'] = 'ugv_bringup not running'
        return out

    pid_list = ' '.join(str(p) for p in pids)
    kill_script = (
        'set +e\n'
        f'kill -TERM {pid_list} 2>/dev/null || true\n'
        'sleep 0.4\n'
        f'kill -KILL {pid_list} 2>/dev/null || true\n'
        'sleep 0.15\n'
        'ps -eo pid,args 2>/dev/null | grep -F ugv_bringup || true\n'
    )
    _ok_st, so2, se2, _code = _docker_exec(kill_script, detach=False, timeout=20)
    leftover = parse_ugv_bringup_pids(so2)
    out['stopped'] = not leftover
    out['ok'] = not leftover
    if leftover:
        out['detail'] = f'still running after kill: {leftover}'
        olog.warn(
            'ros_autostop',
            out['detail'][:240],
            component='bringup', leftover=leftover, pids=pids,
        )
    else:
        out['detail'] = f'stopped pids {pids}'
        olog.info(
            'ros_autostop',
            'ugv_bringup stopped in container',
            component='bringup', stopped=True, pids=pids,
        )
    return out


def _ensure_ros2_sidecar_stack():
    """When entering ros2 mode: ensure rosbridge (+ optionally bringup) are up."""
    result = {
        'rosbridge': _ensure_rosbridge_running(),
        'bringup': None,
    }
    # bringup needs UART free (caller should have released serial already)
    try:
        result['bringup'] = _ensure_ugv_bringup_running()
    except Exception as e:
        result['bringup'] = {'wanted': True, 'ok': False, 'detail': str(e)}
    return result


# Booted in Direct: drop leftover bringup so a Flask restart can reclaim UART.
# (set_control_mode() at import time cannot call this — helpers are defined here.)
if get_control_mode() == 'direct':
    try:
        _startup_stop = _stop_ugv_bringup()
        if _startup_stop.get('stopped'):
            base.claim_serial_for_flask()
            olog.info(
                'ros_autostop',
                'startup Direct: stopped leftover ugv_bringup',
                **{k: _startup_stop.get(k) for k in ('ok', 'stopped', 'detail', 'pids')},
            )
    except Exception as e:
        olog.warn(
            'ros_autostop',
            f'startup Direct bringup stop failed: {e}',
            error=str(e)[:200],
        )


# Background ROS2 heal: keep rosbridge/bringup up while control_mode=ros2.
# Without this, a dead :9090 after a one-shot autostart left PTZ on "dropped serial".
_ros_autoheal_lock = threading.Lock()
_ros_autoheal_state = {
    'enabled': True,
    'last_tick_at': None,
    'last_action': None,
    'last_detail': '',
    'last_ok': None,
    'consecutive_down': 0,
    'heals_attempted': 0,
    'heals_ok': 0,
}


def _ros2_autoheal_tick():
    """One heal cycle. Safe to call from a background thread.

    Policy when control_mode is ros2:
      - rosbridge up  → prefer UART released for bringup; re-release if we held fallback serial
      - rosbridge down → restart sidecar stack; if still down, reclaim serial (PTZ lives)
    Disabled with UGV_ROS_AUTOHEAL=0. Interval: UGV_ROS_AUTOHEAL_S (default 15).
    """
    global _ros_autoheal_state
    if not _env_flag('UGV_ROS_AUTOHEAL', '1'):
        with _ros_autoheal_lock:
            _ros_autoheal_state['enabled'] = False
            _ros_autoheal_state['last_action'] = 'disabled'
        return {'ok': True, 'action': 'disabled'}

    if get_control_mode() != 'ros2':
        with _ros_autoheal_lock:
            _ros_autoheal_state['last_action'] = 'skip_not_ros2'
            _ros_autoheal_state['last_ok'] = True
            _ros_autoheal_state['consecutive_down'] = 0
        return {'ok': True, 'action': 'skip_not_ros2'}

    bridge_ok = _rosbridge_reachable()
    action = 'healthy'
    detail = ''

    if bridge_ok:
        # Prefer ROS owning UART when the bridge is healthy.
        if not getattr(base, 'serial_released_for_ros', False) or base.serial_is_open():
            # We were on serial fallback — hand UART back so bringup can drive
            base.enable_motor_control = False
            base.release_serial_for_ros()
            try:
                _ensure_ugv_bringup_running()
            except Exception as e:
                detail = f'released serial; bringup ensure: {e}'
            else:
                detail = 'rosbridge healthy; UART released for bringup'
            action = 'released_serial_for_ros'
            olog.info(
                'ros_autoheal',
                detail,
                rosbridge_ok=True,
                serial_released=True,
            )
        else:
            detail = 'rosbridge healthy; UART already released'
            action = 'healthy'
        with _ros_autoheal_lock:
            _ros_autoheal_state.update({
                'enabled': True,
                'last_tick_at': time.time(),
                'last_action': action,
                'last_detail': detail,
                'last_ok': True,
                'consecutive_down': 0,
            })
        return {'ok': True, 'action': action, 'detail': detail}

    # Bridge down — try restart sidecars (may need UART free for bringup)
    with _ros_autoheal_lock:
        _ros_autoheal_state['heals_attempted'] = int(
            _ros_autoheal_state.get('heals_attempted') or 0
        ) + 1
        _ros_autoheal_state['consecutive_down'] = int(
            _ros_autoheal_state.get('consecutive_down') or 0
        ) + 1
        down_n = _ros_autoheal_state['consecutive_down']

    olog.warn(
        'ros_autoheal',
        f'rosbridge down (tick #{down_n}) — ensuring sidecar stack',
        consecutive_down=down_n,
        serial_open=base.serial_is_open(),
        serial_released=bool(getattr(base, 'serial_released_for_ros', False)),
    )

    # Free UART before bringup restart if we hold it
    if base.serial_is_open() or not getattr(base, 'serial_released_for_ros', False):
        base.enable_motor_control = False
        base.release_serial_for_ros()

    try:
        stack = _ensure_ros2_sidecar_stack()
    except Exception as e:
        stack = {'error': str(e)}
        olog.warn('ros_autoheal', f'sidecar ensure failed: {e}', error=str(e)[:200])

    bridge_ok = _rosbridge_reachable()
    if bridge_ok:
        action = 'healed_rosbridge'
        detail = 'rosbridge restored by autoheal'
        with _ros_autoheal_lock:
            _ros_autoheal_state['heals_ok'] = int(
                _ros_autoheal_state.get('heals_ok') or 0
            ) + 1
            _ros_autoheal_state['consecutive_down'] = 0
        olog.info('ros_autoheal', detail, stack=str(stack)[:200])
    else:
        # Stay useful: serial fallback so PTZ is not dead
        reclaimed = _ensure_flask_serial(reason='ros_autoheal_fallback')
        action = 'serial_fallback'
        detail = (
            'rosbridge still down after ensure — serial reclaimed for PTZ/drive. '
            f'stack={str(stack)[:160]}'
        )
        olog.warn(
            'ros_autoheal',
            detail,
            reclaimed=reclaimed,
            serial_open=base.serial_is_open(),
        )

    with _ros_autoheal_lock:
        _ros_autoheal_state.update({
            'enabled': True,
            'last_tick_at': time.time(),
            'last_action': action,
            'last_detail': detail[:300],
            'last_ok': bridge_ok,
        })
    return {
        'ok': bridge_ok,
        'action': action,
        'detail': detail,
        'stack': stack,
    }


def _ros2_autoheal_loop():
    """Daemon: periodic rosbridge heal while ROS2 control mode is selected."""
    # First tick after short delay so startup autostart can finish
    time.sleep(8.0)
    while True:
        try:
            interval = float(os.environ.get('UGV_ROS_AUTOHEAL_S') or 15)
        except (TypeError, ValueError):
            interval = 15.0
        interval = max(5.0, min(120.0, interval))
        try:
            if _env_flag('UGV_ROS_AUTOHEAL', '1') and get_control_mode() == 'ros2':
                _ros2_autoheal_tick()
        except Exception as e:
            try:
                olog.warn('ros_autoheal', f'tick error: {e}', error=str(e)[:200])
            except Exception:
                print(f'[ros_autoheal] tick error: {e}')
        time.sleep(interval)


def _start_ros2_autoheal_thread():
    if getattr(_start_ros2_autoheal_thread, '_started', False):
        return
    if not _env_flag('UGV_ROS_AUTOHEAL', '1'):
        olog.info('ros_autoheal', 'UGV_ROS_AUTOHEAL disabled')
        return
    t = threading.Thread(target=_ros2_autoheal_loop, name='ros2-autoheal', daemon=True)
    t.start()
    _start_ros2_autoheal_thread._started = True  # type: ignore[attr-defined]
    olog.info(
        'ros_autoheal',
        'ROS2 autoheal thread started '
        f'(interval≈{os.environ.get("UGV_ROS_AUTOHEAL_S") or 15}s, only when control_mode=ros2)',
    )


# Start after function defs (module load order) — heal while control_mode=ros2
_start_ros2_autoheal_thread()


@app.route('/api/control_mode', methods=['GET', 'POST'])
def api_control_mode():
    """Get or set unified routing: direct (serial) vs ros2 (rosbridge)."""
    if request.method == 'GET':
        return jsonify(_control_mode_payload(mode_changed=False))
    data = request.get_json(silent=True) or {}
    mode = data.get('mode') or data.get('control_mode')
    if not mode and data.get('toggle'):
        mode = 'direct' if get_control_mode() == 'ros2' else 'ros2'
    if not mode:
        return jsonify({'success': False, 'error': "provide mode: 'direct' or 'ros2'"}), 400
    prev = get_control_mode()
    try:
        mode = set_control_mode(mode, source='api')
    except ValueError as e:
        return jsonify({'success': False, 'error': str(e)}), 400
    payload = _control_mode_payload(mode, mode_changed=(prev != mode), prev_mode=prev)
    payload['mode_changed'] = prev != mode
    payload['prev_mode'] = prev
    # Only force restart banner if autostart failed / bridge still down
    if mode == 'ros2':
        auto = payload.get('ros_autostart') or {}
        rb_ok = bool((payload.get('rosbridge') or {}).get('ok'))
        if not rb_ok and (data.get('toggle') or prev != mode):
            payload['restart_required'] = True
        elif rb_ok:
            payload['restart_required'] = False
    elif data.get('force_restart_hint'):
        payload['restart_required'] = True
    return jsonify(payload)

@app.route('/api/toggle_motors', methods=['POST'])
def api_toggle_motors():
    """Legacy: flip between direct serial and ROS 2 relay (same as control_mode toggle)."""
    prev = get_control_mode()
    mode = 'direct' if prev == 'ros2' else 'ros2'
    mode = set_control_mode(mode, source='ui_toggle')
    payload = _control_mode_payload(mode, mode_changed=True, prev_mode=prev)
    if mode == 'ros2' and (payload.get('rosbridge') or {}).get('ok'):
        payload['restart_required'] = False
    elif mode == 'direct':
        # Bringup may still hold UART until container restart
        payload['restart_required'] = True
    return jsonify(payload)


@app.route('/api/stack_restart', methods=['POST'])
def api_stack_restart():
    """Restart ROS docker container so it re-opens UART after control_mode change.

    Body: {"target": "ugv_ros2"}  (default from UGV_ROS_CONTAINER)
    Requires docker CLI access for the Flask user (group docker).
    """
    data = request.get_json(silent=True) or {}
    target = (data.get('target') or data.get('container')
              or os.environ.get('UGV_ROS_CONTAINER') or 'ugv_ros2')
    ok, msg, detail = _docker_restart_container(target)
    mode = get_control_mode()
    return jsonify({
        'success': ok,
        'target': target,
        'message': msg,
        'detail': detail,
        'control_mode': mode,
        'serial_open': base.serial_is_open() if hasattr(base, 'serial_is_open') else None,
        'serial_released_for_ros': bool(getattr(base, 'serial_released_for_ros', False)),
        'note': (
            'After restart, re-launch ugv_bringup + rosbridge inside the container if they '
            'are not started by the container entrypoint.'
            if ok and mode == 'ros2' else
            ('ROS container restarted; UART should be free for Flask direct mode.'
             if ok else msg)
        ),
    }), (200 if ok else 500)


# ---------------------------------------------------------------------------
# ESP32 WiFi (lower computer SoftAP "UGV" etc.)
#
# Safe for boot AP behavior:
#   {"T":408}  CMD_WIFI_STOP  → runtime disconnect only (wifiStop() in firmware).
#   Does NOT write LittleFS /wifiConfig.json. Next power-on still uses boot mode.
#
# Do NOT use for temporary off:
#   {"T":401,"cmd":0}  CMD_WIFI_ON_BOOT → configWifiModeOnBoot() PERSISTS off to
#   wifiConfig.json. That would stop the AP advertising on every future boot.
# ---------------------------------------------------------------------------
_esp32_wifi_session = {
    'stopped': False,   # True after we sent T:408 this Flask process lifetime
    'last_action': None,
    'last_error': None,
}


def _esp32_wifi_stop_session(*, source='api'):
    """Runtime WiFi stop on ESP32. Non-persistent. Returns result dict."""
    try:
        base.base_json_ctrl({'T': 408})
        _esp32_wifi_session['stopped'] = True
        _esp32_wifi_session['last_action'] = 'stop'
        _esp32_wifi_session['last_error'] = None
        olog.info(
            'esp32_wifi',
            'ESP32 WiFi STOP (T:408) — session only; boot AP config unchanged',
            action='stop', T=408, persistent=False, source=source, success=True,
        )
        return {
            'success': True,
            'action': 'stop',
            'command': {'T': 408},
            'persistent': False,
            'stopped': True,
            'note': (
                'Sent CMD_WIFI_STOP (T:408). Runtime only — does not change '
                'wifi_mode_on_boot / wifiConfig.json. AP should return after ESP32 reboot.'
            ),
        }
    except Exception as e:
        _esp32_wifi_session['last_error'] = str(e)
        olog.error('esp32_wifi', f'ESP32 WiFi STOP failed: {e}',
                   action='stop', T=408, persistent=False, source=source, success=False, error=str(e))
        return {'success': False, 'action': 'stop', 'error': str(e), 'persistent': False}


def _esp32_wifi_start_ap_session(ssid=None, password=None, *, source='api'):
    """Re-enable SoftAP for this power cycle only (T:402). Does not write boot config."""
    ssid = ssid or os.environ.get('UGV_ESP32_AP_SSID') or 'UGV'
    password = password or os.environ.get('UGV_ESP32_AP_PASSWORD') or '12345678'
    try:
        cmd = {'T': 402, 'ssid': ssid, 'password': password}
        base.base_json_ctrl(cmd)
        _esp32_wifi_session['stopped'] = False
        _esp32_wifi_session['last_action'] = 'start_ap'
        _esp32_wifi_session['last_error'] = None
        olog.info(
            'esp32_wifi',
            f'ESP32 WiFi START AP (T:402 ssid={ssid!r}) — session only',
            action='start_ap', T=402, ssid=ssid, persistent=False, source=source, success=True,
        )
        return {
            'success': True,
            'action': 'start_ap',
            'command': {'T': 402, 'ssid': ssid},
            'persistent': False,
            'stopped': False,
            'note': (
                'Sent CMD_SET_AP (T:402). Runtime SoftAP only — does not rewrite '
                'wifiConfig.json boot settings.'
            ),
        }
    except Exception as e:
        _esp32_wifi_session['last_error'] = str(e)
        olog.error('esp32_wifi', f'ESP32 WiFi START AP failed: {e}',
                   action='start_ap', T=402, source=source, success=False, error=str(e))
        return {'success': False, 'action': 'start_ap', 'error': str(e), 'persistent': False}


@app.route('/api/esp32_wifi', methods=['GET', 'POST'])
def api_esp32_wifi():
    """Session-only ESP32 WiFi control over serial (never persists boot-off).

    GET  → last known session state (Flask-side; ESP32 not polled by default)
    POST {"action":"stop"}     → T:408 disconnect (safe; non-persistent)
    POST {"action":"start_ap"} → T:402 SoftAP for this power cycle
    POST {"action":"info"}     → T:405 request WiFi info on serial (debug)

    Never exposes T:401 (wifi_mode_on_boot) — that writes LittleFS and would
    kill the boot AP permanently until reconfigured.
    """
    if request.method == 'GET':
        return jsonify({
            'success': True,
            'stopped': bool(_esp32_wifi_session.get('stopped')),
            'last_action': _esp32_wifi_session.get('last_action'),
            'last_error': _esp32_wifi_session.get('last_error'),
            'persistent': False,
            'safe_commands': {
                'stop': {'T': 408},
                'start_ap': {'T': 402, 'ssid': 'UGV', 'password': '(default or env)'},
                'info': {'T': 405},
            },
            'danger_do_not_use_for_session_off': {
                'wifi_on_boot_off': {'T': 401, 'cmd': 0},
                'why': 'Writes wifiConfig.json; AP stays off across reboots.',
            },
            'note': (
                'Stop uses firmware wifiStop() only. Boot AP (SSID UGV) returns '
                'after ESP32 power cycle / reboot.'
            ),
        })

    data = request.get_json(silent=True) or {}
    action = (data.get('action') or data.get('cmd') or '').strip().lower()
    if action in ('stop', 'off', 'disable'):
        result = _esp32_wifi_stop_session(source='api')
        return jsonify(result), (200 if result.get('success') else 500)
    if action in ('start_ap', 'start', 'on', 'enable', 'ap'):
        result = _esp32_wifi_start_ap_session(
            ssid=data.get('ssid'),
            password=data.get('password'),
            source='api',
        )
        return jsonify(result), (200 if result.get('success') else 500)
    if action in ('info', 'status_query'):
        try:
            base.base_json_ctrl({'T': 405})
            olog.info('esp32_wifi', 'ESP32 WiFi INFO requested (T:405)', action='info', T=405)
            return jsonify({
                'success': True,
                'action': 'info',
                'command': {'T': 405},
                'persistent': False,
                'note': 'CMD_WIFI_INFO sent; response prints on ESP32 serial if echo/debug on.',
            })
        except Exception as e:
            olog.error('esp32_wifi', f'ESP32 WiFi INFO failed: {e}', action='info', error=str(e))
            return jsonify({'success': False, 'action': 'info', 'error': str(e)}), 500
    if action in ('on_boot', 'persist', '401'):
        olog.warn(
            'esp32_wifi',
            'Refused persistent WiFi-off (T:401) — would kill boot AP',
            action=action, refused_T=401,
        )
        return jsonify({
            'success': False,
            'error': (
                'Refusing CMD_WIFI_ON_BOOT (T:401). It writes wifiConfig.json and '
                'changes boot WiFi permanently. Use action=stop (T:408) for session-only off.'
            ),
        }), 400
    return jsonify({
        'success': False,
        'error': "action must be 'stop', 'start_ap', or 'info'",
    }), 400


@app.route('/api/ptz', methods=['GET', 'POST'])
def api_ptz():
    """Interrogate / command pan-tilt.

    GET  → hardware pan/tilt from ESP32 feedback (T:1001) + last commanded angles
    POST {"action":"status"} same as GET
    POST {"action":"goto","x":20,"y":0}  → T:133 then re-read positions
    POST {"action":"center"}             → T:133 X=0 Y=0
    POST {"action":"feedback"}           → force T:130 only
    """
    if request.method == 'GET':
        snap = _ptz_status_snapshot(request_feedback=True)
        # Polled often (UI/3D twin); keep out of App log at info+
        olog.debug(
            'ptz_status',
            f'PTZ status hw_pan={snap["hardware"].get("pan_deg")} '
            f'hw_tilt={snap["hardware"].get("tilt_deg")}',
            hw_pan=snap['hardware'].get('pan_deg'),
            hw_tilt=snap['hardware'].get('tilt_deg'),
            feedback_T=snap['hardware'].get('feedback_T'),
            control_mode=snap['control_mode'],
        )
        return jsonify(snap)

    data = request.get_json(silent=True) or {}
    action = data.get('action')
    if not action:
        if any(k in data for k in ('x', 'X', 'pan', 'y', 'Y', 'tilt')):
            action = 'goto'
        else:
            action = 'status'
    action = str(action).strip().lower()
    if action in ('status', 'read', 'query'):
        return jsonify(_ptz_status_snapshot(request_feedback=True))
    if action in ('feedback',):
        return jsonify(_ptz_status_snapshot(request_feedback=True, wait_s=0.5))
    if action in ('center', 'home', 'goto', 'move', 'set'):
        if action in ('center', 'home'):
            x, y = 0.0, 0.0
        else:
            x = float(data.get('x', data.get('X', data.get('pan', 0))) or 0)
            y = float(data.get('y', data.get('Y', data.get('tilt', 0))) or 0)
        spd = float(data.get('spd', data.get('SPD', 0)) or 0)
        acc = float(data.get('acc', data.get('ACC', 0)) or 0)
        cmd = {'T': 133, 'X': x, 'Y': y, 'SPD': spd, 'ACC': acc}
        mode = get_control_mode()
        path = 'serial'
        before = _ptz_status_snapshot(request_feedback=(mode == 'direct'))
        before_pan = (before.get('hardware') or {}).get('pan_deg')
        before_tilt = (before.get('hardware') or {}).get('tilt_deg')
        try:
            used_ros = False
            if mode == 'ros2' and _rosbridge_reachable():
                import ros_motion
                result = ros_motion.publish_gimbal_from_ui(x, y, throttle=False)
                if result.get('ok'):
                    path = 'ros2'
                    used_ros = True
                    olog.info('ptz_cmd', f'PTZ goto X={x} Y={y} (ros2)', X=x, Y=y, path='ros2', ok=True)
                else:
                    olog.warn(
                        'ptz_cmd',
                        f'PTZ ros2 failed, serial fallback: {result.get("error")}',
                        X=x, Y=y, error=str(result.get('error') or '')[:160],
                    )
            if not used_ros:
                if mode == 'ros2' and (
                    getattr(base, 'serial_released_for_ros', False) or not base.serial_is_open()
                ):
                    if not _ensure_flask_serial(reason='ptz_api'):
                        return jsonify({
                            'success': False,
                            'error': 'rosbridge down and serial reclaim failed — set Control: Direct',
                            'command': cmd,
                            'path': 'none',
                        }), 500
                base.base_json_ctrl({'T': 4, 'cmd': 2})  # gimbal module
                base.base_json_ctrl(cmd)
                path = 'serial' if mode == 'direct' else 'serial_fallback'
                olog.info('ptz_cmd', f'PTZ goto X={x} Y={y} ({path} T:133)', X=x, Y=y, path=path)
            _publish_ptz_aim(x, y, settled=False, source='api_ptz')
        except Exception as e:
            olog.error('ptz_cmd', f'PTZ goto failed: {e}', error=str(e))
            return jsonify({'success': False, 'error': str(e), 'command': cmd}), 500
        time.sleep(float(data.get('wait_s', 0.6) or 0.6))
        snap = _ptz_status_snapshot(request_feedback=(path == 'serial'))
        after_pan = (snap.get('hardware') or {}).get('pan_deg')
        after_tilt = (snap.get('hardware') or {}).get('tilt_deg')
        def _delta(a, b):
            try:
                return abs(float(a) - float(b))
            except (TypeError, ValueError):
                return None
        d_pan = _delta(before_pan, after_pan)
        d_tilt = _delta(before_tilt, after_tilt)
        # Command accepted ≠ servos moved. Flag no-op for hardware debug.
        moved = bool((d_pan is not None and d_pan > 2.0) or (d_tilt is not None and d_tilt > 2.0))
        snap['command_sent'] = cmd
        snap['path'] = path
        snap['before'] = {'pan_deg': before_pan, 'tilt_deg': before_tilt}
        snap['delta_deg'] = {'pan': d_pan, 'tilt': d_tilt}
        snap['moved'] = moved
        if not moved:
            snap['warning'] = (
                'Command was sent on the software path but hardware pan/tilt feedback '
                'did not change >2°. Check bus-servo power, wiring, IDs, and that the '
                'PT module is fitted; pan stuck at one LSB often means no servo reply.'
            )
            olog.warn(
                'ptz_cmd',
                f'PTZ goto X={x} Y={y} sent but no HW movement (pan {before_pan}→{after_pan})',
                X=x, Y=y, path=path, before_pan=before_pan, after_pan=after_pan,
                before_tilt=before_tilt, after_tilt=after_tilt, moved=False,
            )
        _publish_ptz_aim(
            x, y, hw_pan=after_pan, hw_tilt=after_tilt,
            settled=moved, source='api_ptz',
        )
        return jsonify(snap)
    return jsonify({'success': False, 'error': "action must be status|goto|center|feedback"}), 400


def _ptz_goto_raw(x, y, wait_s=1.5):
    """Send T:133 (or ROS gimbal) and return movement diagnostics + HW snapshot."""
    x, y = float(x), float(y)
    wait_s = max(0.3, min(5.0, float(wait_s)))
    cmd = {'T': 133, 'X': x, 'Y': y, 'SPD': 0.0, 'ACC': 0.0}
    mode = get_control_mode()
    path = 'serial'
    before = _ptz_status_snapshot(request_feedback=(mode == 'direct'))
    before_pan = (before.get('hardware') or {}).get('pan_deg')
    before_tilt = (before.get('hardware') or {}).get('tilt_deg')
    try:
        used_ros = False
        if mode == 'ros2' and _rosbridge_reachable():
            import ros_motion
            result = ros_motion.publish_gimbal_from_ui(x, y, throttle=False)
            if result.get('ok'):
                path = 'ros2'
                used_ros = True
            else:
                olog.warn(
                    'ptz_cmd',
                    f'_ptz_goto_raw ros2 fail → serial: {result.get("error")}',
                    error=str(result.get('error') or '')[:160],
                )
        if not used_ros:
            if mode == 'ros2' and (
                getattr(base, 'serial_released_for_ros', False) or not base.serial_is_open()
            ):
                if not _ensure_flask_serial(reason='ptz_goto_raw'):
                    return {
                        'success': False,
                        'error': 'rosbridge down and serial reclaim failed — set Control: Direct',
                        'command_sent': cmd,
                        'path': 'none',
                        'before': {'pan_deg': before_pan, 'tilt_deg': before_tilt},
                        'moved': False,
                    }
            path = 'serial' if mode == 'direct' else 'serial_fallback'
            base.base_json_ctrl({'T': 4, 'cmd': 2})
            base.base_json_ctrl(cmd)
        _publish_ptz_aim(x, y, settled=False, source='ptz_goto_raw')
    except Exception as e:
        return {
            'success': False, 'error': str(e), 'command_sent': cmd, 'path': path,
            'before': {'pan_deg': before_pan, 'tilt_deg': before_tilt}, 'moved': False,
        }
    time.sleep(wait_s)
    snap = _ptz_status_snapshot(request_feedback=(path == 'serial'))
    after_pan = (snap.get('hardware') or {}).get('pan_deg')
    after_tilt = (snap.get('hardware') or {}).get('tilt_deg')

    def _delta(a, b):
        try:
            return abs(float(a) - float(b))
        except (TypeError, ValueError):
            return None

    d_pan = _delta(before_pan, after_pan)
    d_tilt = _delta(before_tilt, after_tilt)
    moved = bool((d_pan is not None and d_pan > 2.0) or (d_tilt is not None and d_tilt > 2.0))
    return {
        'success': True,
        'command_sent': cmd,
        'path': path,
        'before': {'pan_deg': before_pan, 'tilt_deg': before_tilt},
        'after': {'pan_deg': after_pan, 'tilt_deg': after_tilt},
        'delta_deg': {'pan': d_pan, 'tilt': d_tilt},
        'moved': moved,
        'hardware': snap.get('hardware'),
        'control_mode': snap.get('control_mode'),
    }


def _jpeg_compare(jpeg_a, jpeg_b):
    """Compare two JPEGs; return metrics used by PTZ photo self-test."""
    try:
        import cv2
        import numpy as np
        ia = cv2.imdecode(np.frombuffer(jpeg_a, dtype=np.uint8), cv2.IMREAD_COLOR)
        ib = cv2.imdecode(np.frombuffer(jpeg_b, dtype=np.uint8), cv2.IMREAD_COLOR)
        if ia is None or ib is None:
            return {'error': 'decode failed', 'visual_change': False}
        if ia.shape != ib.shape:
            ib = cv2.resize(ib, (ia.shape[1], ia.shape[0]))
        ga = cv2.cvtColor(ia, cv2.COLOR_BGR2GRAY).astype(np.float32)
        gb = cv2.cvtColor(ib, cv2.COLOR_BGR2GRAY).astype(np.float32)
        diff = np.abs(ga - gb)
        mad = float(diff.mean())
        changed = float((diff > 15).mean())
        try:
            shift, response = cv2.phaseCorrelate(ga, gb)
            shift_xy = [float(shift[0]), float(shift[1])]
            response = float(response)
        except Exception:
            shift_xy, response = [0.0, 0.0], 0.0
        visual = mad > 3.0 or changed > 0.05 or abs(shift_xy[0]) > 2.0 or abs(shift_xy[1]) > 2.0
        # Compact side-by-side JPEG for UI
        try:
            scale = min(1.0, 320.0 / max(ia.shape[1], 1))
            if scale < 1.0:
                ia_s = cv2.resize(ia, None, fx=scale, fy=scale)
                ib_s = cv2.resize(ib, None, fx=scale, fy=scale)
            else:
                ia_s, ib_s = ia, ib
            panel = np.hstack([ia_s, ib_s])
            ok, buf = cv2.imencode('.jpg', panel, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            panel_b64 = base64.b64encode(buf.tobytes()).decode('ascii') if ok else None
        except Exception:
            panel_b64 = None
        return {
            'mean_abs_diff': round(mad, 3),
            'frac_changed_gt15': round(changed, 5),
            'phase_shift_xy': shift_xy,
            'phase_response': round(response, 4),
            'visual_change': visual,
            'panel_data_url': (f'data:image/jpeg;base64,{panel_b64}' if panel_b64 else None),
        }
    except Exception as e:
        return {'error': str(e)[:200], 'visual_change': False}


def _snap_jpeg_b64():
    jpeg = _grab_jpeg_bytes(max_width=640, quality=70)
    if not jpeg:
        return None, None
    return jpeg, f'data:image/jpeg;base64,{base64.b64encode(jpeg).decode("ascii")}'


@app.route('/api/ptz/self_test', methods=['POST'])
def api_ptz_self_test():
    """Photo + status self-test for pan and tilt.

    Sequence: center → snap → pan L/R → snaps → center → tilt up/down → snaps → center.
    Returns per-step HW deltas, image metrics, side-by-side panels, and a verdict.
    """
    data = request.get_json(silent=True) or {}
    wait_s = float(data.get('wait_s', 1.8) or 1.8)
    wait_s = max(0.8, min(4.0, wait_s))
    pan_deg = float(data.get('pan_deg', 35) or 35)
    tilt_deg = float(data.get('tilt_deg', 30) or 30)
    pan_deg = max(10.0, min(60.0, abs(pan_deg)))
    tilt_deg = max(10.0, min(50.0, abs(tilt_deg)))
    include_photos = data.get('include_photos', True)

    steps = []
    t0 = time.time()

    def _step(name, cmd_x, cmd_y, prev_jpeg=None):
        goto = _ptz_goto_raw(cmd_x, cmd_y, wait_s=wait_s)
        jpeg, data_url = (None, None)
        if include_photos:
            try:
                jpeg, data_url = _snap_jpeg_b64()
            except Exception as e:
                jpeg, data_url = None, None
                goto['photo_error'] = str(e)[:160]
        photo_cmp = None
        if prev_jpeg and jpeg:
            photo_cmp = _jpeg_compare(prev_jpeg, jpeg)
        entry = {
            'name': name,
            'command': goto.get('command_sent'),
            'path': goto.get('path'),
            'success': goto.get('success'),
            'moved_hw': bool(goto.get('moved')),
            'delta_deg': goto.get('delta_deg'),
            'before': goto.get('before'),
            'after': goto.get('after'),
            'error': goto.get('error'),
            'photo_data_url': data_url if include_photos else None,
            'photo_compare': photo_cmp,
        }
        steps.append(entry)
        return jpeg, entry

    try:
        # Center / baseline
        j_base, _ = _step('baseline_center', 0, 0, None)
        # Pan left / right
        j_pl, _ = _step('pan_left', -pan_deg, 0, j_base)
        j_pr, _ = _step('pan_right', pan_deg, 0, j_pl)
        # Re-center before tilt
        j_mid, _ = _step('recenter', 0, 0, j_pr)
        # Tilt up / down
        j_tu, _ = _step('tilt_up', 0, tilt_deg, j_mid)
        j_td, _ = _step('tilt_down', 0, -min(tilt_deg, 25.0), j_tu)
        # Final center
        _step('final_center', 0, 0, j_td)
    except Exception as e:
        olog.error('ptz_self_test', f'Self-test crashed: {e}', error=str(e)[:300])
        return jsonify({'success': False, 'error': str(e)[:300], 'steps': steps}), 500

    pan_hw = any(
        s.get('moved_hw') and s.get('name', '').startswith('pan')
        for s in steps
    )
    tilt_hw = any(
        s.get('moved_hw') and s.get('name', '').startswith('tilt')
        for s in steps
    )
    # Also accept large HW span across steps even if pairwise moved failed thresholds oddly
    pans = []
    tilts = []
    for s in steps:
        a = s.get('after') or {}
        if a.get('pan_deg') is not None:
            try:
                pans.append(float(a['pan_deg']))
            except (TypeError, ValueError):
                pass
        if a.get('tilt_deg') is not None:
            try:
                tilts.append(float(a['tilt_deg']))
            except (TypeError, ValueError):
                pass
    pan_span = (max(pans) - min(pans)) if pans else 0.0
    tilt_span = (max(tilts) - min(tilts)) if tilts else 0.0
    if pan_span > 5.0:
        pan_hw = True
    if tilt_span > 5.0:
        tilt_hw = True

    pan_photo = any(
        (s.get('photo_compare') or {}).get('visual_change')
        and s.get('name', '').startswith('pan')
        for s in steps
    )
    tilt_photo = any(
        (s.get('photo_compare') or {}).get('visual_change')
        and s.get('name', '').startswith('tilt')
        for s in steps
    )

    if pan_hw or pan_photo:
        pan_result = 'pass'
    else:
        pan_result = 'fail'
    if tilt_hw or tilt_photo:
        tilt_result = 'pass'
    else:
        tilt_result = 'fail'

    if pan_result == 'pass' and tilt_result == 'pass':
        overall = 'both_axes_ok'
        summary = 'Pan and tilt both show movement (HW and/or photo).'
    elif pan_result == 'pass':
        overall = 'pan_ok_tilt_fail'
        summary = 'Pan moved; tilt did not — check tilt servo wiring/power/ID.'
    elif tilt_result == 'pass':
        overall = 'tilt_ok_pan_fail'
        summary = 'Tilt moved; pan did not — check pan servo wiring/power/ID.'
    else:
        overall = 'both_axes_fail'
        summary = (
            'Neither pan nor tilt proved movement. Commands were sent; check bus-servo '
            'power, daisy-chain plugs, and that the PT module is powered.'
        )

    verdict = {
        'overall': overall,
        'summary': summary,
        'pan': {
            'result': pan_result,
            'hw_moved': pan_hw,
            'photo_changed': pan_photo,
            'hw_span_deg': round(pan_span, 3),
        },
        'tilt': {
            'result': tilt_result,
            'hw_moved': tilt_hw,
            'photo_changed': tilt_photo,
            'hw_span_deg': round(tilt_span, 3),
        },
        'elapsed_s': round(time.time() - t0, 2),
        'control_mode': get_control_mode(),
        'pan_cmd_deg': pan_deg,
        'tilt_cmd_deg': tilt_deg,
    }
    olog.info(
        'ptz_self_test',
        f'PTZ self-test {overall}: {summary}',
        overall=overall, pan=pan_result, tilt=tilt_result,
        pan_span=round(pan_span, 3), tilt_span=round(tilt_span, 3),
        elapsed_s=verdict['elapsed_s'],
    )
    return jsonify({
        'success': True,
        'verdict': verdict,
        'steps': steps,
    })


@app.route('/api/logs', methods=['GET', 'POST', 'DELETE'])
def api_logs():
    """In-app ops log ring buffer.

    GET  ?since_id=0&limit=200&min_level=info
    POST {"level","event","msg", ...fields}  — allow UI to note client events
    DELETE — clear buffer
    """
    if request.method == 'DELETE':
        n = olog.clear()
        olog.info('app_log', f'Log cleared ({n} entries removed)', cleared=n, source='api')
        return jsonify({'success': True, 'cleared': n, **olog.stats()})
    if request.method == 'POST':
        data = request.get_json(silent=True) or {}
        level = data.get('level') or 'info'
        event = (data.get('event') or 'client').strip() or 'client'
        # Only allow a few client events to avoid spam
        allowed = {
            'ui', 'client', 'gamepad', 'ui_error', 'ui_warn',
            'esp32_wifi', 'control_mode', 'rtsp_toggle',
        }
        if event not in allowed:
            event = 'client'
        msg = data.get('msg') or data.get('message')
        fields = {k: v for k, v in data.items()
                  if k not in ('level', 'event', 'msg', 'message') and not str(k).startswith('_')}
        fields['source'] = fields.get('source') or 'browser'
        entry = olog.log(level, event, msg, **fields)
        return jsonify({'success': True, 'entry': entry, **olog.stats()})
    since_id = request.args.get('since_id', 0, type=int) or 0
    limit = request.args.get('limit', 200, type=int) or 200
    min_level = request.args.get('min_level') or request.args.get('level') or 'debug'
    event = request.args.get('event') or None
    entries = olog.get(since_id=since_id, limit=limit, min_level=min_level, event=event)
    return jsonify({
        'success': True,
        'entries': entries,
        **olog.stats(),
    })


# Optional: kill ESP32 SoftAP once Flask is up (session only). Boot AP still returns next power-on.
if os.environ.get('UGV_ESP32_WIFI_STOP_ON_START', '').lower() in ('1', 'true', 'yes'):
    try:
        _esp32_wifi_stop_session(source='env_on_start')
    except Exception as e:
        olog.error('esp32_wifi', f'UGV_ESP32_WIFI_STOP_ON_START failed: {e}', error=str(e))

# ---------- AI agent (local OpenAI-compatible LLM + camera vision) ----------
_snapshot_lock = threading.Lock()

def _ai_env_config():
    """LLM settings: stock OpenAI defaults in code; custom endpoint/model only via .env.

    Code only cares about OPENAI_API_KEY for default OpenAI usage.
    Set OPENAI_BASE_URL / OPENAI_MODEL in ugv_rpi/.env for local LiteLLM/Ollama/etc.
    """
    return {
        'api_key': os.environ.get('OPENAI_API_KEY') or '',
        # Official OpenAI default when OPENAI_BASE_URL is unset
        'base_url': (os.environ.get('OPENAI_BASE_URL') or 'https://api.openai.com/v1').rstrip('/'),
        # OpenAI requires a model name; override via OPENAI_MODEL in .env for custom infra
        'model': os.environ.get('OPENAI_MODEL') or 'gpt-4o-mini',
        # Chat / agent tool loops need a large completion budget so function
        # arguments are not truncated (reasoning models eat tokens first).
        'chat_max_tokens': _chat_max_tokens(),
    }


def _chat_max_tokens():
    """Max completion tokens for Chat / agent tool rounds. Default 8192."""
    raw = os.environ.get('UGV_CHAT_MAX_TOKENS') or os.environ.get('OPENAI_MAX_TOKENS')
    try:
        n = int(raw) if raw not in (None, '') else 8192
    except (TypeError, ValueError):
        n = 8192
    return max(1024, min(32768, n))

def _grab_jpeg_bytes(max_width=640, quality=70):
    """Capture one clean JPEG for AI/snapshot (no HUD, no failure placeholders).

    Uses grab_bgr_frame() so camera-unavailable placeholders from frame_process
    (human MJPEG feed) are never treated as real snapshots.
    """
    import cv2
    with _snapshot_lock:
        frame = cvf.grab_bgr_frame()
    if frame is None:
        raise RuntimeError('camera frame unavailable')
    h, w = frame.shape[:2]
    if w > max_width:
        frame = cv2.resize(frame, (max_width, int(h * max_width / w)))
    ok, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        raise RuntimeError('jpeg encode failed')
    return buf.tobytes()

_AI_SYSTEM_PROMPT = (
    "You are a vision-capable scout on a Waveshare UGV rover with a forward camera. "
    "When an image is attached, describe what you see clearly and briefly. "
    "Use tools when available: get_cv_detections for MobileNet-SSD labels (includes dog, person, cat, …), "
    "send_motor_command for short timed drives, send_gimbal_command to look around, stop_motors for safety. "
    "For search tasks: (1) inspect the current view / run detection, (2) if the target is missing drive "
    "forward in punchy timed steps (duration_ms 800–1600, linear_x ~0.22–0.28) down open hallways, "
    "(3) re-check with get_cv_detections after each move, (4) stop when the target is found or the path is blocked. "
    "Do not claim you cannot see or move if the matching tools are listed as callable. "
    "Prefer several short moves over one long continuous drive."
)

# Toggle tree: groups + leaf tools. Persisted under ugv_rpi/.ai_capabilities.json
_CAPABILITIES_PATH = os.path.join(thisPath, '.ai_capabilities.json')
_ai_capabilities_lock = threading.Lock()

# Hierarchy: enabling a child auto-enables parents; disabling a parent disables descendants.
# requires: extra nodes that must turn on with this node (e.g. drive implies stop).
_TOOL_TREE = [
    {
        'name': 'group_telemetry',
        'label': 'Telemetry',
        'kind': 'group',
        'description': 'On-board sensor readout tools.',
        'children': [
            {
                'name': 'get_telemetry',
                'label': 'get_telemetry',
                'kind': 'tool',
                'description': 'CPU, RAM, temp, voltage, pan/tilt, RSSI, FPS.',
            },
        ],
    },
    {
        'name': 'group_computer_vision',
        'label': 'Computer vision',
        'kind': 'group',
        'description': 'On-board OpenCV / MobileNet-SSD tools.',
        'children': [
            {
                'name': 'get_cv_detections',
                'label': 'get_cv_detections',
                'kind': 'tool',
                'description': 'MobileNet-SSD detections (labels, confidence, bboxes).',
            },
            {
                'name': 'get_camera_snapshot',
                'label': 'get_camera_snapshot',
                'kind': 'tool',
                'description': 'Live JPEG capture metadata.',
            },
        ],
    },
    {
        'name': 'group_ros2_motion',
        'label': 'Motion (direct / ROS 2)',
        'kind': 'group',
        'description': 'Chassis + gimbal. Routed by control_mode: direct serial or ROS 2 relay.',
        'needs_motion': True,
        'children': [
            {
                'name': 'send_motor_command',
                'label': 'send_motor_command',
                'kind': 'tool',
                'description': 'Drive chassis (serial T:13 or ROS /cmd_vel per control mode).',
                'needs_motion': True,
                'requires': ['stop_motors'],  # safety: driving implies stop tool on
            },
            {
                'name': 'send_gimbal_command',
                'label': 'send_gimbal_command',
                'kind': 'tool',
                'description': 'Pan/tilt (serial T:133 or ROS joints per control mode).',
                'needs_motion': True,
            },
            {
                'name': 'stop_motors',
                'label': 'stop_motors',
                'kind': 'tool',
                'description': 'Emergency stop (zero wheels on active path).',
                'needs_motion': True,
            },
        ],
    },
]

_MOTION_TOOLS = frozenset({'send_motor_command', 'send_gimbal_command', 'stop_motors'})

# AI timed-drive auto-stop (direct serial path). Cancelled on new drive/stop.
_DEFAULT_AI_DRIVE_MS = 1000
_ai_drive_timer_lock = threading.Lock()
_ai_drive_timer = None  # threading.Timer
_ai_drive_timer_gen = 0
# While set, zero-velocity UI chassis heartbeats (T:1 L=0/R=0 etc.) are ignored so
# the browser's 2s idle resend cannot cut short an AI timed/continuous drive.
_ai_motion_lock_until = 0.0
_AI_MOTION_LOCK_GRACE_S = 0.25


def _arm_ai_motion_lock(duration_ms=0, continuous=False):
    """Hold off zero UI chassis cmds for the AI maneuver window."""
    global _ai_motion_lock_until
    if continuous:
        # Until stop_motors / explicit clear (1h safety cap)
        _ai_motion_lock_until = time.time() + 3600.0
    else:
        _ai_motion_lock_until = (
            time.time() + max(0.0, float(duration_ms)) / 1000.0 + _AI_MOTION_LOCK_GRACE_S
        )


def _clear_ai_motion_lock():
    global _ai_motion_lock_until
    _ai_motion_lock_until = 0.0


def _ai_motion_lock_active():
    return time.time() < _ai_motion_lock_until


def _chassis_cmd_is_zero(cmd):
    """True for neutral/stop chassis payloads (UI idle heartbeat)."""
    if not isinstance(cmd, dict):
        return False
    try:
        L = float(cmd.get('L', 0) or 0)
        R = float(cmd.get('R', 0) or 0)
        X = float(cmd.get('X', cmd.get('x', 0)) or 0)
        Z = float(cmd.get('Z', cmd.get('z', 0)) or 0)
    except (TypeError, ValueError):
        return False
    eps = 1e-9
    return abs(L) < eps and abs(R) < eps and abs(X) < eps and abs(Z) < eps


def _cancel_ai_drive_timer():
    """Cancel any pending AI timed auto-stop on the direct serial path."""
    global _ai_drive_timer, _ai_drive_timer_gen
    with _ai_drive_timer_lock:
        _ai_drive_timer_gen += 1
        if _ai_drive_timer is not None:
            try:
                _ai_drive_timer.cancel()
            except Exception:
                pass
            _ai_drive_timer = None


def _schedule_ai_drive_stop(duration_ms):
    """Schedule serial zero-velocity after duration_ms; supersedes prior timer."""
    global _ai_drive_timer, _ai_drive_timer_gen
    with _ai_drive_timer_lock:
        _ai_drive_timer_gen += 1
        gen = _ai_drive_timer_gen
        if _ai_drive_timer is not None:
            try:
                _ai_drive_timer.cancel()
            except Exception:
                pass
            _ai_drive_timer = None

        def _fire():
            global _ai_drive_timer
            with _ai_drive_timer_lock:
                if gen != _ai_drive_timer_gen:
                    return
                _ai_drive_timer = None
            try:
                base.base_json_ctrl({'T': 13, 'X': 0, 'Z': 0})
            except Exception:
                pass
            # Allow UI heartbeats again shortly after our own stop
            # (lock window already covers duration + grace from arm time)

        t = threading.Timer(max(0.0, float(duration_ms)) / 1000.0, _fire)
        t.daemon = True
        _ai_drive_timer = t
        t.start()


def _iter_tree_nodes(nodes=None, parent=None, depth=0):
    for node in (nodes if nodes is not None else _TOOL_TREE):
        yield node, parent, depth
        for child in node.get('children') or []:
            yield from _iter_tree_nodes([child], parent=node['name'], depth=depth + 1)


def _all_node_names():
    return [n['name'] for n, _, _ in _iter_tree_nodes()]


def _node_by_name(name):
    for n, parent, depth in _iter_tree_nodes():
        if n['name'] == name:
            return n, parent, depth
    return None, None, None


def _parent_chain(name):
    """Ancestors from immediate parent up to root."""
    chain = []
    _, parent, _ = _node_by_name(name)
    while parent:
        chain.append(parent)
        _, parent, _ = _node_by_name(parent)
    return chain


def _descendant_names(name):
    node, _, _ = _node_by_name(name)
    if not node:
        return []
    out = []
    for n, _, _ in _iter_tree_nodes(node.get('children') or []):
        out.append(n['name'])
    return out


def _required_names(name, seen=None):
    """Transitive requires[] for a node."""
    seen = seen if seen is not None else set()
    node, _, _ = _node_by_name(name)
    if not node or name in seen:
        return []
    seen.add(name)
    reqs = []
    for r in node.get('requires') or []:
        reqs.append(r)
        reqs.extend(_required_names(r, seen))
    return reqs


# Motion tools default off for safer first boot; telemetry stays on.
_DEFAULT_TOOL_CAPS = {n: True for n in _all_node_names()}
_DEFAULT_TOOL_CAPS.update({
    'group_ros2_motion': False,
    'send_motor_command': False,
    'send_gimbal_command': False,
    'stop_motors': False,
})
_ai_capabilities = dict(_DEFAULT_TOOL_CAPS)


def _migrate_legacy_caps(data):
    """Map old group / tool keys into the tree."""
    out = {}
    if 'telemetry' in data or 'group_telemetry' in data:
        v = bool(data.get('group_telemetry', data.get('telemetry', True)))
        out['group_telemetry'] = v
        out['get_telemetry'] = bool(data.get('get_telemetry', v))
    if 'computer_vision' in data or 'group_computer_vision' in data:
        v = bool(data.get('group_computer_vision', data.get('computer_vision', True)))
        out['group_computer_vision'] = v
        out['get_cv_detections'] = bool(data.get('get_cv_detections', v))
        out['get_camera_snapshot'] = bool(data.get('get_camera_snapshot', v))
    if 'ros2_motion' in data or 'group_ros2_motion' in data:
        v = bool(data.get('group_ros2_motion', data.get('ros2_motion', True)))
        out['group_ros2_motion'] = v
        for n in _MOTION_TOOLS:
            out[n] = bool(data.get(n, v))
    return out


def _load_ai_capabilities():
    global _ai_capabilities
    try:
        if os.path.isfile(_CAPABILITIES_PATH):
            with open(_CAPABILITIES_PATH, 'r') as fh:
                data = json.load(fh)
            if isinstance(data, dict):
                merged = dict(_DEFAULT_TOOL_CAPS)
                merged.update(_migrate_legacy_caps(data))
                for k in _DEFAULT_TOOL_CAPS:
                    if k in data:
                        merged[k] = bool(data[k])
                _ai_capabilities = merged
    except Exception as e:
        print(f'[app.py] load capabilities: {e}')


def _save_ai_capabilities():
    try:
        with open(_CAPABILITIES_PATH, 'w') as fh:
            json.dump(_ai_capabilities, fh, indent=2)
    except Exception as e:
        print(f'[app.py] save capabilities: {e}')


_load_ai_capabilities()


def _get_capabilities():
    with _ai_capabilities_lock:
        return dict(_ai_capabilities)


def _apply_toggle(name, enabled, state):
    """Mutate state dict with dependency cascade. Returns list of changed keys."""
    if name not in state:
        return []
    enabled = bool(enabled)
    changed = []

    def _set(n, val):
        if n in state and state[n] != val:
            state[n] = val
            changed.append(n)

    if enabled:
        _set(name, True)
        # parents on
        for p in _parent_chain(name):
            _set(p, True)
        # required siblings/tools on (then their parents too)
        for r in _required_names(name):
            _set(r, True)
            for p in _parent_chain(r):
                _set(p, True)
        # if enabling a group, enable all descendants
        node, _, _ = _node_by_name(name)
        if node and node.get('kind') == 'group':
            for d in _descendant_names(name):
                _set(d, True)
    else:
        _set(name, False)
        # descendants off
        for d in _descendant_names(name):
            _set(d, False)
        # if all children of a parent are off, parent goes off
        for p in _parent_chain(name):
            kids = _descendant_names(p)
            # only immediate children for parent off? turn off parent if no enabled descendants
            if kids and not any(state.get(k, False) for k in kids):
                _set(p, False)
    return changed


def _set_capabilities(updates):
    with _ai_capabilities_lock:
        prev = dict(_ai_capabilities)
        state = dict(_ai_capabilities)
        for k, v in (updates or {}).items():
            if k in state:
                _apply_toggle(k, v, state)
        _ai_capabilities.clear()
        _ai_capabilities.update(state)
        snap = dict(_ai_capabilities)
    _save_ai_capabilities()
    changed = {k: snap[k] for k in snap if prev.get(k) != snap.get(k)}
    if changed:
        parts = [f'{k}={"on" if v else "off"}' for k, v in list(changed.items())[:8]]
        olog.info(
            'ai_capabilities',
            'AI tools toggled: ' + ', '.join(parts),
            changed=','.join(changed.keys()),
            motion_tools_on=any(snap.get(n, False) for n in _MOTION_TOOLS),
        )
    return snap


def _motion_backend_info():
    """Report active control path + rosbridge (when relevant)."""
    mode = get_control_mode()
    if mode == 'direct':
        return 'direct', {'ok': True, 'path': 'serial', 'control_mode': 'direct'}
    try:
        import ros_motion
        bridge = ros_motion.rosbridge_status()
        bridge = {**bridge, 'control_mode': 'ros2', 'path': 'ros2'}
        return 'ros2', bridge
    except Exception as e:
        return 'ros2', {'ok': False, 'error': str(e), 'control_mode': 'ros2'}


def _motion_infra_ready():
    """True when the active control_mode path can execute motion tools."""
    backend, bridge = _motion_backend_info()
    if backend == 'direct':
        return True, backend, bridge
    return bool(bridge.get('ok')), backend, bridge


def _motion_tools_available():
    """Any motion tool user-enabled AND infrastructure ready for current mode."""
    caps = _get_capabilities()
    user_any = any(caps.get(n, True) for n in _MOTION_TOOLS)
    infra_ok, backend, bridge = _motion_infra_ready()
    if not user_any:
        return False, backend, {**bridge, 'user_disabled': True}
    return infra_ok and user_any, backend, bridge


def _resolve_node_status(meta, caps, infra_ok, backend, bridge):
    name = meta['name']
    user_on = bool(caps.get(name, True))
    needs_motion = bool(meta.get('needs_motion') or meta.get('needs_ros2'))
    if not user_on:
        return 'off', 'Toggled off — not offered to the LLM.', user_on
    if needs_motion:
        if infra_ok:
            if backend == 'direct':
                return 'active', 'Direct serial path (control_mode=direct).', user_on
            return 'active', f'ROS 2 relay ready (rosbridge {bridge.get("url") or "up"}).', user_on
        err = bridge.get('error') or bridge.get('reason') or 'rosbridge not reachable'
        return 'unavailable', (
            f'On, but control_mode=ros2 and rosbridge down ({err}). '
            'Toggle Control to Direct serial, or start rosbridge + ugv_bringup.'
        ), user_on
    return 'active', 'Offered to the LLM.', user_on


# Flat + tree catalog for UI / LLM.
# status: active | unavailable | off
def _ai_tools_catalog():
    """Leaf tools only (for LLM tool list / flat chips)."""
    caps = _get_capabilities()
    infra_ok, backend, bridge = _motion_infra_ready()
    tools = []
    for meta, parent, depth in _iter_tree_nodes():
        if meta.get('kind') != 'tool':
            continue
        status, reason, user_on = _resolve_node_status(meta, caps, infra_ok, backend, bridge)
        tools.append({
            'name': meta['name'],
            'label': meta['label'],
            'description': meta.get('description', ''),
            'kind': 'tool',
            'parent': parent,
            'depth': depth,
            'toggleable': True,
            'needs_ros2': bool(meta.get('needs_motion') or meta.get('needs_ros2')),
            'needs_motion': bool(meta.get('needs_motion') or meta.get('needs_ros2')),
            'requires': list(meta.get('requires') or []),
            'enabled': user_on,
            'status': status,
            'reason': reason,
        })
    return tools


def _ai_tools_tree():
    """Hierarchical tree for UI pills (groups + tools)."""
    caps = _get_capabilities()
    infra_ok, backend, bridge = _motion_infra_ready()

    def build(nodes, parent=None, depth=0):
        out = []
        for meta in nodes:
            status, reason, user_on = _resolve_node_status(meta, caps, infra_ok, backend, bridge)
            # Group status: active if any active child; unavailable if any on but blocked; else off
            children = build(meta.get('children') or [], parent=meta['name'], depth=depth + 1)
            if meta.get('kind') == 'group' and children:
                if any(c['status'] == 'active' for c in children):
                    status = 'active'
                    reason = 'Group on — one or more children offered to the LLM.'
                elif any(c['enabled'] for c in children):
                    status = 'unavailable'
                    reason = 'Group on but children blocked (see amber children).'
                elif not user_on:
                    status = 'off'
                    reason = 'Group toggled off.'
                else:
                    status = 'off'
                    reason = 'No children enabled.'
            item = {
                'name': meta['name'],
                'label': meta['label'],
                'description': meta.get('description', ''),
                'kind': meta.get('kind', 'tool'),
                'parent': parent,
                'depth': depth,
                'toggleable': True,
                'needs_ros2': bool(meta.get('needs_ros2')),
                'requires': list(meta.get('requires') or []),
                'enabled': user_on,
                'status': status,
                'reason': reason,
                'children': children,
            }
            out.append(item)
        return out

    return build(_TOOL_TREE)

_tiktoken_enc = None
_tiktoken_method = None

def _get_token_encoder():
    """Prefer tiktoken (cl100k_base); fall back to char/4 heuristic."""
    global _tiktoken_enc, _tiktoken_method
    if _tiktoken_method is not None:
        return _tiktoken_enc, _tiktoken_method
    try:
        import tiktoken
        _tiktoken_enc = tiktoken.get_encoding('cl100k_base')
        _tiktoken_method = 'tiktoken/cl100k_base'
    except Exception:
        _tiktoken_enc = None
        _tiktoken_method = 'heuristic_chars/4'
    return _tiktoken_enc, _tiktoken_method

def _count_text_tokens(text):
    if not text:
        return 0
    enc, method = _get_token_encoder()
    if enc is not None:
        try:
            return len(enc.encode(str(text)))
        except Exception:
            pass
    # ~4 chars per token heuristic
    return max(1, (len(str(text)) + 3) // 4)

def _estimate_image_tokens(jpeg_bytes=None, data_url=None):
    """Rough vision-token estimate (OpenAI-style tile math is model-specific)."""
    size = 0
    if jpeg_bytes is not None:
        size = len(jpeg_bytes)
    elif data_url and isinstance(data_url, str) and 'base64,' in data_url:
        try:
            size = len(base64.b64decode(data_url.split('base64,', 1)[1], validate=False))
        except Exception:
            size = len(data_url) * 3 // 4
    if size <= 0:
        # typical low-res JPEG attach budget
        return 765
    # Empirical ballpark: base ~85 + ~170 per ~512px tile; scale with file size
    return int(85 + min(4000, size / 40))

def _message_text_parts(content):
    """Flatten OpenAI message content (str or multimodal list) to plain text."""
    if content is None:
        return ''
    if isinstance(content, str):
        return content
    if isinstance(content, list):
        bits = []
        for part in content:
            if isinstance(part, dict):
                if part.get('type') == 'text':
                    bits.append(part.get('text') or '')
                elif part.get('type') == 'image_url':
                    bits.append('[image]')
            else:
                bits.append(str(part))
        return '\n'.join(bits)
    return str(content)

def _estimate_messages_tokens(messages, include_image_bytes=None):
    """Estimate prompt tokens for a chat.completions messages list."""
    _, method = _get_token_encoder()
    total = 0
    image_tokens = 0
    text_tokens = 0
    # per-message overhead ~4 tokens (OpenAI cookbook-ish)
    for m in messages:
        total += 4
        content = m.get('content')
        if isinstance(content, list):
            for part in content:
                if not isinstance(part, dict):
                    t = _count_text_tokens(str(part))
                    text_tokens += t
                    total += t
                    continue
                if part.get('type') == 'text':
                    t = _count_text_tokens(part.get('text') or '')
                    text_tokens += t
                    total += t
                elif part.get('type') == 'image_url':
                    url = (part.get('image_url') or {}).get('url')
                    it = _estimate_image_tokens(data_url=url)
                    image_tokens += it
                    total += it
        else:
            t = _count_text_tokens(_message_text_parts(content))
            text_tokens += t
            total += t
    if include_image_bytes:
        it = _estimate_image_tokens(jpeg_bytes=include_image_bytes)
        # if messages already counted an image, don't double-count; only for pending attach
        pass
    # reply priming
    total += 3
    return {
        'tokens_est': total,
        'text_tokens_est': text_tokens,
        'image_tokens_est': image_tokens,
        'messages': len(messages),
        'method': method,
    }

def _normalize_history(history, limit=24):
    out = []
    for h in (history or [])[-limit:]:
        role = h.get('role')
        content = h.get('content')
        if role in ('user', 'assistant', 'system') and isinstance(content, str) and content.strip():
            out.append({'role': role, 'content': content.strip()})
    return out

def _build_chat_messages(history, user_msg, attach_snapshot=False, jpeg_bytes=None):
    messages = [{'role': 'system', 'content': _AI_SYSTEM_PROMPT}]
    for h in _normalize_history(history, limit=12):
        if h['role'] in ('user', 'assistant'):
            messages.append({'role': h['role'], 'content': h['content']})
    if attach_snapshot and jpeg_bytes:
        b64 = base64.b64encode(jpeg_bytes).decode('ascii')
        data_url = f'data:image/jpeg;base64,{b64}'
        messages.append({
            'role': 'user',
            'content': [
                {'type': 'text', 'text': user_msg},
                {'type': 'image_url', 'image_url': {'url': data_url}},
            ],
        })
        return messages, data_url, len(jpeg_bytes)
    messages.append({'role': 'user', 'content': user_msg})
    return messages, None, 0

def _openai_chat(
    messages, max_tokens=512, temperature=0.4, tools=None,
    response_format=None, timeout=120, tool_choice=None,
):
    """Chat Completions via OpenAI-compatible HTTP API. Settings from env/.env only.

    Returns (assistant_message_dict, raw_body, cfg).
    assistant_message_dict may include content and/or tool_calls.
    Optional response_format enables JSON mode / json_schema when the server supports it.
    """
    cfg = _ai_env_config()
    if not cfg['api_key']:
        raise RuntimeError('OPENAI_API_KEY is not set (add it to ugv_rpi/.env)')
    url = cfg['base_url'] + '/chat/completions'
    payload = {
        'model': cfg['model'],
        'messages': messages,
        'max_tokens': max_tokens,
        'temperature': temperature,
        'stream': False,
    }
    if tools:
        payload['tools'] = tools
        payload['tool_choice'] = tool_choice if tool_choice is not None else 'auto'
    if response_format:
        payload['response_format'] = response_format
    req = urllib.request.Request(
        url,
        data=json.dumps(payload).encode('utf-8'),
        headers={
            'Content-Type': 'application/json',
            'Authorization': f"Bearer {cfg['api_key']}",
        },
        method='POST',
    )
    try:
        with urllib.request.urlopen(req, timeout=float(timeout or 120)) as resp:
            body = json.loads(resp.read().decode('utf-8'))
    except urllib.error.HTTPError as e:
        err_body = e.read().decode('utf-8', errors='replace')
        raise RuntimeError(f'LLM HTTP {e.code}: {err_body[:800]}') from e
    except Exception as e:
        raise RuntimeError(f'LLM request failed: {e}') from e
    try:
        msg = body['choices'][0]['message']
    except Exception:
        raise RuntimeError(f'LLM bad response: {str(body)[:500]}')
    return msg, body, cfg


def _parse_tool_call_args(msg, name=None):
    """Extract JSON arguments from OpenAI-style tool_calls / function_call."""
    if not isinstance(msg, dict):
        return None
    calls = msg.get('tool_calls') or []
    if isinstance(calls, dict):
        calls = [calls]
    for tc in calls:
        if not isinstance(tc, dict):
            continue
        fn = tc.get('function') or tc
        if name and fn.get('name') and fn.get('name') != name:
            continue
        args = fn.get('arguments')
        if isinstance(args, dict):
            return args
        if isinstance(args, str) and args.strip():
            try:
                parsed = json.loads(args)
                if isinstance(parsed, dict):
                    return parsed
            except Exception:
                parsed = _parse_json_from_text(args)
                if isinstance(parsed, dict):
                    return parsed
    fc = msg.get('function_call')
    if isinstance(fc, dict):
        if name and fc.get('name') and fc.get('name') != name:
            return None
        args = fc.get('arguments')
        if isinstance(args, dict):
            return args
        if isinstance(args, str) and args.strip():
            try:
                parsed = json.loads(args)
                return parsed if isinstance(parsed, dict) else None
            except Exception:
                parsed = _parse_json_from_text(args)
                return parsed if isinstance(parsed, dict) else None
    return None


def _message_text_content(msg):
    content = (msg.get('content') or '').strip() if isinstance(msg, dict) else ''
    if not content and isinstance(msg, dict):
        content = (msg.get('reasoning') or msg.get('reasoning_content') or '').strip()
    return content


def _get_telemetry_payload():
    """Local sensor snapshot for the get_telemetry tool (no ROS required)."""
    data = {
        'cpu_load': getattr(si, 'cpu_load', None),
        'cpu_temp': getattr(si, 'cpu_temp', None),
        'ram': getattr(si, 'ram', None),
        'wifi_rssi': getattr(si, 'wifi_rssi', None),
        'pan_angle_cmd': getattr(cvf, 'pan_angle', None),  # last UI/CV command (not always HW)
        'tilt_angle_cmd': getattr(cvf, 'tilt_angle', None),
        'pan_angle': getattr(cvf, 'pan_angle', None),
        'tilt_angle': getattr(cvf, 'tilt_angle', None),
        'video_fps': getattr(cvf, 'video_fps', None),
        'motor_enabled': getattr(base, 'enable_motor_control', None),
        'control_mode': get_control_mode(),
    }
    try:
        bd = base.base_data if isinstance(getattr(base, 'base_data', None), dict) else {}
        data['voltage_raw'] = bd.get('v')
        data['base_feedback_T'] = bd.get('T')
        # Hardware pan/tilt when module_type=2 and feedback flow is on (T:1001)
        if 'pan' in bd:
            data['hw_pan_deg'] = bd.get('pan')
        if 'tilt' in bd:
            data['hw_tilt_deg'] = bd.get('tilt')
        data['base_feedback'] = {
            k: bd.get(k) for k in ('T', 'L', 'R', 'v', 'pan', 'tilt', 'odl', 'odr')
            if k in bd
        }
    except Exception:
        pass
    return data


def _ptz_status_snapshot(request_feedback=True, wait_s=0.35):
    """Read pan/tilt: request ESP32 feedback (T:130) and return last base_data + ROS if any."""
    serial_open = base.serial_is_open() if hasattr(base, 'serial_is_open') else bool(getattr(base, 'ser', None))
    if request_feedback and serial_open:
        try:
            # Ensure gimbal module + feedback flow, then one-shot feedback
            base.base_json_ctrl({'T': 4, 'cmd': int(f['base_config'].get('module_type') or 2)})
            base.base_json_ctrl({'T': 131, 'cmd': 1})
            base.base_json_ctrl({'T': 130})
            time.sleep(max(0.05, float(wait_s)))
            # Drain a few feedback frames so pan/tilt fields land in base.base_data
            for _ in range(8):
                fb = base.feedback_data()
                if isinstance(fb, dict) and fb.get('T') == 1001:
                    break
                time.sleep(0.05)
        except Exception as e:
            olog.warn('ptz_status', f'Feedback request failed: {e}', error=str(e))

    bd = base.base_data if isinstance(getattr(base, 'base_data', None), dict) else {}
    hw_pan = bd.get('pan')
    hw_tilt = bd.get('tilt')
    out = {
        'success': True,
        'control_mode': get_control_mode(),
        'module_type_cfg': f['base_config'].get('module_type'),
        'serial_open': serial_open,
        'uart_owner': 'flask' if serial_open else 'ros2_or_none',
        'serial_released_for_ros': bool(getattr(base, 'serial_released_for_ros', False)),
        'commanded': {
            'pan_deg': getattr(cvf, 'pan_angle', None),
            'tilt_deg': getattr(cvf, 'tilt_angle', None),
            'note': 'Last angles commanded by UI/CV in this process (not encoder readback).',
        },
        'hardware': {
            'pan_deg': hw_pan,
            'tilt_deg': hw_tilt,
            'feedback_T': bd.get('T'),
            'voltage_raw': bd.get('v'),
            'wheel_L': bd.get('L'),
            'wheel_R': bd.get('R'),
            'raw': {k: bd.get(k) for k in bd} if bd else {},
            'note': (
                'From ESP32 T:1001 when module_type=2. pan/tilt absent means module not set '
                'to gimbal, feedback flow off, or bus servos not responding (watch for T:1005).'
            ),
        },
        'how_to_command': {
            'serial_simple': {'T': 133, 'X': 20, 'Y': 0, 'SPD': 0, 'ACC': 0},
            'feedback_on': {'T': 131, 'cmd': 1},
            'feedback_once': {'T': 130},
            'module_gimbal': {'T': 4, 'cmd': 2},
        },
    }
    # Optional ROS joint readback via rosbridge subscribe is heavy; just probe topics exist
    if get_control_mode() == 'ros2':
        try:
            import ros_motion
            out['ros'] = {
                'bridge': ros_motion.rosbridge_status(),
                'joint_states_topic': ros_motion.joint_states_topic(),
                'pt_joint_topic': ros_motion.pt_joint_topic(),
                'note': 'Publish path only; position subscribe not polled here.',
            }
        except Exception as e:
            out['ros'] = {'error': str(e)}
    return out


def _openai_tools_for_agent():
    """Function tools offered to the model = catalog entries with status active."""
    tools = []
    by_name = {t['name']: t for t in _ai_tools_catalog()}

    if by_name.get('get_telemetry', {}).get('status') == 'active':
        tools.append({
            'type': 'function',
            'function': {
                'name': 'get_telemetry',
                'description': 'Read live robot/SBC telemetry (CPU, RAM, temp, voltage, pan/tilt, RSSI, FPS).',
                'parameters': {'type': 'object', 'properties': {}},
            },
        })
    if by_name.get('get_cv_detections', {}).get('status') == 'active':
        tools.append({
            'type': 'function',
            'function': {
                'name': 'get_cv_detections',
                'description': (
                    'Run on-board MobileNet-SSD on the live camera. Returns labels, confidences, '
                    'normalized bboxes [x1,y1,x2,y2], and center_x/center_y (0–1, image coords). '
                    'Useful labels include dog, person, cat, chair, bottle, … Use conf_threshold '
                    '0.15–0.25 when searching; raise to 0.4+ to reduce false positives. '
                    'Optional filter_label (e.g. \"dog\") keeps only that class.'
                ),
                'parameters': {
                    'type': 'object',
                    'properties': {
                        'conf_threshold': {
                            'type': 'number',
                            'description': 'Minimum confidence 0-1 (default 0.18 for search)',
                        },
                        'filter_label': {
                            'type': 'string',
                            'description': 'If set, only return this label (case-insensitive), e.g. dog',
                        },
                    },
                },
            },
        })
    if by_name.get('get_camera_snapshot', {}).get('status') == 'active':
        tools.append({
            'type': 'function',
            'function': {
                'name': 'get_camera_snapshot',
                'description': (
                    'Capture a live camera JPEG. Returns size metadata. '
                    'For full vision, enable Attach snapshot on the chat UI.'
                ),
                'parameters': {'type': 'object', 'properties': {}},
            },
        })
    # Motion group — register each leaf independently by catalog status
    active_motion = {
        n for n in _MOTION_TOOLS
        if by_name.get(n, {}).get('status') == 'active'
    }
    if active_motion:
        try:
            import ros_motion
            tools.extend(
                t for t in ros_motion.openai_motion_tools()
                if t.get('function', {}).get('name') in active_motion
            )
        except Exception as e:
            print(f'[app.py] ros_motion tools unavailable: {e}')
    return tools


def _execute_agent_tool(name, arguments):
    args = arguments or {}
    # Respect capability toggles at execution time too
    catalog = {t['name']: t for t in _ai_tools_catalog()}
    entry = catalog.get(name)
    if entry and entry.get('status') != 'active':
        reason = entry.get('reason') or entry.get('status')
        olog.warn(
            'ai_tool_blocked',
            f'AI tool blocked: {name} ({reason})',
            tool=name, status=entry.get('status'), reason=reason,
            control_mode=get_control_mode(),
        )
        return {
            'ok': False,
            'error': f'tool unavailable: {reason}',
            'tool': name,
        }

    if name == 'get_telemetry':
        return {'ok': True, 'telemetry': _get_telemetry_payload()}

    if name == 'get_cv_detections':
        try:
            conf = float(args.get('conf_threshold', 0.18))
            conf = max(0.05, min(0.95, conf))
            filter_label = (args.get('filter_label') or args.get('label') or '').strip().lower()
            frame = cvf.grab_bgr_frame()
            if frame is None:
                dets = list(getattr(cvf, 'last_detections', []) or [])
                warning = 'live frame unavailable; returning last_detections if any'
            else:
                dets = cvf.detect_objects_structured(frame, conf_threshold=conf)
                warning = None
            # Enrich with centers for steering; optional class filter
            enriched = []
            for d in dets:
                if not isinstance(d, dict):
                    continue
                lab = (d.get('label') or '').lower()
                if filter_label and lab != filter_label:
                    continue
                bb = d.get('bbox_norm') or d.get('bbox') or [0, 0, 0, 0]
                try:
                    x1, y1, x2, y2 = [float(v) for v in bb[:4]]
                except (TypeError, ValueError):
                    x1 = y1 = x2 = y2 = 0.0
                item = dict(d)
                item['center_x'] = round((x1 + x2) / 2.0, 3)
                item['center_y'] = round((y1 + y2) / 2.0, 3)
                # +center_x = target right of image center → turn right to face it
                item['offset_x'] = round(item['center_x'] - 0.5, 3)
                enriched.append(item)
            labels = sorted({(d.get('label') or '') for d in enriched if d.get('label')})
            out = {
                'ok': True,
                'engine': 'mobilenet-ssd',
                'conf_threshold': conf,
                'filter_label': filter_label or None,
                'detections': enriched,
                'count': len(enriched),
                'labels_found': labels,
                'found_dog': any(l.lower() == 'dog' for l in labels),
            }
            if warning:
                out['warning'] = warning
            return out
        except Exception as e:
            return {'ok': False, 'error': str(e), 'tool': name}

    if name == 'get_camera_snapshot':
        try:
            jpeg = _grab_jpeg_bytes()
            return {
                'ok': True,
                'mime': 'image/jpeg',
                'bytes': len(jpeg),
                'note': 'Frame captured. For vision, enable Attach snapshot on the next user message.',
            }
        except Exception as e:
            return {'ok': False, 'error': str(e), 'tool': name}

    if name in ('send_motor_command', 'stop_motors', 'send_gimbal_command'):
        try:
            return _execute_motion_via_mode(name, args)
        except Exception as e:
            return {'ok': False, 'error': str(e), 'tool': name}

    return {'ok': False, 'error': f'unmapped tool: {name}'}


def _execute_motion_via_mode(name, args):
    """AI motion tools follow the same control_mode as UI sticks."""
    mode = get_control_mode()
    args = args or {}
    level = 'warn' if name == 'stop_motors' else 'info'

    if mode == 'ros2':
        import ros_motion
        if name == 'stop_motors':
            _clear_ai_motion_lock()
            _cancel_ai_drive_timer()
        ros_args = dict(args)
        if name == 'send_motor_command':
            # Same body→hardware mapping as direct serial T:13
            body_lin = float(ros_args.get('linear_x', 0.0) or 0.0)
            body_ang = float(ros_args.get('angular_z', 0.0) or 0.0)
            hw_lin, hw_ang = body_to_hw_twist(body_lin, body_ang)
            ros_args['linear_x'] = hw_lin
            ros_args['angular_z'] = hw_ang
        result = ros_motion.execute_motion_tool(name, ros_args)
        if isinstance(result, dict):
            result.setdefault('control_mode', mode)
            ok = bool(result.get('ok', True)) and not result.get('error')
            if name == 'send_motor_command' and ok:
                cont = bool(result.get('continuous'))
                dur = int(result.get('duration_ms') or 0)
                _arm_ai_motion_lock(duration_ms=dur, continuous=cont)
                result['ui_heartbeat_lock_s'] = max(
                    0.0, _ai_motion_lock_until - time.time()
                )
                result['drive_linear_sign'] = _drive_sign('linear')
            if name == 'send_gimbal_command' and ok:
                try:
                    _pr = float(args.get('pan_rad') or 0.0)
                    _tr = float(args.get('tilt_rad') or 0.0)
                    _publish_ptz_aim(
                        -_pr * 180.0 / math.pi, _tr * 180.0 / math.pi,
                        source='ros_gimbal',
                    )
                except Exception:
                    pass
            olog.log(
                'error' if not ok else level,
                'ai_motion',
                f'AI tool {name} via ros2' + (f' — {result.get("error")}' if not ok else ''),
                tool=name, control_mode=mode, path='ros2', ok=ok,
                **{k: args.get(k) for k in ('linear_x', 'angular_z', 'duration_ms', 'pan_rad', 'tilt_rad') if k in args},
                error=result.get('error'),
            )
        return result

    # ---- direct serial (ESP32 UART) ----
    import math

    max_lin = float(os.environ.get('UGV_MAX_LINEAR') or 0.35)
    max_ang = float(os.environ.get('UGV_MAX_ANGULAR') or 0.8)
    max_ms = int(os.environ.get('UGV_MAX_DRIVE_MS') or 4000)

    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))

    if name == 'stop_motors':
        _cancel_ai_drive_timer()
        _clear_ai_motion_lock()
        base.base_json_ctrl({'T': 13, 'X': 0, 'Z': 0})
        base.base_json_ctrl({'T': 1, 'L': 0, 'R': 0})
        olog.warn('ai_motion', 'AI stop_motors via direct serial',
                  tool=name, control_mode=mode, path='serial', ok=True)
        return {
            'ok': True,
            'backend': 'direct',
            'control_mode': mode,
            'path': 'serial',
            'pending_stop_cancelled': True,
        }

    if name == 'send_motor_command':
        # Body-frame (camera-forward +linear) before hardware mapping
        body_lin = _clamp(float(args.get('linear_x', 0.0)), -max_lin, max_lin)
        body_ang = _clamp(float(args.get('angular_z', 0.0)), -max_ang, max_ang)
        lin, ang = body_to_hw_twist(body_lin, body_ang)
        cont = args.get('continuous')
        if isinstance(cont, str):
            cont = cont.strip().lower() in ('1', 'true', 'yes', 'on')
        else:
            cont = bool(cont)
        # Default missing/0 → short timed move; continuous only if continuous=true
        if cont:
            dur = 0
            is_continuous = True
        else:
            is_continuous = False
            raw = args.get('duration_ms')
            try:
                if raw is None or raw == '' or int(raw) == 0:
                    dur = _DEFAULT_AI_DRIVE_MS
                else:
                    dur = int(raw)
            except (TypeError, ValueError):
                dur = _DEFAULT_AI_DRIVE_MS
            if dur < 0:
                dur = _DEFAULT_AI_DRIVE_MS
            if dur > max_ms:
                dur = max_ms
        # Supersede any previous scheduled auto-stop, then start motion
        _cancel_ai_drive_timer()
        # Block UI idle L=0/R=0 heartbeats for the full maneuver window
        _arm_ai_motion_lock(duration_ms=dur, continuous=is_continuous)
        base.base_json_ctrl({'T': 13, 'X': lin, 'Z': ang})
        async_stop = False
        scheduled_stop_ms = None
        if not is_continuous and dur > 0:
            _schedule_ai_drive_stop(dur)
            async_stop = True
            scheduled_stop_ms = dur
        olog.info(
            'ai_motion',
            f'AI drive body_lin={body_lin:.3f}→hw_X={lin:.3f} ang={body_ang:.3f}→{ang:.3f} '
            f'dur={dur}ms continuous={is_continuous} (direct)',
            tool=name, control_mode=mode, path='serial', ok=True,
            linear_x=body_lin, angular_z=body_ang, hw_linear_x=lin, hw_angular_z=ang,
            drive_linear_sign=_drive_sign('linear'), duration_ms=dur,
            continuous=is_continuous, async_stop=async_stop, stopped=False,
            ui_heartbeat_lock_s=max(0.0, _ai_motion_lock_until - time.time()),
        )
        out = {
            'ok': True,
            'backend': 'direct',
            'control_mode': mode,
            'path': 'serial',
            'linear_x': body_lin,
            'angular_z': body_ang,
            'hw_linear_x': lin,
            'hw_angular_z': ang,
            'drive_linear_sign': _drive_sign('linear'),
            'duration_ms': dur,
            'continuous': is_continuous,
            'stopped': False,
            'async_stop': async_stop,
            'ui_heartbeat_lock_s': max(0.0, _ai_motion_lock_until - time.time()),
        }
        if scheduled_stop_ms is not None:
            out['scheduled_stop_ms'] = scheduled_stop_ms
        return out

    if name == 'send_gimbal_command':
        # ±2.6 rad ≈ ±149° — enough for a ~270°+ sweep (hardware pan is ±180°)
        pan = _clamp(float(args.get('pan_rad', 0.0)), -2.6, 2.6)
        tilt = _clamp(float(args.get('tilt_rad', 0.0)), -1.0, 0.6)
        # Inverse of ros_motion.ui_xy_to_radians
        x_deg = -pan * 180.0 / math.pi
        y_deg = tilt * 180.0 / math.pi
        base.base_json_ctrl({'T': 133, 'X': x_deg, 'Y': y_deg, 'SPD': 0, 'ACC': 0})
        _publish_ptz_aim(x_deg, y_deg, settled=False, source='send_gimbal')
        olog.info(
            'ai_motion',
            f'AI gimbal pan={pan:.3f} tilt={tilt:.3f} rad (direct)',
            tool=name, control_mode=mode, path='serial', ok=True,
            pan_rad=pan, tilt_rad=tilt, x_deg=round(x_deg, 2), y_deg=round(y_deg, 2),
        )
        return {
            'ok': True,
            'backend': 'direct',
            'control_mode': mode,
            'path': 'serial',
            'pan_rad': pan,
            'tilt_rad': tilt,
            'x_deg': x_deg,
            'y_deg': y_deg,
        }

    olog.warn('ai_motion', f'Unknown motion tool: {name}', tool=name, control_mode=mode, ok=False)
    return {'ok': False, 'error': f'unknown motion tool: {name}', 'control_mode': mode}


def _run_agent_loop(messages, max_rounds=6):
    """Chat with optional tool calls until final text reply."""
    tools = _openai_tools_for_agent()
    tool_trace = []
    cfg = _ai_env_config()
    out_tokens = int(cfg.get('chat_max_tokens') or _chat_max_tokens())
    for _round in range(max_rounds):
        msg, body, cfg = _openai_chat(
            messages,
            max_tokens=out_tokens,
            temperature=0.4,
            tools=tools if tools else None,
        )
        finish = None
        try:
            finish = (body.get('choices') or [{}])[0].get('finish_reason')
        except Exception:
            finish = None
        if finish == 'length':
            olog.warn(
                'ai_chat',
                f'LLM hit max_tokens={out_tokens} (finish_reason=length) — tool JSON may be truncated',
                max_tokens=out_tokens, round=_round,
            )
        tool_calls = msg.get('tool_calls') or []
        if tool_calls:
            # Append assistant turn with tool_calls, then tool results
            messages.append({
                'role': 'assistant',
                'content': msg.get('content'),
                'tool_calls': tool_calls,
            })
            for tc in tool_calls:
                fn = (tc.get('function') or {})
                name = fn.get('name') or ''
                raw_args = fn.get('arguments') or '{}'
                try:
                    args = json.loads(raw_args) if isinstance(raw_args, str) else (raw_args or {})
                except Exception:
                    olog.warn(
                        'ai_chat',
                        f'Tool {name} arguments not valid JSON (likely truncated)',
                        tool=name, finish_reason=finish,
                        args_prefix=str(raw_args)[:160],
                    )
                    args = {}
                    result = {
                        'ok': False,
                        'error': (
                            'tool arguments truncated or invalid JSON '
                            f'(finish_reason={finish}, max_tokens={out_tokens})'
                        ),
                    }
                    tool_trace.append({'name': name, 'arguments': args, 'result': result})
                    messages.append({
                        'role': 'tool',
                        'tool_call_id': tc.get('id') or name,
                        'content': json.dumps(result),
                    })
                    continue
                result = _execute_agent_tool(name, args)
                tool_trace.append({'name': name, 'arguments': args, 'result': result})
                messages.append({
                    'role': 'tool',
                    'tool_call_id': tc.get('id') or name,
                    'content': json.dumps(result),
                })
            continue
        content = _message_text_content(msg)
        if not content and not tool_trace:
            raise RuntimeError(f'LLM empty content: {str(body)[:500]}')
        if not content:
            content = '(tools ran; no final text)'
        return content, body, cfg, tool_trace
    return (
        'Stopped after max tool rounds. Last tools: ' + json.dumps(tool_trace[-3:]),
        {},
        cfg,
        tool_trace,
    )

# ---------------------------------------------------------------------------
# Optional AI API auth: when UGV_AI_TOKEN is set/non-empty, require it on
# /api/ai/* and /api/snapshot. Accept X-UGV-Token or Authorization: Bearer.
# ---------------------------------------------------------------------------
def _ai_auth_token():
    return (os.environ.get('UGV_AI_TOKEN') or '').strip()


def _request_provided_ai_token():
    hdr = (request.headers.get('X-UGV-Token') or '').strip()
    if hdr:
        return hdr
    auth = (request.headers.get('Authorization') or '').strip()
    if auth.lower().startswith('bearer '):
        return auth[7:].strip()
    return ''


def _ai_request_authorized():
    expected = _ai_auth_token()
    if not expected:
        return True  # open LAN mode when unset
    provided = _request_provided_ai_token()
    if not provided or len(provided) != len(expected):
        return False
    return hmac.compare_digest(provided, expected)


@app.before_request
def _gate_ai_apis():
    path = request.path or ''
    if not (path.startswith('/api/ai/') or path == '/api/snapshot'):
        return None
    if _ai_request_authorized():
        return None
    return jsonify({'success': False, 'error': 'unauthorized'}), 401


@app.route('/api/ai/config', methods=['GET'])
def api_ai_config():
    cfg = _ai_env_config()
    # Never send full key to browser — only a masked hint / boolean
    key = cfg['api_key'] or ''
    masked = (key[:3] + '…' + key[-2:]) if len(key) > 6 else ('***' if key else '')
    _, method = _get_token_encoder()
    motion_ok, backend, bridge = _motion_tools_available()
    caps = _get_capabilities()
    mode = get_control_mode()
    return jsonify({
        'base_url': cfg['base_url'],
        'model': cfg['model'],
        'chat_max_tokens': cfg.get('chat_max_tokens') or _chat_max_tokens(),
        'api_key_set': bool(key),
        'api_key_masked': masked,
        'token_estimate_method': method,
        'tools': _ai_tools_catalog(),
        'tool_tree': _ai_tools_tree(),
        'capabilities': caps,
        'control_mode': mode,
        'motion_backend': backend,
        'motion_available': motion_ok,
        'rosbridge': bridge,
        'note': (
            'Click tool pills to toggle. Motion path follows control_mode '
            '(Direct serial or ROS 2 relay — toggle on main UI). '
            'ROS 2 mode needs rosbridge + ugv_bringup.'
        ),
    })


@app.route('/api/ai/capabilities', methods=['GET', 'POST'])
def api_ai_capabilities():
    """Get/set tool/group toggles (with parent/child cascade)."""
    if request.method == 'GET':
        motion_ok, backend, bridge = _motion_tools_available()
        infra_ok, _, bridge_raw = _motion_infra_ready()
        return jsonify({
            'success': True,
            'capabilities': _get_capabilities(),
            'tools': _ai_tools_catalog(),
            'tool_tree': _ai_tools_tree(),
            'control_mode': get_control_mode(),
            'motion_available': motion_ok,
            'motion_infra_ready': infra_ok,
            'motion_backend': backend,
            'rosbridge': bridge_raw,
        })
    data = request.get_json(silent=True) or {}
    allowed = set(_DEFAULT_TOOL_CAPS.keys())
    updates = {}
    src = data.get('tools') if isinstance(data.get('tools'), dict) else data
    for k in allowed:
        if k in src:
            updates[k] = src[k]
    # Legacy group keys
    if 'telemetry' in data:
        updates['group_telemetry'] = data['telemetry']
    if 'computer_vision' in data:
        updates['group_computer_vision'] = data['computer_vision']
    if 'ros2_motion' in data:
        updates['group_ros2_motion'] = data['ros2_motion']
    if not updates:
        return jsonify({'success': False, 'error': 'no valid tool/group keys'}), 400
    caps = _set_capabilities(updates)
    motion_ok, backend, bridge = _motion_tools_available()
    infra_ok, _, bridge_raw = _motion_infra_ready()
    return jsonify({
        'success': True,
        'capabilities': caps,
        'tools': _ai_tools_catalog(),
        'tool_tree': _ai_tools_tree(),
        'control_mode': get_control_mode(),
        'motion_available': motion_ok,
        'motion_infra_ready': infra_ok,
        'motion_backend': backend,
        'rosbridge': bridge_raw,
    })

@app.route('/api/ai/estimate', methods=['POST'])
def api_ai_estimate():
    """Estimate context tokens for current history (+ optional draft message / snapshot)."""
    data = request.get_json(silent=True) or {}
    history = data.get('history') or []
    draft = (data.get('message') or data.get('draft') or '').strip()
    attach = bool(data.get('attach_snapshot', False))
    keep_limit = int(data.get('history_limit') or 12)

    jpeg = None
    snap_bytes = 0
    if attach:
        try:
            jpeg = _grab_jpeg_bytes()
            snap_bytes = len(jpeg)
        except Exception as e:
            return jsonify({'success': False, 'error': f'snapshot for estimate failed: {e}'}), 500

    user_msg = draft if draft else '(empty draft)'
    messages, _url, _b = _build_chat_messages(
        history[-keep_limit:], user_msg, attach_snapshot=attach and jpeg is not None, jpeg_bytes=jpeg
    )
    # If no draft, still estimate system+history only (without a fake user turn)
    if not draft:
        messages = [{'role': 'system', 'content': _AI_SYSTEM_PROMPT}]
        for h in _normalize_history(history, limit=keep_limit):
            if h['role'] in ('user', 'assistant'):
                messages.append(h)
        if attach and jpeg is not None:
            # pending vision attach cost even without typed draft
            b64 = base64.b64encode(jpeg).decode('ascii')
            messages.append({
                'role': 'user',
                'content': [
                    {'type': 'text', 'text': ''},
                    {'type': 'image_url', 'image_url': {'url': f'data:image/jpeg;base64,{b64}'}},
                ],
            })

    est = _estimate_messages_tokens(messages)
    hist_chars = sum(len(h.get('content') or '') for h in _normalize_history(history, limit=99))
    return jsonify({
        'success': True,
        'tokens_est': est['tokens_est'],
        'text_tokens_est': est['text_tokens_est'],
        'image_tokens_est': est['image_tokens_est'],
        'method': est['method'],
        'messages_counted': est['messages'],
        'history_messages': len(_normalize_history(history, limit=99)),
        'history_chars': hist_chars,
        'snapshot_bytes': snap_bytes,
        'attach_snapshot': attach and snap_bytes > 0,
        'label': (
            f"~{est['tokens_est']} tokens"
            f" (text ~{est['text_tokens_est']}"
            + (f", image ~{est['image_tokens_est']}" if est['image_tokens_est'] else '')
            + f", {est['method']})"
        ),
    })

@app.route('/api/ai/compress', methods=['POST'])
def api_ai_compress():
    """Summarize older chat turns into a short context block; keep recent turns."""
    data = request.get_json(silent=True) or {}
    history = _normalize_history(data.get('history') or [], limit=100)
    keep_recent = max(2, min(8, int(data.get('keep_recent') or 4)))
    if len(history) <= keep_recent:
        est = _estimate_messages_tokens(
            [{'role': 'system', 'content': _AI_SYSTEM_PROMPT}] + history
        )
        return jsonify({
            'success': True,
            'compressed': False,
            'reason': 'history already short',
            'history': history,
            'before': est,
            'after': est,
            'label_before': f"~{est['tokens_est']} tokens",
            'label_after': f"~{est['tokens_est']} tokens",
        })

    older = history[:-keep_recent]
    recent = history[-keep_recent:]
    transcript = []
    for h in older:
        transcript.append(f"{h['role'].upper()}: {h['content']}")
    compress_messages = [
        {
            'role': 'system',
            'content': (
                "You compress robot teleop chat history. "
                "Write a concise bullet summary of facts, observations, and user goals. "
                "No preamble. Max ~180 words."
            ),
        },
        {
            'role': 'user',
            'content': "Summarize this conversation for future context:\n\n" + "\n".join(transcript),
        },
    ]
    before_msgs = [{'role': 'system', 'content': _AI_SYSTEM_PROMPT}] + history
    before = _estimate_messages_tokens(before_msgs)
    try:
        summary_msg, _raw, used_cfg = _openai_chat(compress_messages, max_tokens=320, temperature=0.2)
        summary = _message_text_content(summary_msg)
        if not summary:
            olog.error('ai_compress', 'Compress produced empty summary')
            return jsonify({'success': False, 'error': 'compress produced empty summary'}), 502
    except Exception as e:
        olog.error('ai_compress', f'Compress failed: {e}', error=str(e)[:200])
        return jsonify({'success': False, 'error': str(e)}), 502

    new_history = [
        {
            'role': 'user',
            'content': f"[Compressed earlier context]\n{summary}",
        },
        {
            'role': 'assistant',
            'content': 'Understood — I will use that summary as prior context.',
        },
    ] + recent
    after_msgs = [{'role': 'system', 'content': _AI_SYSTEM_PROMPT}] + new_history
    after = _estimate_messages_tokens(after_msgs)
    saved = max(0, before['tokens_est'] - after['tokens_est'])
    olog.info(
        'ai_compress',
        f'Chat compressed ~{before["tokens_est"]}→{after["tokens_est"]} tok (saved ~{saved})',
        before_tokens=before['tokens_est'],
        after_tokens=after['tokens_est'],
        saved_tokens_est=saved,
        model=used_cfg.get('model'),
    )
    return jsonify({
        'success': True,
        'compressed': True,
        'history': new_history,
        'summary': summary,
        'model': used_cfg['model'],
        'before': before,
        'after': after,
        'label_before': f"~{before['tokens_est']} tokens ({before['method']})",
        'label_after': f"~{after['tokens_est']} tokens ({after['method']})",
        'saved_tokens_est': saved,
    })

@app.route('/api/snapshot', methods=['GET'])
def api_snapshot():
    try:
        jpeg = _grab_jpeg_bytes()
        b64 = base64.b64encode(jpeg).decode('ascii')
        return jsonify({
            'success': True,
            'mime': 'image/jpeg',
            'width_hint': 640,
            'bytes': len(jpeg),
            'image_base64': b64,
            'data_url': f'data:image/jpeg;base64,{b64}',
        })
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500

@app.route('/api/ai/chat', methods=['POST'])
def api_ai_chat():
    """
    Body:
      {
        "message": "what do you see?",
        "history": [{"role":"user"|"assistant","content":"..."}, ...],
        "attach_snapshot": true
      }
    LLM endpoint/model/key are read only from process env / ugv_rpi/.env
    Motion tools follow control_mode (direct serial or ROS 2 relay).
    """
    data = request.get_json(silent=True) or {}
    user_msg = (data.get('message') or data.get('goal') or '').strip()
    if not user_msg:
        return jsonify({'success': False, 'error': 'message is required'}), 400

    history = data.get('history') or []
    attach = data.get('attach_snapshot', True)

    # Enrich system prompt with control path status
    mode = get_control_mode()
    backend, bridge = _motion_backend_info()
    system = _AI_SYSTEM_PROMPT
    active = [t['name'] for t in _ai_tools_catalog() if t.get('status') == 'active']
    inactive = [t['name'] for t in _ai_tools_catalog() if t.get('status') != 'active']
    system += (
        f"\n\nControl mode: {mode} ({'ESP32 serial' if mode == 'direct' else 'ROS 2 / rosbridge'}). "
        f"Callable tools right now: {', '.join(active) if active else '(none)'}. "
        "Use tools instead of claiming you lack ones that are listed. "
    )
    if inactive:
        system += f"Unavailable (do not claim you have these): {', '.join(inactive)}. "
    if any(n in active for n in ('send_motor_command', 'stop_motors')):
        system += (
            "Prefer punchy timed moves (duration_ms 800–1600, linear_x 0.22–0.28). "
            "After each drive, re-check with get_cv_detections before moving again. "
            "Call stop_motors if unsure or when done."
        )
    elif any(n in inactive for n in ('send_motor_command', 'stop_motors')):
        system += (
            "Motion is unavailable; tell the user to toggle Control to Direct serial, "
            "or enable ROS 2 mode with rosbridge + ugv_bringup."
        )
    if 'get_cv_detections' in active:
        system += (
            " get_cv_detections: use filter_label when hunting a class (e.g. dog); "
            "center_x/offset_x help aim (offset_x>0 means target is to the right)."
        )

    snapshot_data_url = None
    snapshot_bytes = 0
    jpeg = None
    if attach:
        try:
            jpeg = _grab_jpeg_bytes()
            snapshot_bytes = len(jpeg)
        except Exception as e:
            messages = [{'role': 'system', 'content': system}]
            for h in _normalize_history(history, limit=12):
                if h['role'] in ('user', 'assistant'):
                    messages.append(h)
            messages.append({
                'role': 'user',
                'content': f'{user_msg}\n\n[camera snapshot failed: {e}]',
            })
            try:
                content, raw, used_cfg, tool_trace = _run_agent_loop(messages)
            except Exception as e2:
                return jsonify({'success': False, 'error': str(e2)}), 502
            est = _estimate_messages_tokens(messages)
            return jsonify({
                'success': True,
                'reply': content,
                'model': used_cfg['model'],
                'base_url': used_cfg['base_url'],
                'snapshot_attached': False,
                'snapshot_bytes': 0,
                'snapshot_data_url': None,
                'context_tokens_est': est['tokens_est'],
                'context_method': est['method'],
                'tool_calls': tool_trace,
                'motion_backend': backend,
            })

    # Build messages with custom system (override default in _build_chat_messages)
    messages, snapshot_data_url, snapshot_bytes = _build_chat_messages(
        history, user_msg, attach_snapshot=bool(attach and jpeg), jpeg_bytes=jpeg
    )
    if messages and messages[0].get('role') == 'system':
        messages[0]['content'] = system
    est = _estimate_messages_tokens(messages)

    try:
        content, raw, used_cfg, tool_trace = _run_agent_loop(messages)
    except Exception as e:
        olog.error(
            'ai_chat', f'AI chat failed: {e}',
            control_mode=mode, error=str(e)[:300],
            tokens_est=est.get('tokens_est'),
        )
        return jsonify({
            'success': False,
            'error': str(e),
            'context_tokens_est': est['tokens_est'],
            'motion_backend': backend,
        }), 502

    olog.info(
        'ai_chat',
        f'AI chat ok · tools={len(tool_trace or [])} · ~{est.get("tokens_est")} tok',
        control_mode=mode,
        model=used_cfg.get('model'),
        snapshot_attached=bool(snapshot_data_url),
        snapshot_bytes=snapshot_bytes or 0,
        tokens_est=est.get('tokens_est'),
        tool_count=len(tool_trace or []),
        tools=','.join(t.get('name', '') for t in (tool_trace or [])[:8]) or None,
    )
    return jsonify({
        'success': True,
        'reply': content,
        'model': used_cfg['model'],
        'base_url': used_cfg['base_url'],
        'snapshot_attached': bool(snapshot_data_url),
        'snapshot_bytes': snapshot_bytes,
        'snapshot_data_url': snapshot_data_url,
        'context_tokens_est': est['tokens_est'],
        'context_method': est['method'],
        'tool_calls': tool_trace,
        'motion_backend': backend,
    })


# ---------- Seek mode: pilot LLM steers; referee is detector OR vision LLM JSON ----------
from ai_track import (
    track_controller,
    resolve_track_goal,
    bbox_offsets,
    ptz_delta_from_offsets,
    clamp_ptz,
    DEFAULT_TRACK_MAX_STEPS,
    DEFAULT_TRACK_TIMEOUT_S,
    DEFAULT_TRACK_CONF,
)
from ai_seek import (
    seek_controller,
    parse_seek_goal,
    parse_llm_goal,
    parse_seek_referee,
    parse_llm_found_payload,
    parse_on_found,
    format_on_found_tts,
    evaluate_goal_detections,
    detector_labels,
    REFEREE_DETECTOR,
    REFEREE_LLM,
    ON_FOUND_NONE,
    ON_FOUND_TTS,
    DEFAULT_SEEK_MAX_STEPS,
    DEFAULT_SEEK_TIMEOUT_S,
    DEFAULT_SEEK_CONF,
    DEFAULT_SEEK_STEP_PAUSE_S,
    normalize_seek_max_steps,
    normalize_seek_timeout_s,
    motion_lock_should_ignore_zero,
    DEFAULT_ON_FOUND,
    DEFAULT_ON_FOUND_TTS,
)
from seek_nav import (  # noqa: E402
    seek_nav_plan as _seek_nav_plan,
    seek_normalize_action as _seek_normalize_action,
    seek_normalize_distance as _seek_normalize_distance,
    seek_normalize_obstacle_range as _seek_normalize_obstacle_range,
    SEEK_TURN_MS_BY_DIST as _SEEK_TURN_MS_BY_DIST,
    SEEK_TURN_DEG_BY_DIST as _SEEK_TURN_DEG_BY_DIST,
    interpret_base_voltage as _interpret_base_voltage,
    seek_battery_block_reason as _seek_battery_block_reason_pure,
    reset_escape_cycle as _seek_reset_escape_cycle,
    seek_commit_through_opening as _seek_commit_through_opening,
    seek_prefer_away_from_wall as _seek_prefer_away_from_wall,
    seek_may_reverse as _seek_may_reverse,
    SEEK_NAV_TOOL as _SEEK_NAV_TOOL,
    SEEK_NAV_TOOL_NAME as _SEEK_NAV_TOOL_NAME,
    seek_action_from_schema as _seek_action_from_schema,
)

_SEEK_JUDGE_SYSTEM = (
    "You are a visual goal referee for a robot camera. "
    "Look only at the provided image. Decide if the described target is clearly visible. "
    "Reply with JSON only — no markdown, no extra keys beyond the schema. "
    "found must be a boolean: true only if the target is clearly present in the image; "
    "false if absent, uncertain, occluded, or too small to be sure. "
    "reason is one short sentence."
)

_SEEK_NAV_SYSTEM = (
    "You are a navigation advisor for a small UGV with a pan-tilt camera. "
    "You are given THREE stills from the same pose: LEFT, STRAIGHT (center), and RIGHT. "
    "Decide the safest move AND how far to drive before the next scan. "
    "PRIMARY GOAL when seeking: say HOW FAR to go forward, or if blocked which way to turn. "
    "Prefer forward when the straight view is open (hallway/path clear enough for the robot). "
    "If the empty space is off-center in STRAIGHT, a short turn toward that empty lane first, "
    "then forward long on the next step. "
    "Choose turn_left or turn_right if straight is blocked/cluttered/unsafe and that side looks more open "
    "or more likely to reveal the goal. "
    "Also set drive_distance for how long the next timed hop should be before re-scanning: "
    "short = tight space / near obstacles / a small Fast aiming spin (~0.3s); "
    "medium = punchy hop over floor cables / a solid Fast room turn (~0.7s); "
    "long = clear hallway — USE THIS when the path looks free (go farther, not a 4s creep). "
    "Turns should usually use short (or medium at most). Long is mainly for clear forward. "
    "Do not pick short-forward when a corridor is open — this chassis stalls if it creeps. "
    "INDOOR NAV (this chassis): "
    "Drive AWAY from walls — if CENTRE is a wall/door slab, turn toward the emptier side; never inch into paint. "
    "Doorways/halls: after you enter a frame, take another hop of similar length so the body is fully PAST the jambs "
    "(stopping in the doorway wedges the wheels). "
    "Low hazards (bowls, cables, thresholds, baseboards) sit at bumper height — treat floor clutter in CENTRE as near. "
    "When approaching a wall, object, or door the robot will look DOWN at left/front/right (~±55°) "
    "before the next hop; honour that bumper inspect (floor_blocked → do not go forward). "
    "When a PERSON is nearby, look UP at left/front/right to keep the face in frame for identification — "
    "do not tilt down onto their feet. "
    "Reply with JSON only. action must be exactly one of: forward, turn_left, turn_right. "
    "drive_distance must be exactly one of: short, medium, long. "
    "Do not claim the goal is found — another system handles that."
)

_SEEK_NAV_JSON_SCHEMA = {
    'type': 'json_schema',
    'json_schema': {
        'name': 'seek_nav',
        'strict': True,
        'schema': {
            'type': 'object',
            'properties': {
                'action': {
                    'type': 'string',
                    'enum': ['forward', 'turn_left', 'turn_right'],
                    'description': 'Drive choice after comparing the three views.',
                },
                'drive_distance': {
                    'type': 'string',
                    'enum': ['short', 'medium', 'long'],
                    'description': (
                        'How far/long to drive before the next L/straight/R scan. '
                        'short≈near obstacles; medium≈open; long≈long clear hallway.'
                    ),
                },
                'reason': {
                    'type': 'string',
                    'description': 'One short sentence covering direction and distance.',
                },
                'path_clear_forward': {
                    'type': 'boolean',
                    'description': 'True if straight view looks open enough to drive forward.',
                },
            },
            'required': ['action', 'drive_distance', 'reason', 'path_clear_forward'],
            'additionalProperties': False,
        },
    },
}

# Fraction of args_config max_speed*max_rate for seek spins (1.0 = UI Fast)
_SEEK_TURN_SPEED_SCALE = 1.0
# Nav-plan + hop tables live in seek_nav.py (unit-tested). Aliases imported above.


def _seek_side_openness(views):
    """Score LEFT vs RIGHT near-field openness (higher = more free space / less structure).

    Returns dict: {left: float, right: float, prefer: 'left'|'right', scores_ok: bool}
    """
    scores = {}
    for name in ('left', 'right'):
        try:
            v = next((item for item in (views or []) if item.get('name') == name), None)
            jpeg = v.get('jpeg') if isinstance(v, dict) else None
            if not jpeg:
                scores[name] = None
                continue
            arr = np.frombuffer(jpeg, np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if img is None:
                scores[name] = None
                continue
            h, w = img.shape[:2]
            roi = img[int(h * 0.35):int(h * 0.95), int(w * 0.15):int(w * 0.85)]
            if roi.size == 0:
                scores[name] = None
                continue
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 40, 120)
            edge_density = float(np.count_nonzero(edges)) / float(edges.size)
            std = float(np.std(gray))
            # More open ≈ fewer edges and a bit of natural floor texture (not a blank wall)
            # Blank close wall: very low std + low edges → penalize
            open_score = 1.0 - min(1.0, edge_density * 4.0)
            if std < 14.0 and edge_density < 0.05:
                open_score *= 0.35  # likely a near flat wall
            scores[name] = round(open_score, 3)
        except Exception:
            scores[name] = None

    left_s = scores.get('left')
    right_s = scores.get('right')
    if left_s is None and right_s is None:
        return {'left': None, 'right': None, 'prefer': 'left', 'scores_ok': False}
    if left_s is None:
        prefer = 'right'
    elif right_s is None:
        prefer = 'left'
    elif right_s > left_s + 0.04:
        prefer = 'right'
    elif left_s > right_s + 0.04:
        prefer = 'left'
    else:
        prefer = 'left'  # slight default bias
    return {
        'left': left_s,
        'right': right_s,
        'prefer': prefer,
        'scores_ok': True,
    }


def _seek_prefer_open_turn(views):
    """Return turn_left or turn_right toward the more open side view."""
    info = _seek_side_openness(views)
    return 'turn_right' if info.get('prefer') == 'right' else 'turn_left'


def _seek_half_open_score(jpeg, *, half):
    """Openness of the left or right half of a still (0..1, higher = clearer)."""
    if not jpeg:
        return None
    try:
        arr = np.frombuffer(jpeg, np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None:
            return None
        h, w = img.shape[:2]
        if half == 'left':
            roi = img[int(h * 0.30):int(h * 0.95), 0:int(w * 0.50)]
        else:
            roi = img[int(h * 0.30):int(h * 0.95), int(w * 0.50):w]
        if roi.size == 0:
            return None
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        return _seek_roi_open_score(gray)
    except Exception:
        return None


def _seek_rear_clearance(views, *, min_score=0.36):
    """Can we reverse? Need BOTH rear quarters clear.

    −135° photo → its LEFT half is behind-left.
    +135° photo → its RIGHT half is behind-right.
    """
    left_v = next((v for v in (views or []) if isinstance(v, dict) and v.get('name') == 'left'), None)
    right_v = next((v for v in (views or []) if isinstance(v, dict) and v.get('name') == 'right'), None)
    left_half = _seek_half_open_score((left_v or {}).get('jpeg'), half='left')
    right_half = _seek_half_open_score((right_v or {}).get('jpeg'), half='right')
    left_ok = left_half is not None and float(left_half) >= float(min_score)
    right_ok = right_half is not None and float(right_half) >= float(min_score)
    return {
        'left_half': left_half,
        'right_half': right_half,
        'left_clear': left_ok,
        'right_clear': right_ok,
        'all_clear': bool(left_ok and right_ok),
        'min_score': min_score,
    }


_SEEK_FOUND_JSON_SCHEMA = {
    'type': 'json_schema',
    'json_schema': {
        'name': 'seek_found',
        'strict': True,
        'schema': {
            'type': 'object',
            'properties': {
                'found': {
                    'type': 'boolean',
                    'description': 'True only if the target is clearly visible in the image.',
                },
                'reason': {
                    'type': 'string',
                    'description': 'One short sentence explaining the decision.',
                },
            },
            'required': ['found', 'reason'],
            'additionalProperties': False,
        },
    },
}

_SEEK_FOUND_JSON_OBJECT = {'type': 'json_object'}


def opencv_goal_check(goal_label, conf_threshold=DEFAULT_SEEK_CONF, frame=None):
    """On-device detector oracle (MobileNet-SSD). Closed class list only."""
    if frame is None:
        frame = cvf.grab_bgr_frame()
    dets = []
    if frame is not None:
        dets = cvf.detect_objects_structured(frame, conf_threshold=0.12)
    else:
        dets = list(getattr(cvf, 'last_detections', []) or [])
    
    raw_labels = [
        f"{d.get('label')}: {round(float(d.get('confidence', 0))*100)}%"
        for d in (dets or []) if isinstance(d, dict) and d.get('label')
    ]
    olog.info(
        'ai_seek',
        f'CV scan for target "{goal_label}" (conf>={conf_threshold}) — saw {len(dets)} objects: {", ".join(raw_labels) or "none"}',
        goal=goal_label, conf_threshold=conf_threshold, raw_count=len(dets), detected_objects=raw_labels,
    )
    result = evaluate_goal_detections(dets, goal_label, conf_threshold=conf_threshold)
    result['frame_ok'] = frame is not None
    result['referee'] = REFEREE_DETECTOR
    result['raw_detections'] = raw_labels
    return result


def llm_goal_check(goal_text, jpeg=None):
    """Vision LLM oracle: snapshot + structured JSON {found: bool, reason: str}.

    Uses response_format json_schema when the server accepts it; falls back to
    json_object, then plain completion + robust parse. found defaults false on
    parse failure so Seek does not end on garbage.
    """
    goal = (goal_text or '').strip()
    if jpeg is None:
        jpeg = _grab_jpeg_bytes(max_width=640, quality=70)
    if not jpeg:
        return {
            'found': False,
            'goal_label': goal,
            'referee': REFEREE_LLM,
            'frame_ok': False,
            'reason': 'no camera frame',
            'parse_ok': False,
            'labels_found': [],
            'match_count': 0,
            'all_count': 0,
        }
    b64 = base64.b64encode(jpeg).decode('ascii')
    data_url = f'data:image/jpeg;base64,{b64}'
    user_content = [
        {
            'type': 'text',
            'text': (
                f'Target to find: {goal}\n'
                'Is this target clearly visible in the image? '
                'Respond with JSON: {"found": true|false, "reason": "..."}. '
                'found=true only if clearly present.'
            ),
        },
        {'type': 'image_url', 'image_url': {'url': data_url}},
    ]
    messages = [
        {'role': 'system', 'content': _SEEK_JUDGE_SYSTEM},
        {'role': 'user', 'content': user_content},
    ]
    content = ''
    format_used = None
    last_err = None
    for fmt_name, fmt in (
        ('json_schema', _SEEK_FOUND_JSON_SCHEMA),
        ('json_object', _SEEK_FOUND_JSON_OBJECT),
        ('plain', None),
    ):
        try:
            msg, _body, _cfg = _openai_chat(
                messages,
                max_tokens=80,
                temperature=0.0,
                tools=None,
                response_format=fmt,
                timeout=12,
            )
            content = _message_text_content(msg) or ''
            format_used = fmt_name
            last_err = None
            break
        except Exception as e:
            last_err = e
            # Schema often unsupported on local OpenAI-compat servers — try next.
            continue
    if last_err and not content:
        return {
            'found': False,
            'goal_label': goal,
            'referee': REFEREE_LLM,
            'frame_ok': True,
            'reason': f'judge LLM failed: {last_err}'[:300],
            'parse_ok': False,
            'error': str(last_err)[:300],
            'labels_found': [],
            'match_count': 0,
            'all_count': 0,
        }
    parsed = parse_llm_found_payload(content)
    found = bool(parsed.get('found'))
    return {
        'found': found,
        'goal_label': goal,
        'referee': REFEREE_LLM,
        'frame_ok': True,
        'reason': parsed.get('reason') or '',
        'parse_ok': bool(parsed.get('parse_ok')),
        'response_format': format_used,
        'raw_reply': (content or '')[:400],
        'labels_found': [goal] if found else [],
        'match_count': 1 if found else 0,
        'all_count': 1 if found else 0,
        'best': {'label': goal, 'confidence': 1.0 if found else 0.0} if found else None,
    }


def seek_goal_check(goal, referee=REFEREE_DETECTOR, conf_threshold=DEFAULT_SEEK_CONF, jpeg=None):
    """Dispatch to detector or LLM vision referee."""
    ref = parse_seek_referee(referee)
    if ref == REFEREE_LLM:
        return llm_goal_check(goal, jpeg=jpeg)
    frame = None
    if jpeg is not None:
        try:
            arr = np.frombuffer(jpeg, np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        except Exception:
            pass
    return opencv_goal_check(goal, conf_threshold=conf_threshold, frame=frame)


def _seek_disable_steady():
    """Turn off gimbal IMU steady so absolute PTZ looks are not fought by horizon hold."""
    try:
        base.base_json_ctrl({'T': f['cmd_config'].get('cmd_gimbal_steady', 137), 's': 0, 'y': 0})
    except Exception as e:
        olog.warn('ai_seek', f'Could not disable gimbal steady: {e}', error=str(e)[:160])


# ---------- ESP32 OLED (T:3) overlay during Seek ----------
# Small 4-line panel — only use 3 dedicated lines (line 3 blank):
#   0: SEEK          (mode label)
#   1: <goal>        (what we seek)
#   2: <DO>          (one-word decision: FWD/BACK/LEFT/RIGHT/PAN/THINK/…)
# While active, update_data_loop yields the panel so network/IP status does not race.
_SEEK_OLED_COLS = 16
_SEEK_OLED_ALT_S = 2.2  # unused (fixed layout; kept for compat)
_SEEK_OLED_KEYS = frozenset({
    'active', 'goal', 'referee', 'phase', 'step', 'activity', 'detail',
    'message', 'nav_summary', 'obstacle', 'frame', 'last_frame_at',
    'last_paint_sig', 'lines',
})
_SEEK_OLED_PHASE_SHORT = {
    'idle': 'IDLE',
    'running': 'RUN',
    'start': 'START',
    'triple_scan': 'PAN',
    'scan': 'SCAN',
    'goal_check': 'SCAN',
    'nav_decide': 'THINK',
    'llm': 'THINK',
    'drive': 'DRIVE',
    'found': 'FOUND',
    'stopped': 'STOP',
    'timeout': 'TIME',
    'failed': 'FAIL',
}
# Map free-form activity / action strings → one OLED word
_SEEK_OLED_DO_WORDS = {
    'forward': 'FWD', 'fwd': 'FWD', 'go_forward': 'FWD', 'straight': 'FWD', 'ahead': 'FWD',
    'backward': 'BACK', 'back': 'BACK', 'reverse': 'BACK', 'retreat': 'BACK',
    'turn_left': 'LEFT', 'left': 'LEFT',
    'turn_right': 'RIGHT', 'right': 'RIGHT',
    'pan': 'PAN', 'pan left': 'PAN', 'pan right': 'PAN', 'pan straight': 'PAN',
    'snap': 'SNAP', 'photo': 'SNAP', 'shutter': 'SNAP',
    'detect': 'SCAN', 'check': 'SCAN', 'scan': 'SCAN',
    'llm': 'THINK', 'think': 'THINK', 'nav': 'THINK', 'llm think': 'THINK',
    'drive': 'DRIVE',
    'found': 'FOUND', 'tts': 'SPEAK', 'speak': 'SPEAK',
    'start': 'START', 'stop': 'STOP', 'stopped': 'STOP',
    'failed': 'FAIL', 'fail': 'FAIL', 'timeout': 'TIME',
    're-center': 'CENTER', 'recenter': 'CENTER', 'center': 'CENTER', 'centre': 'CENTER',
}
_seek_oled_lock = threading.Lock()
_seek_oled = {
    'active': False,
    'goal': '',
    'referee': '',
    'phase': 'idle',
    'step': 0,
    'activity': '',      # one-word decision
    'detail': '',
    'message': '',
    'nav_summary': '',
    'obstacle': '',
    'frame': 0,
    'last_frame_at': 0.0,
    'last_paint_sig': '',
    'lines': ['', '', '', ''],
}
# Last non-seek (network/IP) OLED contents — restored when seek exits
_net_oled_lock = threading.Lock()
_net_oled_lines = ['', '', 'F/J:5000/8888', '']
_pre_seek_oled_lines = None  # snapshot taken when seek first grabs the panel


def _oled_fit(text, width=_SEEK_OLED_COLS):
    """Fit text to OLED line width (ASCII only, no unicode ellipsis)."""
    s = (text or '').replace('\n', ' ').replace('\r', ' ').strip()
    # Degree / dash / quotes common in nav text → plain ASCII
    for a, b in (
        ('\u00b0', 'd'), ('°', 'd'),
        ('\u2013', '-'), ('\u2014', '-'), ('\u2026', '..'),
        ('\u2018', "'"), ('\u2019', "'"), ('\u201c', '"'), ('\u201d', '"'),
        ('\u00b7', ' '), ('·', ' '),
    ):
        s = s.replace(a, b)
    # Drop remaining non-ascii so ESP32 glyph set doesn't show garbage
    s = ''.join(ch if 32 <= ord(ch) < 127 else ' ' for ch in s)
    s = ' '.join(s.split())
    if len(s) <= width:
        return s
    return s[: max(0, width - 2)] + '..'


def _oled_phase_short(phase):
    p = (phase or 'run').strip().lower()
    return _SEEK_OLED_PHASE_SHORT.get(p, (p[:8] or 'RUN').upper())


def _oled_do_word(activity=None, phase=None, nav_summary=None, message=None):
    """Collapse seek state into ONE short uppercase decision word for OLED line 2."""
    candidates = [
        (activity or '').strip().lower(),
        (nav_summary or '').strip().lower().split()[0] if nav_summary else '',
        (message or '').strip().lower(),
        (phase or '').strip().lower(),
    ]
    for raw in candidates:
        if not raw:
            continue
        # Exact map
        if raw in _SEEK_OLED_DO_WORDS:
            return _SEEK_OLED_DO_WORDS[raw]
        # Action token from "turn_left/medium" or "pan left (-135)"
        tok = raw.replace('-', ' ').replace('_', ' ').split('/')[0].strip()
        first = tok.split()[0] if tok else ''
        if first in _SEEK_OLED_DO_WORDS:
            return _SEEK_OLED_DO_WORDS[first]
        joined = ' '.join(tok.split()[:2])
        if joined in _SEEK_OLED_DO_WORDS:
            return _SEEK_OLED_DO_WORDS[joined]
        # Substring cues
        if 'turn left' in raw or raw.startswith('turn_left') or raw == 'left':
            return 'LEFT'
        if 'turn right' in raw or raw.startswith('turn_right') or raw == 'right':
            return 'RIGHT'
        if 'backward' in raw or 'reverse' in raw:
            return 'BACK'
        if 'forward' in raw:
            return 'FWD'
        if raw.startswith('pan') or ' pan ' in f' {raw} ':
            return 'PAN'
        if 'found' in raw:
            return 'FOUND'
        if 'tts' in raw or 'speak' in raw or 'announc' in raw:
            return 'SPEAK'
        if 'llm' in raw or 'think' in raw or 'consult' in raw:
            return 'THINK'
        if 'detect' in raw or 'scan' in raw or 'check' in raw:
            return 'SCAN'
        if 'snap' in raw or 'photo' in raw:
            return 'SNAP'
        if 'center' in raw or 'centre' in raw:
            return 'CENTER'
        if 'drive' in raw:
            return 'DRIVE'
        if 'stop' in raw:
            return 'STOP'
        if 'fail' in raw:
            return 'FAIL'
    # Phase table fallback
    ph = _oled_phase_short(phase)
    if ph and ph not in ('RUN',):
        return ph
    # Last resort: first ASCII word uppercased
    seed = (activity or phase or 'RUN').strip()
    seed = re.sub(r'[^A-Za-z0-9]+', '', seed.split()[0] if seed else 'RUN')
    return (seed or 'RUN')[:8].upper()


def _seek_oled_is_active():
    with _seek_oled_lock:
        return bool(_seek_oled.get('active'))


def _oled_write_lines(lines):
    """Write up to 4 OLED lines (ASCII-fitted)."""
    out = []
    for i in range(4):
        line = ''
        if lines and i < len(lines) and lines[i] is not None:
            line = _oled_fit(str(lines[i]))
        out.append(line)
        try:
            base.base_oled(i, line)
        except Exception:
            pass
    return out


def _net_oled_compose_lines(start_time=None):
    """Build the usual network/IP status lines (what shows outside Seek)."""
    eth0 = getattr(si, 'eth0_ip', None)
    wlan = getattr(si, 'wlan_ip', None)
    if eth0:
        l0 = f'E:{eth0}'
    else:
        l0 = 'E: No Ethernet'
    if wlan:
        l1 = f'W:{wlan}'
    else:
        iface = getattr(si, 'net_interface', 'wlan0')
        l1 = f'W: NO {iface}'
    l2 = 'F/J:5000/8888'
    if start_time is None:
        # Prefer last known uptime line if we have one
        with _net_oled_lock:
            prev = list(_net_oled_lines)
        if prev and len(prev) > 3 and prev[3]:
            l3 = prev[3]
        else:
            mode = getattr(si, 'wifi_mode', 'AP')
            rssi = getattr(si, 'wifi_rssi', 0)
            l3 = f'{mode} 00:00:00 {rssi}dBm'
    else:
        elapsed_time = time.time() - float(start_time)
        hours = int(elapsed_time // 3600)
        minutes = int((elapsed_time % 3600) // 60)
        seconds = int(elapsed_time % 60)
        mode = getattr(si, 'wifi_mode', 'AP')
        rssi = getattr(si, 'wifi_rssi', 0)
        l3 = f'{mode} {hours:02d}:{minutes:02d}:{seconds:02d} {rssi}dBm'
    return [l0, l1, l2, l3]


def _net_oled_remember(lines):
    """Store last network OLED paint for pre-seek snapshot / restore."""
    global _net_oled_lines
    with _net_oled_lock:
        _net_oled_lines = [str(x or '') for x in (lines or [''] * 4)[:4]]
        while len(_net_oled_lines) < 4:
            _net_oled_lines.append('')


def _net_oled_paint(start_time=None, remember=True):
    """Paint network status and optionally remember it as the pre-seek baseline."""
    lines = _net_oled_compose_lines(start_time=start_time)
    written = _oled_write_lines(lines)
    if remember and not _seek_oled_is_active():
        _net_oled_remember(written)
    return written


def _seek_oled_snapshot_pre_seek():
    """Capture what is on the OLED before Seek takes over."""
    global _pre_seek_oled_lines
    with _net_oled_lock:
        snap = list(_net_oled_lines)
    # If we never painted net status yet, compose live now
    if not any(snap):
        snap = _net_oled_compose_lines()
    _pre_seek_oled_lines = list(snap)
    try:
        olog.info(
            'ai_seek',
            f'OLED pre-seek snapshot: {snap[0]!r} | {snap[1]!r} | {snap[2]!r} | {snap[3]!r}',
        )
    except Exception:
        pass
    return list(snap)


def _seek_oled_restore_pre_seek():
    """Restore OLED to the snapshot taken when Seek started (or live net status)."""
    global _pre_seek_oled_lines
    with _net_oled_lock:
        snap = list(_pre_seek_oled_lines) if _pre_seek_oled_lines else list(_net_oled_lines)
    if not any(snap):
        snap = _net_oled_compose_lines()
    written = _oled_write_lines(snap)
    _net_oled_remember(written)
    _pre_seek_oled_lines = None
    try:
        olog.info(
            'ai_seek',
            f'OLED restored after seek: {written[0]!r} | {written[1]!r} | {written[2]!r} | {written[3]!r}',
        )
    except Exception:
        pass
    return written


def _seek_oled_snapshot():
    """Debug snapshot for /api/ai/seek/status (no lock held across paint)."""
    with _seek_oled_lock:
        lines = list(_seek_oled.get('lines') or ['', '', '', ''])
        do = (_seek_oled.get('activity') or (lines[2] if len(lines) > 2 else '') or '')
        return {
            'active': bool(_seek_oled.get('active')),
            'layout': ['SEEK', '<goal>', '<do>', ''],
            'goal': _seek_oled.get('goal') or '',
            'do': do,
            'activity': do,
            'phase': _seek_oled.get('phase') or '',
            'step': int(_seek_oled.get('step') or 0),
            'lines': lines,
            'nav_summary': _seek_oled.get('nav_summary') or '',
            'obstacle': _seek_oled.get('obstacle') or '',
        }


def _seek_oled_set(**kwargs):
    """Update seek OLED state and paint immediately (best-effort)."""
    with _seek_oled_lock:
        was_active = bool(_seek_oled.get('active'))
        for k, v in kwargs.items():
            if k in _SEEK_OLED_KEYS and k not in ('lines', 'last_paint_sig', 'last_frame_at', 'frame'):
                _seek_oled[k] = v
        # Collapse activity/message into one decision word for the tiny panel
        do = _oled_do_word(
            activity=_seek_oled.get('activity'),
            phase=_seek_oled.get('phase'),
            nav_summary=_seek_oled.get('nav_summary'),
            message=_seek_oled.get('message'),
        )
        _seek_oled['activity'] = do
        _seek_oled['detail'] = do
        _seek_oled['message'] = do
        _seek_oled['active'] = True
        if not was_active:
            _seek_oled['frame'] = 0
            _seek_oled['last_frame_at'] = time.time()
            _seek_oled['last_paint_sig'] = ''
    # First takeover: remember what the panel showed (network/IP status)
    if not was_active:
        try:
            _seek_oled_snapshot_pre_seek()
        except Exception:
            pass
    try:
        _seek_oled_paint(force=True)
    except Exception as e:
        try:
            olog.warn('ai_seek', f'OLED set/paint failed: {e}', error=str(e)[:160])
        except Exception:
            pass


def _seek_oled_set_nav(action, dist, *, plan=None, nav=None, obstacle=None, **extra):
    """Convenience: set OLED from a nav action (line 2 becomes one-word FWD/LEFT/…)."""
    plan = plan or {}
    nav = nav or {}
    act = action or plan.get('action') or nav.get('action') or ''
    d = dist or plan.get('drive_distance') or nav.get('drive_distance') or ''
    nav_s = f'{act}/{d}' if act and d else (act or plan.get('summary') or nav.get('summary') or '')
    obs = obstacle
    if obs is None:
        obs = plan.get('obstacle_range') or nav.get('obstacle_range') or nav.get('obstacle_ahead_range') or ''
    do_word = _oled_do_word(activity=act, phase=extra.get('phase'), nav_summary=nav_s)
    payload = {
        'activity': do_word,  # already one word
        'detail': do_word,
        'nav_summary': str(nav_s)[:48],
        'obstacle': str(obs or '')[:16],
        'message': do_word,
    }
    for k in ('goal', 'referee', 'phase', 'step'):
        if k in extra and extra[k] is not None:
            payload[k] = extra[k]
    _seek_oled_set(**payload)


def _seek_oled_clear(restore_hint=True):
    """Release OLED overlay and restore what was on screen before Seek."""
    with _seek_oled_lock:
        _seek_oled['active'] = False
        _seek_oled['goal'] = ''
        _seek_oled['referee'] = ''
        _seek_oled['activity'] = ''
        _seek_oled['detail'] = ''
        _seek_oled['message'] = ''
        _seek_oled['nav_summary'] = ''
        _seek_oled['obstacle'] = ''
        _seek_oled['last_paint_sig'] = ''
        _seek_oled['lines'] = ['', '', '', '']
        _seek_oled['phase'] = 'idle'
        _seek_oled['step'] = 0
        _seek_oled['frame'] = 0
    if restore_hint:
        try:
            # Immediate restore of pre-seek (usually network/IP) lines
            _seek_oled_restore_pre_seek()
        except Exception:
            try:
                _net_oled_paint(remember=True)
            except Exception:
                pass
        try:
            olog.info('ai_seek', 'OLED overlay released; pre-seek screen restored')
        except Exception:
            pass


def _seek_oled_paint(force=False, advance_frame=None):
    """Write fixed 3-line seek status (line 3 blank).

      0: SEEK
      1: <goal>
      2: <one-word decision>   e.g. FWD / BACK / LEFT / RIGHT / PAN / THINK / FOUND
      3: (blank)
    """
    with _seek_oled_lock:
        if not _seek_oled.get('active'):
            return False
        st = dict(_seek_oled)
        # Fixed layout — no frame flip
        _seek_oled['frame'] = 0
        if advance_frame is not None:
            pass

    goal = (st.get('goal') or '?').strip() or '?'
    do_word = _oled_do_word(
        activity=st.get('activity'),
        phase=st.get('phase'),
        nav_summary=st.get('nav_summary'),
        message=st.get('message'),
    )

    lines = [
        _oled_fit('SEEK'),
        _oled_fit(goal),
        _oled_fit(do_word),
        '',  # keep 4th line clear on small panel
    ]

    sig = '|'.join(lines)
    with _seek_oled_lock:
        if not force and sig == _seek_oled.get('last_paint_sig'):
            return True
        _seek_oled['last_paint_sig'] = sig
        _seek_oled['lines'] = list(lines)

    try:
        for i, line in enumerate(lines):
            base.base_oled(i, line if line is not None else '')
        try:
            olog.debug(
                'ai_seek',
                f'OLED paint: {lines[0]} | {lines[1]} | {lines[2]}',
                goal=goal, do=do_word, phase=st.get('phase'), step=st.get('step'),
            )
        except Exception:
            pass
    except Exception as e:
        olog.warn('ai_seek', f'OLED paint failed: {e}', error=str(e)[:160])
        return False
    return True


def _seek_oled_from_ctrl(ctrl, **extra):
    """Push current controller status into OLED overlay (prefers last_nav)."""
    try:
        st = ctrl.status() if ctrl is not None else {}
    except Exception:
        st = {}
    nav = st.get('last_nav') or {}
    phase = extra.get('phase') or st.get('seek_phase') or st.get('phase') or 'run'
    action = extra.get('action') or nav.get('action') or ''
    dist = extra.get('dist') or nav.get('drive_distance') or ''
    obs = extra.get('obstacle')
    if obs is None:
        obs = nav.get('obstacle_range') or nav.get('obstacle_ahead_range') or ''

    activity = extra.get('activity') or action or phase
    nav_summary = extra.get('nav_summary')
    if nav_summary is None:
        nav_summary = f'{action}/{dist}' if action and dist else (action or nav.get('summary') or '')

    do_word = _oled_do_word(
        activity=activity,
        phase=phase,
        nav_summary=nav_summary,
        message=extra.get('message') or st.get('message'),
    )

    _seek_oled_set(
        goal=extra.get('goal') or st.get('goal_label') or st.get('goal_text') or '',
        referee=extra.get('referee') or st.get('referee') or '',
        phase=phase,
        step=extra.get('step', st.get('step') or 0),
        activity=do_word,
        detail=do_word,
        message=do_word,
        nav_summary=str(nav_summary or '')[:48],
        obstacle=str(obs or '')[:16],
        active=True,
    )


def _seek_run_on_found(ctrl, label):
    """Run configured post-found action. Default is TTS; honor explicit 'none'."""
    action = ctrl.on_found_action()
    if not action:
        action = DEFAULT_ON_FOUND
    try:
        _seek_oled_set(
            goal=label, phase='found', activity='FOUND',
            detail='FOUND', message='FOUND',
            nav_summary='found', obstacle='',
        )
    except Exception:
        pass
    if action == ON_FOUND_NONE:
        ctrl.append_log('found', f'Found {label} — on_found=none (no announcement)')
        ctrl.update(on_found_done=True, on_found_phrase=None)
        return
    if action == ON_FOUND_TTS:
        phrase = format_on_found_tts(ctrl.on_found_tts_template(), label)
        ctrl.append_log('tts', f'Announcing: “{phrase}”…')
        ctrl.update(on_found_phrase=phrase)
        try:
            _seek_oled_set(
                goal=label, phase='found', activity='SPEAK',
                detail='SPEAK', message='SPEAK',
                nav_summary='announce',
            )
        except Exception:
            pass
        try:
            # Block so the announcement is not dropped by a race with halt/stop
            result = audio_ctrl.speak(phrase, force=True, block=True)
            if result.get('ok'):
                ctrl.update(
                    on_found_done=True,
                    on_found_error=None,
                    on_found_backend=result.get('backend'),
                )
                ctrl.append_log(
                    'tts',
                    f'TTS OK ({result.get("backend")}): “{phrase}”',
                    backend=result.get('backend') or '',
                )
                olog.info(
                    'ai_seek',
                    f'On-found TTS spoken via {result.get("backend")}: "{phrase}"',
                    goal=label, action='tts', backend=result.get('backend'),
                )
            else:
                err = result.get('error') or 'TTS failed'
                ctrl.update(on_found_done=False, on_found_error=str(err)[:200])
                ctrl.append_log('tts', f'TTS FAILED: {err} — phrase was “{phrase}”')
                olog.warn(
                    'ai_seek',
                    f'On-found TTS failed: {err}',
                    error=str(err)[:200], goal=label,
                )
        except Exception as e:
            olog.warn('ai_seek', f'On-found TTS failed: {e}', error=str(e)[:200], goal=label)
            ctrl.update(on_found_done=False, on_found_error=str(e)[:200])
            ctrl.append_log('tts', f'TTS exception: {e}')
        return
    olog.warn('ai_seek', f'Unknown on_found action: {action}', action=action)
    ctrl.append_log('warn', f'Unknown on_found action: {action}')


def _seek_force_tools_on():
    """Ensure CV + motion + gimbal tools are offered during Seek pilot turns."""
    try:
        _set_capabilities({
            'group_computer_vision': True,
            'get_cv_detections': True,
            'get_camera_snapshot': True,
            'group_ros2_motion': True,
            'send_motor_command': True,
            'stop_motors': True,
            'send_gimbal_command': True,
        })
    except Exception as e:
        olog.warn('ai_seek', f'Could not enable seek tools: {e}', error=str(e)[:200])


# Triple-view nav: left / straight / right pans (degrees on T:133 X)
# Side pans ±135°: view centres span 270°, and with camera FOV (~60–90°) this
# approximates a full 360° look-around (left/right see well into the rear).
_SEEK_VIEW_PAN_DEG = 135
# T:133 Y — slight look-down so bowls, cable runners, and baseboards are in frame
_SEEK_LOOK_TILT_DEG = -12.0
# Steeper bumper-height inspect when approaching a wall / object / door
_SEEK_LOOKDOWN_TILT_DEG = -22.0
_SEEK_LOOKUP_TILT_DEG = 18.0  # face height when a person is nearby
_SEEK_LOOKDOWN_PANS = (
    ('left', -55.0),
    ('straight', 0.0),
    ('right', 55.0),
)
_SEEK_LOOKUP_PANS = _SEEK_LOOKDOWN_PANS
_SEEK_VIEW_PANS = (
    ('left', -_SEEK_VIEW_PAN_DEG),
    ('straight', 0),
    ('right', _SEEK_VIEW_PAN_DEG),
)
# Wait for HW pan feedback after each aim (not a blind sleep-only “settle”)
_SEEK_PAN_TOL_DEG = 14.0       # larger swings to ±135°
_SEEK_PAN_WAIT_MAX_S = 4.0     # allow time for full left/right pan
_SEEK_PAN_POLL_S = 0.08
_SEEK_PAN_POST_ARRIVE_S = 0.12  # brief damp after arrival before shutter
_SEEK_PAN_FALLBACK_SLEEP_S = 0.55  # if no HW pan feedback at all
# Live cam_aim publish throttle (UI polls ~5Hz; wait loop is ~12Hz)
_SEEK_CAM_AIM_MIN_INTERVAL_S = 0.12
_SEEK_CAM_AIM_MIN_LIVE_DELTA_DEG = 3.0
# ESP32 pan feedback is often stuck near 0 while T:133 is mid-swing — estimate
# live angle so the UI needle moves (≈ 135° in ~2.5s on this gimbal).
_SEEK_PAN_EST_DPS = 54.0
# Throttle + motion state for the single cam_aim publisher
_cam_aim_pub = {
    't': 0.0,
    'cmd': None,
    'live': None,
    'settled': None,
    'motion_t0': 0.0,
    'motion_from': 0.0,
    'motion_cmd': None,
}
_SEEK_DRIVE_MS = 1100  # legacy default; hop tables live in seek_nav.py


def _seek_pan_deg_to_rad(pan_deg, tilt_deg=0.0):
    """Match send_gimbal_command convention: x_deg = -pan_rad * 180/pi."""
    pan_rad = -float(pan_deg) * math.pi / 180.0
    tilt_rad = float(tilt_deg) * math.pi / 180.0
    return {'pan_rad': round(pan_rad, 4), 'tilt_rad': round(tilt_rad, 4)}


def _seek_read_hw_pan_tilt(request_feedback=True):
    """Best-effort hardware pan/tilt degrees from ESP32 feedback."""
    try:
        snap = _ptz_status_snapshot(request_feedback=request_feedback, wait_s=0.08)
        hw = (snap or {}).get('hardware') or {}
        return hw.get('pan_deg'), hw.get('tilt_deg'), snap
    except Exception:
        return None, None, None


def _seek_wait_pan_arrived(target_pan_deg, tol_deg=None, max_s=None, should_stop=None):
    """Poll HW pan until near target_pan_deg (T:133 X convention) or timeout.

    Returns dict: settled, hw_pan, err_deg, waited_s, samples, reason.
    Live UI updates go only through cam_aim (throttled).
    """
    tol = float(tol_deg if tol_deg is not None else _SEEK_PAN_TOL_DEG)
    max_s = float(max_s if max_s is not None else _SEEK_PAN_WAIT_MAX_S)
    t0 = time.time()
    samples = 0
    last_pan = None
    saw_feedback = False
    target = float(target_pan_deg)
    while (time.time() - t0) < max_s:
        if should_stop and should_stop():
            return {
                'settled': False,
                'hw_pan': last_pan,
                'err_deg': None,
                'waited_s': round(time.time() - t0, 3),
                'samples': samples,
                'reason': 'stop_requested',
            }
        pan, _tilt, _snap = _seek_read_hw_pan_tilt(request_feedback=True)
        samples += 1
        if pan is not None:
            saw_feedback = True
            last_pan = float(pan)
            try:
                _seek_publish_cam_aim(
                    target, float(_cam_aim_pub.get('tilt') or 0.0),
                    hw_pan=last_pan,
                    settled=False,
                    live_pan=last_pan,
                )
            except Exception:
                pass
            err = abs(last_pan - target)
            if err <= tol:
                time.sleep(_SEEK_PAN_POST_ARRIVE_S)
                return {
                    'settled': True,
                    'hw_pan': last_pan,
                    'err_deg': round(err, 2),
                    'waited_s': round(time.time() - t0, 3),
                    'samples': samples,
                    'reason': 'within_tol',
                }
        else:
            # No HW yet — keep cmd visible (publisher throttles)
            try:
                _seek_publish_cam_aim(
                    target, float(_cam_aim_pub.get('tilt') or 0.0), settled=False,
                )
            except Exception:
                pass
        time.sleep(_SEEK_PAN_POLL_S)
    err = None
    if last_pan is not None:
        err = round(abs(last_pan - target), 2)
    return {
        'settled': False,
        'hw_pan': last_pan,
        'err_deg': err,
        'waited_s': round(time.time() - t0, 3),
        'samples': samples,
        'reason': 'timeout_no_feedback' if not saw_feedback else 'timeout_not_within_tol',
    }


def _seek_pan_label(pan_deg):
    """Human label for camera pan (front / rear-left / rear-right)."""
    try:
        p = float(pan_deg)
    except (TypeError, ValueError):
        return 'pan?'
    if abs(p) < 15:
        return 'FRONT'
    if p <= -90:
        return 'REAR-L'
    if p >= 90:
        return 'REAR-R'
    if p < 0:
        return 'LEFT'
    return 'RIGHT'


def _seek_estimate_live_pan(cmd, hw_pan=None, settled=False, live_pan=None):
    """Best needle angle for UI: trust HW only when it tracks the command.

    ESP32 often reports ~0° throughout a ±135° pan, so raw HW freezes the
    overlay. Fall back to a time-based sweep from the last known angle.
    """
    p = float(cmd)
    now = time.time()
    prev = _cam_aim_pub
    hw_f = None
    if hw_pan is not None:
        try:
            hw_f = float(hw_pan)
        except (TypeError, ValueError):
            hw_f = None

    # New target → start motion estimate from last live/cmd
    prev_cmd = prev.get('motion_cmd')
    if prev_cmd is None or abs(float(prev_cmd) - p) > 1.5:
        from_deg = prev.get('live')
        if from_deg is None:
            from_deg = prev.get('cmd')
        if from_deg is None:
            from_deg = hw_f if hw_f is not None else 0.0
        prev['motion_from'] = float(from_deg)
        prev['motion_t0'] = now
        prev['motion_cmd'] = p

    if settled:
        return p, hw_f

    # Explicit live_pan from caller only if it is not "stuck at 0 while cmd far"
    if live_pan is not None:
        try:
            lp = float(live_pan)
            if not (abs(p) > 40 and abs(lp) < 12 and abs(lp - p) > 40):
                return lp, hw_f
        except (TypeError, ValueError):
            pass

    # HW useful: near target, or has moved away from motion start toward cmd
    if hw_f is not None:
        err = abs(hw_f - p)
        moved = abs(hw_f - float(prev.get('motion_from') or 0.0))
        if err <= float(_SEEK_PAN_TOL_DEG) or err < 18:
            return hw_f, hw_f
        if moved > 12 and abs(hw_f) > 10:
            # Feedback is actually sweeping
            return hw_f, hw_f
        # else: HW stuck — ignore for needle

    # Time-based estimate toward command
    t0 = float(prev.get('motion_t0') or now)
    from_deg = float(prev.get('motion_from') or 0.0)
    elapsed = max(0.0, now - t0)
    span = p - from_deg
    if abs(span) < 0.5:
        return p, hw_f
    step = float(_SEEK_PAN_EST_DPS) * elapsed
    if step >= abs(span):
        est = p
    else:
        est = from_deg + (span / abs(span)) * step
    return est, hw_f


_ptz_aim_lock = threading.Lock()
_ptz_aim = {
    'x': 0.0,
    'y': 0.0,
    'hw_pan': None,
    'hw_tilt': None,
    'settled': True,
    'source': 'init',
    't': 0.0,
}


def _ptz_aim_public():
    """Last commanded T:133 X/Y + last HW if known. For /api/status and HUD."""
    with _ptz_aim_lock:
        d = dict(_ptz_aim)
    return {
        'pan_deg': d.get('x'),
        'tilt_deg': d.get('y'),
        'hw_pan': d.get('hw_pan'),
        'hw_tilt': d.get('hw_tilt'),
        'settled': d.get('settled'),
        'source': d.get('source'),
        'cmd': d.get('x'),
        'live': d.get('x'),
        'hw': d.get('hw_pan'),
        'tilt': d.get('y'),
    }


def _publish_ptz_aim(x, y, *, hw_pan=None, hw_tilt=None, settled=None, source='cmd'):
    """Central PTZ SoT: Raw HUD, Seek overlay, /api/status, /api/ptz.

    x,y are T:133 degrees (same numbers the Raw stick writes to #Pan / #Tilt).
    """
    try:
        x = float(x)
        y = float(y)
    except (TypeError, ValueError):
        return None
    settled_b = True if settled is None else bool(settled)
    with _ptz_aim_lock:
        _ptz_aim.update({
            'x': x, 'y': y,
            'hw_pan': hw_pan, 'hw_tilt': hw_tilt,
            'settled': settled_b, 'source': source, 't': time.time(),
        })
    try:
        cvf.pan_angle = x
        cvf.tilt_angle = y
    except Exception:
        pass
    try:
        _seek_publish_cam_aim(x, y, hw_pan=hw_pan, settled=settled_b, force=True)
    except Exception:
        pass
    try:
        socketio.emit(
            'ptz_aim',
            {
                'pan_deg': round(x, 1),
                'tilt_deg': round(y, 1),
                'cmd': round(x, 1),
                'live': round(x, 1),
                'hw': None if hw_pan is None else round(float(hw_pan), 1),
                'tilt': round(y, 1),
                'settled': settled_b,
                'source': source,
            },
            namespace='/ctrl',
        )
    except Exception:
        pass
    return _ptz_aim_public()


def _seek_publish_cam_aim(pan_deg, tilt_deg=0.0, hw_pan=None, settled=None, live_pan=None, force=False):
    """Seek overlay + last-aim cache. Also used when Seek is idle (API/stick PTZ).

    Publishes cmd + estimated live (HW when trustworthy, else timed sweep).
    force=True always publishes (command start / final settle).
    """
    try:
        p = float(pan_deg)
        t = float(tilt_deg or 0.0)
        settled_b = False if settled is None else bool(settled)
        show_f, hw_f = _seek_estimate_live_pan(
            p, hw_pan=hw_pan, settled=settled_b, live_pan=live_pan,
        )
        now = time.time()
        prev = _cam_aim_pub
        if not force:
            same_cmd = prev['cmd'] is not None and abs(float(prev['cmd']) - p) < 0.5
            same_settled = prev['settled'] is settled_b
            live_delta = (
                abs(float(prev['live']) - show_f)
                if prev['live'] is not None else 999.0
            )
            age = now - float(prev['t'] or 0)
            if (
                same_cmd
                and same_settled
                and live_delta < _SEEK_CAM_AIM_MIN_LIVE_DELTA_DEG
                and age < _SEEK_CAM_AIM_MIN_INTERVAL_S
            ):
                return

        hw_out = None
        if hw_f is not None:
            hw_out = round(float(hw_f), 1)
        elif hw_pan is not None:
            try:
                hw_out = round(float(hw_pan), 1)
            except (TypeError, ValueError):
                hw_out = None

        aim = {
            'cmd': round(p, 1),
            'live': round(float(show_f), 1),
            'hw': hw_out,
            'tilt': round(t, 1),
            'settled': settled_b,
            'label': _seek_pan_label(show_f),
            'target': _seek_pan_label(p),
            # true when live is timed estimate (HW ignored / missing)
            'est': bool(
                not settled_b
                and (
                    hw_out is None
                    or (abs(p) > 40 and abs(hw_out) < 12 and abs(hw_out - p) > 40)
                )
            ),
        }
        seek_controller.update(cam_aim=aim)
        try:
            if track_controller.is_running():
                track_controller.update(cam_aim=aim)
        except Exception:
            pass
        _cam_aim_pub['t'] = now
        _cam_aim_pub['cmd'] = p
        _cam_aim_pub['live'] = float(show_f)
        _cam_aim_pub['tilt'] = t
        _cam_aim_pub['settled'] = settled_b
    except Exception:
        pass


def _seek_look_deg(pan_deg, tilt_deg=None, settle_s=None, wait_hw=True, should_stop=None):
    """Point gimbal (degrees), optionally wait until HW pan confirms arrival, then return.

    Sequence: command → (confirm pan completed | timeout) → ready for photo.
    settle_s only used as blind fallback when wait_hw is False.
    Overlay state is only published via cam_aim (not phase_meta / drive args).
    Default tilt looks slightly down (bowls / cable / baseboards).
    """
    if tilt_deg is None:
        tilt_deg = float(_SEEK_LOOK_TILT_DEG)
    look = _seek_pan_deg_to_rad(pan_deg, tilt_deg)
    _seek_publish_cam_aim(pan_deg, tilt_deg, settled=False, force=True)
    res = _execute_agent_tool('send_gimbal_command', look)
    arrival = None
    if wait_hw:
        arrival = _seek_wait_pan_arrived(
            float(pan_deg),
            should_stop=should_stop,
        )
        # If firmware never reports pan, still give a minimal settle so we don't snap mid-command
        if arrival.get('reason') == 'timeout_no_feedback':
            time.sleep(
                settle_s if settle_s is not None else _SEEK_PAN_FALLBACK_SLEEP_S
            )
    else:
        time.sleep(settle_s if settle_s is not None else _SEEK_PAN_FALLBACK_SLEEP_S)
        arrival = {'settled': False, 'reason': 'wait_hw_disabled', 'hw_pan': None}
    _seek_publish_cam_aim(
        pan_deg, tilt_deg,
        hw_pan=(arrival or {}).get('hw_pan'),
        settled=bool(arrival and arrival.get('settled')),
        force=True,
    )
    return look, res, arrival


def _seek_grab_jpeg():
    try:
        return _grab_jpeg_bytes(max_width=480, quality=65)
    except Exception:
        return None


def _seek_capture_triple_views(ctrl, step, steps_label, goal_label=None, conf_threshold=DEFAULT_SEEK_CONF, referee=REFEREE_DETECTOR):
    """For each view: pan → confirm pan completed → take photo → run goal check. Then re-center.

    Order: left → straight → right.
    """
    views = []
    found_check = None
    for i, (name, pan_deg) in enumerate(_SEEK_VIEW_PANS, start=1):
        if ctrl.should_stop():
            break
        ctrl.update(
            seek_phase='triple_scan',
            message=(
                f'Step {step}/{steps_label}: pan {name} ({i}/{len(_SEEK_VIEW_PANS)}, '
                f'target≈{pan_deg}°) — waiting for pan complete…'
            ),
            # Scan progress only — live pan angle lives exclusively in cam_aim
            phase_meta={
                'view': name, 'index': i,
                'total': len(_SEEK_VIEW_PANS), 'sub': 'aim_wait',
            },
        )
        try:
            _seek_oled_set(
                phase='triple_scan', step=step,
                activity=f'pan {name}',
                detail=f'{int(pan_deg) if float(pan_deg) == int(pan_deg) else pan_deg} deg',
                message=f'{i}/{len(_SEEK_VIEW_PANS)} {name}',
                nav_summary=f'scan {name}',
            )
        except Exception:
            pass
        look, gres, arrival = _seek_look_deg(
            pan_deg, wait_hw=True, should_stop=ctrl.should_stop,
        )
        settled = bool(arrival and arrival.get('settled'))
        ctrl.update(
            message=(
                f'Step {step}/{steps_label}: photo {name} '
                f'(pan settled={settled}, hw={arrival.get("hw_pan") if arrival else "?"}°)…'
            ),
            phase_meta={
                'view': name, 'index': i,
                'total': len(_SEEK_VIEW_PANS), 'sub': 'shutter',
            },
        )
        try:
            _seek_oled_set(
                phase='triple_scan', step=step,
                activity=f'snap {name}',
                detail=f'settled={1 if settled else 0}',
                message=f'photo {name}',
                nav_summary=f'scan {name}',
            )
        except Exception:
            pass
        jpeg = _seek_grab_jpeg()
        data_url = None
        has_target = False
        det_labels = []
        chk = None
        raw_dets = []
        if jpeg:
            b64 = base64.b64encode(jpeg).decode('ascii')
            data_url = f'data:image/jpeg;base64,{b64}'
            if goal_label:
                chk = seek_goal_check(goal_label, referee=REFEREE_DETECTOR, conf_threshold=conf_threshold, jpeg=jpeg)
                if chk.get('found'):
                    has_target = True
                    if not found_check:
                        found_check = chk
                        found_check['found_view'] = name
                det_labels = chk.get('labels_found') or []
                raw_dets = list(chk.get('raw_detections') or [])

        views.append({
            'name': name,
            'pan_deg': pan_deg,
            'look': look,
            'gimbal_ok': bool(isinstance(gres, dict) and gres.get('ok', True)),
            'pan_settled': settled,
            'hw_pan': arrival.get('hw_pan') if arrival else None,
            'pan_err_deg': arrival.get('err_deg') if arrival else None,
            'pan_wait_s': arrival.get('waited_s') if arrival else None,
            'pan_wait_reason': arrival.get('reason') if arrival else None,
            'jpeg': jpeg,
            'bytes': len(jpeg) if jpeg else 0,
            'data_url': data_url,
            'has_target': has_target,
            'detected_labels': det_labels,
            'raw_detections': raw_dets,
            'check': chk,
        })
        olog.info(
            'ai_seek',
            f'Triple view {name}: target={pan_deg} settled={settled} has_target={has_target}',
            view=name, pan_deg=pan_deg, settled=settled, has_target=has_target,
        )
        # Per-view detection line for UI seek log
        det_txt = ', '.join(raw_dets) if raw_dets else (
            ', '.join(det_labels) if det_labels else 'none'
        )
        try:
            ctrl.append_log(
                'detect',
                f'Step {step} · {name} ({pan_deg}°): saw [{det_txt}]'
                + (' · GOAL MATCH' if has_target else ''),
                view=name, step=step,
            )
        except Exception:
            pass
        try:
            saw = (det_txt or 'none')[:12]
            _seek_oled_set(
                phase='triple_scan', step=step,
                activity=f'det {name}',
                detail=f'saw {saw}',
                message=('GOAL!' if has_target else f'{i}/{len(_SEEK_VIEW_PANS)} {name}'),
                nav_summary=f'scan {name}',
            )
        except Exception:
            pass
        partial_pano = _stitch_panorama_views(views)
        if partial_pano:
            b64_p = base64.b64encode(partial_pano).decode('ascii')
            ctrl.update(
                panorama_data_url=f'data:image/jpeg;base64,{b64_p}',
                last_views=[{
                    'name': v['name'], 'pan_deg': v['pan_deg'], 'bytes': v['bytes'],
                    'data_url': v.get('data_url'), 'has_target': v.get('has_target'),
                    'detected_labels': v.get('detected_labels', []),
                } for v in views]
            )
    # Re-center
    try:
        ctrl.update(
            seek_phase='triple_scan',
            message=f'Step {step}/{steps_label}: re-center pan before drive…',
            phase_meta={'view': 'center', 'sub': 'aim_wait'},
        )
        try:
            _seek_oled_set(
                phase='triple_scan', step=step,
                activity='re-center', detail='pan 0 deg',
                message=f'step {step} center', nav_summary='scan center',
            )
        except Exception:
            pass
        _seek_look_deg(0.0, wait_hw=True, should_stop=ctrl.should_stop)
    except Exception:
        pass
    return views, found_check


def _seek_should_inspect_floor(views, last_action=None, *, step=None, last_lookdown_step=None):
    """Look-down only when cruise scan says we are *close* to a wall/object/door.

    Do not fire after every forward hop — that spent minutes panning instead of driving.
    Throttle to at most once every 3 steps.
    """
    if step is not None and last_lookdown_step is not None:
        if int(step) - int(last_lookdown_step) < 3:
            return False
    blocked = _seek_centre_obstacle_hint(views)
    if blocked is True:
        return True
    cor = _seek_centre_corridor_hint(views)
    if cor.get('blocked') is True:
        return True
    sides = _seek_side_openness(views)
    lo, ro = sides.get('left'), sides.get('right')
    if lo is not None and ro is not None and lo < 0.40 and ro < 0.40:
        return True
    return False


def _seek_capture_band_views(
    ctrl, step, steps_label, *, tilt, pans, band, goal_label=None,
    conf_threshold=DEFAULT_SEEK_CONF,
):
    """L / front / R stills at a fixed tilt (look-down or look-up)."""
    views = []
    found_check = None
    tilt = float(tilt)
    n_pans = len(pans)
    verb = 'look-down' if band == 'lookdown' else 'look-up'
    for i, (name, pan_deg) in enumerate(pans, start=1):
        if ctrl.should_stop():
            break
        ctrl.update(
            seek_phase=band,
            message=(
                f'Step {step}/{steps_label}: {verb} {name} '
                f'({i}/{n_pans}, pan≈{int(pan_deg)}° tilt={int(tilt)}°)…'
            ),
            phase_meta={'view': name, 'index': i, 'total': n_pans, 'sub': band},
        )
        try:
            _seek_oled_set(
                phase=band, step=step,
                activity=f'{band[:4]} {name}',
                detail=f'{int(pan_deg)}/{int(tilt)}',
                message=f'{band[:4]} {i}/{n_pans}',
                nav_summary=f'{band} {name}',
            )
        except Exception:
            pass
        _seek_look_deg(
            pan_deg, tilt_deg=tilt, wait_hw=True, should_stop=ctrl.should_stop,
        )
        jpeg = _seek_grab_jpeg()
        data_url = None
        has_target = False
        det_labels = []
        raw_dets = []
        chk = None
        if jpeg:
            b64 = base64.b64encode(jpeg).decode('ascii')
            data_url = f'data:image/jpeg;base64,{b64}'
            if goal_label:
                chk = seek_goal_check(
                    goal_label, referee=REFEREE_DETECTOR,
                    conf_threshold=conf_threshold, jpeg=jpeg,
                )
                if chk.get('found'):
                    has_target = True
                    if not found_check:
                        found_check = dict(chk)
                        found_check['found_view'] = f'{band}_{name}'
                det_labels = chk.get('labels_found') or []
                raw_dets = list(chk.get('raw_detections') or [])
            # Always also note person (even if goal is dog) for look-up ID
            if band == 'lookup' and not det_labels:
                try:
                    pchk = seek_goal_check(
                        'person', referee=REFEREE_DETECTOR,
                        conf_threshold=conf_threshold, jpeg=jpeg,
                    )
                    det_labels = pchk.get('labels_found') or []
                    raw_dets = list(pchk.get('raw_detections') or raw_dets)
                except Exception:
                    pass
        views.append({
            'name': name,
            'pan_deg': pan_deg,
            'tilt_deg': tilt,
            'jpeg': jpeg,
            'bytes': len(jpeg) if jpeg else 0,
            'data_url': data_url,
            'has_target': has_target,
            'detected_labels': det_labels,
            'raw_detections': raw_dets,
            'check': chk,
            band: True,
        })
        det_txt = ', '.join(raw_dets) if raw_dets else (
            ', '.join(det_labels) if det_labels else 'none'
        )
        try:
            ctrl.append_log(
                'detect',
                f'Step {step} · {verb} {name} (pan {int(pan_deg)}° tilt {int(tilt)}°): '
                f'saw [{det_txt}]'
                + (' · GOAL MATCH' if has_target else ''),
                view=f'{band}_{name}', step=step,
            )
        except Exception:
            pass
    try:
        _seek_look_deg(0.0, wait_hw=True, should_stop=ctrl.should_stop)
    except Exception:
        pass
    return views, found_check


def _seek_capture_lookdown_views(ctrl, step, steps_label, goal_label=None,
                                 conf_threshold=DEFAULT_SEEK_CONF):
    """Bumper-height L / front / R stills (not the rear ±135° cruise scan)."""
    return _seek_capture_band_views(
        ctrl, step, steps_label,
        tilt=_SEEK_LOOKDOWN_TILT_DEG, pans=_SEEK_LOOKDOWN_PANS, band='lookdown',
        goal_label=goal_label, conf_threshold=conf_threshold,
    )


def _seek_capture_lookup_views(ctrl, step, steps_label, goal_label=None,
                               conf_threshold=DEFAULT_SEEK_CONF):
    """Face-height L / front / R stills when a person is nearby."""
    return _seek_capture_band_views(
        ctrl, step, steps_label,
        tilt=_SEEK_LOOKUP_TILT_DEG, pans=_SEEK_LOOKUP_PANS, band='lookup',
        goal_label=goal_label, conf_threshold=conf_threshold,
    )


def _seek_lookdown_floor_hint(views):
    """Openness at bumper height for look-down L/C/R (higher = more free floor)."""
    scores = {}
    for name in ('left', 'straight', 'right'):
        scores[name] = None
        try:
            v = next((item for item in (views or []) if item.get('name') == name), None)
            jpeg = v.get('jpeg') if isinstance(v, dict) else None
            if not jpeg:
                continue
            arr = np.frombuffer(jpeg, np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if img is None:
                continue
            h, w = img.shape[:2]
            roi = img[int(h * 0.40):int(h * 0.98), int(w * 0.12):int(w * 0.88)]
            if roi.size == 0:
                continue
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            scores[name] = _seek_roi_open_score(gray)
        except Exception:
            scores[name] = None
    left_s, mid_s, right_s = scores.get('left'), scores.get('straight'), scores.get('right')
    prefer = 'left'
    if left_s is None and right_s is None:
        prefer = 'left'
    elif left_s is None:
        prefer = 'right'
    elif right_s is None:
        prefer = 'left'
    elif right_s > left_s + 0.04:
        prefer = 'right'
    elif left_s > right_s + 0.04:
        prefer = 'left'
    # 0.38 marked almost every indoor floor as blocked and froze the chassis.
    floor_blocked = mid_s is not None and mid_s < 0.18
    return {
        'left': left_s,
        'straight': mid_s,
        'right': right_s,
        'prefer': prefer,
        'floor_blocked': floor_blocked,
        'prefer_turn': 'turn_right' if prefer == 'right' else 'turn_left',
    }


def _seek_maybe_lookdown(ctrl, step, steps_label, views, label, conf, last_action=None,
                         last_lookdown_step=None):
    """If approaching wall/object/door, sweep look-down L/C/R. Returns (found, hint)."""
    if not _seek_should_inspect_floor(
        views, last_action=last_action, step=step, last_lookdown_step=last_lookdown_step,
    ):
        return None, None
    try:
        ctrl.append_log(
            'nav',
            f'Step {step}: approaching wall/object/door — look-down L/front/R',
            step=step,
        )
    except Exception:
        pass
    ld_views, ld_found = _seek_capture_lookdown_views(
        ctrl, step, steps_label, goal_label=label, conf_threshold=conf,
    )
    hint = _seek_lookdown_floor_hint(ld_views)
    try:
        ctrl.append_log(
            'nav',
            f'Step {step}: look-down scores L={hint.get("left")} C={hint.get("straight")} '
            f'R={hint.get("right")} blocked={hint.get("floor_blocked")} '
            f'prefer={hint.get("prefer")}',
            step=step,
        )
    except Exception:
        pass
    return ld_found, hint


def _seek_views_mention_person(views):
    """True if any cruise/inspect still reported a person."""
    for v in views or []:
        if not isinstance(v, dict):
            continue
        blob = ' '.join(
            str(x).lower()
            for x in list(v.get('detected_labels') or []) + list(v.get('raw_detections') or [])
        )
        if 'person' in blob or 'people' in blob:
            return True
    return False


def _seek_lookup_person_hint(views):
    """Which look-up panel still has a person (for keeping them in frame)."""
    seen = {}
    for name in ('left', 'straight', 'right'):
        v = next((item for item in (views or []) if item.get('name') == name), None)
        seen[name] = bool(v and _seek_views_mention_person([v]))
    if seen.get('straight'):
        prefer = 'straight'
    elif seen.get('left') and not seen.get('right'):
        prefer = 'left'
    elif seen.get('right') and not seen.get('left'):
        prefer = 'right'
    elif seen.get('left'):
        prefer = 'left'
    else:
        prefer = None
    return {
        'person_left': seen.get('left'),
        'person_straight': seen.get('straight'),
        'person_right': seen.get('right'),
        'any': any(seen.values()),
        'prefer': prefer,
        'prefer_turn': (
            'turn_left' if prefer == 'left'
            else ('turn_right' if prefer == 'right' else None)
        ),
    }


def _seek_maybe_lookup(ctrl, step, steps_label, views, label, conf):
    """If a person is in the cruise scan, look up L/front/R to keep the face."""
    if not _seek_views_mention_person(views):
        return None, None
    try:
        ctrl.append_log(
            'nav',
            f'Step {step}: person nearby — look-up L/front/R (face height)',
            step=step,
        )
    except Exception:
        pass
    lu_views, lu_found = _seek_capture_lookup_views(
        ctrl, step, steps_label, goal_label=label, conf_threshold=conf,
    )
    hint = _seek_lookup_person_hint(lu_views)
    try:
        ctrl.append_log(
            'nav',
            f'Step {step}: look-up person L={hint.get("person_left")} '
            f'C={hint.get("person_straight")} R={hint.get("person_right")} '
            f'prefer={hint.get("prefer")}',
            step=step,
        )
    except Exception:
        pass
    return lu_found, hint


def _seek_parse_nav_action(raw_text):
    """Parse nav JSON into action, drive_distance, reason, path_clear_forward."""
    action = 'forward'
    drive_distance = 'medium'
    reason = ''
    path_clear = True
    parsed = None
    text = (raw_text or '').strip()
    if text.startswith('```'):
        text = re.sub(r'^```(?:json)?\s*', '', text)
        text = re.sub(r'\s*```$', '', text)
    try:
        parsed = json.loads(text)
    except Exception:
        m = re.search(r'\{[^{}]+\}', text, re.S)
        if m:
            try:
                parsed = json.loads(m.group(0))
            except Exception:
                parsed = None
    if isinstance(parsed, dict):
        action = _seek_normalize_action(
            parsed.get('action') or parsed.get('recommended_direction') or 'forward'
        )
        drive_distance = _seek_normalize_distance(
            parsed.get('drive_distance')
            or parsed.get('distance')
            or parsed.get('range')
            or 'medium'
        )
        reason = str(parsed.get('reason') or '')[:240]
        pc = parsed.get('path_clear_forward')
        if isinstance(pc, str):
            path_clear = pc.strip().lower() in ('1', 'true', 'yes')
        elif pc is not None:
            path_clear = bool(pc)
    plan = _seek_nav_plan(action, drive_distance, path_clear_forward=path_clear)
    return {
        'action': plan['action'],
        'drive_distance': plan['drive_distance'],
        'reason': reason,
        'path_clear_forward': path_clear,
        'magnitude': plan['magnitude'],
        'summary': plan['summary'],
        'repeats': plan['repeats'],
        'duration_ms': plan['duration_ms'],
        'turn_deg': plan['turn_deg'],
        'raw': (raw_text or '')[:400],
    }


def _parse_json_from_text(text):
    text = (text or '').strip()
    if text.startswith('```'):
        text = re.sub(r'^```(?:json)?\s*', '', text)
        text = re.sub(r'\s*```$', '', text)
    try:
        return json.loads(text)
    except Exception:
        m = re.search(r'\{[^{}]+\}', text, re.S)
        if m:
            try:
                return json.loads(m.group(0))
            except Exception:
                pass
    return None


def _stitch_panorama_views(views):
    """Stitch Left (-135°), Straight (0°), Right (+135°) into an annotated near-360° strip for the LLM.

    View centres span 270°; combined with camera FOV this covers essentially all around.
    views: list of view dicts [{'name': 'left'|'straight'|'right', 'jpeg': bytes, 'pan_deg': float}]
    Returns: stitched_jpeg_bytes (or None if no images)
    """
    pan = int(_SEEK_VIEW_PAN_DEG)
    frames = []
    have_any = False
    for name in ('left', 'straight', 'right'):
        v = next((item for item in views if item.get('name') == name), None)
        jpeg = v.get('jpeg') if (isinstance(v, dict) and v.get('jpeg')) else None
        img = None
        if jpeg:
            try:
                arr = np.frombuffer(jpeg, np.uint8)
                img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if img is not None:
                    have_any = True
            except Exception:
                img = None

        if img is None:
            img = np.zeros((240, 320, 3), dtype=np.uint8)
            cv2.putText(img, f'{name.upper()} (N/A)', (60, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (120, 120, 120), 1)

        # Compact tiles for faster VLM round-trips (local Ollama)
        img = cv2.resize(img, (240, 180))

        # Banners: outer panels are rear-looking (±135°), centre is front
        banner_text = (
            f'REAR-L (-{pan}°)' if name == 'left'
            else ('FRONT (0°)' if name == 'straight' else f'REAR-R (+{pan}°)')
        )
        cv2.rectangle(img, (0, 0), (240, 20), (18, 20, 26), -1)
        cv2.putText(img, banner_text, (6, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 230, 255), 1, cv2.LINE_AA)
        cv2.line(img, (239, 0), (239, 180), (50, 50, 65), 2)
        frames.append(img)

    if not have_any:
        return None

    # Concatenate horizontally into a 960x240 panorama
    panorama = cv2.hconcat(frames)

    # Encode as JPEG (keep small for local VLMs / Ollama)
    ok, buf = cv2.imencode('.jpg', panorama, [int(cv2.IMWRITE_JPEG_QUALITY), 55])
    if ok:
        return buf.tobytes()
    return None


def _seek_centre_frame(views):
    """Decode straight/centre view BGR image, or None."""
    try:
        v = next((item for item in (views or []) if item.get('name') == 'straight'), None)
        jpeg = v.get('jpeg') if isinstance(v, dict) else None
        if not jpeg:
            return None
        arr = np.frombuffer(jpeg, np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        return img
    except Exception:
        return None


def _seek_roi_open_score(gray_roi):
    """Higher = more free floor / less structure (0..1-ish)."""
    if gray_roi is None or gray_roi.size == 0:
        return None
    edges = cv2.Canny(gray_roi, 40, 120)
    edge_density = float(np.count_nonzero(edges)) / float(edges.size)
    std = float(np.std(gray_roi))
    mean = float(np.mean(gray_roi))
    open_score = 1.0 - min(1.0, edge_density * 5.0)
    # Blank near wall: low texture + low edges
    if std < 14.0 and edge_density < 0.05:
        open_score *= 0.30
    # Bright painted wall/door (few edges, moderate std) is NOT free corridor
    elif mean >= 130.0 and edge_density < 0.07 and std < 48.0:
        open_score *= 0.25
    return round(open_score, 3)


def _seek_centre_obstacle_hint(views):
    """Cheap vision hint: does CENTRE look blocked by a near obstacle/wall?

    Returns True/False/None (None = could not judge). Used to correct an optimistic LLM.
    Blank white walls/doors have few edges and were previously misread as open.
    """
    try:
        img = _seek_centre_frame(views)
        if img is None:
            return None
        h, w = img.shape[:2]
        # Near-field ROI: lower-middle of centre view (floor/obstacle zone)
        roi = img[int(h * 0.40):int(h * 0.95), int(w * 0.20):int(w * 0.80)]
        if roi.size == 0:
            return None
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 40, 120)
        edge_density = float(np.count_nonzero(edges)) / float(edges.size)
        std = float(np.std(gray))
        mean = float(np.mean(gray))
        # High structure filling lower FOV → furniture/wall close
        if edge_density >= 0.10:
            return True
        # Very flat/uniform fill often means a close blank wall/door
        if std < 16.0 and edge_density < 0.04:
            return True
        # Mid-frame wall body: bright painted surface filling the view (common indoors)
        wall = img[int(h * 0.22):int(h * 0.62), int(w * 0.18):int(w * 0.82)]
        if wall.size:
            wg = cv2.cvtColor(wall, cv2.COLOR_BGR2GRAY)
            we = cv2.Canny(wg, 40, 120)
            w_dens = float(np.count_nonzero(we)) / float(we.size)
            w_mean = float(np.mean(wg))
            w_std = float(np.std(wg))
            if w_mean >= 125.0 and w_dens < 0.075 and w_std < 50.0:
                return True
        # Bright low-edge lower FOV (carpet + close pale wall) — not free path
        if mean >= 145.0 and edge_density < 0.055 and std < 40.0:
            return True
        return False
    except Exception:
        return None


def _seek_centre_corridor_hint(views):
    """Score empty-space / corridor in the CENTRE (forward) view.

    Returns dict:
      open_score: float|None (higher = freer floor ahead)
      lane: 'left'|'right'|'center'|None — emptier half of forward FOV
      blocked: True|False|None
      hop: suggested 'short'|'medium'|'long' when going forward
    """
    out = {
        'open_score': None,
        'lane': None,
        'blocked': None,
        'hop': 'medium',
        'left_score': None,
        'right_score': None,
    }
    try:
        img = _seek_centre_frame(views)
        if img is None:
            return out
        h, w = img.shape[:2]
        # Floor band: lower 55% of frame
        y0, y1 = int(h * 0.42), int(h * 0.95)
        mid = w // 2
        # Full centre corridor band
        full = img[y0:y1, int(w * 0.18):int(w * 0.82)]
        left = img[y0:y1, int(w * 0.10):mid]
        right = img[y0:y1, mid:int(w * 0.90)]
        g_full = cv2.cvtColor(full, cv2.COLOR_BGR2GRAY) if full.size else None
        g_l = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY) if left.size else None
        g_r = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY) if right.size else None
        s_full = _seek_roi_open_score(g_full)
        s_l = _seek_roi_open_score(g_l)
        s_r = _seek_roi_open_score(g_r)
        out['open_score'] = s_full
        out['left_score'] = s_l
        out['right_score'] = s_r
        out['blocked'] = _seek_centre_obstacle_hint(views)
        # Lane bias: which half of the forward view is emptier
        if s_l is not None and s_r is not None:
            if s_l > s_r + 0.08:
                out['lane'] = 'left'
            elif s_r > s_l + 0.08:
                out['lane'] = 'right'
            else:
                out['lane'] = 'center'
        # Hop length from openness
        if s_full is None:
            out['hop'] = 'medium'
        elif s_full >= 0.72 and out['blocked'] is not True:
            out['hop'] = 'long'
        elif s_full >= 0.45 and out['blocked'] is not True:
            out['hop'] = 'medium'
        else:
            out['hop'] = 'short'
        return out
    except Exception:
        return out


# Dead-reckoning heading for exploration bias (no ROS odom required).
# 0=N, 90=E, 180=S, 270=W — updated from executed seek moves.
_SEEK_HEADING_DEG = 0.0
_SEEK_EXPLORE_TRAIL = []  # newest last: {action, dist, heading, t}
_SEEK_TRAIL_MAX = 16
_SEEK_TURN_HEADING_DELTA = {
    'short': float(_SEEK_TURN_DEG_BY_DIST.get('short', 35)),
    'medium': float(_SEEK_TURN_DEG_BY_DIST.get('medium', 80)),
    'long': float(_SEEK_TURN_DEG_BY_DIST.get('long', 130)),
}


def _seek_heading_cardinal(deg=None):
    d = float(_SEEK_HEADING_DEG if deg is None else deg) % 360.0
    dirs = ('N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW')
    return dirs[int((d + 22.5) // 45) % 8]


def _seek_explore_reset():
    global _SEEK_HEADING_DEG, _SEEK_EXPLORE_TRAIL
    _SEEK_HEADING_DEG = 0.0
    _SEEK_EXPLORE_TRAIL = []


def _seek_explore_record(action, dist='medium'):
    """Update heading + trail after a move executes."""
    global _SEEK_HEADING_DEG, _SEEK_EXPLORE_TRAIL
    action = _seek_normalize_action(action)
    dist = _seek_normalize_distance(dist, default='medium')
    h = float(_SEEK_HEADING_DEG)
    if action == 'turn_left':
        h = (h - float(_SEEK_TURN_HEADING_DELTA.get(dist, 20.0))) % 360.0
    elif action == 'turn_right':
        h = (h + float(_SEEK_TURN_HEADING_DELTA.get(dist, 20.0))) % 360.0
    # forward/backward keep heading (reverse doesn't flip map heading for trail bias)
    _SEEK_HEADING_DEG = h
    _SEEK_EXPLORE_TRAIL.append({
        'action': action,
        'dist': dist,
        'heading': round(h, 1),
        'cardinal': _seek_heading_cardinal(h),
        't': time.time(),
    })
    if len(_SEEK_EXPLORE_TRAIL) > _SEEK_TRAIL_MAX:
        _SEEK_EXPLORE_TRAIL = _SEEK_EXPLORE_TRAIL[-_SEEK_TRAIL_MAX:]
    return h


def _seek_explore_summary():
    """Text + stats for LLM / heuristic exploration bias."""
    trail = list(_SEEK_EXPLORE_TRAIL)
    heading = float(_SEEK_HEADING_DEG)
    card = _seek_heading_cardinal(heading)
    # Count recent headings used while going forward
    recent = trail[-8:]
    fwd_cards = [
        e.get('cardinal') for e in recent
        if e.get('action') == 'forward' and e.get('cardinal')
    ]
    from collections import Counter
    counts = Counter(fwd_cards)
    # How often current heading was used for forward
    same_heading_fwd = counts.get(card, 0)
    # Suggest turn away from overused forward headings
    overused = [c for c, n in counts.most_common() if n >= 2]
    parts = []
    for e in recent:
        parts.append(f"{e.get('action')}/{e.get('dist')}@{e.get('cardinal')}")
    trail_s = ' → '.join(parts) if parts else '(empty — start exploring)'
    return {
        'heading_deg': round(heading, 1),
        'cardinal': card,
        'trail_text': trail_s,
        'forward_counts': dict(counts),
        'same_heading_fwd': same_heading_fwd,
        'overused': overused,
        'prefer_new': same_heading_fwd >= 2,
    }


def _seek_unified_llm_analysis(
    current_views, prev_forward_jpeg, goal_label, last_action=None,
    lookdown=None, lookup=None,
):
    """Single unified LLM query: near-360° stitch + previous view.

    last_action: previous executed move so recovery can turn toward open space
    after a reverse instead of reversing forever.
    """
    pan = int(_SEEK_VIEW_PAN_DEG)
    last = _seek_normalize_action(last_action) if last_action else None
    side_info = _seek_side_openness(current_views)
    prefer_turn = _seek_prefer_open_turn(current_views)
    prefer_side = 'right' if prefer_turn == 'turn_right' else 'left'
    rear = _seek_rear_clearance(current_views)
    explore = _seek_explore_summary()

    stitched_jpeg = _stitch_panorama_views(current_views)
    if not stitched_jpeg:
        return {
            'goal_found': False,
            'goal_found_view': None,
            'path_forward_clear': False,
            'obstacle_ahead_range': 'near',
            'is_identical_to_previous': False,
            'recommended_direction': 'left' if prefer_turn == 'turn_left' else 'right',
            'drive_distance': 'short',
            'open_side': prefer_side,
            'prefer_turn': prefer_turn,
            'reason': 'no camera images — turn short toward open (avoid reverse into wall)',
            'explore': explore,
        }

    last_note = ''
    if last == 'backward':
        last_note = (
            f'\nLAST MOVE: the robot just REVERSED. Do NOT reverse again. '
            f'Turn toward open side (hint: {prefer_side}) then try forward when '
            f'CENTRE looks clear.\n'
        )
    elif last in ('turn_left', 'turn_right'):
        last_note = (
            f'\nLAST MOVE: just turned ({last}). If CENTRE (front) is open, prefer '
            f'forward medium/long into NEW space. Do not reverse just because LEFT/RIGHT show walls.\n'
        )
    elif last == 'forward':
        last_note = '\nLAST MOVE: forward hop.\n'

    explore_note = (
        f'\nEXPLORATION (visit places not yet seen):\n'
        f'  Heading now ≈ {explore["cardinal"]} ({explore["heading_deg"]}°).\n'
        f'  Recent trail (old→new): {explore["trail_text"]}\n'
        f'  Forward counts by heading: {explore["forward_counts"] or "{}"}.\n'
        f'  Prefer open corridors facing headings NOT already used a lot.\n'
        f'  If CENTRE is open but heading {explore["cardinal"]} was used ≥2 times recently, '
        f'turn short toward the emptier unexplored side, then forward long.\n'
        f'  Goal of seek: cover more of the environment, not only re-drive the same open path.\n'
    )
    last_note = last_note + explore_note

    openness_hint = ''
    if side_info.get('scores_ok'):
        openness_hint = (
            f'\nLocal side scores (for turning only; higher=more free): '
            f'LEFT/rear-left={side_info.get("left")} '
            f'RIGHT/rear-right={side_info.get("right")} '
            f'→ prefer turn {prefer_side}.\n'
        )
    rear_note = (
        f'\nREAR CLEARANCE (for reverse only):\n'
        f'  LEFT photo (−135°) left-half score={rear.get("left_half")} '
        f'clear={rear.get("left_clear")}.\n'
        f'  RIGHT photo (+135°) right-half score={rear.get("right_half")} '
        f'clear={rear.get("right_clear")}.\n'
        f'  both_clear={rear.get("all_clear")} — reverse is ILLEGAL unless both_clear '
        f'and you cannot go forward or turn.\n'
    )
    openness_hint = (openness_hint or '') + rear_note
    lookdown_hint = ''
    if lookdown:
        lookdown_hint = (
            f'\nLOOK-DOWN bumper inspect (L/front/R at tilt≈−22°):\n'
            f'  scores L={lookdown.get("left")} C={lookdown.get("straight")} '
            f'R={lookdown.get("right")} floor_blocked={lookdown.get("floor_blocked")} '
            f'prefer={lookdown.get("prefer")}.\n'
            f'  If floor_blocked, CENTRE is NEAR even if the high panorama looked open '
            f'(bowls, cables, jambs, baseboards).\n'
        )
    if lookup:
        lookdown_hint += (
            f'\nLOOK-UP face inspect (L/front/R at tilt≈+18°): '
            f'person L={lookup.get("person_left")} C={lookup.get("person_straight")} '
            f'R={lookup.get("person_right")} prefer={lookup.get("prefer")}.\n'
            f'  A human is nearby — keep the camera UP on them; do not stare at their feet. '
            f'If the goal is a person and they are in a look-up panel, that is found.\n'
        )

    user_content = [
        {
            'type': 'text',
            'text': (
                f'Target object to seek: "{goal_label}".\n\n'
                '=== HOW TO READ THIS IMAGE (CRITICAL) ===\n'
                f'This is ONE stitched near-360° panorama of THREE photos the robot took '
                f'by panning its camera. Layout left→right:\n'
                f'  [ LEFT panel | CENTRE panel | RIGHT panel ]\n'
                f'  LEFT   = camera aimed {pan}° LEFT of drive direction '
                f'(≈ rear-left / over the left shoulder — NOT the front).\n'
                f'  CENTRE = camera aimed STRAIGHT AHEAD (0°) = the ONLY view of '
                f'where driving FORWARD goes.\n'
                f'  RIGHT  = camera aimed {pan}° RIGHT of drive direction '
                f'(≈ rear-right / over the right shoulder — NOT the front).\n'
                'Together LEFT+CENTRE+RIGHT cover roughly all around the robot. '
                'The outer edges of the strip are the REAR half of the world; '
                'the middle strip is the FRONT.\n\n'
                'Motion mapping:\n'
                '  forward  → into what you see in CENTRE only\n'
                '  backward → into the REAR (content that appears in LEFT/RIGHT edges, '
                'behind the robot). If LEFT/RIGHT show close walls, do NOT reverse.\n'
                '  left/right turns → rotate toward freer space (use LEFT vs RIGHT panels)\n\n'
                f'{last_note}{openness_hint}{lookdown_hint}\n'
                'YOUR NAV CONTRACT (the robot will execute this, not a heuristic):\n'
                '  • If CENTRE is clear enough to drive: recommended_direction="forward" and '
                'drive_distance is HOW FAR (short≈0.8s, medium≈1.1s, long≈1.6s punchy hops).\n'
                '    Use long for open halls/rooms; medium for mixed; short only if something is close.\n'
                '  • If CENTRE is blocked (wall, door slab, furniture, bowl): '
                'recommended_direction="left" or "right" — whichever side is more open. '
                'Set open_side to that same side. Do not pick forward.\n'
                '  • backward ONLY if you cannot go forward AND cannot usefully turn, AND '
                'BOTH rear quarters are clear: LEFT half of the LEFT (−135°) photo AND '
                'RIGHT half of the RIGHT (+135°) photo. If either half is cluttered, turn instead.\n'
                'Answer in ONE JSON object.\n\n'
                'obstacle_ahead_range (CENTRE panel ONLY — ignore left/right for this field):\n'
                '  none   = open floor ahead in CENTRE\n'
                '  far    = something distant in CENTRE\n'
                '  medium = mid-range in CENTRE\n'
                '  near   = immediately blocking the CENTRE / filling the front view\n'
                '  NEVER set near just because LEFT or RIGHT show furniture/walls behind.\n\n'
                'HARD RULES:\n'
                '  1. path_forward_clear and obstacle_ahead_range use CENTRE only.\n'
                '  2. Obstacles only on LEFT/RIGHT = behind/side → do NOT reverse; '
                'turn or go forward if CENTRE is clear.\n'
                '  3. Prefer turn toward open_side when CENTRE is near — do NOT reverse '
                'into a rear wall. Reverse only as a last short nudge after a turn failed.\n'
                '  4. If reverse is used: drive_distance must be short only; never reverse twice.\n'
                '  5. CENTRE clear (none/far) or open corridor / empty floor → prefer '
                'forward medium or long (navigate into the empty space; do not creep short).\n'
                '  6. If the empty corridor is offset left/right in the CENTRE panel, '
                'turn short toward that empty lane first, then forward long next.\n'
                '  7. medium range in CENTRE → forward medium (not reverse) unless blocked near.\n'
                '  8. Explore: when several opens exist, pick the direction least used in the trail.\n'
                '  9. never omit drive_distance, obstacle_ahead_range, open_side.\n'
                ' 10. Drive AWAY from walls. A pale wall filling CENTRE is NEAR — turn to open_side, '
                'do not creep toward the paint.\n'
                ' 11. Doorway / hall chute (CENTRE open, LEFT+RIGHT look like jambs/walls): after a forward '
                'into the frame, take another hop of similar length so the chassis is fully PAST the jambs. '
                'Stopping in the doorway wedges this rover.\n'
                ' 12. Floor hazards: bowls, cable runners, and baseboards are at bumper height. '
                'If CENTRE lower frame shows an object on the floor, treat as near and turn around it.\n'
                '  13. On approach the robot also takes a LOOK-DOWN left/front/right (~±55°, tilt −22°). '
                'If that inspect says floor_blocked, do not go forward.\n'
                '  14. If a PERSON is nearby, a LOOK-UP left/front/right (~±55°, tilt +18°) keeps the face '
                'in frame. Identify them from that band; do not look down at their feet.\n\n'
                'JSON schema:\n'
                '{\n'
                '  "goal_found": true|false,\n'
                '  "goal_found_view": "straight"|"left"|"right"|null,\n'
                '  "obstacle_ahead_range": "none"|"far"|"medium"|"near",\n'
                '  "open_side": "left"|"right"|"neither",\n'
                '  "path_forward_clear": true|false,\n'
                '  "is_identical_to_previous": true|false,\n'
                '  "recommended_direction": "forward"|"left"|"right"|"backward",\n'
                '  "drive_distance": "short"|"medium"|"long",\n'
                '  "reason": "cite CENTRE vs rear panels + why this move"\n'
                '}'
            ),
        }
    ]

    if prev_forward_jpeg:
        b64_prev = base64.b64encode(prev_forward_jpeg).decode('ascii')
        user_content.append({
            'type': 'text',
            'text': (
                'PREVIOUS CENTRE / FORWARD VIEW only (0° drive direction — not the rear):'
            ),
        })
        user_content.append({'type': 'image_url', 'image_url': {'url': f'data:image/jpeg;base64,{b64_prev}'}})

    b64_pano = base64.b64encode(stitched_jpeg).decode('ascii')
    user_content.append({
        'type': 'text',
        'text': (
            f'CURRENT NEAR-360° PANORAMA — call seek_nav_answer:\n'
            f'  LEFT third  = camera −{pan}° REAR-LEFT. rear_left_clear = LEFT HALF of this panel.\n'
            f'  MIDDLE third = camera 0° FRONT. forward_clear_cm / can_forward / forward_hop from this only.\n'
            f'  RIGHT third = camera +{pan}° REAR-RIGHT. rear_right_clear = RIGHT HALF of this panel.\n'
            f'can_backward only if both rear halves are clear AND we cannot go forward or turn.'
        ),
    })
    user_content.append({'type': 'image_url', 'image_url': {'url': f'data:image/jpeg;base64,{b64_pano}'}})

    messages = [
        {
            'role': 'system',
            'content': (
                'You are the sole navigator for a small UGV. '
                'You MUST call the function seek_nav_answer. Do not write prose. '
                'Each field is a one-token / enum answer. '
                'CENTRE panel = path ahead (cm until collision, can_forward, forward_hop). '
                'LEFT photo is camera −135° rear-left — judge rear_left_clear from its LEFT HALF. '
                'RIGHT photo is camera +135° rear-right — judge rear_right_clear from its RIGHT HALF. '
                'can_backward only if cannot go forward, cannot turn, and BOTH rear halves are clear.'
            ),
        },
        {'role': 'user', 'content': user_content},
    ]

    centre_blocked_hint = _seek_centre_obstacle_hint(current_views)

    try:
        try:
            msg, _body, _cfg = _openai_chat(
                messages,
                max_tokens=1024,
                temperature=0.0,
                timeout=20,
                tools=[_SEEK_NAV_TOOL],
                tool_choice={'type': 'function', 'function': {'name': _SEEK_NAV_TOOL_NAME}},
            )
        except Exception as e:
            olog.warn('ai_seek', f'seek_nav tool_choice=required failed, retry auto: {e}')
            msg, _body, _cfg = _openai_chat(
                messages,
                max_tokens=1024,
                temperature=0.0,
                timeout=20,
                tools=[_SEEK_NAV_TOOL],
                tool_choice='auto',
            )
        parsed = _parse_tool_call_args(msg, _SEEK_NAV_TOOL_NAME)
        if not isinstance(parsed, dict):
            txt = _message_text_content(msg) or ''
            parsed = _parse_json_from_text(txt)
        if not isinstance(parsed, dict):
            # 96-token-era failure mode: reasoning ate the tool JSON
            olog.warn('ai_seek', 'seek_nav parse empty — retry larger completion')
            msg, _body, _cfg = _openai_chat(
                messages,
                max_tokens=2048,
                temperature=0.0,
                timeout=25,
                tools=[_SEEK_NAV_TOOL],
                tool_choice='auto',
            )
            parsed = _parse_tool_call_args(msg, _SEEK_NAV_TOOL_NAME)
            if not isinstance(parsed, dict):
                parsed = _parse_json_from_text(_message_text_content(msg) or '')
        if isinstance(parsed, dict) and (
            'can_forward' in parsed
            or 'forward_clear_cm' in parsed
            or parsed.get('recommended_direction')
            or parsed.get('action')
            or parsed.get('drive_distance')
            or parsed.get('open_side')
        ):
            prefer_turn_llm = prefer_turn
            open_side = prefer_side
            fwd_clear = False
            obs_range = 'unknown'
            found = bool(parsed.get('goal_found'))
            found_view = str(
                parsed.get('goal_view') or parsed.get('goal_found_view') or 'straight'
            ).lower().strip()
            if found_view in ('', 'null', 'none'):
                found_view = 'straight' if found else None

            identical = bool(parsed.get('is_identical_to_previous'))
            if 'can_forward' in parsed or 'forward_clear_cm' in parsed:
                schema_move = _seek_action_from_schema(parsed, last_action=last)
                if schema_move.get('action') == 'backward' and not rear.get('all_clear'):
                    schema_move = _seek_action_from_schema(
                        {**parsed, 'can_backward': False, 'backward_hop': 'none',
                         'can_turn_left': parsed.get('can_turn_left', True)},
                        last_action=last,
                    )
                direction = schema_move['action']
                dist = schema_move['drive_distance']
                if schema_move.get('open_side') in ('left', 'right'):
                    open_side = schema_move['open_side']
                    prefer_turn_llm = 'turn_' + open_side
                fwd_clear = bool(schema_move.get('can_forward'))
                cm = schema_move.get('forward_clear_cm')
                if cm is None:
                    obs_range = 'unknown'
                elif cm <= 15:
                    obs_range = 'near'
                elif cm <= 60:
                    obs_range = 'medium'
                elif cm <= 120:
                    obs_range = 'far'
                else:
                    obs_range = 'none'
            else:
                obs_range = _seek_normalize_obstacle_range(
                    parsed.get('obstacle_ahead_range')
                    or parsed.get('obstacle_range')
                    or parsed.get('forward_obstacle_range')
                    or parsed.get('clearance')
                    or parsed.get('how_close')
                )
                open_raw = str(parsed.get('open_side') or parsed.get('preferred_side') or '').strip().lower()
                if open_raw in ('left', 'l'):
                    open_side = 'left'
                    prefer_turn_llm = 'turn_left'
                elif open_raw in ('right', 'r'):
                    open_side = 'right'
                    prefer_turn_llm = 'turn_right'
                else:
                    open_side = prefer_side
                    prefer_turn_llm = prefer_turn
                fwd_clear = parsed.get('path_forward_clear')
                if isinstance(fwd_clear, str):
                    fwd_clear = fwd_clear.lower() in ('true', '1', 'yes')
                elif fwd_clear is None:
                    fwd_clear = obs_range in ('none', 'far', 'medium')
                else:
                    fwd_clear = bool(fwd_clear)
                direction = _seek_normalize_action(
                    parsed.get('recommended_direction')
                    or parsed.get('action')
                    or ('forward' if fwd_clear else prefer_turn_llm)
                )
                if direction == 'backward' and not rear.get('all_clear'):
                    direction = prefer_turn_llm if prefer_turn_llm in ('turn_left', 'turn_right') else 'turn_left'
                dist = _seek_normalize_distance(
                    parsed.get('drive_distance')
                    or parsed.get('distance')
                    or parsed.get('range')
                    or ('medium' if direction == 'forward' else 'short')
                )
            # Tables only — do not rewrite the LLM's hop length / turn side via obstacle heuristics
            plan = _seek_nav_plan(direction, dist)
            reason = str(parsed.get('reason') or '')[:200]
            cm = parsed.get('forward_clear_cm')
            range_note = f'obstacle={obs_range} open={open_side} cm={cm}'
            reason = f'LLM {plan["summary"]} ({range_note})' + (
                f' | {reason}' if reason else ''
            )

            return {
                'goal_found': found,
                'goal_found_view': found_view if found else None,
                'path_forward_clear': bool(fwd_clear),
                'obstacle_ahead_range': obs_range,
                'open_side': open_side,
                'prefer_turn': prefer_turn_llm,
                'side_openness': side_info,
                'is_identical_to_previous': identical,
                'recommended_direction': plan['action'],
                'drive_distance': plan['drive_distance'],
                'action': plan['action'],
                'magnitude': plan['magnitude'],
                'summary': plan['summary'],
                'repeats': plan['repeats'],
                'duration_ms': plan['duration_ms'],
                'turn_deg': plan['turn_deg'],
                'safety_override': None,
                'vision_blocked_hint': centre_blocked_hint,
                'source': 'llm',
                'reason': reason[:240],
            }
    except Exception as e:
        olog.warn('ai_seek', f'Unified LLM analysis error: {e}')

    # LLM failed/slow: local vision heuristic that can still exit a room
    return _seek_heuristic_room_nav(
        current_views, last_action=last, last_dist=None, prefer_turn=prefer_turn,
        prefer_side=prefer_side, side_info=side_info, reason_prefix='LLM failed — ',
        lookdown=lookdown,
    )


def _seek_heuristic_room_nav(
    views,
    *,
    last_action=None,
    last_dist=None,
    prefer_turn=None,
    prefer_side=None,
    side_info=None,
    reason_prefix='',
    lookdown=None,
):
    """Navigate without LLM: reverse once if blocked, aim into empty corridor, drive longer.

    Goal: leave clutter toward free hallway space — longer hops when the centre is open.
    """
    side_info = side_info or _seek_side_openness(views)
    prefer_turn = prefer_turn or _seek_prefer_open_turn(views)
    if lookdown and lookdown.get('prefer_turn') in ('turn_left', 'turn_right'):
        prefer_turn = lookdown['prefer_turn']
    prefer_side = prefer_side or ('right' if prefer_turn == 'turn_right' else 'left')
    if lookdown and lookdown.get('prefer') in ('left', 'right'):
        prefer_side = lookdown['prefer']
    last = _seek_normalize_action(last_action) if last_action else None
    last_dist = _seek_normalize_distance(last_dist, default='medium') if last_dist else None
    corridor = _seek_centre_corridor_hint(views)
    blocked = corridor.get('blocked')
    if blocked is None:
        blocked = _seek_centre_obstacle_hint(views)
    if lookdown and lookdown.get('floor_blocked'):
        blocked = True
    hop = corridor.get('hop') or 'medium'
    lane = corridor.get('lane')
    open_score = corridor.get('open_score')

    if blocked is True:
        if last == 'backward':
            action, dist, obs = prefer_turn, 'short', 'near'
            reason = f'heuristic: after reverse, turn {prefer_side} toward open space'
        elif last in ('turn_left', 'turn_right'):
            rear = _seek_rear_clearance(views)
            if _seek_may_reverse(
                can_forward=False,
                can_turn=False,
                rear_left_clear=bool(rear.get('left_clear')),
                rear_right_clear=bool(rear.get('right_clear')),
                last_action=last,
            ):
                action, dist, obs = 'backward', 'short', 'near'
                reason = (
                    f'heuristic: still blocked after {last}; rear L/R both clear '
                    f'(Lhalf={rear.get("left_half")} Rhalf={rear.get("right_half")}) → reverse short'
                )
            else:
                other = 'turn_right' if last == 'turn_left' else 'turn_left'
                action, dist, obs = other, 'short', 'near'
                reason = (
                    f'heuristic: still blocked after {last}; rear not both-clear '
                    f'(Lhalf={rear.get("left_half")} clear={rear.get("left_clear")} '
                    f'Rhalf={rear.get("right_half")} clear={rear.get("right_clear")}) '
                    f'→ {other} instead of reverse'
                )
        else:
            # Prefer turn-to-open; reverse often hits rear wall in rooms/corridors
            action, dist, obs = prefer_turn, 'short', 'near'
            reason = f'heuristic: centre near-blocked → turn {prefer_side} (not reverse)'
    elif blocked is False:
        # Open floor / corridor — aim into emptier lane, then drive longer
        explore = _seek_explore_summary()
        if explore.get('prefer_new') and last == 'forward':
            # Same heading overused — peel off toward open side for new space
            action, dist, obs = prefer_turn, 'short', 'none'
            reason = (
                f'heuristic: explore leave {explore.get("cardinal")} '
                f'(fwd×{explore.get("same_heading_fwd")}) → turn {prefer_side}'
            )
        elif last in ('turn_left', 'turn_right'):
            # Just aimed: commit down the open path
            action, dist, obs = 'forward', hop if hop in ('medium', 'long') else 'long', 'none'
            if dist == 'short':
                dist = 'medium'
            reason = (
                f'heuristic: after turn, centre open (score={open_score}) '
                f'→ forward {dist} into corridor'
            )
        elif lane in ('left', 'right') and last not in ('turn_left', 'turn_right'):
            # Empty space is off-center — small turn to face the free lane first
            action = 'turn_left' if lane == 'left' else 'turn_right'
            dist, obs = 'short', 'none'
            reason = (
                f'heuristic: empty corridor lane={lane} (L={corridor.get("left_score")} '
                f'R={corridor.get("right_score")}) → aim short then go long'
            )
        else:
            action, dist, obs = 'forward', hop if hop != 'short' else 'medium', 'none'
            if open_score is not None and open_score >= 0.65:
                dist = 'long'
            reason = (
                f'heuristic: centre open corridor (score={open_score}) '
                f'→ forward {dist} into empty space hdg={explore.get("cardinal")}'
            )
    else:
        # Unknown: prefer progressing into likely free space rather than spinning short
        if last in ('turn_left', 'turn_right', 'backward'):
            action, dist, obs = 'forward', 'medium', 'unknown'
            reason = 'heuristic: after recovery, probe forward medium into space'
        elif lane in ('left', 'right'):
            action = 'turn_left' if lane == 'left' else 'turn_right'
            dist, obs = 'short', 'unknown'
            reason = f'heuristic: unclear but emptier lane={lane} → aim short'
        else:
            action, dist, obs = 'forward', 'medium', 'unknown'
            reason = 'heuristic: unclear centre → forward medium (corridor progress)'

    # Doorway/hall chute: don't stop in the frame (live: wedges on jambs).
    commit = _seek_commit_through_opening(
        last, last_dist or 'medium',
        obstacle_range='near' if blocked is True else ('none' if blocked is False else 'unknown'),
        left_open=side_info.get('left'),
        right_open=side_info.get('right'),
        centre_open=(blocked is False),
    )
    if commit:
        action, dist = commit['action'], commit['drive_distance']
        obs = 'none' if blocked is False else 'unknown'
        reason = commit['reason']

    away = _seek_prefer_away_from_wall(
        action,
        obstacle_range='near' if blocked is True else 'unknown',
        left_open=side_info.get('left'),
        right_open=side_info.get('right'),
        prefer_turn=prefer_turn,
    )
    if away:
        action, dist, obs = away, 'short', 'near'
        reason = f'heuristic: wall ahead — turn {away} away (do not drive into paint)'

    plan = _seek_nav_plan(
        action, dist,
        path_clear_forward=(obs in ('none', 'far')),
        obstacle_range=obs,
        last_action=last,
        prefer_turn=prefer_turn,
    )
    dir_token = {
        'forward': 'forward',
        'turn_left': 'left',
        'turn_right': 'right',
        'backward': 'backward',
    }.get(plan['action'], 'forward')

    return {
        'goal_found': False,
        'goal_found_view': None,
        'path_forward_clear': obs in ('none', 'far'),
        'obstacle_ahead_range': plan.get('obstacle_range') or obs,
        'open_side': prefer_side,
        'prefer_turn': prefer_turn,
        'side_openness': side_info,
        'is_identical_to_previous': False,
        'recommended_direction': dir_token,
        'drive_distance': plan['drive_distance'],
        'action': plan['action'],
        'magnitude': plan['magnitude'],
        'summary': plan['summary'],
        'repeats': plan['repeats'],
        'duration_ms': plan['duration_ms'],
        'turn_deg': plan['turn_deg'],
        'safety_override': plan.get('safety_override'),
        'source': 'heuristic',
        'reason': (reason_prefix + reason + f' | open={prefer_side}')[:240],
    }


def _seek_nav_decide(views, goal_label, labels_hint=None):
    """Fallback nav when LLM scene nav is off: always returns action + short|medium|long."""
    labels_hint = labels_hint or []
    for v in views or []:
        if v.get('has_target'):
            name = (v.get('name') or '').lower()
            if name == 'left':
                plan = _seek_nav_plan('turn_left', 'short')
                plan['reason'] = f'detector saw goal on left → {plan["summary"]}; labels={labels_hint}'
                plan['source'] = 'detector_heuristic'
                return plan
            if name == 'right':
                plan = _seek_nav_plan('turn_right', 'short')
                plan['reason'] = f'detector saw goal on right → {plan["summary"]}; labels={labels_hint}'
                plan['source'] = 'detector_heuristic'
                return plan
            plan = _seek_nav_plan('forward', 'short')
            plan['reason'] = f'detector saw goal ahead → {plan["summary"]}; labels={labels_hint}'
            plan['source'] = 'detector_heuristic'
            return plan
    # No goal yet: bias into open corridor space (longer hop when centre free)
    corridor = _seek_centre_corridor_hint(views)
    if corridor.get('blocked') is True:
        prefer = _seek_prefer_open_turn(views)
        plan = _seek_nav_plan(
            prefer, 'short',
            obstacle_range='near', path_clear_forward=False,
            prefer_turn=prefer,
        )
        plan['reason'] = (
            f'no goal + centre blocked → {plan["summary"]} (saw {labels_hint or "nothing"})'
        )
    elif corridor.get('lane') in ('left', 'right'):
        act = 'turn_left' if corridor['lane'] == 'left' else 'turn_right'
        plan = _seek_nav_plan(act, 'short', obstacle_range='none', path_clear_forward=True)
        plan['reason'] = (
            f'no goal + empty lane={corridor["lane"]} → {plan["summary"]} '
            f'(saw {labels_hint or "nothing"})'
        )
    else:
        hop = corridor.get('hop') or 'long'
        if hop == 'short':
            hop = 'medium'
        plan = _seek_nav_plan(
            'forward', hop,
            obstacle_range='none' if corridor.get('blocked') is False else 'unknown',
            path_clear_forward=corridor.get('blocked') is not True,
        )
        plan['reason'] = (
            f'no goal in L/C/R → {plan["summary"]} corridor_score={corridor.get("open_score")} '
            f'(saw {labels_hint or "nothing"})'
        )
    plan['source'] = 'detector_heuristic'
    return plan


def _seek_fast_tank_turn(direction, duration_ms, should_stop=None):
    """In-place tank turn at UI Fast speed (max_speed × max_rate on T:1).

    Matches Raw-mode left/right at Fast. Soft T:13 arcs (angular≈0.4–0.7) do
    not yaw this rover — live 2026-08-13: 350ms nudge, 700ms room turn, 1100ms large.
    """
    args_cfg = f.get('args_config') or {}
    max_speed = float(args_cfg.get('max_speed', 1.3) or 1.3)
    max_rate = float(args_cfg.get('max_rate', 1.0) or 1.0)
    scale = float(_SEEK_TURN_SPEED_SCALE or 1.0)
    spd = abs(max_speed) * abs(max_rate) * max(0.5, min(1.0, scale))
    # Same body L/R convention as control.js left/right buttons
    if direction in ('left', 'turn_left'):
        body_L, body_R = -spd, +spd
        side = 'left'
    else:
        body_L, body_R = +spd, -spd
        side = 'right'
    hw_L, hw_R = body_to_hw_diff(body_L, body_R)
    dur = max(200, int(duration_ms or 600))

    try:
        _cancel_ai_drive_timer()
    except Exception:
        pass
    _arm_ai_motion_lock(duration_ms=dur, continuous=False)
    try:
        base.base_json_ctrl({'T': 1, 'L': hw_L, 'R': hw_R})
        olog.info(
            'ai_seek',
            f'Fast tank turn {side} L={hw_L:.2f} R={hw_R:.2f} {dur}ms (UI fast scale)',
            side=side, L=hw_L, R=hw_R, duration_ms=dur, speed=spd,
        )
    except Exception as e:
        olog.warn('ai_seek', f'Fast tank turn cmd failed: {e}', error=str(e)[:160])
        return {
            'ok': False, 'error': str(e)[:200],
            'side': side, 'duration_ms': dur, 'path': 't1_fast',
        }

    t0 = time.time()
    while (time.time() - t0) < (dur / 1000.0):
        if should_stop and should_stop():
            break
        time.sleep(0.04)
    try:
        base.base_json_ctrl({'T': 1, 'L': 0, 'R': 0})
        base.base_json_ctrl({'T': 13, 'X': 0, 'Z': 0})
    except Exception:
        pass
    return {
        'ok': True,
        'side': side,
        'body_L': body_L,
        'body_R': body_R,
        'hw_L': hw_L,
        'hw_R': hw_R,
        'duration_ms': dur,
        'speed': spd,
        'path': 't1_fast',
        'speed_ctrl': 'fast',
    }


def _seek_aim_for_motion(action, open_side=None, should_stop=None):
    """Point camera for the upcoming chassis move.

    - forward / turn_left / turn_right → face drive direction (0°)
    - backward → look rearward (±_SEEK_VIEW_PAN_DEG, prefer open_side)
      so reverse is watched on the rear panels, not with the camera stuck
      sideways after a scan.
    """
    action = _seek_normalize_action(action)
    pan_rear = float(_SEEK_VIEW_PAN_DEG)  # 135°
    if action == 'backward':
        side = (open_side or 'left').strip().lower()
        # Rear-left / rear-right of the 360 stitch
        pan = -pan_rear if side in ('left', 'l') else pan_rear
        look_label = f'rear {side} ({int(pan)}d)'
    else:
        # Driving or turning: always look where the body is going (front)
        pan = 0.0
        look_label = 'front (0d)'

    try:
        _seek_oled_set(activity='CENTER' if pan == 0.0 else 'PAN', phase='drive',
                       message=look_label, detail=look_label)
    except Exception:
        pass

    arrival = None
    try:
        _look, _gres, arrival = _seek_look_deg(
            pan, wait_hw=True, should_stop=should_stop,
        )
    except Exception:
        try:
            _seek_look_deg(pan, wait_hw=False, settle_s=0.45)
            arrival = {'settled': False, 'reason': 'exception_fallback'}
        except Exception:
            arrival = {'settled': False, 'reason': 'aim_failed'}

    settled = bool(arrival and arrival.get('settled'))
    # If HW never confirmed centre, still give a short settle before driving
    if pan == 0.0 and not settled:
        time.sleep(0.35)
    try:
        olog.info(
            'ai_seek',
            f'Aim for {action}: pan={pan} settled={settled} ({look_label})',
            action=action, pan_deg=pan, settled=settled, open_side=open_side,
        )
    except Exception:
        pass
    return {'pan_deg': pan, 'settled': settled, 'label': look_label, 'arrival': arrival}


def _seek_execute_nav_action(action, drive_distance='medium', should_stop=None, open_side=None):
    """Execute body move from a normalized plan:
    - turn_left/right: T:1 UI-Fast tank spin (soft T:13 yaw does not turn this chassis)
    - forward/backward: punchy timed hop (cable runner needs ~0.26, not 0.12)

    Camera: forward+turns look 0°; reverse looks ±135° (rear).
    """
    plan = _seek_nav_plan(action, drive_distance)
    action = plan['action']
    dist = plan['drive_distance']
    aim = _seek_aim_for_motion(action, open_side=open_side, should_stop=should_stop)
    last_res = None
    last_args = {}

    # Aim already published via cam_aim inside _seek_aim_for_motion / _seek_look_deg.
    # Keep a short cam_look string only for log lines — not a second pan SoT.
    cam_look = aim.get('label')

    if action == 'turn_left':
        dur = int(plan['duration_ms'])
        last_res = _seek_fast_tank_turn('left', dur, should_stop=should_stop)
        last_args = {
            'action': 'turn_left',
            'drive_distance': dist,
            'duration_ms': dur,
            'turn_deg': plan['turn_deg'],
            'speed_ctrl': 'fast',
            'path': 't1_fast',
        }

    elif action == 'turn_right':
        dur = int(plan['duration_ms'])
        last_res = _seek_fast_tank_turn('right', dur, should_stop=should_stop)
        last_args = {
            'action': 'turn_right',
            'drive_distance': dist,
            'duration_ms': dur,
            'turn_deg': plan['turn_deg'],
            'speed_ctrl': 'fast',
            'path': 't1_fast',
        }

    elif action == 'backward':
        dur = plan['duration_ms']
        lin = plan['linear_x']  # already negative
        last_args = {
            'linear_x': lin, 'angular_z': 0.0, 'duration_ms': dur,
            'drive_distance': dist,
        }
        last_res = _execute_agent_tool('send_motor_command', last_args)
        time.sleep(float(dur) / 1000.0 + 0.3)
        # After reverse, return camera to front for the next step
        try:
            _seek_look_deg(0.0, wait_hw=True, should_stop=should_stop)
        except Exception:
            try:
                _seek_look_deg(0.0, wait_hw=False, settle_s=0.3)
            except Exception:
                pass

    else:  # forward
        dur = plan['duration_ms']
        lin = plan['linear_x']
        last_args = {
            'linear_x': lin, 'angular_z': 0.0, 'duration_ms': dur,
            'drive_distance': dist,
        }
        last_res = _execute_agent_tool('send_motor_command', last_args)
        time.sleep(float(dur) / 1000.0 + 0.3)

    return {
        'name': 'send_motor_command' if action in ('forward', 'backward') else 'seek_fast_tank_turn',
        'arguments': last_args,
        'result': last_res,
        'action': action,
        'drive_distance': dist,
        'magnitude': plan['magnitude'],
        'summary': plan['summary'],
        'repeats': 1,
        'duration_ms': last_args.get('duration_ms', plan['duration_ms']),
        'turn_deg': plan['turn_deg'],
        'speed_ctrl': 'fast' if action in ('turn_left', 'turn_right') else 'normal',
        'cam_look': cam_look,
    }


def _seek_loop(ctrl, label, conf, max_steps, timeout_s):
    """Seek loop with support for single unified LLM scene navigation & triple scan thumbnail status."""
    t0 = time.time()
    referee = ctrl.referee()
    max_steps = int(max_steps or 0)
    timeout_s = float(timeout_s or 0)
    unlimited = max_steps <= 0
    steps_label = '∞' if unlimited else str(max_steps)

    st_dict = ctrl.status()
    llm_scene_nav = bool(st_dict.get('llm_scene_nav', True))
    llm_nav_interval = max(1, int(st_dict.get('llm_nav_interval', 10)))

    prev_center_jpeg = None
    cached_nav = None
    last_drive_action = None  # for recovery: after reverse, turn to open
    last_drive_distance = None
    last_lookdown_step = None
    try:
        _seek_reset_escape_cycle()
    except Exception:
        pass

    def _halt(phase, message, step=0, **kwargs):
        try:
            _execute_agent_tool('stop_motors', {})
        except Exception:
            pass
        try:
            _seek_look_deg(0.0, settle_s=0.2)
        except Exception:
            pass
        # Final OLED frame then release overlay back to network status loop
        try:
            act = 'FOUND' if phase == 'found' else (phase or 'end').upper()[:12]
            _seek_oled_set(
                goal=label,
                referee=referee,
                phase=phase,
                step=step,
                activity=act,
                detail=(message or '')[:40],
                message=message or '',
                nav_summary=phase or 'end',
                obstacle='',
            )
            _seek_oled_paint(force=True, advance_frame=1)
            time.sleep(1.2)
        except Exception:
            pass
        try:
            _seek_oled_clear(restore_hint=True)
        except Exception:
            pass
        ctrl.finish(phase, message=message, step=step, **kwargs)

    try:
        _seek_force_tools_on()
        _seek_disable_steady()
        _seek_explore_reset()
        _seek_oled_set(
            goal=label,
            referee=referee,
            phase='running',
            step=0,
            activity='start',
            detail=f'ref {referee}',
            message=f'Seeking {label}',
            nav_summary='start',
            obstacle='',
        )
        try:
            olog.info('ai_seek', f'OLED seek overlay ON goal={label} ref={referee}')
        except Exception:
            pass
        step = 0
        while True:
            step += 1
            if not unlimited and step > max_steps:
                break
            if ctrl.should_stop():
                _halt('stopped', 'Stopped by user', step=step - 1)
                return
            if timeout_s > 0 and (time.time() - t0) >= timeout_s:
                _halt('timeout', f'Timeout after {timeout_s}s', step=step - 1)
                return
            batt_block = _seek_battery_block_reason()
            if batt_block:
                _halt('failed', batt_block, step=step - 1)
                return

            if llm_scene_nav:
                # Never reuse a reverse escape: after backing up we must re-plan (turn/open).
                # Also re-plan immediately after any safety escape so we don't crawl reverse×N.
                force_replan = False
                if cached_nav:
                    ca = _seek_normalize_action(cached_nav.get('action'))
                    # Re-plan after reverse only. After a turn we rewrite cache to a
                    # forward hop so we cover ground instead of another 3-view+LLM wait.
                    if ca == 'backward':
                        force_replan = True
                is_calc_step = (
                    (step == 1)
                    or ((step - 1) % llm_nav_interval == 0)
                    or (cached_nav is None)
                    or force_replan
                )
                if is_calc_step:
                    ctrl.update(
                        step=step,
                        seek_phase='triple_scan',
                        message=f'Step {step}/{steps_label}: Capturing Left, Centre & Right views for LLM Scene Nav…'
                    )
                    _seek_oled_set(
                        goal=label, referee=referee, phase='triple_scan', step=step,
                        activity='scan L/C/R', detail='waiting pan',
                        message=f'step {step} scan', nav_summary='triple_scan',
                        obstacle='',
                    )
                    views_to_analyze, found_det = _seek_capture_triple_views(
                        ctrl, step, steps_label, goal_label=label, conf_threshold=conf, referee=referee
                    )
                    lu_found, lookup_hint = _seek_maybe_lookup(
                        ctrl, step, steps_label, views_to_analyze, label, conf,
                    )
                    if lu_found and lu_found.get('found') and not (found_det and found_det.get('found')):
                        found_det = lu_found
                    ld_found, lookdown_hint = _seek_maybe_lookdown(
                        ctrl, step, steps_label, views_to_analyze, label, conf,
                        last_action=last_drive_action,
                        last_lookdown_step=last_lookdown_step,
                    )
                    if lookdown_hint is not None:
                        last_lookdown_step = step
                    if ld_found and ld_found.get('found') and not (found_det and found_det.get('found')):
                        found_det = ld_found
                    
                    if found_det and found_det.get('found'):
                        found_v = found_det.get('found_view', 'scan')
                        raw = found_det.get('raw_detections') or found_det.get('labels_found') or []
                        ctrl.append_log(
                            'found',
                            f'FOUND “{label}” via detector at step {step} ({found_v} view)'
                            f' · saw {raw or "match"}',
                            step=step, view=found_v,
                        )
                        _seek_run_on_found(ctrl, label)
                        _halt('found', message=f'Found {label} via detector at step {step} ({found_v} view)', step=step, last_detection=found_det)
                        return

                    stitched_jpeg = _stitch_panorama_views(views_to_analyze)
                    b64_pano = base64.b64encode(stitched_jpeg).decode('ascii') if stitched_jpeg else ''
                    pano_url = f'data:image/jpeg;base64,{b64_pano}' if b64_pano else ''

                    ctrl.update(
                        panorama_data_url=pano_url,
                        last_views=[{
                            'name': v['name'], 'pan_deg': v['pan_deg'], 'bytes': v['bytes'],
                            'data_url': v.get('data_url'), 'has_target': v.get('has_target'),
                            'detected_labels': v.get('detected_labels', []),
                            'raw_detections': v.get('raw_detections', []),
                        } for v in views_to_analyze]
                    )

                    straight_v = next((v for v in views_to_analyze if v.get('name') == 'straight'), None)
                    straight_jpeg = straight_v.get('jpeg') if straight_v else None

                    ctrl.append_log('nav', f'Step {step}: consulting LLM for scene navigation…', step=step)
                    _seek_oled_set(
                        goal=label, referee=referee, phase='nav_decide', step=step,
                        activity='LLM think', detail='scene nav',
                        message=f'step {step} LLM', nav_summary='llm',
                        obstacle='',
                    )
                    analysis = _seek_unified_llm_analysis(
                        views_to_analyze, prev_center_jpeg, label,
                        last_action=last_drive_action,
                        lookdown=lookdown_hint,
                        lookup=lookup_hint,
                    )
                    
                    if (referee == REFEREE_LLM and analysis.get('goal_found')) or (found_det and found_det.get('found')):
                        found_v = analysis.get('goal_found_view') or (found_det.get('found_view') if found_det else 'straight')
                        for v in views_to_analyze:
                            if v.get('name') == found_v:
                                v['has_target'] = True
                        res_check = found_det or {
                            'found': True, 'goal_label': label, 'referee': REFEREE_LLM,
                            'reason': analysis.get('reason') or 'LLM unified query identified goal',
                            'found_view': found_v
                        }
                        ctrl.append_log(
                            'found',
                            f'FOUND “{label}” via {referee} at step {step} ({found_v})'
                            f' — {analysis.get("reason") or ""}',
                            step=step,
                        )
                        _seek_run_on_found(ctrl, label)
                        _halt('found', message=f'Found {label} via {referee} at step {step} ({found_v} view)', step=step, last_detection=res_check)
                        return

                    prefer_turn = analysis.get('prefer_turn') or _seek_prefer_open_turn(views_to_analyze)
                    llm_said = (analysis.get('source') == 'llm')
                    plan = _seek_nav_plan(
                        analysis.get('action') or analysis.get('recommended_direction') or 'forward',
                        analysis.get('drive_distance') or 'medium',
                    )
                    # Heuristic-only: doorway commit / turn-away. Do not rewrite an LLM hop/turn.
                    if not llm_said:
                        side = analysis.get('side_openness') or {}
                        if lookdown_hint:
                            side = {
                                'left': lookdown_hint.get('left', side.get('left')),
                                'right': lookdown_hint.get('right', side.get('right')),
                            }
                        commit = _seek_commit_through_opening(
                            last_drive_action, last_drive_distance,
                            obstacle_range=analysis.get('obstacle_ahead_range'),
                            left_open=side.get('left'),
                            right_open=side.get('right'),
                            centre_open=analysis.get('path_forward_clear'),
                        )
                        if commit:
                            plan = _seek_nav_plan(commit['action'], commit['drive_distance'])
                            plan['safety_override'] = commit['reason']
                        else:
                            away = _seek_prefer_away_from_wall(
                                plan.get('action'),
                                obstacle_range=analysis.get('obstacle_ahead_range'),
                                left_open=side.get('left'),
                                right_open=side.get('right'),
                                prefer_turn=prefer_turn,
                            )
                            if away:
                                plan = _seek_nav_plan(away, 'short')
                                plan['safety_override'] = (
                                    f'wall ahead — turn {away} away (do not drive into paint)'
                                )
                    action = plan['action']
                    dist = plan['drive_distance']
                    obs_r = plan.get('obstacle_range') or analysis.get('obstacle_ahead_range') or '?'
                    open_side = analysis.get('open_side') or (
                        'right' if prefer_turn == 'turn_right' else 'left'
                    )
                    reason = analysis.get('reason') or ''
                    if plan.get('safety_override'):
                        reason = f"{plan['safety_override']}" + (f' | {reason}' if reason else '')
                    if analysis.get('is_identical_to_previous'):
                        reason = (
                            f'STUCK DETECTED — recovery {plan["summary"]}'
                            + (f' ({reason})' if reason else '')
                        )

                    prev_center_jpeg = straight_jpeg
                    cached_nav = {
                        'action': action,
                        'drive_distance': dist,
                        'reason': reason,
                        'magnitude': plan['magnitude'],
                        'summary': plan['summary'],
                        'repeats': plan['repeats'],
                        'duration_ms': plan['duration_ms'],
                        'turn_deg': plan['turn_deg'],
                        'path_clear_forward': analysis.get('path_forward_clear'),
                        'obstacle_range': obs_r,
                        'open_side': open_side,
                        'prefer_turn': prefer_turn,
                        'last_action': last_drive_action,
                        'stuck': bool(analysis.get('is_identical_to_previous')),
                        'goal_found': bool(analysis.get('goal_found')),
                        'safety_override': plan.get('safety_override'),
                        'vision_blocked_hint': analysis.get('vision_blocked_hint'),
                        'source': 'llm',
                    }
                    clear_s = 'clear' if analysis.get('path_forward_clear') else 'blocked'
                    stuck_s = ' · STUCK' if analysis.get('is_identical_to_previous') else ''
                    safe_s = ' · SAFETY' if plan.get('safety_override') else ''
                    ctrl.append_log(
                        'nav',
                        f'Step {step} LLM nav → {plan["summary"]} · obstacle={obs_r}'
                        f' · open={open_side} · path {clear_s}{stuck_s}{safe_s}'
                        f' — {reason or "(no reason)"}',
                        step=step, action=action, dist=dist,
                    )
                    ctrl.update(
                        last_nav=cached_nav,
                        last_llm_reply=reason[:500],
                        message=(
                            f'Step {step}/{steps_label}: Nav {plan["summary"]}'
                            f' — {(reason or "")[:50]}'
                        ),
                    )
                    _seek_oled_set_nav(
                        action, dist, plan=plan, nav=cached_nav, obstacle=obs_r,
                        goal=label, referee=referee, phase='nav_decide', step=step,
                        message=(reason or plan['summary'])[:48],
                    )
                else:
                    # Intermediate step: run 0° goal check and reuse cached_nav
                    ctrl.update(step=step, seek_phase='goal_check', message=f'Step {step}/{steps_label}: goal check for {label}…')
                    _cn = cached_nav or {}
                    _seek_oled_set(
                        goal=label, referee=referee, phase='goal_check', step=step,
                        activity='SCAN', detail='SCAN', message='SCAN',
                        nav_summary=str(
                            f"{_cn.get('action') or ''}/{_cn.get('drive_distance') or ''}"
                        ).strip('/')[:48],
                        obstacle=str(_cn.get('obstacle_range') or '')[:16],
                    )
                    _seek_look_deg(0.0, wait_hw=True, should_stop=ctrl.should_stop)
                    straight_jpeg = _seek_grab_jpeg()
                    chk_centre = seek_goal_check(label, referee=REFEREE_DETECTOR, conf_threshold=conf, jpeg=straight_jpeg)
                    raw_c = chk_centre.get('raw_detections') or []
                    ctrl.append_log(
                        'detect',
                        f'Step {step} · centre check: saw [{", ".join(raw_c) if raw_c else "none"}]'
                        + (' · GOAL MATCH' if chk_centre.get('found') else ''),
                        step=step,
                    )
                    if chk_centre.get('found'):
                        ctrl.append_log('found', f'FOUND “{label}” via detector at step {step} (centre)', step=step)
                        _seek_run_on_found(ctrl, label)
                        _halt('found', message=f'Found {label} via detector at step {step}', step=step, last_detection=chk_centre)
                        return

                    # Update CENTRE image section in 180° panoramic scan live as we drive forward!
                    if straight_jpeg and views_to_analyze:
                        b64_s = base64.b64encode(straight_jpeg).decode('ascii')
                        for v in views_to_analyze:
                            if v.get('name') == 'straight':
                                v['jpeg'] = straight_jpeg
                                v['bytes'] = len(straight_jpeg)
                                v['data_url'] = f'data:image/jpeg;base64,{b64_s}'
                                v['detected_labels'] = chk_centre.get('labels_found') or []
                                v['raw_detections'] = list(chk_centre.get('raw_detections') or [])
                        
                        partial_pano = _stitch_panorama_views(views_to_analyze)
                        if partial_pano:
                            b64_p = base64.b64encode(partial_pano).decode('ascii')
                            ctrl.update(
                                panorama_data_url=f'data:image/jpeg;base64,{b64_p}',
                                last_views=[{
                                    'name': v['name'], 'pan_deg': v['pan_deg'], 'bytes': v['bytes'],
                                    'data_url': v.get('data_url'), 'has_target': v.get('has_target'),
                                    'detected_labels': v.get('detected_labels', []),
                                    'raw_detections': v.get('raw_detections', []),
                                } for v in views_to_analyze]
                            )

                    action = cached_nav['action']
                    dist = cached_nav['drive_distance']
                    reason = cached_nav.get('reason') or ''
                    mag = cached_nav.get('summary') or f'{action}/{dist}'
                    ctrl.append_log(
                        'nav',
                        f'Step {step}: reusing nav {mag} — {reason or ""}',
                        step=step, action=action, dist=dist,
                    )
                    ctrl.update(message=f'Step {step}/{steps_label}: Executing {mag}…')
                    _seek_oled_set_nav(
                        action, dist, nav=cached_nav,
                        obstacle=cached_nav.get('obstacle_range'),
                        goal=label, referee=referee, phase='drive', step=step,
                        message=f'reuse {action}/{dist}'[:48],
                    )
            else:
                # LLM Scene Nav disabled: use standard 3-view scan every step
                ctrl.update(step=step, seek_phase='goal_check', message=f'Step {step}/{steps_label}: 3-view scan for {label}…')
                _seek_oled_set(
                    goal=label, referee=referee, phase='triple_scan', step=step,
                    activity='scan L/C/R', detail='heuristic',
                    message=f'step {step} scan', nav_summary='triple_scan',
                    obstacle='',
                )
                views, found_chk = _seek_capture_triple_views(ctrl, step, steps_label, goal_label=label, conf_threshold=conf, referee=referee)
                lu_found, lookup_hint = _seek_maybe_lookup(
                    ctrl, step, steps_label, views, label, conf,
                )
                if lu_found and lu_found.get('found') and not (found_chk and found_chk.get('found')):
                    found_chk = lu_found
                ld_found, lookdown_hint = _seek_maybe_lookdown(
                    ctrl, step, steps_label, views, label, conf,
                    last_action=last_drive_action,
                    last_lookdown_step=last_lookdown_step,
                )
                if lookdown_hint is not None:
                    last_lookdown_step = step
                if ld_found and ld_found.get('found') and not (found_chk and found_chk.get('found')):
                    found_chk = ld_found
                ctrl.update(
                    last_views=[{
                        'name': v['name'], 'pan_deg': v['pan_deg'], 'bytes': v['bytes'],
                        'data_url': v.get('data_url'), 'has_target': v.get('has_target'),
                        'detected_labels': v.get('detected_labels', []),
                        'raw_detections': v.get('raw_detections', []),
                    } for v in views]
                )
                if found_chk and found_chk.get('found'):
                    found_v = found_chk.get('found_view', 'scan')
                    ctrl.append_log(
                        'found',
                        f'FOUND “{label}” in {found_v} view at step {step}',
                        step=step, view=found_v,
                    )
                    _seek_run_on_found(ctrl, label)
                    _halt('found', message=f'Found {label} in {found_v} view at step {step}', step=step, last_detection=found_chk)
                    return

                labels_h = (found_chk.get('labels_found') if isinstance(found_chk, dict) else []) or []
                nav = _seek_nav_decide(views, label, labels_hint=labels_h)
                nav = nav or {}
                plan = _seek_nav_plan(
                    nav.get('action') or 'forward',
                    nav.get('drive_distance') or 'medium',
                )
                if lookdown_hint and lookdown_hint.get('floor_blocked'):
                    away = lookdown_hint.get('prefer_turn') or 'turn_left'
                    plan = _seek_nav_plan(
                        away, 'short',
                        path_clear_forward=False,
                        obstacle_range='near',
                        last_action=last_drive_action,
                        prefer_turn=away,
                    )
                    plan['safety_override'] = (
                        f'look-down: floor blocked — turn {away} (bowl/wall/jamb)'
                    )
                action = plan['action']
                dist = plan['drive_distance']
                reason = plan.get('safety_override') or nav.get('reason') or plan['summary']
                nav.update({
                    'action': action,
                    'drive_distance': dist,
                    'magnitude': plan['magnitude'],
                    'summary': plan['summary'],
                    'repeats': plan['repeats'],
                    'duration_ms': plan['duration_ms'],
                    'turn_deg': plan['turn_deg'],
                    'source': 'detector_heuristic',
                    'reason': reason,
                })
                ctrl.append_log(
                    'nav',
                    f'Step {step} heuristic nav → {plan["summary"]} — {reason}',
                    step=step, action=action, dist=dist,
                )
                ctrl.update(last_nav=nav, last_llm_reply=reason[:500])
                _seek_oled_set_nav(
                    action, dist, plan=plan, nav=nav,
                    goal=label, referee=referee, phase='nav_decide', step=step,
                    message=(reason or plan['summary'])[:48],
                )

            if ctrl.should_stop():
                _halt('stopped', 'Stopped by user', step=step)
                return

            # Drive — always show short|medium|long + physical magnitude
            drive_plan = _seek_nav_plan(action, dist)
            action = drive_plan['action']
            dist = drive_plan['drive_distance']
            # Carry obstacle from cached/last nav when plan has no range context
            _drive_obs = ''
            try:
                _ln = {}
                try:
                    _ln = (ctrl.status() or {}).get('last_nav') or {}
                except Exception:
                    _ln = {}
                _drive_obs = (
                    (cached_nav or {}).get('obstacle_range')
                    or _ln.get('obstacle_range')
                    or drive_plan.get('obstacle_range')
                    or ''
                )
            except Exception:
                _drive_obs = drive_plan.get('obstacle_range') or ''
            ctrl.append_log(
                'drive',
                f'Step {step}: driving {drive_plan["summary"]}'
                + (f' — {reason[:100]}' if reason else ''),
                step=step, action=action, dist=dist,
            )
            _seek_oled_set_nav(
                action, dist, plan=drive_plan,
                obstacle=_drive_obs,
                goal=label, referee=referee, phase='drive', step=step,
                message=(reason or drive_plan['summary'])[:48],
            )
            try:
                st_now = ctrl.status()
                prev_nav = dict(st_now.get('last_nav') or {})
            except Exception:
                prev_nav = {}
            prev_nav.update({
                'action': action,
                'drive_distance': dist,
                'magnitude': drive_plan['magnitude'],
                'summary': drive_plan['summary'],
                'repeats': drive_plan['repeats'],
                'duration_ms': drive_plan['duration_ms'],
                'turn_deg': drive_plan['turn_deg'],
            })
            if reason:
                prev_nav.setdefault('reason', reason)
            ctrl.update(
                seek_phase='drive',
                message=f'Step {step}/{steps_label}: driving {drive_plan["summary"]}…',
                last_nav=prev_nav,
            )
            # open_side for reverse camera aim (±135° rear); front for FWD/turns
            open_side = 'left'
            try:
                open_side = (
                    (cached_nav or {}).get('open_side')
                    or prev_nav.get('open_side')
                    or ('right' if (cached_nav or {}).get('prefer_turn') == 'turn_right' else 'left')
                )
            except Exception:
                open_side = 'left'
            try:
                drive = _seek_execute_nav_action(
                    action, dist,
                    should_stop=ctrl.should_stop,
                    open_side=open_side,
                )
                last_drive_action = action
                last_drive_distance = dist
                # Exploration trail + heading (dead-reckoning for LLM bias)
                try:
                    _seek_explore_record(action, dist)
                    ex = _seek_explore_summary()
                    ctrl.update(
                        explore_heading=ex.get('cardinal'),
                        explore_heading_deg=ex.get('heading_deg'),
                        explore_trail=ex.get('trail_text'),
                    )
                except Exception:
                    pass
                # After reverse or a turn, drop cache so the next step *looks*
                # again. Blind forward-after-turn is how we drove into the pot.
                if action in ('backward', 'turn_left', 'turn_right'):
                    cached_nav = None
                ctrl.update(last_tools=[drive])
                cam_note = ''
                if isinstance(drive, dict) and drive.get('cam_look'):
                    cam_note = f' cam={drive.get("cam_look")}'
                try:
                    ex = _seek_explore_summary()
                    cam_note += f' hdg={ex.get("cardinal")}'
                except Exception:
                    pass
                ctrl.append_log(
                    'drive',
                    f'Step {step}: done {drive.get("summary") or drive_plan["summary"]}{cam_note}',
                    step=step,
                )
            except Exception as e:
                olog.warn('ai_seek', f'Drive failed: {e}', error=str(e)[:200], step=step)
                ctrl.append_log('warn', f'Step {step}: drive failed: {e}', step=step)
                try:
                    alt = 'turn_left' if action != 'turn_left' else 'turn_right'
                    alt_plan = _seek_nav_plan(
                        alt, 'short',
                        last_action=last_drive_action,
                        prefer_turn=alt,
                    )
                    _seek_oled_set_nav(
                        alt, 'short', plan=alt_plan,
                        goal=label, referee=referee, phase='drive', step=step,
                        message=f'fallback {alt}/short',
                    )
                    drive = _seek_execute_nav_action(
                        alt, 'short',
                        should_stop=ctrl.should_stop,
                        open_side=open_side,
                    )
                    last_drive_action = alt
                    ctrl.update(last_tools=[drive], message=f'Step {step}: fallback {alt_plan["summary"]}')
                    ctrl.append_log('drive', f'Step {step}: fallback {alt_plan["summary"]}', step=step)
                except Exception as e2:
                    _halt('failed', f'Drive failed: {e2}', step=step, error=str(e2)[:300])
                    return

            for _ in range(int(DEFAULT_SEEK_STEP_PAUSE_S / 0.1) or 1):
                if ctrl.should_stop():
                    _halt('stopped', 'Stopped by user', step=step)
                    return
                time.sleep(0.1)

        # Max steps exhausted
        _halt('timeout', message=f'Gave up after {max_steps} steps without match for {label}', step=max_steps)
    except Exception as e:
        olog.error('ai_seek', f'Seek loop crashed: {e}', error=str(e)[:300])
        try:
            _execute_agent_tool('stop_motors', {})
        except Exception:
            pass
        try:
            _seek_oled_set(
                goal=label, referee=referee, phase='failed', step=0,
                activity='FAILED', detail=_oled_fit(str(e), 40),
                message=_oled_fit(str(e), 48), nav_summary='failed',
            )
            _seek_oled_paint(force=True, advance_frame=1)
            time.sleep(0.8)
            _seek_oled_clear(restore_hint=True)
        except Exception:
            pass
        ctrl.finish('failed', message=str(e)[:300], error=str(e)[:300])


@app.route('/api/ai/seek/labels', methods=['GET'])
def api_ai_seek_labels():
    """Closed vocabulary for on-device detector referee (dropdown)."""
    return jsonify({
        'success': True,
        'referee_modes': [REFEREE_DETECTOR, REFEREE_LLM],
        'detector_labels': detector_labels(),
        'detector_backend': 'mobilenet-ssd',
        'note': (
            'detector mode only accepts these labels; '
            'llm mode accepts free-text goals and uses vision JSON found:true|false'
        ),
    })


# Battery-low log + Seek gate (volts). Unknown / ADC-ish readings do not block.
_battery_low_active = False
_BATTERY_LOW_V = float(os.environ.get('UGV_BATTERY_LOW_V') or 9.5)


def _read_battery_voltage_v():
    """Live pack volts from ESP32 feedback, or None if unknown."""
    try:
        bd = base.base_data if isinstance(getattr(base, 'base_data', None), dict) else {}
        return _interpret_base_voltage(bd.get('v'))
    except Exception:
        return None


def _seek_battery_block_reason():
    """Refuse Seek drive when pack voltage is known and below threshold."""
    if not _env_flag('UGV_SEEK_BATTERY_GATE', '1'):
        return None
    return _seek_battery_block_reason_pure(
        _read_battery_voltage_v(),
        low_v=_BATTERY_LOW_V,
        gate_enabled=True,
    )


@app.route('/api/ai/seek/start', methods=['POST'])
def api_ai_seek_start():
    data = request.get_json(silent=True) or {}
    batt_block = _seek_battery_block_reason()
    if batt_block:
        volts = _read_battery_voltage_v()
        olog.warn('ai_seek', batt_block, voltage_v=volts, threshold=_BATTERY_LOW_V)
        return jsonify({
            'success': False,
            'error': batt_block,
            'battery_blocked': True,
            'voltage_v': volts,
            'threshold_v': _BATTERY_LOW_V,
        }), 400
    if track_controller.is_running():
        return jsonify({'success': False, 'error': 'Track is running — stop it first'}), 409
    goal = data.get('goal') or data.get('target') or data.get('label') or ''
    referee = parse_seek_referee(data.get('referee') or data.get('mode') or REFEREE_DETECTOR)
    # Missing → finite pilot defaults. Explicit 0 / "unlimited" → unlimited.
    if 'max_steps' in data:
        max_steps = normalize_seek_max_steps(data.get('max_steps'))
    else:
        max_steps = normalize_seek_max_steps(None, default=DEFAULT_SEEK_MAX_STEPS)
    if 'timeout_s' in data:
        timeout_s = normalize_seek_timeout_s(data.get('timeout_s'))
    else:
        timeout_s = normalize_seek_timeout_s(None, default=DEFAULT_SEEK_TIMEOUT_S)
    conf = float(data.get('conf_threshold') or DEFAULT_SEEK_CONF)
    conf = max(0.05, min(0.95, conf))
    on_found = parse_on_found(data.get('on_found') or data.get('upon_found') or DEFAULT_ON_FOUND)
    on_found_tts = data.get('on_found_tts') or data.get('tts_phrase') or DEFAULT_ON_FOUND_TTS
    llm_scene_nav = bool(data.get('llm_scene_nav', True))
    try:
        llm_nav_interval = int(data.get('llm_nav_interval', 10))
    except (ValueError, TypeError):
        llm_nav_interval = 10
    result = seek_controller.start(
        goal,
        loop_fn=_seek_loop,
        max_steps=max_steps,
        timeout_s=timeout_s,
        conf_threshold=conf,
        referee=referee,
        on_found=on_found,
        on_found_tts=on_found_tts,
        llm_scene_nav=llm_scene_nav,
        llm_nav_interval=llm_nav_interval,
    )
    code = 200 if result.get('success') else 400
    if result.get('success'):
        olog.info(
            'ai_seek',
            f'Seek started ({referee}) for {result.get("status", {}).get("goal_label")} on_found={on_found}',
            goal=goal, referee=referee, on_found=on_found,
        )
    return jsonify(result), code


@app.route('/api/ai/seek/status', methods=['GET'])
def api_ai_seek_status():
    st = seek_controller.status()
    try:
        st['oled'] = _seek_oled_snapshot()
    except Exception:
        st['oled'] = {'active': False, 'lines': ['', '', '', ''], 'frame': 0}
    return jsonify({'success': True, 'status': st})


@app.route('/api/ai/seek/stop', methods=['POST'])
def api_ai_seek_stop():
    st = seek_controller.stop()
    # Always clear AI motion lock and force-zero chassis so Stop is not blocked by lock.
    result = _emergency_stop_motion(source='seek_stop', stop_seek=False)
    olog.info('ai_seek', 'Seek stop requested', phase=st.get('phase'))
    return jsonify({
        'success': True,
        'status': seek_controller.status(),
        'motion': result,
    })


def _emergency_stop_motion(source='api', stop_seek=True):
    """Zero chassis, cancel AI drive timer/lock, optionally cancel Seek.

    Always bypasses AI motion lock so operators can stop-by-STOP even during
    continuous AI drives. Used by /api/emergency_stop and Seek stop.
    """
    _cancel_ai_drive_timer()
    _clear_ai_motion_lock()
    seek_phase = None
    if stop_seek:
        try:
            st = seek_controller.stop()
            seek_phase = (st or {}).get('phase')
        except Exception as e:
            olog.warn('emergency_stop', f'Seek stop failed: {e}', source=source, error=str(e))
        try:
            track_controller.stop()
        except Exception as e:
            olog.warn('emergency_stop', f'Track stop failed: {e}', source=source, error=str(e)[:120])
    # Force-zero via router (bypasses lock) and direct serial belt-and-suspenders.
    route_results = []
    for cmd in (
        {'T': 1, 'L': 0, 'R': 0, '_force_stop': True},
        {'T': 13, 'X': 0, 'Z': 0, '_force_stop': True},
    ):
        try:
            route_results.append(_route_json_command(cmd))
        except Exception as e:
            route_results.append({'ok': False, 'error': str(e)})
    try:
        base.base_json_ctrl({'T': 13, 'X': 0, 'Z': 0})
        base.base_json_ctrl({'T': 1, 'L': 0, 'R': 0})
    except Exception as e:
        olog.warn('emergency_stop', f'Direct zero failed: {e}', source=source, error=str(e))
    lock_rem = max(0.0, _ai_motion_lock_until - time.time())
    olog.warn(
        'emergency_stop',
        f'Emergency STOP ({source})',
        source=source,
        seek_phase=seek_phase,
        lock_remaining_s=round(lock_rem, 3),
    )
    return {
        'ok': True,
        'source': source,
        'ai_motion_lock_active': lock_rem > 0,
        'ai_motion_lock_remaining_s': round(lock_rem, 3),
        'seek_phase': seek_phase,
        'routes': route_results,
    }


@app.route('/api/emergency_stop', methods=['POST'])
def api_emergency_stop():
    """Global STOP: cancel Seek, clear AI motion lock, zero chassis (bypass lock)."""
    result = _emergency_stop_motion(source='api_emergency_stop', stop_seek=True)
    try:
        seek_st = seek_controller.status()
    except Exception:
        seek_st = None
    return jsonify({
        'success': True,
        'motion': result,
        'status': seek_st,
        'ai_motion_lock_active': _ai_motion_lock_active(),
    })


@app.route('/api/ai/seek/check', methods=['POST'])
def api_ai_seek_check():
    """One-shot goal check (no motion). Body: {goal, referee?, conf_threshold?}."""
    data = request.get_json(silent=True) or {}
    goal = data.get('goal') or data.get('label') or ''
    referee = parse_seek_referee(data.get('referee') or data.get('mode') or REFEREE_DETECTOR)
    if referee == REFEREE_LLM:
        label, err = parse_llm_goal(goal)
    else:
        label, err = parse_seek_goal(goal)
    if err:
        return jsonify({'success': False, 'error': err}), 400
    conf = float(data.get('conf_threshold') or DEFAULT_SEEK_CONF)
    check = seek_goal_check(label, referee=referee, conf_threshold=conf)
    return jsonify({'success': True, 'goal_label': label, 'referee': referee, 'check': check})


_TRACK_SCAN_PANS = (-90.0, -45.0, 0.0, 45.0, 90.0)
_TRACK_SCAN_TILTS = (-18.0, 0.0, 14.0)


def _track_current_ptz():
    try:
        snap = _ptz_aim_public()
        return float(snap.get('pan_deg') or 0.0), float(snap.get('tilt_deg') or 0.0)
    except Exception:
        return 0.0, 0.0


def _track_goto(pan, tilt, should_stop=None):
    pan, tilt = clamp_ptz(pan, tilt)
    try:
        _publish_ptz_aim(pan, tilt, settled=False, source='track')
    except Exception:
        pass
    try:
        _seek_look_deg(pan, tilt_deg=tilt, wait_hw=True, should_stop=should_stop)
    except Exception as e:
        olog.warn('ai_track', f'Track PTZ goto failed: {e}', error=str(e)[:160])
    return pan, tilt


def _track_center_on_best(best, should_stop=None):
    ox, oy = bbox_offsets(best)
    if ox is None:
        return None
    if abs(ox) <= 0.10 and abs(oy or 0) <= 0.10:
        return {'centered': True, 'offset_x': ox, 'offset_y': oy}
    dpan, dtilt = ptz_delta_from_offsets(ox, oy)
    cur_p, cur_t = _track_current_ptz()
    npan, ntilt = _track_goto(cur_p + dpan, cur_t + dtilt, should_stop=should_stop)
    return {
        'centered': False,
        'offset_x': ox,
        'offset_y': oy,
        'pan': npan,
        'tilt': ntilt,
        'd_pan': dpan,
        'd_tilt': dtilt,
    }


def _track_loop(ctrl, label, conf, max_steps, timeout_s):
    """PTZ-only scan + lock. Never commands the chassis."""
    t0 = time.time()
    referee = (ctrl.status() or {}).get('referee') or REFEREE_DETECTOR
    max_steps = int(max_steps or 0)
    timeout_s = float(timeout_s or 0)
    locked = False
    lost = 0
    step = 0
    check_seq = 0
    pans = _TRACK_SCAN_PANS
    tilts = _TRACK_SCAN_TILTS
    if referee == REFEREE_LLM:
        # Vision LLM is slow — fewer poses so a sweep finishes this decade
        pans = (-60.0, 0.0, 60.0)
        tilts = (-12.0, 12.0)

    def _halt(phase, message):
        try:
            _track_goto(0.0, 0.0)
        except Exception:
            pass
        ctrl.finish(phase, message=message, step=step, locked=False)

    try:
        while True:
            if ctrl.should_stop():
                _halt('stopped', 'Stopped by user')
                return
            if timeout_s > 0 and (time.time() - t0) >= timeout_s:
                _halt('timeout', f'Timeout after {timeout_s}s')
                return
            if max_steps > 0 and step >= max_steps and not locked:
                _halt('timeout', f'Gave up after {max_steps} scan steps')
                return

            if not locked:
                step += 1
                ctrl.update(step=step, locked=False, message=f'Scan {step}: sweeping PTZ for {label}…')
                found = False
                for tilt in tilts:
                    if ctrl.should_stop():
                        break
                    for pan in pans:
                        if ctrl.should_stop():
                            break
                        ctrl.update(
                            message=f'Scan {step}: look pan={int(pan)}° tilt={int(tilt)}°',
                        )
                        _track_goto(pan, tilt, should_stop=ctrl.should_stop)
                        jpeg = _seek_grab_jpeg()
                        chk = seek_goal_check(
                            label, referee=referee, conf_threshold=conf, jpeg=jpeg,
                        )
                        check_seq += 1
                        ctrl.update(
                            last_detection=chk,
                            last_check_seq=check_seq,
                        )
                        raw = chk.get('raw_detections') or chk.get('labels_found') or []
                        ctrl.append_log(
                            'detect',
                            f'Scan {step} · pan {int(pan)}° tilt {int(tilt)}° · '
                            f'{referee} saw [{", ".join(str(x) for x in raw) or "none"}]'
                            + (' · LOCK' if chk.get('found') else ''),
                        )
                        if chk.get('found'):
                            found = True
                            locked = True
                            lost = 0
                            adj = _track_center_on_best(chk.get('best'), should_stop=ctrl.should_stop)
                            p, t = _track_current_ptz()
                            ctrl.update(locked=True, lock_pan=p, lock_tilt=t,
                                        message=f'Locked on {label} at pan={p:.0f}° tilt={t:.0f}°')
                            ctrl.append_log(
                                'found',
                                f'Locked “{label}” via {referee} at pan={p:.0f} tilt={t:.0f}'
                                + (f' · refine Δpan={adj.get("d_pan")}' if adj else ''),
                            )
                            break
                    if found:
                        break
                if not found:
                    ctrl.append_log('nav', f'Scan {step}: no {label} — next sweep')
                continue

            # Locked: re-check and keep bbox in the middle of the frame
            time.sleep(0.35)
            if ctrl.should_stop():
                _halt('stopped', 'Stopped by user')
                return
            jpeg = _seek_grab_jpeg()
            chk = seek_goal_check(label, referee=referee, conf_threshold=conf, jpeg=jpeg)
            ctrl.update(last_detection=chk)
            if chk.get('found'):
                lost = 0
                adj = _track_center_on_best(chk.get('best'), should_stop=ctrl.should_stop)
                p, t = _track_current_ptz()
                ctrl.update(
                    locked=True, lock_pan=p, lock_tilt=t,
                    message=f'Tracking {label} · pan={p:.0f}° tilt={t:.0f}°',
                )
                if adj and not adj.get('centered'):
                    ctrl.append_log(
                        'nav',
                        f'Refine centre Δpan={adj.get("d_pan")} Δtilt={adj.get("d_tilt")}',
                    )
            else:
                lost += 1
                ctrl.append_log('nav', f'Lock lost ({lost}/4) — {label} not in frame')
                if lost >= 4:
                    locked = False
                    ctrl.update(locked=False, message=f'Lost {label} — resuming PTZ sweep')
                    ctrl.append_log('nav', 'Resume sweep')
    except Exception as e:
        olog.error('ai_track', f'Track loop crashed: {e}', error=str(e)[:300])
        try:
            _execute_agent_tool('stop_motors', {})
        except Exception:
            pass
        ctrl.finish('failed', message=str(e)[:200], step=step)


@app.route('/api/ai/track/start', methods=['POST'])
def api_ai_track_start():
    data = request.get_json(silent=True) or {}
    if seek_controller.is_running():
        return jsonify({'success': False, 'error': 'Seek is running — stop it first'}), 409
    goal = data.get('goal') or data.get('target') or data.get('label') or ''
    resolved = resolve_track_goal(goal)
    if resolved.get('error'):
        return jsonify({'success': False, 'error': resolved['error']}), 400
    max_steps = int(data.get('max_steps') if data.get('max_steps') is not None else DEFAULT_TRACK_MAX_STEPS)
    timeout_s = float(data.get('timeout_s') if data.get('timeout_s') is not None else DEFAULT_TRACK_TIMEOUT_S)
    conf = float(data.get('conf_threshold') or DEFAULT_TRACK_CONF)
    result = track_controller.start(
        goal,
        loop_fn=_track_loop,
        max_steps=max_steps,
        timeout_s=timeout_s,
        conf_threshold=conf,
    )
    code = 200 if result.get('success') else 400
    return jsonify(result), code


@app.route('/api/ai/track/status', methods=['GET'])
def api_ai_track_status():
    return jsonify({'success': True, 'status': track_controller.status()})


@app.route('/api/ai/track/stop', methods=['POST'])
def api_ai_track_stop():
    track_controller.stop()
    return jsonify({'success': True, 'status': track_controller.status()})


@app.route('/api/ai/track/check', methods=['POST'])
def api_ai_track_check():
    data = request.get_json(silent=True) or {}
    goal = data.get('goal') or ''
    resolved = resolve_track_goal(goal)
    if resolved.get('error'):
        return jsonify({'success': False, 'error': resolved['error']}), 400
    chk = seek_goal_check(
        resolved['goal'],
        referee=resolved['referee'],
        conf_threshold=float(data.get('conf_threshold') or DEFAULT_TRACK_CONF),
    )
    return jsonify({
        'success': True,
        'goal_label': resolved['goal'],
        'referee': resolved['referee'],
        'check': chk,
    })


@app.route('/api/ai/motion_status', methods=['GET'])
def api_ai_motion_status():
    try:
        mode = get_control_mode()
        backend, bridge = _motion_backend_info()
        lock_rem = max(0.0, _ai_motion_lock_until - time.time())
        out = {
            'success': True,
            'control_mode': mode,
            'backend': backend,
            'rosbridge': bridge if mode == 'ros2' else {'ok': True, 'skipped': True, 'path': 'serial'},
            'cmd_vel_topic': None,
            'pt_joint_topic': None,
            'ai_motion_lock_active': lock_rem > 0,
            'ai_motion_lock_remaining_s': round(lock_rem, 3),
            'note': (
                'While ai_motion_lock_active, zero-velocity UI chassis heartbeats '
                '(T:1 L=0/R=0) are ignored so AI timed drives complete. '
                'Emergency STOP and stop_motors always clear the lock. '
                'Also set "Idle heartbeat" OFF on the main dashboard when using AI timed drives.'
            ),
            'tools': [
                t for t in _ai_tools_catalog()
                if t['name'].startswith('send_') or t['name'] in ('stop_motors', 'get_telemetry')
            ],
        }
        if mode == 'ros2':
            try:
                import ros_motion
                out['cmd_vel_topic'] = ros_motion.cmd_vel_topic()
                out['pt_joint_topic'] = ros_motion.pt_joint_topic()
            except Exception:
                pass
        return jsonify(out)
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500

@app.route('/ai')
def ai_agent_page():
    # Same-origin single-operator: inject token so page fetches can auth when set.
    return render_template('ai_agent.html', ugv_ai_token=_ai_auth_token())

# Route to render the HTML template
@app.route('/')
def index():
    audio_ctrl.play_random_audio("connected", False)
    return render_template('index.html')

@app.route('/3d')
def digital_twin_3d():
    return render_template('3d_twin.html')

@app.route('/config')
def get_config():
    with open(thisPath + '/config.yaml', 'r') as file:
        yaml_content = file.read()
    return yaml_content

# Catch-all: serve any file from templates/ (photo.html, video.html, settings.html, JS, CSS, etc.)
# Flask routes are matched most-specific first, so /api/* and /config etc. take priority.
@app.route('/<path:filename>')
def serve_static(filename):
    resp = send_from_directory('templates', filename)
    if _HOT_RELOAD:
        resp.headers['Cache-Control'] = 'no-store, no-cache, must-revalidate, max-age=0'
        resp.headers['Pragma'] = 'no-cache'
        resp.headers['Expires'] = '0'
    return resp

@app.route('/get_photo_names')
def get_photo_names():
    photo_files = sorted(os.listdir(thisPath + '/templates/pictures'), key=lambda x: os.path.getmtime(os.path.join(thisPath + '/templates/pictures', x)), reverse=True)
    return jsonify(photo_files)

@app.route('/delete_photo', methods=['POST'])
def delete_photo():
    filename = request.form.get('filename')
    try:
        os.remove(os.path.join(thisPath + '/templates/pictures', filename))
        olog.info('media_delete', f'Deleted photo {filename}', kind='photo', filename=filename, success=True)
        return jsonify(success=True)
    except Exception as e:
        olog.error('media_delete', f'Photo delete failed: {e}', kind='photo', filename=filename, error=str(e))
        return jsonify(success=False)

@app.route('/videos/<path:filename>')
def videos(filename):
    return send_from_directory(thisPath + '/templates/videos', filename)

@app.route('/get_video_names')
def get_video_names():
    video_files = sorted(
        [filename for filename in os.listdir(thisPath + '/templates/videos/') if filename.endswith('.mp4')],
        key=lambda filename: os.path.getctime(os.path.join(thisPath + '/templates/videos/', filename)),
        reverse=True
    )
    return jsonify(video_files)

@app.route('/delete_video', methods=['POST'])
def delete_video():
    filename = request.form.get('filename')
    try:
        os.remove(os.path.join(thisPath + '/templates/videos', filename))
        olog.info('media_delete', f'Deleted video {filename}', kind='video', filename=filename, success=True)
        return jsonify(success=True)
    except Exception as e:
        olog.error('media_delete', f'Video delete failed: {e}', kind='video', filename=filename, error=str(e))
        return jsonify(success=False)




# Video WebRTC
# Function to manage connections
def manage_connections(pc_id):
    if len(active_pcs) >= MAX_CONNECTIONS:
        # If maximum connections reached, terminate the oldest connection
        oldest_pc_id = next(iter(active_pcs))
        old_pc = active_pcs.pop(oldest_pc_id)
        old_pc.close()

    # Add new connection to active connections
    active_pcs[pc_id] = pc

# Asynchronous function to handle offer exchange
async def offer_async():
    params = await request.json
    offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

    # Create an RTCPeerConnection instance
    pc = RTCPeerConnection()

    # Generate a unique ID for the RTCPeerConnection
    pc_id = "PeerConnection(%s)" % uuid.uuid4()
    pc_id = pc_id[:8]

    # Manage connections
    manage_connections(pc_id)

    # Create and set the local description
    await pc.createOffer(offer)
    await pc.setLocalDescription(offer)

    # Prepare the response data with local SDP and type
    response_data = {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}

    return jsonify(response_data)

# Wrapper function for running the asynchronous offer function
def offer():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    
    future = asyncio.run_coroutine_threadsafe(offer_async(), loop)
    return future.result()

# set product version
def set_version(input_main, input_module):
    base.base_json_ctrl({"T":900,"main":input_main,"module":input_module})
    olog.info(
        'product_version',
        f'Set chassis main={input_main} module={input_module}',
        main_type=input_main, module_type=input_module, T=900,
    )
    if input_main == 1:
        cvf.info_update("RaspRover", (0,255,255), 0.36)
    elif input_main == 2:
        cvf.info_update("UGV Rover", (0,255,255), 0.36)
    elif input_main == 3:
        cvf.info_update("UGV Beast", (0,255,255), 0.36)
    if input_module == 0:
        cvf.info_update("No Module", (0,255,255), 0.36)
    elif input_module == 1:
        cvf.info_update("ARM", (0,255,255), 0.36)
    elif input_module == 2:
        cvf.info_update("PT", (0,255,255), 0.36)

# main cmdline for robot ctrl
def cmdline_ctrl(args_string):
    if not args_string:
        return
    args = args_string.split()
    # base -c {"T":1,"L":0.5,"R":0.5}
    if args[0] == 'base':
        if args[1] == '-c' or args[1] == '--cmd':
            payload = json.loads(args[2])
            t_code = payload.get('T') if isinstance(payload, dict) else None
            olog.info(
                'cli',
                f'CLI base -c T:{t_code}',
                command=args_string[:200], T=t_code, source='cli',
            )
            base.base_json_ctrl(payload)
        elif args[1] == '-r' or args[1] == '--recv':
            if args[2] == 'on':
                cvf.show_recv_info(True)
            else:
                cvf.show_recv_info(False)

    elif args[0] == 'audio':
        if args[1] == '-s' or args[1] == '--say':
            text = ' '.join(args[2:])
            olog.info('audio', f'TTS: {text[:80]}', kind='tts', text=text[:120])
            audio_ctrl.play_speech_thread(text)
        elif args[1] == '-v' or args[1] == '--volume':
            audio_ctrl.set_audio_volume(args[2])
        elif args[1] == '-p' or args[1] == '--play_file':
            olog.info('audio', f'Play file {args[2]}', kind='file', file=args[2])
            audio_ctrl.play_file(args[2])

    elif args[0] == 'send':
        if args[1] == '-a' or args[1] == '--add':
            if args[2] == '-b' or args[2] == '--broadcast':
                base.base_json_ctrl({"T":303,"mac":"FF:FF:FF:FF:FF:FF"})
            else:
                base.base_json_ctrl({"T":303,"mac":args[2]})
        elif args[1] == '-rm' or args[1] == '--remove':
            if args[2] == '-b' or args[2] == '--broadcast':
                base.base_json_ctrl({"T":304,"mac":"FF:FF:FF:FF:FF:FF"})
            else:
                base.base_json_ctrl({"T":304,"mac":args[2]})
        elif args[1] == '-b' or args[1] == '--broadcast':
            base.base_json_ctrl({"T":306,"mac":"FF:FF:FF:FF:FF:FF","dev":0,"b":0,"s":0,"e":0,"h":0,"cmd":3,"megs":' '.join(args[2:])})
        elif args[1] == '-g' or args[1] == '--group':
            base.base_json_ctrl({"T":305,"dev":0,"b":0,"s":0,"e":0,"h":0,"cmd":3,"megs":' '.join(args[2:])})
        else:
            base.base_json_ctrl({"T":306,"mac":args[1],"dev":0,"b":0,"s":0,"e":0,"h":0,"cmd":3,"megs":' '.join(args[2:])})

    elif args[0] == 'cv':
        if args[1] == '-r' or args[1] == '--range':
            try:
                lower_trimmed = args[2].strip("[]")
                lower_nums = [int(lower_num) for lower_num in lower_trimmed.split(",")]
                if all(0 <= num <= 255 for num in lower_nums):
                    pass
                else:
                    return
            except:
                return
            try:
                upper_trimmed = args[3].strip("[]")
                upper_nums = [int(upper_num) for upper_num in upper_trimmed.split(",")]
                if all(0 <= num <= 255 for num in upper_nums):
                    pass
                else:
                    return
            except:
                return
            cvf.change_target_color(lower_nums, upper_nums)
        elif args[1] == '-s' or args[1] == '--select':
            cvf.selet_target_color(args[2])

    elif args[0] == 'video' or args[0] == 'v':
        if args[1] == '-q' or args[1] == '--quality':
            try:
                int(args[2])
            except:
                return
            cvf.set_video_quality(int(args[2]))

    elif args[0] == 'line':
        if args[1] == '-r' or args[1] == '--range':
            try:
                lower_trimmed = args[2].strip("[]")
                lower_nums = [int(lower_num) for lower_num in lower_trimmed.split(",")]
                if all(0 <= num <= 255 for num in lower_nums):
                    pass
                else:
                    return
            except:
                return
            try:
                upper_trimmed = args[3].strip("[]")
                upper_nums = [int(upper_num) for upper_num in upper_trimmed.split(",")]
                if all(0 <= num <= 255 for num in upper_nums):
                    pass
                else:
                    return
            except:
                return
            cvf.change_line_color(lower_nums, upper_nums)
        elif args[1] == '-s' or args[1] == '--set':
            if len(args) != 9:
                return
            try:
                for i in range(2,9):
                    float(args[i])
            except:
                return
            # line -s 0.7 0.8 1.6 0.0006 0.6 0.4 0.2
            cvf.set_line_track_args(float(args[2]), float(args[3]), float(args[4]), float(args[5]), float(args[6]), float(args[7]), float(args[8]))

    elif args[0] == 'track':
        cvf.set_pt_track_args(args[1], args[2])

    elif args[0] == 'timelapse':
        if args[1] == '-s' or args[1] == '--start':
            if len(args) != 6:
                return
            try:
                move_speed = float(args[2])
                move_time  = float(args[3])
                t_interval = float(args[4])
                loop_times = int(args[5])
            except:
                return
            cvf.timelapse(move_speed, move_time, t_interval, loop_times)
        elif args[1] == '-e' or args[1] == '--end' or args[1] == '--stop':
            cvf.mission_stop()

    elif args[0] == 'p':
        main_type = int(args[1][0])
        module_type = int(args[1][1])
        set_version(main_type, module_type)

    # s 20
    elif args[0] == 's':
        main_type = int(args[1][0])
        module_type = int(args[1][1])
        if main_type == 1:
            f['base_config']['robot_name'] = "RaspRover"
            f['args_config']['max_speed'] = 0.65
            f['args_config']['slow_speed'] = 0.3
        elif main_type == 2:
            f['base_config']['robot_name'] = "UGV Rover"
            f['args_config']['max_speed'] = 1.3
            f['args_config']['slow_speed'] = 0.2
        elif main_type == 3:
            f['base_config']['robot_name'] = "UGV Beast"
            f['args_config']['max_speed'] = 1.0
            f['args_config']['slow_speed'] = 0.2
        f['base_config']['main_type'] = main_type
        f['base_config']['module_type'] = module_type
        with open(thisPath + '/config.yaml', "w") as yaml_file:
            yaml.dump(f, yaml_file)
        olog.warn(
            'config_write',
            f'Wrote config.yaml robot type main={main_type} module={module_type}',
            main_type=main_type, module_type=module_type,
            robot_name=f['base_config'].get('robot_name'),
        )
        set_version(main_type, module_type)

    elif args[0] == 'test':
        cvf.update_base_data({"T":1003,"mac":1111,"megs":"helllo aaaaaaaa"})


# Route to handle the offer request
@app.route('/offer', methods=['POST'])
def offer_route():
    return offer()

# Route to stream video frames
@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/send_command', methods=['POST'])
def handle_command():
    command = request.form['command']
    print("Received command:", command)
    olog.info('cli', f'Web CLI: {command[:160]}', command=command[:200], source='web')
    cvf.info_update("CMD:" + command, (0,255,255), 0.36)
    try:
        cmdline_ctrl(command)
    except Exception as e:
        olog.error('cli', f'Web CLI error: {e}', command=command[:200], error=str(e))
    return jsonify({"status": "success", "message": "Command received"})

@app.route('/getAudioFiles', methods=['GET'])
def get_audio_files():
    files = [f for f in os.listdir(UPLOAD_FOLDER) if os.path.isfile(os.path.join(UPLOAD_FOLDER, f)) and (f.endswith('.mp3') or f.endswith('.wav'))]
    return jsonify(files)

@app.route('/uploadAudio', methods=['POST'])
def upload_audio():
    if 'file' not in request.files:
        return jsonify({'error': 'No file part'})
    file = request.files['file']
    if file.filename == '':
        return jsonify({'error': 'No selected file'})
    if file:
        filename = secure_filename(file.filename)
        file.save(os.path.join(UPLOAD_FOLDER, filename))
        return jsonify({'success': 'File uploaded successfully'})

@app.route('/playAudio', methods=['POST'])
def play_audio():
    audio_file = request.form['audio_file']
    print(thisPath + '/sounds/others/' + audio_file)
    olog.info('audio', f'Play {audio_file}', kind='file', file=audio_file, source='ui')
    audio_ctrl.play_audio_thread(thisPath + '/sounds/others/' + audio_file)
    return jsonify({'success': 'Audio is playing'})

@app.route('/stop_audio', methods=['POST'])
def audio_stop():
    audio_ctrl.stop()
    olog.info('audio', 'Audio stop', kind='stop', source='ui')
    return jsonify({'success': 'Audio stop'})

@app.route('/settings/<path:filename>')
def serve_static_settings(filename):
    return send_from_directory('templates', filename)



def _route_json_command(cmd):
    """Route motion JSON via get_control_mode(): direct serial or ROS 2 relay.

    - T:1 / T:13  chassis → serial (if direct) or /cmd_vel (if ros2)
    - T:133 / T:141 gimbal → serial or ROS joint_states + pt controller topic
    - everything else → serial always

    Zero chassis cmds are ignored while AI motion lock is active (so the UI 2s
    idle heartbeat cannot cut a timed drive). Pass ``_force_stop: true`` (or
    ``force_stop``) to always deliver a stop — used by emergency STOP / stop_motors.
    """
    if not isinstance(cmd, dict):
        base.base_json_ctrl(cmd)
        return {'path': 'serial', 'ok': True, 'mode': get_control_mode()}

    # Strip internal flags before hardware; keep force_stop for lock policy.
    force_stop = bool(cmd.get('_force_stop') or cmd.get('force_stop') or cmd.get('_emergency'))
    if any(k in cmd for k in ('_force_stop', 'force_stop', '_emergency')):
        cmd = {
            k: v for k, v in cmd.items()
            if k not in ('_force_stop', 'force_stop', '_emergency')
        }

    mode = get_control_mode()
    t = cmd.get('T')
    gimbal_types = {
        133, 141, '133', '141',
        f.get('cmd_config', {}).get('cmd_gimbal_ctrl'),
        f.get('cmd_config', {}).get('cmd_gimbal_base_ctrl'),
    }
    chassis_types = {1, 13, '1', '13', f.get('cmd_config', {}).get('cmd_movition_ctrl')}

    # UI 2s idle heartbeat sends T:1 L=0 R=0; do not clobber an in-flight AI drive.
    # Emergency STOP sets force_stop so zeros always land and clear motion.
    if t in chassis_types and motion_lock_should_ignore_zero(
        _ai_motion_lock_active(),
        _chassis_cmd_is_zero(cmd),
        force_stop=force_stop,
    ):
        return {
            'path': 'ignored',
            'ok': True,
            'reason': 'ai_motion_active',
            'lock_remaining_s': max(0.0, _ai_motion_lock_until - time.time()),
        }

    # force_stop zeros also clear lock so subsequent heartbeats work normally
    if force_stop and t in chassis_types and _chassis_cmd_is_zero(cmd):
        _clear_ai_motion_lock()
        _cancel_ai_drive_timer()

    # ---- Gimbal / pan-tilt ----
    if t in gimbal_types:
        x = float(cmd.get('X', cmd.get('x', 0)) or 0)
        y = float(cmd.get('Y', cmd.get('y', 0)) or 0)
        tilt_sign_y = -y if t in (133, '133', f.get('cmd_config', {}).get('cmd_gimbal_ctrl')) else y

        def _apply_cvf_pt():
            # T:133 X/Y are the HUD numbers (same as Raw stick).
            _publish_ptz_aim(x, y, settled=False, source='socket_t133')

        import ros_motion as _rm
        path_choice = _rm.preferred_motion_path(mode, _rosbridge_reachable())
        if path_choice == 'ros2':
            try:
                result = _rm.publish_gimbal_from_ui(x, y, throttle=True)
                if result.get('ok'):
                    _apply_cvf_pt()
                    return {'path': 'ros2', 'ok': True, 'mode': mode, 'result': result}
                olog.warn(
                    'motion_route',
                    f'ROS2 gimbal returned not-ok; falling back to serial: {result.get("error")}',
                    path='ros2', T=t, mode=mode, error=str(result.get('error') or '')[:160],
                    throttle_s=3.0, throttle_key='ros2_gimbal_fallback',
                )
                path_choice = 'serial_fallback'
            except Exception as e:
                olog.error(
                    'motion_route', f'ROS2 gimbal failed; falling back to serial: {e}',
                    path='ros2', T=t, mode=mode, error=str(e),
                    throttle_s=3.0, throttle_key='ros2_gimbal_fail',
                )
                path_choice = 'serial_fallback'
        elif path_choice == 'serial_fallback':
            olog.warn(
                'motion_route',
                'ROS2 gimbal: rosbridge down — using serial fallback',
                path='serial_fallback', T=t, mode=mode,
                throttle_s=5.0, throttle_key='ros2_gimbal_no_bridge',
            )

        # Direct mode, or ROS mode with dead/failed bridge
        if path_choice != 'direct' and (
            getattr(base, 'serial_released_for_ros', False) or not base.serial_is_open()
        ):
            if not _ensure_flask_serial(reason='gimbal_route'):
                return {
                    'path': 'none',
                    'ok': False,
                    'mode': mode,
                    'error': 'rosbridge down and serial reclaim failed — switch Control to Direct',
                }
        base.base_json_ctrl(cmd)
        _apply_cvf_pt()
        return {
            'path': path_choice if path_choice in ('direct', 'serial_fallback') else 'serial_fallback',
            'ok': True,
            'mode': mode,
        }

    # ---- Chassis wheels ----
    if t in chassis_types:
        # Apply universal body→hardware signs so UI stick, AI T:13, and ROS agree.
        if t in (13, '13') or (('X' in cmd or 'x' in cmd) and t not in (1, '1')):
            body_lin = float(cmd.get('X', cmd.get('x', 0)) or 0)
            body_ang = float(cmd.get('Z', cmd.get('z', 0)) or 0)
            hw_lin, hw_ang = body_to_hw_twist(body_lin, body_ang)
            cmd = dict(cmd)
            cmd['X'] = hw_lin
            cmd['Z'] = hw_ang
            if 'x' in cmd:
                cmd['x'] = hw_lin
            if 'z' in cmd:
                cmd['z'] = hw_ang
        elif t in (1, '1') or 'L' in cmd or 'R' in cmd:
            L = float(cmd.get('L', 0) or 0)
            R = float(cmd.get('R', 0) or 0)
            hw_L, hw_R = body_to_hw_diff(L, R)
            cmd = dict(cmd)
            cmd['L'] = hw_L
            cmd['R'] = hw_R

        if mode == 'ros2' and _rosbridge_reachable():
            try:
                import ros_motion
                # Values already sign-mapped above for T:13; for T:1 derive twist from HW L/R
                # (same as body after sign). Prefer twist fields if present.
                if t in (13, '13') or 'X' in cmd or 'x' in cmd:
                    lin = float(cmd.get('X', cmd.get('x', 0)) or 0)
                    ang = float(cmd.get('Z', cmd.get('z', 0)) or 0)
                else:
                    L = float(cmd.get('L', 0) or 0)
                    R = float(cmd.get('R', 0) or 0)
                    lin = (L + R) / 2.0
                    ang = (R - L)
                result = ros_motion.publish_cmd_vel(lin, ang)
                if result.get('ok'):
                    return {'path': 'ros2', 'ok': True, 'mode': mode, 'result': result}
                olog.warn(
                    'motion_route',
                    f'ROS2 chassis not-ok; serial fallback: {result.get("error")}',
                    path='ros2', T=t, error=str(result.get('error') or '')[:160],
                    throttle_s=3.0, throttle_key='ros2_chassis_fallback',
                )
            except Exception as e:
                olog.error(
                    'motion_route', f'ROS2 chassis failed; serial fallback: {e}',
                    path='ros2', T=t, mode=mode, error=str(e),
                    throttle_s=3.0, throttle_key='ros2_chassis_fail',
                )
        if mode == 'ros2' and (
            getattr(base, 'serial_released_for_ros', False) or not base.serial_is_open()
        ):
            if not _ensure_flask_serial(reason='chassis_route'):
                return {
                    'path': 'none',
                    'ok': False,
                    'mode': mode,
                    'error': 'rosbridge down and serial reclaim failed — switch Control to Direct',
                }
        base.base_json_ctrl(cmd)
        return {
            'path': 'direct' if mode == 'direct' else 'serial_fallback',
            'ok': True,
            'mode': mode,
        }

    # Non-motion JSON always serial (lights, module select, etc.)
    # Log interesting T codes only (not high-rate noise)
    try:
        t_int = int(t) if t is not None else None
    except (TypeError, ValueError):
        t_int = None
    if t_int is not None and t_int in (4, 132, 136, 137, 401, 402, 403, 404, 405, 406, 407, 408, 600, 604, 900):
        olog.info('serial_cmd', f'Serial JSON T:{t_int}', T=t_int, path='serial', mode=mode)
    base.base_json_ctrl(cmd)
    return {'path': 'serial', 'ok': True, 'mode': mode}


# Web socket
@socketio.on('json', namespace='/json')
def handle_socket_json(json_data):
    try:
        _route_json_command(json_data)
    except Exception as e:
        olog.error('socket_json', f'Error handling JSON data: {e}', error=str(e))
        return

# Battery low edge (log once when crossing below threshold, again when recovered)
def _check_battery_edge():
    global _battery_low_active
    try:
        bd = base.base_data if isinstance(getattr(base, 'base_data', None), dict) else {}
        voltage = _interpret_base_voltage(bd.get('v'))
        if voltage is None:
            return
        if voltage <= _BATTERY_LOW_V and not _battery_low_active:
            _battery_low_active = True
            olog.warn('battery', f'Battery low: {voltage:.2f} V (≤ {_BATTERY_LOW_V} V)',
                      voltage_v=voltage, threshold=_BATTERY_LOW_V)
        elif voltage > (_BATTERY_LOW_V + 0.3) and _battery_low_active:
            _battery_low_active = False
            olog.info('battery', f'Battery recovered: {voltage:.2f} V',
                      voltage_v=voltage, threshold=_BATTERY_LOW_V)
    except Exception:
        pass


# info update single
def update_data_websocket_single():
    # {'T':1001,'L':0,'R':0,'r':0,'p':0,'v': 11,'pan':0,'tilt':0}
    try:
        _check_battery_edge()
        socket_data = {
            f['fb']['picture_size']:si.pictures_size,
            f['fb']['video_size']:  si.videos_size,
            f['fb']['cpu_load']:    si.cpu_load,
            f['fb']['cpu_temp']:    si.cpu_temp,
            f['fb']['ram_usage']:   si.ram,
            f['fb']['wifi_rssi']:   si.wifi_rssi,

            f['fb']['led_mode']:    cvf.cv_light_mode,
            f['fb']['detect_type']: cvf.cv_mode,
            f['fb']['detect_react']:cvf.detection_reaction_mode,
            f['fb']['pan_angle']:   _ptz_aim_public().get('pan_deg', cvf.pan_angle),
            f['fb']['tilt_angle']:  _ptz_aim_public().get('tilt_deg', cvf.tilt_angle),
            f['fb']['base_voltage']:base.base_data['v'] if (base.base_data and isinstance(base.base_data, dict) and 'v' in base.base_data) else 0,
            f['fb']['video_fps']:   cvf.video_fps,
            f['fb']['cv_movtion_mode']: cvf.cv_movtion_lock,
            f['fb']['base_light']:  base.base_light_status
        }
        socketio.emit('update', socket_data, namespace='/ctrl')
    except Exception as e:
        print("An [app.update_data_websocket_single] error occurred:", e)

# info feedback
def update_data_loop():
    # Seed remembered network OLED so seek exit can restore it
    try:
        _net_oled_paint(start_time=time.time(), remember=True)
    except Exception:
        try:
            base.base_oled(2, "F/J:5000/8888")
        except Exception:
            pass
    start_time = time.time()
    time.sleep(1)
    while 1:
        update_data_websocket_single()
        # During Seek, seek owns the panel (SEEK / goal / DO)
        if _seek_oled_is_active():
            try:
                _seek_oled_paint(force=False)
            except Exception:
                pass
            time.sleep(1.0)
            continue
        try:
            _net_oled_paint(start_time=start_time, remember=True)
        except Exception:
            # Fallback to original inline paint
            try:
                eth0 = si.eth0_ip
                wlan = si.wlan_ip
                base.base_oled(0, f"E:{eth0}" if eth0 else "E: No Ethernet")
                base.base_oled(1, f"W:{wlan}" if wlan else f"W: NO {si.net_interface}")
                elapsed_time = time.time() - start_time
                hours = int(elapsed_time // 3600)
                minutes = int((elapsed_time % 3600) // 60)
                seconds = int(elapsed_time % 60)
                base.base_oled(2, "F/J:5000/8888")
                base.base_oled(3, f"{si.wifi_mode} {hours:02d}:{minutes:02d}:{seconds:02d} {si.wifi_rssi}dBm")
            except Exception:
                pass
        time.sleep(5)

def base_data_loop():
    sensor_interval = 1
    sensor_read_time = time.time()
    while True:
        cvf.update_base_data(base.feedback_data())

        # get sensor data
        if base.extra_sensor:
            if time.time() - sensor_read_time > sensor_interval:
                base.rl.read_sensor_data()
                sensor_read_time = time.time()
        
        # get lidar data
        if base.use_lidar:
            base.rl.lidar_data_recv()
        
        time.sleep(0.025)

@socketio.on('message', namespace='/ctrl')
def handle_socket_cmd(message):
    try:
        json_data = json.loads(message)
    except json.JSONDecodeError:
        olog.warn('socket_cmd', 'JSON decode error on /ctrl', source='socket')
        return
    cmd_a = float(json_data.get("A", 0))
    if cmd_a in cmd_actions:
        # Log safety-relevant UI codes (CV / motion lock / capture) — not every zoom click
        try:
            codes = f.get('code') or {}
            interesting = {
                codes.get('cv_none'), codes.get('cv_moti'), codes.get('cv_face'),
                codes.get('cv_objs'), codes.get('cv_clor'), codes.get('mp_hand'),
                codes.get('cv_auto'), codes.get('mp_face'), codes.get('mp_pose'),
                codes.get('re_none'), codes.get('re_capt'), codes.get('re_reco'),
                codes.get('mc_lock'), codes.get('mc_unlo'),
                codes.get('pic_cap'), codes.get('vid_sta'), codes.get('vid_end'),
                codes.get('release'), codes.get('s_panid'), codes.get('s_tilid'),
                codes.get('set_mid'),
            }
            if cmd_a in interesting:
                label = None
                for k, v in codes.items():
                    if v == cmd_a:
                        label = k
                        break
                level = 'warn' if label in ('cv_auto', 'release', 's_panid', 's_tilid', 'set_mid') else 'info'
                olog.log(level, 'ui_cmd', f'UI cmd {label or cmd_a}',
                         cmd_a=cmd_a, label=label or str(cmd_a), source='socket')
        except Exception:
            pass
        cmd_actions[cmd_a]()
    else:
        pass
    if cmd_a in cmd_feedback_actions:
        threading.Thread(target=update_data_websocket_single, daemon=True).start()



# commandline on boot
def cmd_on_boot():
    cmd_list = [
        'base -c {"T":142,"cmd":50}',   # set feedback interval
        'base -c {"T":131,"cmd":1}',    # serial feedback flow on
        'base -c {"T":143,"cmd":0}',    # serial echo off
        'base -c {{"T":4,"cmd":{}}}'.format(f['base_config']['module_type']),      # select the module - 0:None 1:RoArm-M2-S 2:Gimbal
        'base -c {"T":300,"mode":0,"mac":"EF:EF:EF:EF:EF:EF"}',  # the base won't be ctrl by esp-now broadcast cmd, but it can still recv broadcast megs.
        'send -a -b'    # add broadcast mac addr to peer
    ]
    print('base -c {{"T":4,"cmd":{}}}'.format(f['base_config']['module_type']))
    for i in range(0, len(cmd_list)):
        cmdline_ctrl(cmd_list[i])
        cvf.info_update(cmd_list[i], (0,255,255), 0.36)
    set_version(f['base_config']['main_type'], f['base_config']['module_type'])



# Run the Flask app
if __name__ == "__main__":
    # lights off
    base.lights_ctrl(255, 255)
    
    # play a audio file in /sounds/robot_started/
    audio_ctrl.play_random_audio("robot_started", False)

    # update the size of videos and pictures
    si.update_folder(thisPath)

    # pt/arm looks forward
    if f['base_config']['module_type'] == 1:
        base.base_json_ctrl({"T":f['cmd_config']['cmd_arm_ctrl_ui'],"E":f['args_config']['arm_default_e'],"Z":f['args_config']['arm_default_z'],"R":f['args_config']['arm_default_r']})
    else:
        base.gimbal_ctrl(0, 0, 200, 10)

    # feedback loop starts
    si.start()
    si.resume()
    data_update_thread = threading.Thread(target=update_data_loop, daemon=True)
    data_update_thread.start()

    # base data update
    base_update_thread = threading.Thread(target=base_data_loop, daemon=True)
    base_update_thread.start()

    # lights off
    base.lights_ctrl(0, 0)
    cmd_on_boot()

    # run the main web app
    port = int(os.environ.get('UGV_PORT') or os.environ.get('PORT') or 5000)
    olog.info(
        'startup',
        f'UGV app ready on :{port}',
        port=port,
        control_mode=get_control_mode(),
        serial_open=bool(getattr(base, 'ser', None)),
        module_type=f['base_config'].get('module_type'),
        main_type=f['base_config'].get('main_type'),
        hot_reload=bool(_HOT_RELOAD),
        esp32_wifi_stop_on_start=os.environ.get('UGV_ESP32_WIFI_STOP_ON_START', '0'),
    )
    # HTML/JS/CSS: browser refresh is enough (TEMPLATES_AUTO_RELOAD + no-store headers).
    # Python process restart is opt-in via UGV_RELOADER=1 (re-inits serial/camera).
    if _HOT_RELOAD:
        print(f'[app.py] HOT RELOAD on :{port} — edit templates/* then refresh the browser'
              + ('; UGV_RELOADER=1 (*.py auto-restart)' if _USE_RELOADER else ''))
    socketio.run(
        app,
        host='0.0.0.0',
        port=port,
        allow_unsafe_werkzeug=True,
        debug=_HOT_RELOAD and _USE_RELOADER,
        use_reloader=_USE_RELOADER,
    )
