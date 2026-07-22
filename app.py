import serial
import sys
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
socketio = SocketIO(app)

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


def set_control_mode(mode, *, source='api'):
    """Set 'direct' (serial) or 'ros2' (rosbridge). Syncs chassis bypass flag."""
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
        # direct: Flask may drive wheels on serial; ros2: bypass chassis T:1/T:13
        base.enable_motor_control = (mode == 'direct')
    _save_control_mode()
    fields = {
        'mode': mode,
        'prev_mode': prev,
        'chassis_serial': 'on' if mode == 'direct' else 'bypassed',
        'source': source,
    }
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
    if prev != mode:
        olog.info(
            'control_mode',
            f'Control mode → {mode} (chassis serial {"on" if mode == "direct" else "bypassed"})',
            **fields,
        )
    return mode


_load_control_mode()
# Apply side effects for loaded mode
base.enable_motor_control = (get_control_mode() == 'direct')
olog.info(
    'startup',
    f'control_mode={get_control_mode()} '
    f'(chassis_serial={"on" if base.enable_motor_control else "bypassed for ROS 2"})',
    control_mode=get_control_mode(),
    chassis_serial='on' if base.enable_motor_control else 'bypassed',
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
    return jsonify({
        'enable_rtsp_stream': enable_rtsp_stream,
        'enable_motor_control': base.enable_motor_control,
        'control_mode': mode,  # 'direct' | 'ros2'
        'control_mode_label': 'Direct serial' if mode == 'direct' else 'ROS 2 relay',
        'esp32_wifi_stopped': bool(_esp32_wifi_session.get('stopped')),
    })

@app.route('/api/toggle_rtsp', methods=['POST'])
def api_toggle_rtsp():
    global enable_rtsp_stream
    enable_rtsp_stream = not enable_rtsp_stream
    olog.info('rtsp_toggle', f'RTSP stream {"ON" if enable_rtsp_stream else "OFF"}',
              enable_rtsp_stream=enable_rtsp_stream)
    return jsonify({'success': True, 'enable_rtsp_stream': enable_rtsp_stream})

@app.route('/api/control_mode', methods=['GET', 'POST'])
def api_control_mode():
    """Get or set unified routing: direct (serial) vs ros2 (rosbridge)."""
    if request.method == 'GET':
        mode = get_control_mode()
        bridge = {}
        try:
            import ros_motion
            bridge = ros_motion.rosbridge_status() if mode == 'ros2' else {}
        except Exception as e:
            bridge = {'ok': False, 'error': str(e)}
        return jsonify({
            'success': True,
            'control_mode': mode,
            'enable_motor_control': base.enable_motor_control,
            'rosbridge': bridge,
        })
    data = request.get_json(silent=True) or {}
    mode = data.get('mode') or data.get('control_mode')
    if not mode and data.get('toggle'):
        mode = 'direct' if get_control_mode() == 'ros2' else 'ros2'
    if not mode:
        return jsonify({'success': False, 'error': "provide mode: 'direct' or 'ros2'"}), 400
    try:
        mode = set_control_mode(mode, source='api')
    except ValueError as e:
        return jsonify({'success': False, 'error': str(e)}), 400
    bridge = {}
    if mode == 'ros2':
        try:
            import ros_motion
            bridge = ros_motion.rosbridge_status()
        except Exception as e:
            bridge = {'ok': False, 'error': str(e)}
    return jsonify({
        'success': True,
        'control_mode': mode,
        'enable_motor_control': base.enable_motor_control,
        'rosbridge': bridge,
    })

@app.route('/api/toggle_motors', methods=['POST'])
def api_toggle_motors():
    """Legacy: flip between direct serial and ROS 2 relay (same as control_mode toggle)."""
    mode = 'direct' if get_control_mode() == 'ros2' else 'ros2'
    mode = set_control_mode(mode, source='ui_toggle')
    return jsonify({
        'success': True,
        'enable_motor_control': base.enable_motor_control,
        'control_mode': mode,
    })


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
    }

def _grab_jpeg_bytes(max_width=640, quality=70):
    """Capture one JPEG from the live camera pipeline (thread-safe-ish)."""
    with _snapshot_lock:
        frame = cvf.frame_process()
    if not frame:
        raise RuntimeError('empty frame from camera')
    # frame_process already returns JPEG bytes when successful
    if isinstance(frame, (bytes, bytearray)) and frame[:2] == b'\xff\xd8':
        return bytes(frame)
    # Fallback: raw ndarray path (unlikely with current cv_ctrl)
    import cv2
    import numpy as np
    if isinstance(frame, np.ndarray):
        h, w = frame.shape[:2]
        if w > max_width:
            frame = cv2.resize(frame, (max_width, int(h * max_width / w)))
        ok, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        if not ok:
            raise RuntimeError('jpeg encode failed')
        return buf.tobytes()
    raise RuntimeError(f'unexpected frame type: {type(frame)}')

_AI_SYSTEM_PROMPT = (
    "You are a helpful vision assistant on a Waveshare UGV rover. "
    "When an image is attached, describe what you see clearly and briefly. "
    "You do not control motors yet — only describe and advise."
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


_DEFAULT_TOOL_CAPS = {n: True for n in _all_node_names()}
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
        state = dict(_ai_capabilities)
        for k, v in (updates or {}).items():
            if k in state:
                _apply_toggle(k, v, state)
        _ai_capabilities.clear()
        _ai_capabilities.update(state)
        snap = dict(_ai_capabilities)
    _save_ai_capabilities()
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

def _openai_chat(messages, max_tokens=512, temperature=0.4, tools=None):
    """Chat Completions via OpenAI-compatible HTTP API. Settings from env/.env only.

    Returns (assistant_message_dict, raw_body, cfg).
    assistant_message_dict may include content and/or tool_calls.
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
        payload['tool_choice'] = 'auto'
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
        with urllib.request.urlopen(req, timeout=120) as resp:
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
        'pan_angle': getattr(cvf, 'pan_angle', None),
        'tilt_angle': getattr(cvf, 'tilt_angle', None),
        'video_fps': getattr(cvf, 'video_fps', None),
        'motor_enabled': getattr(base, 'enable_motor_control', None),
    }
    try:
        bd = base.base_data if isinstance(getattr(base, 'base_data', None), dict) else {}
        data['voltage_raw'] = bd.get('v')
        data['base_feedback_T'] = bd.get('T')
    except Exception:
        pass
    return data


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
                    'Run on-board MobileNet-SSD object detection on the live camera. '
                    'Returns labels, confidences, and normalized bounding boxes [x1,y1,x2,y2].'
                ),
                'parameters': {
                    'type': 'object',
                    'properties': {
                        'conf_threshold': {
                            'type': 'number',
                            'description': 'Minimum confidence 0-1 (default 0.25)',
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
    # Motion group
    if by_name.get('send_motor_command', {}).get('status') == 'active':
        try:
            import ros_motion
            tools.extend(ros_motion.openai_motion_tools())
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
            conf = float(args.get('conf_threshold', 0.25))
            conf = max(0.05, min(0.95, conf))
            frame = cvf.grab_bgr_frame()
            if frame is None:
                dets = getattr(cvf, 'last_detections', []) or []
                return {
                    'ok': bool(dets),
                    'engine': 'mobilenet-ssd',
                    'detections': dets,
                    'count': len(dets),
                    'warning': 'live frame unavailable; returning last_detections if any',
                }
            dets = cvf.detect_objects_structured(frame, conf_threshold=conf)
            return {
                'ok': True,
                'engine': 'mobilenet-ssd',
                'conf_threshold': conf,
                'detections': dets,
                'count': len(dets),
            }
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
        result = ros_motion.execute_motion_tool(name, args)
        if isinstance(result, dict):
            result.setdefault('control_mode', mode)
            ok = bool(result.get('ok', True)) and not result.get('error')
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
    import time as _time

    max_lin = float(os.environ.get('UGV_MAX_LINEAR') or 0.35)
    max_ang = float(os.environ.get('UGV_MAX_ANGULAR') or 0.8)
    max_ms = int(os.environ.get('UGV_MAX_DRIVE_MS') or 4000)

    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))

    if name == 'stop_motors':
        base.base_json_ctrl({'T': 13, 'X': 0, 'Z': 0})
        base.base_json_ctrl({'T': 1, 'L': 0, 'R': 0})
        olog.warn('ai_motion', 'AI stop_motors via direct serial',
                  tool=name, control_mode=mode, path='serial', ok=True)
        return {'ok': True, 'backend': 'direct', 'control_mode': mode, 'path': 'serial'}

    if name == 'send_motor_command':
        lin = _clamp(float(args.get('linear_x', 0.0)), -max_lin, max_lin)
        ang = _clamp(float(args.get('angular_z', 0.0)), -max_ang, max_ang)
        dur = int(args.get('duration_ms') or 0)
        if dur < 0:
            dur = 0
        if dur > max_ms:
            dur = max_ms
        base.base_json_ctrl({'T': 13, 'X': lin, 'Z': ang})
        stopped = False
        if dur > 0:
            _time.sleep(dur / 1000.0)
            base.base_json_ctrl({'T': 13, 'X': 0, 'Z': 0})
            stopped = True
        olog.info(
            'ai_motion',
            f'AI drive lin={lin:.3f} ang={ang:.3f} dur={dur}ms (direct)',
            tool=name, control_mode=mode, path='serial', ok=True,
            linear_x=lin, angular_z=ang, duration_ms=dur, stopped=stopped,
        )
        return {
            'ok': True,
            'backend': 'direct',
            'control_mode': mode,
            'path': 'serial',
            'linear_x': lin,
            'angular_z': ang,
            'duration_ms': dur,
            'stopped': stopped,
        }

    if name == 'send_gimbal_command':
        pan = _clamp(float(args.get('pan_rad', 0.0)), -1.2, 1.2)
        tilt = _clamp(float(args.get('tilt_rad', 0.0)), -1.0, 0.6)
        # Inverse of ros_motion.ui_xy_to_radians
        x_deg = -pan * 180.0 / math.pi
        y_deg = tilt * 180.0 / math.pi
        base.base_json_ctrl({'T': 133, 'X': x_deg, 'Y': y_deg, 'SPD': 0, 'ACC': 0})
        try:
            cvf.pan_angle = x_deg
            cvf.tilt_angle = -y_deg
        except Exception:
            pass
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
    for _round in range(max_rounds):
        msg, body, cfg = _openai_chat(
            messages,
            max_tokens=512,
            temperature=0.4,
            tools=tools if tools else None,
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
                    args = {}
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

@app.route('/api/ai/config', methods=['GET'])
def api_ai_config():
    cfg = _ai_env_config()
    # Never send full key to browser — only a masked hint
    key = cfg['api_key'] or ''
    masked = (key[:3] + '…' + key[-2:]) if len(key) > 6 else ('***' if key else '')
    _, method = _get_token_encoder()
    motion_ok, backend, bridge = _motion_tools_available()
    caps = _get_capabilities()
    mode = get_control_mode()
    return jsonify({
        'base_url': cfg['base_url'],
        'model': cfg['model'],
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
            return jsonify({'success': False, 'error': 'compress produced empty summary'}), 502
    except Exception as e:
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
        'saved_tokens_est': max(0, before['tokens_est'] - after['tokens_est']),
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
        system += "Prefer short timed moves (duration_ms 500–2000). Call stop_motors if unsure."
    elif any(n in inactive for n in ('send_motor_command', 'stop_motors')):
        system += (
            "Motion is unavailable; tell the user to toggle Control to Direct serial, "
            "or enable ROS 2 mode with rosbridge + ugv_bringup."
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


@app.route('/api/ai/motion_status', methods=['GET'])
def api_ai_motion_status():
    try:
        mode = get_control_mode()
        backend, bridge = _motion_backend_info()
        out = {
            'success': True,
            'control_mode': mode,
            'backend': backend,
            'rosbridge': bridge if mode == 'ros2' else {'ok': True, 'skipped': True, 'path': 'serial'},
            'cmd_vel_topic': None,
            'pt_joint_topic': None,
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
    return render_template('ai_agent.html')

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
        return jsonify(success=True)
    except Exception as e:
        print(e)
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
        return jsonify(success=True)
    except Exception as e:
        print(e)
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
            base.base_json_ctrl(json.loads(args[2]))
        elif args[1] == '-r' or args[1] == '--recv':
            if args[2] == 'on':
                cvf.show_recv_info(True)
            else:
                cvf.show_recv_info(False)

    elif args[0] == 'audio':
        if args[1] == '-s' or args[1] == '--say':
            audio_ctrl.play_speech_thread(' '.join(args[2:]))
        elif args[1] == '-v' or args[1] == '--volume':
            audio_ctrl.set_audio_volume(args[2])
        elif args[1] == '-p' or args[1] == '--play_file':
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
    cvf.info_update("CMD:" + command, (0,255,255), 0.36)
    try:
        cmdline_ctrl(command)
    except Exception as e:
        print(f"[app.handle_command] error: {e}")
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
    audio_ctrl.play_audio_thread(thisPath + '/sounds/others/' + audio_file)
    return jsonify({'success': 'Audio is playing'})

@app.route('/stop_audio', methods=['POST'])
def audio_stop():
    audio_ctrl.stop()
    return jsonify({'success': 'Audio stop'})

@app.route('/settings/<path:filename>')
def serve_static_settings(filename):
    return send_from_directory('templates', filename)



def _route_json_command(cmd):
    """Route motion JSON via get_control_mode(): direct serial or ROS 2 relay.

    - T:1 / T:13  chassis → serial (if direct) or /cmd_vel (if ros2)
    - T:133 / T:141 gimbal → serial or ROS joint_states + pt controller topic
    - everything else → serial always
    """
    if not isinstance(cmd, dict):
        base.base_json_ctrl(cmd)
        return {'path': 'serial', 'ok': True, 'mode': get_control_mode()}

    mode = get_control_mode()
    t = cmd.get('T')
    gimbal_types = {
        133, 141, '133', '141',
        f.get('cmd_config', {}).get('cmd_gimbal_ctrl'),
        f.get('cmd_config', {}).get('cmd_gimbal_base_ctrl'),
    }
    chassis_types = {1, 13, '1', '13', f.get('cmd_config', {}).get('cmd_movition_ctrl')}

    # ---- Gimbal / pan-tilt ----
    if t in gimbal_types:
        x = float(cmd.get('X', cmd.get('x', 0)) or 0)
        y = float(cmd.get('Y', cmd.get('y', 0)) or 0)
        if mode == 'ros2':
            try:
                import ros_motion
                result = ros_motion.publish_gimbal_from_ui(x, y, throttle=True)
                try:
                    cvf.pan_angle = x
                    cvf.tilt_angle = -y if t in (133, '133', f.get('cmd_config', {}).get('cmd_gimbal_ctrl')) else y
                except Exception:
                    pass
                return {'path': 'ros2', 'ok': bool(result.get('ok')), 'mode': mode, 'result': result}
            except Exception as e:
                olog.error(
                    'motion_route', f'ROS2 gimbal failed: {e}',
                    path='ros2', T=t, mode=mode, error=str(e),
                    throttle_s=3.0, throttle_key='ros2_gimbal_fail',
                )
                return {'path': 'ros2', 'ok': False, 'mode': mode, 'error': str(e)}
        base.base_json_ctrl(cmd)
        return {'path': 'direct', 'ok': True, 'mode': mode}

    # ---- Chassis wheels ----
    if t in chassis_types:
        if mode == 'ros2':
            try:
                import ros_motion
                # Map T:1 L/R roughly to twist, or T:13 X/Z
                if t in (13, '13') or 'X' in cmd or 'x' in cmd:
                    lin = float(cmd.get('X', cmd.get('x', 0)) or 0)
                    ang = float(cmd.get('Z', cmd.get('z', 0)) or 0)
                else:
                    # Differential L/R → approximate unicycle
                    L = float(cmd.get('L', 0) or 0)
                    R = float(cmd.get('R', 0) or 0)
                    lin = (L + R) / 2.0
                    ang = (R - L)  # crude; good enough for stick heartbeats
                result = ros_motion.publish_cmd_vel(lin, ang)
                return {'path': 'ros2', 'ok': bool(result.get('ok')), 'mode': mode, 'result': result}
            except Exception as e:
                olog.error(
                    'motion_route', f'ROS2 chassis failed: {e}',
                    path='ros2', T=t, mode=mode, error=str(e),
                    throttle_s=3.0, throttle_key='ros2_chassis_fail',
                )
                return {'path': 'ros2', 'ok': False, 'mode': mode, 'error': str(e)}
        base.base_json_ctrl(cmd)
        return {'path': 'direct', 'ok': True, 'mode': mode}

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
_battery_low_active = False
_BATTERY_LOW_V = float(os.environ.get('UGV_BATTERY_LOW_V') or 9.5)


def _check_battery_edge():
    global _battery_low_active
    try:
        bd = base.base_data if isinstance(getattr(base, 'base_data', None), dict) else {}
        v = bd.get('v')
        if v is None:
            return
        voltage = float(v)
        # Waveshare often reports raw ADC-ish values; if > 30 treat as raw and skip
        if voltage > 30:
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
            f['fb']['pan_angle']:   cvf.pan_angle,
            f['fb']['tilt_angle']:  cvf.tilt_angle,
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
    base.base_oled(2, "F/J:5000/8888")
    start_time = time.time()
    time.sleep(1)
    while 1:
        update_data_websocket_single()
        eth0 = si.eth0_ip
        wlan = si.wlan_ip
        if eth0:
            base.base_oled(0, f"E:{eth0}")
        else:
            base.base_oled(0, f"E: No Ethernet")
        if wlan:
            base.base_oled(1, f"W:{wlan}")
        else:
            base.base_oled(1, f"W: NO {si.net_interface}")
        elapsed_time = time.time() - start_time
        hours = int(elapsed_time // 3600)
        minutes = int((elapsed_time % 3600) // 60)
        seconds = int(elapsed_time % 60)
        base.base_oled(3, f"{si.wifi_mode} {hours:02d}:{minutes:02d}:{seconds:02d} {si.wifi_rssi}dBm")
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
        print("Error decoding JSON.[app.handle_socket_cmd]")
        return
    cmd_a = float(json_data.get("A", 0))
    if cmd_a in cmd_actions:
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
