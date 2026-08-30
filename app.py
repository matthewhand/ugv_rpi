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

import loadout as loadout_mod
_LOADOUT_PATH = os.path.join(thisPath, loadout_mod.LOADOUT_FILENAME)
_loadout_store = loadout_mod.LoadoutStore(thisPath, _LOADOUT_PATH)
_loadout_store.load(fallback_from_config=f)
loadout_mod.apply_loadout_to_config(f, _loadout_store.get())
# Honor persisted use_lidar without forcing lidar hardware at install time.
try:
    base.set_use_lidar(bool(f['base_config'].get('use_lidar')))
except Exception:
    try:
        base.use_lidar = bool(f['base_config'].get('use_lidar'))
    except Exception:
        pass

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

# ---------------------------------------------------------------------------
# USB RoArm (hangar-gated). Start ONLY when attachment=roarm2.
# Rover + ptz/none keep roarm_started=false and never open CP2102.
# Live handle lives in roarm_ctrl (current_roarm / get_roarm / shutdown_roarm).
# ---------------------------------------------------------------------------
_AIM_MODE_PATH = os.path.join(thisPath, '.ui_aim_mode.json')
_aim_mode_lock = threading.Lock()
_ui_aim_mode = 'pt'


def _arm_cfg():
    return f.get('arm_config') or {}


def arm_usb_enabled():
    """True when hangar wants USB RoArm (attachment=roarm2)."""
    try:
        return loadout_mod.wants_roarm(_loadout_store.get())
    except Exception:
        return False


def arm_transport():
    return 'usb_serial' if arm_usb_enabled() else 'base_uart'


def _roarm_usb_owner() -> str:
    try:
        import ros_motion
        return ros_motion.roarm_usb_owner()
    except Exception:
        raw = (os.environ.get('UGV_ROARM_USB_OWNER') or 'flask').strip().lower()
        if raw in ('driver', 'ros', 'ros2', 'bridge', 'exclusive'):
            return 'driver'
        return 'flask'


def _default_ui_aim_mode():
    raw = (_arm_cfg().get('ui_aim_default') or 'auto').strip().lower()
    if raw in ('roarm', 'arm', 'usb'):
        return 'roarm'
    if raw in ('pt', 'gimbal', 'pan_tilt'):
        return 'pt'
    if arm_usb_enabled() or int(f.get('base_config', {}).get('module_type') or 0) == 1:
        return 'roarm'
    return 'pt'


def _load_ui_aim_mode():
    global _ui_aim_mode
    try:
        if os.path.isfile(_AIM_MODE_PATH):
            with open(_AIM_MODE_PATH, 'r', encoding='utf-8') as fh:
                data = json.load(fh)
            m = (data.get('mode') or '').strip().lower()
            if m in ('roarm', 'pt'):
                _ui_aim_mode = m
                return
    except Exception as e:
        print(f'[app.py] load ui_aim_mode: {e}')
    _ui_aim_mode = _default_ui_aim_mode()


def _save_ui_aim_mode():
    try:
        with open(_AIM_MODE_PATH, 'w', encoding='utf-8') as fh:
            json.dump({'mode': _ui_aim_mode}, fh)
    except Exception as e:
        print(f'[app.py] save ui_aim_mode: {e}')


def get_ui_aim_mode():
    with _aim_mode_lock:
        return _ui_aim_mode


def set_ui_aim_mode(mode, *, source='api'):
    global _ui_aim_mode
    mode = (mode or '').strip().lower()
    if mode in ('arm', 'usb', 'roarm_usb'):
        mode = 'roarm'
    if mode in ('gimbal', 'pan_tilt', 'pan-tilt'):
        mode = 'pt'
    if mode not in ('roarm', 'pt'):
        raise ValueError("mode must be 'roarm' or 'pt'")
    if mode == 'roarm' and not arm_usb_enabled():
        raise ValueError('Aim:RoArm requires hangar attachment=roarm2')
    with _aim_mode_lock:
        prev = _ui_aim_mode
        _ui_aim_mode = mode
    _save_ui_aim_mode()
    olog.info(
        'ui_aim',
        f'UI aim mode → {mode} (was {prev})',
        mode=mode, prev_mode=prev, source=source,
        arm_transport=arm_transport(),
    )
    return mode


_load_ui_aim_mode()


def get_roarm():
    """Live USB RoArm controller, or None. Single owner: roarm_ctrl."""
    import roarm_ctrl
    return roarm_ctrl.current_roarm()


def roarm_started() -> bool:
    return get_roarm() is not None


def _stop_roarm(*, reason='loadout'):
    """Close USB RoArm if open. Safe when already stopped."""
    import roarm_ctrl
    arm = roarm_ctrl.current_roarm()
    if arm is None:
        return {'stopped': False, 'already_down': True, 'ok': True}
    try:
        roarm_ctrl.shutdown_roarm()
    except Exception as e:
        olog.warn('roarm', f'RoArm close failed ({reason}): {e}', error=str(e)[:160])
    olog.info('roarm', f'RoArm stopped ({reason})', reason=reason, roarm_started=False)
    return {'stopped': True, 'ok': True, 'reason': reason}


def _start_roarm(*, reason='loadout'):
    """Open USB RoArm only when hangar attachment=roarm2."""
    import roarm_ctrl
    if not arm_usb_enabled():
        return {'started': False, 'ok': False, 'detail': 'attachment is not roarm2', 'roarm_started': False}
    if _roarm_usb_owner() == 'driver':
        olog.info(
            'roarm',
            'RoArm USB owner=driver; Flask skips CP2102 open',
            usb_owner='driver', reason=reason,
        )
        return {'started': False, 'ok': True, 'detail': 'usb_owner=driver', 'roarm_started': False, 'usb_owner': 'driver'}
    existing = roarm_ctrl.current_roarm()
    if existing is not None:
        st = {}
        try:
            st = existing.status() or {}
        except Exception:
            pass
        return {
            'started': False,
            'already_up': True,
            'ok': True,
            'roarm_started': True,
            'status': st,
        }
    try:
        port = (_arm_cfg().get('serial_port') or '').strip() or None
        baud = int(_arm_cfg().get('baud') or roarm_ctrl.DEFAULT_BAUD)
        arm = roarm_ctrl.get_roarm(port=port, baud=baud, enabled=True)
        st = arm.status() if arm else {}
        olog.info(
            'roarm',
            f"RoArm USB init connected={st.get('connected')} port={st.get('port')}",
            connected=st.get('connected'),
            port=st.get('port'),
            reason=reason,
            roarm_started=True,
        )
        return {
            'started': True,
            'ok': True,
            'roarm_started': True,
            'status': st,
            'reason': reason,
        }
    except Exception as e:
        try:
            roarm_ctrl.shutdown_roarm()
        except Exception:
            pass
        olog.error('roarm', f'RoArm USB init failed: {e}', error=str(e), reason=reason)
        return {
            'started': False,
            'ok': False,
            'roarm_started': False,
            'error': str(e),
            'reason': reason,
        }


def _sync_roarm_to_loadout(*, reason='loadout'):
    """Start RoArm iff hangar attachment=roarm2; otherwise ensure stopped."""
    if arm_usb_enabled():
        return _start_roarm(reason=reason)
    return _stop_roarm(reason=reason)


def _route_arm_ui_cmd(cmd):
    """Handle stock UI T:144 E/Z/R — USB RoArm when hangar says roarm2, else base UART."""
    e = float(cmd.get('E', cmd.get('e', f['args_config'].get('arm_default_e', 60))) or 0)
    z = float(cmd.get('Z', cmd.get('z', f['args_config'].get('arm_default_z', 24))) or 0)
    r = float(cmd.get('R', cmd.get('r', f['args_config'].get('arm_default_r', 0))) or 0)
    try:
        cvf.pan_angle = r
        cvf.tilt_angle = z
    except Exception:
        pass

    chassis_mode = get_control_mode()
    default_e = float(f['args_config'].get('arm_default_e', 60))
    default_z = float(f['args_config'].get('arm_default_z', 24))
    default_r = float(f['args_config'].get('arm_default_r', 0))

    if not arm_usb_enabled():
        base.base_json_ctrl(cmd)
        return {'path': 'base_uart', 'ok': True, 'roarm_started': False}

    import roarm_ctrl
    # Preserve current hand position to avoid closing grip on stick/height moves
    arm = get_roarm()
    current_hand = None
    if arm is not None:
        current_hand = getattr(arm, '_last_joints', {}).get("hand")
    joints = roarm_ctrl.e_z_r_to_joints(
        e, z, r, default_e=default_e, default_z=default_z, default_r=default_r,
        hand=current_hand
    )
    usb_owner = _roarm_usb_owner()
    ros_cmd = None
    ros_mirror = None
    ok = False
    text = ''
    path = 'roarm_usb'

    if chassis_mode == 'ros2':
        try:
            import ros_motion
            ros_cmd = ros_motion.publish_roarm_joint_command(
                joints.get('base', 0),
                joints.get('shoulder', 0),
                joints.get('elbow', 0),
                joints.get('hand', 0),
                throttle=True,
                also_states=(usb_owner == 'driver'),
            )
        except Exception as ex:
            ros_cmd = {'ok': False, 'error': str(ex)}

    use_usb = True
    if usb_owner == 'driver' and chassis_mode == 'ros2' and ros_cmd and ros_cmd.get('ok'):
        use_usb = False
        ok = True
        path = 'roarm_ros_command'
        text = 'published joint_command (USB owned by driver)'

    if use_usb:
        arm = get_roarm()
        if arm is None:
            _start_roarm(reason='t144')
            arm = get_roarm()
        if arm is not None:
            ok, text, joints = arm.set_from_e_z_r(
                e, z, r,
                default_e=default_e,
                default_z=default_z,
                default_r=default_r,
            )
            path = 'roarm_usb'
            if chassis_mode == 'ros2' and ros_cmd and ros_cmd.get('ok'):
                path = 'roarm_hybrid'
        else:
            ok = bool(ros_cmd and ros_cmd.get('ok'))
            if not ok:
                text = text or 'no RoArm USB and ROS command failed'
                path = 'roarm_unavailable'

    if ok and joints and path in ('roarm_usb', 'roarm_hybrid', 'roarm_usb_fallback'):
        try:
            import ros_motion
            if chassis_mode == 'ros2' or ros_motion.rosbridge_status().get('ok'):
                ros_mirror = ros_motion.publish_roarm_joints(
                    joints.get('base', 0),
                    joints.get('shoulder', 0),
                    joints.get('elbow', 0),
                    joints.get('hand', 0),
                    throttle=True,
                )
        except Exception:
            ros_mirror = None

    return {
        'path': path,
        'ok': bool(ok),
        'text': text,
        'joints': joints,
        'ros_cmd': ros_cmd,
        'ros_mirror': ros_mirror,
        'roarm_started': roarm_started(),
        'status': (arm.status() if arm else None),
    }


def _route_roarm_raw(cmd):
    """Forward USB-native RoArm T-codes when hangar attachment=roarm2.
    
    T:102 (set_joints) is routed through set_joints() to update _last_joints
    so the feedback API returns accurate joint state.
    """
    arm = get_roarm()
    if not arm_usb_enabled() or arm is None:
        base.base_json_ctrl(cmd)
        return {'path': 'base_uart', 'ok': True, 'roarm_started': False}
    
    # Special handling for T:102 to ensure _last_joints is updated
    t = cmd.get('T')
    if t in (102, '102'):
        try:
            base_rad = float(cmd.get('base', 0))
            shoulder_rad = float(cmd.get('shoulder', 0))
            elbow_rad = float(cmd.get('elbow', 1.5708))
            hand_rad = float(cmd.get('hand', 3.1416))
            spd = float(cmd.get('spd', 0))
            acc = float(cmd.get('acc', 12))
            ok, text = arm.set_joints(
                base_rad, shoulder_rad, elbow_rad, hand_rad,
                spd=spd, acc=acc
            )
            return {
                'path': 'roarm_usb_set_joints',
                'ok': bool(ok),
                'text': text,
                'status': arm.status(),
                'roarm_started': True,
            }
        except (ValueError, TypeError) as e:
            return {
                'path': 'roarm_usb',
                'ok': False,
                'error': f'T:102 param error: {e}',
                'status': arm.status(),
                'roarm_started': True,
            }
    
    # All other T-codes go through send_json (no _last_joints update needed)
    ok, text = arm.send_json(cmd, read_s=0.2)
    return {
        'path': 'roarm_usb',
        'ok': bool(ok),
        'text': text,
        'status': arm.status(),
        'roarm_started': True,
    }


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
        'loadout': _status_loadout(),
        'lidar': _lidar_public(),
    })

@app.route('/api/toggle_rtsp', methods=['POST'])
def api_toggle_rtsp():
    global enable_rtsp_stream
    enable_rtsp_stream = not enable_rtsp_stream
    olog.info('rtsp_toggle', f'RTSP stream {"ON" if enable_rtsp_stream else "OFF"}',
              enable_rtsp_stream=enable_rtsp_stream)
    return jsonify({'success': True, 'enable_rtsp_stream': enable_rtsp_stream})


def _lidar_public(include_sample=True):
    """Live USB lidar slice for /api/status, /api/lidar, and the 3D twin.

    `detected` is true when a CP2102/ACM lidar candidate exists even if
    hangar `use_lidar` is still off (so the UI can offer the toggle).
    """
    rl = getattr(base, 'rl', None)
    snap = {}
    if rl is not None and hasattr(rl, 'lidar_snapshot'):
        try:
            snap = rl.lidar_snapshot() or {}
        except Exception as e:
            snap = {'error': str(e)}
    candidates = []
    try:
        from base_ctrl import lidar_port_candidates
        candidates = list(lidar_port_candidates() or [])
    except Exception:
        candidates = []
    detected = bool(snap.get('open') or candidates)
    out = {
        'enabled': bool(getattr(base, 'use_lidar', False)),
        'detected': detected,
        'candidates': candidates,
        **snap,
    }
    if not include_sample:
        out.pop('sample', None)
    return out


def _status_loadout():
    """Compact loadout slice for /api/status (no yaml write)."""
    pub = _loadout_store.public(f, roarm_started=roarm_started())
    return {
        'loadout': pub.get('loadout'),
        'effective': pub.get('effective'),
        'arm_status': pub.get('arm_status'),
        'arm_message': pub.get('arm_message'),
        'roarm_started': bool(pub.get('roarm_started')),
    }


_LOADOUT_PATCH_KEYS = ('base', 'attachment', 'use_lidar', 'camera_prefer')


@app.route('/api/loadout', methods=['GET', 'POST'])
def api_loadout():
    """Get or merge chassis/attachment loadout.

    Side effects (still never write config.yaml / drive signs):
      - overlay in-memory base_config
      - apply camera_prefer live via cv_ctrl re-init
      - start/stop USB RoArm only when attachment is/ was roarm2
      - open/close USB lidar (CP2102 / ttyACM) when use_lidar flips
    """
    if request.method == 'GET':
        payload = _loadout_store.public(f, roarm_started=roarm_started())
        payload['lidar'] = _lidar_public()
        return jsonify(payload)
    data = request.get_json(silent=True)
    if data is None:
        if request.get_data():
            return jsonify({'ok': False, 'error': 'invalid JSON'}), 400
        data = {}
    elif not isinstance(data, dict):
        return jsonify({'ok': False, 'error': 'invalid JSON'}), 400
    prev = _loadout_store.get()
    patch = {k: data[k] for k in _LOADOUT_PATCH_KEYS if k in data}
    _loadout_store.set(patch)
    lo = _loadout_store.get()
    loadout_mod.apply_loadout_to_config(f, lo)
    lidar_apply = None
    try:
        lidar_ok = base.set_use_lidar(bool(f['base_config'].get('use_lidar')))
        lidar_apply = {
            'ok': bool(lidar_ok) if lo.get('use_lidar') else True,
            'enabled': bool(getattr(base, 'use_lidar', False)),
            'port': getattr(base.rl, 'lidar_port', None),
            'error': getattr(base.rl, 'lidar_last_error', None),
        }
    except Exception as e:
        lidar_apply = {'ok': False, 'error': str(e)}
        olog.warn('loadout', f'lidar apply failed: {e}', error=str(e)[:160])

    camera_apply = None
    if patch.get('camera_prefer') is not None or patch.get('base') is not None:
        try:
            camera_apply = cvf.apply_camera_prefer(lo)
        except Exception as e:
            camera_apply = {'ok': False, 'error': str(e)}
            olog.warn('loadout', f'camera_prefer apply failed: {e}', error=str(e)[:160])

    roarm_sync = _sync_roarm_to_loadout(reason='api_loadout')
    if arm_usb_enabled():
        try:
            set_ui_aim_mode('roarm', source='loadout')
        except Exception:
            pass
    elif prev.get('attachment') == 'roarm2':
        try:
            set_ui_aim_mode('pt', source='loadout')
        except Exception:
            pass

    payload = _loadout_store.public(f, roarm_started=roarm_started())
    payload['camera_apply'] = camera_apply
    payload['lidar'] = _lidar_public()
    payload['lidar_apply'] = lidar_apply
    payload['roarm_sync'] = {
        k: roarm_sync.get(k)
        for k in ('started', 'stopped', 'ok', 'already_up', 'already_down', 'detail', 'error', 'usb_owner')
        if k in roarm_sync
    }
    olog.info(
        'loadout',
        f'Loadout → {lo.get("base")}/{lo.get("attachment")} '
        f'lidar={lo.get("use_lidar")} cam={lo.get("camera_prefer")}',
        base=lo.get('base'),
        attachment=lo.get('attachment'),
        use_lidar=lo.get('use_lidar'),
        camera_prefer=lo.get('camera_prefer'),
        roarm_started=roarm_started(),
        arm_status=payload.get('arm_status'),
        camera_ok=(camera_apply or {}).get('ok'),
    )
    return jsonify(payload)


@app.route('/api/lidar', methods=['GET'])
def api_lidar():
    """Last USB lidar revolution (LD19/CP2102). ?fresh=1 waits for one scan."""
    if request.args.get('fresh') in ('1', 'true', 'yes'):
        if getattr(base, 'use_lidar', False):
            try:
                base.rl.lidar_data_recv()
            except Exception as e:
                olog.warn('lidar', f'fresh scan failed: {e}', error=str(e)[:160])
        else:
            return jsonify({
                'ok': False,
                'error': 'use_lidar is off',
                **_lidar_public(),
            }), 409
    snap = _lidar_public()
    return jsonify({'ok': True, **snap})


@app.route('/api/ui_aim_mode', methods=['GET', 'POST'])
def api_ui_aim_mode():
    """Toggle UI overlay target: pan/tilt (pt) vs RoArm (roarm)."""
    if request.method == 'GET':
        return jsonify({
            'ok': True,
            'mode': get_ui_aim_mode(),
            'arm_transport': arm_transport(),
            'roarm_started': roarm_started(),
            'roarm': (get_roarm().status() if get_roarm() else None),
        })
    data = request.get_json(silent=True) or {}
    mode = data.get('mode')
    if not mode:
        mode = 'pt' if get_ui_aim_mode() == 'roarm' else 'roarm'
    try:
        mode = set_ui_aim_mode(mode, source='api')
    except ValueError as e:
        return jsonify({'ok': False, 'error': str(e), 'mode': get_ui_aim_mode()}), 400
    return jsonify({
        'ok': True,
        'mode': mode,
        'arm_transport': arm_transport(),
        'roarm_started': roarm_started(),
        'roarm': (get_roarm().status() if get_roarm() else None),
    })


def _twin_joints_snapshot():
    """Last commanded RoArm joints, or the configured default pose for the overlay."""
    import roarm_ctrl
    arm = get_roarm()
    if arm is not None:
        st = arm.status()
        joints = st.get('last_joints')
        if isinstance(joints, dict) and joints:
            return dict(joints), bool(st.get('connected')), st.get('port'), 'commanded'
        return dict(roarm_ctrl.POSES['home']), bool(st.get('connected')), st.get('port'), 'home'
    pose_name = str((_arm_cfg().get('default_pose') or 'travel_tuck')).strip().lower()
    pose = roarm_ctrl.POSES.get(pose_name) or roarm_ctrl.POSES['home']
    return dict(pose), False, None, pose_name


@app.route('/api/twin')
def api_twin():
    """Shared 3D twin snapshot — hangar overlay and /3d use the same payload.

    Joints are last commanded USB RoArm radians (T:102 / T:144 map). Direct
    serial does not need rosbridge. PTZ robots still get pan/tilt degrees.
    """
    import roarm_ctrl
    lo = _loadout_store.get()
    types = loadout_mod.effective_types(lo)
    joints, connected, port, joints_source = _twin_joints_snapshot()
    fk = roarm_ctrl.forward_kinematics(
        joints.get('base', 0.0),
        joints.get('shoulder', 0.0),
        joints.get('elbow', 1.5708),
        joints.get('hand'),
    )
    return jsonify({
        'ok': True,
        'robot_name': types.get('robot_name') or f.get('base_config', {}).get('robot_name') or 'UGV',
        'chassis': lo.get('base') or 'rover',
        'attachment': lo.get('attachment') or 'ptz',
        'drive': types.get('drive'),
        'main_type': int(types.get('main_type') or 2),
        'module_type': int(types.get('module_type') or 0),
        'roarm_started': roarm_started(),
        'roarm_connected': connected,
        'roarm_port': port,
        'joints': joints,
        'joints_source': joints_source,
        'ee_m': {k: round(fk[k], 4) for k in ('x', 'y', 'z', 'z_world')},
        'ptz': _ptz_aim_public(),
        'kinematics': roarm_ctrl.kinematics_public(),
        'lidar': _lidar_public(include_sample=False),
    })


@app.route('/api/arm/move', methods=['POST'])
def api_arm_move():
    """IK end-effector jog for the USB RoArm.

    Body (all optional, applied together):
      d_reach_mm / d_lift_mm / d_yaw_deg — relative EE deltas.
    Reach keeps gripper height; lift keeps reach; yaw is base-only. Seeded
    from the last commanded pose so branch selection stays continuous.
    Hangar-gated: rover + ptz/none gets 400 and never touches USB.
    """
    import roarm_ctrl

    if not arm_usb_enabled():
        return jsonify({
            'ok': False,
            'error': 'arm move requires hangar attachment=roarm2',
            'roarm_started': False,
        }), 400

    body = request.get_json(silent=True) or {}
    try:
        dr = float(body.get('d_reach_mm') or 0.0)
        dz = float(body.get('d_lift_mm') or 0.0)
        dyaw = float(body.get('d_yaw_deg') or 0.0)
    except (TypeError, ValueError):
        return jsonify({'ok': False, 'error': 'deltas must be numbers'}), 400
    if not (math.isfinite(dr) and math.isfinite(dz) and math.isfinite(dyaw)):
        return jsonify({'ok': False, 'error': 'deltas must be finite'}), 400
    if max(abs(dr), abs(dz)) > 250.0:
        return jsonify({'ok': False, 'error': 'delta too large (max 250mm)'}), 400

    arm = get_roarm()
    if arm is None:
        _start_roarm(reason='api_arm_move')
        arm = get_roarm()
    if arm is None:
        return jsonify({'ok': False, 'error': 'RoArm USB unavailable'}), 503

    seed = getattr(arm, '_last_joints', None)
    if not isinstance(seed, dict) or not seed:
        seed = _twin_joints_snapshot()[0]

    res = roarm_ctrl.relative_move(dr_mm=dr, dz_mm=dz, dyaw_deg=dyaw, joints=seed)
    if not res.get('ok'):
        return jsonify(res), 400

    j = res['joints']
    ok, text = arm.set_joints(j['base'], j['shoulder'], j['elbow'], j['hand'])

    # Mirror to ROS viz when a bridge is up (same as the T:144 path).
    ros_mirror = None
    try:
        import ros_motion
        if get_control_mode() == 'ros2' or ros_motion.rosbridge_status().get('ok'):
            ros_mirror = ros_motion.publish_roarm_joints(
                j['base'], j['shoulder'], j['elbow'], j['hand'], throttle=True,
            )
    except Exception:
        ros_mirror = None

    resp = {
        'ok': bool(ok),
        'text': text[:200],
        'clamped': bool(res.get('clamped')),
        'target_r_mm': round(res['target_r_mm'], 1),
        'target_z_mm': round(res['target_z_mm'], 1),
        'achieved_r_mm': round(res['achieved_r_mm'], 1),
        'achieved_z_mm': round(res['achieved_z_mm'], 1),
        'joints': {k: round(v, 5) for k, v in j.items()},
        'roarm_started': True,
    }
    if ros_mirror:
        resp['ros_mirror_ok'] = bool(ros_mirror.get('ok'))
    return jsonify(resp)


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


def _chassis_driver_ps():
    """ps lines for chassis ROS nodes (not rosbridge, not RoArm)."""
    _ok, so, se, _ = _docker_exec(
        "ps -eo pid,args 2>/dev/null | grep -E 'ugv_bringup|ugv_driver_min' || true",
        detach=False, timeout=15,
    )
    return so, se


def _parse_chassis_driver_pids(ps_text):
    try:
        from ros_motion import parse_chassis_driver_pids
        return parse_chassis_driver_pids(ps_text)
    except Exception:
        from ros_motion import parse_ugv_bringup_pids
        return parse_ugv_bringup_pids(ps_text)


def _host_ugv_driver_min_path():
    return os.path.join(thisPath, 'ros2', 'ugv_driver_min.py')


def _stage_ugv_driver_min_in_container():
    """Return container path to ugv_driver_min.py, docker-cp from host if needed."""
    _ok, so, _se, _ = _docker_exec(
        'if [ -f /opt/ugv_ros2/ugv_driver_min.py ]; then echo /opt/ugv_ros2/ugv_driver_min.py; '
        'elif [ -f /tmp/ugv_driver_min.py ]; then echo /tmp/ugv_driver_min.py; fi',
        detach=False, timeout=15,
    )
    path = ''
    if so:
        for line in so.splitlines():
            line = line.strip()
            if line.endswith('ugv_driver_min.py'):
                path = line
                break
    if path:
        return path
    host = _host_ugv_driver_min_path()
    if not os.path.isfile(host):
        return ''
    import subprocess
    dest = f'{_ros_container_name()}:/tmp/ugv_driver_min.py'
    try:
        cp = subprocess.run(
            ['docker', 'cp', host, dest],
            capture_output=True, text=True, timeout=20,
        )
    except Exception:
        return ''
    if cp.returncode == 0:
        return '/tmp/ugv_driver_min.py'
    return ''


def _probe_chassis_driver_kind():
    """What this ROS image can run. Prefer beast ugv_driver_min; never require ugv_ws.

    Env: UGV_CHASSIS_DRIVER=auto|ugv_driver_min|ugv_bringup
    Positive detections are cached; 'none' is not.
    """
    cached = getattr(_probe_chassis_driver_kind, '_kind', None)
    if cached:
        return cached
    env = (os.environ.get('UGV_CHASSIS_DRIVER') or 'auto').strip().lower()
    if env in ('ugv_driver_min', 'driver_min', 'min'):
        _probe_chassis_driver_kind._kind = 'ugv_driver_min'  # type: ignore[attr-defined]
        return 'ugv_driver_min'
    if env in ('ugv_bringup', 'bringup'):
        _probe_chassis_driver_kind._kind = 'ugv_bringup'  # type: ignore[attr-defined]
        return 'ugv_bringup'

    if _stage_ugv_driver_min_in_container():
        _probe_chassis_driver_kind._kind = 'ugv_driver_min'  # type: ignore[attr-defined]
        return 'ugv_driver_min'

    _ok, so, _se, _ = _docker_exec(
        'source /opt/ros/humble/setup.bash >/dev/null 2>&1; '
        'if ros2 pkg prefix ugv_bringup >/dev/null 2>&1; then echo bringup; else echo none; fi',
        detach=False, timeout=20,
    )
    kind = (so or '').strip().splitlines()[-1] if so else ''
    if kind == 'bringup':
        _probe_chassis_driver_kind._kind = 'ugv_bringup'  # type: ignore[attr-defined]
        return 'ugv_bringup'
    if os.path.isfile(_host_ugv_driver_min_path()):
        return 'ugv_driver_min'
    return 'none'


def _ensure_ugv_bringup_running():
    """Start the chassis ROS node so /cmd_vel + /joint_states reach ESP32.

    Prefers this tree's ugv_driver_min (compose mount). Falls back to
    ugv_bringup only if that package is actually in the image.
    Does not start RoArm drivers. Does not require ugv_ws.

    Env: UGV_AUTOSTART_BRINGUP=1 (default on). Requires UART released (ros2 mode).
    """
    out = {
        'wanted': True,
        'already_up': False,
        'started': False,
        'ok': False,
        'detail': '',
        'kind': None,
    }
    if not _env_flag('UGV_AUTOSTART_BRINGUP', '1'):
        out['wanted'] = False
        out['detail'] = 'UGV_AUTOSTART_BRINGUP disabled'
        return out

    so, _se = _chassis_driver_ps()
    pids = _parse_chassis_driver_pids(so)
    if pids:
        kind = 'ugv_driver_min' if so and 'ugv_driver_min' in so else 'ugv_bringup'
        out['already_up'] = True
        out['ok'] = True
        out['kind'] = kind
        out['detail'] = f'{kind} already running pids={pids}'
        return out

    kind = _probe_chassis_driver_kind()
    out['kind'] = kind
    port = (
        os.environ.get('UGV_SERIAL_PORT')
        or os.environ.get('UGV_SERIAL_DEV')
        or '/dev/ttyAMA0'
    ).strip()
    if not port or not all(c.isalnum() or c in '/._-' for c in port):
        port = '/dev/ttyAMA0'

    if kind == 'ugv_driver_min':
        script = _stage_ugv_driver_min_in_container()
        if (
            not script
            or not script.endswith('ugv_driver_min.py')
            or not all(c.isalnum() or c in '/._-' for c in script)
        ):
            out['detail'] = 'ugv_driver_min.py not in container and docker cp failed'
            olog.warn('ros_autostart', out['detail'], component='chassis')
            return out
        start_script = f'''
set +e
source /opt/ros/humble/setup.bash
mkdir -p /tmp/ugv_ros_logs
export UGV_SERIAL_DEV={port}
export UGV_SERIAL_PORT={port}
nohup python3 {script} \
  > /tmp/ugv_ros_logs/ugv_driver_min.log 2>&1 &
sleep 0.5
if pgrep -af ugv_driver_min | grep -v pgrep | grep -q ugv_driver_min; then
  echo driver_ok
  exit 0
fi
echo driver_start_uncertain
tail -8 /tmp/ugv_ros_logs/ugv_driver_min.log 2>/dev/null || true
exit 0
'''
        ok_st, so2, se2, code = _docker_exec(start_script, detach=False, timeout=40)
        out['started'] = 'driver_ok' in (so2 or '')
        out['ok'] = out['started']
        out['detail'] = so2 or se2 or f'exit {code}'
        if out['ok']:
            olog.info(
                'ros_autostart', 'ugv_driver_min started in container',
                component='chassis', kind='ugv_driver_min', started=True,
            )
        else:
            out['started'] = bool(ok_st)
            out['ok'] = bool(ok_st) and 'driver_ok' in (so2 or '')
            olog.info(
                'ros_autostart',
                f'ugv_driver_min start attempted: {out["detail"][:200]}',
                component='chassis', kind='ugv_driver_min', ok=out['ok'],
            )
        return out

    if kind == 'ugv_bringup':
        start_script = f'''
set +e
source /opt/ros/humble/setup.bash
source /home/ws/ugv_ws/install/setup.bash 2>/dev/null || true
mkdir -p /tmp/ugv_ros_logs
nohup ros2 run ugv_bringup ugv_bringup --ros-args \
  -p serial_port:={port} -p baud_rate:=115200 \
  > /tmp/ugv_ros_logs/bringup.log 2>&1 &
sleep 0.4
if pgrep -f '/lib/ugv_bringup/ugv_bringup' >/dev/null 2>&1; then
  echo bringup_ok
  exit 0
fi
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
        out['ok'] = out['started']
        out['detail'] = so2 or se2 or f'exit {code}'
        if out['ok']:
            olog.info('ros_autostart', 'ugv_bringup started in container',
                      component='chassis', kind='ugv_bringup', started=True)
        else:
            out['started'] = bool(ok_st)
            out['ok'] = bool(ok_st)
            olog.info(
                'ros_autostart',
                f'ugv_bringup start attempted: {out["detail"][:200]}',
                component='chassis', kind='ugv_bringup', ok=out['ok'],
            )
        return out

    out['detail'] = (
        'no chassis ROS node in this image '
        '(need /opt/ugv_ros2/ugv_driver_min.py or ugv_bringup; full workspace image not required)'
    )
    olog.warn('ros_autostart', out['detail'], component='chassis')
    return out


def _stop_ugv_bringup():
    """Stop chassis ROS node in the ROS container so Flask can reclaim UART.

    Stops ugv_driver_min and/or ugv_bringup. Does not stop rosbridge or RoArm.
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

    so, se = _chassis_driver_ps()
    pids = _parse_chassis_driver_pids(so)
    out['pids'] = pids
    if not pids:
        out['already_down'] = True
        out['ok'] = True
        if se and ('docker CLI not found' in se or 'No such container' in se
                   or 'Cannot connect' in se):
            out['detail'] = f'chassis stop skipped: {se[:160]}'
        else:
            out['detail'] = 'chassis driver not running'
        return out

    pid_list = ' '.join(str(p) for p in pids)
    kill_script = (
        'set +e\n'
        f'kill -TERM {pid_list} 2>/dev/null || true\n'
        'sleep 0.4\n'
        f'kill -KILL {pid_list} 2>/dev/null || true\n'
        'sleep 0.15\n'
        "ps -eo pid,args 2>/dev/null | grep -E 'ugv_bringup|ugv_driver_min' || true\n"
    )
    _ok_st, so2, se2, _code = _docker_exec(kill_script, detach=False, timeout=20)
    leftover = _parse_chassis_driver_pids(so2)
    out['stopped'] = not leftover
    out['ok'] = not leftover
    if leftover:
        out['detail'] = f'still running after kill: {leftover}'
        olog.warn(
            'ros_autostop',
            out['detail'][:240],
            component='chassis', leftover=leftover, pids=pids,
        )
    else:
        out['detail'] = f'stopped pids {pids}'
        olog.info(
            'ros_autostop',
            'chassis ROS node stopped in container (rosbridge left up)',
            component='chassis', stopped=True, pids=pids,
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


# Hangar-gated RoArm: start only when attachment=roarm2 (rover+ptz stays off).
try:
    _sync_roarm_to_loadout(reason='startup')
except Exception as e:
    olog.warn('roarm', f'startup RoArm sync failed: {e}', error=str(e)[:160])

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
    # Preload rosbridge only. Bringup stays down so Flask keeps ttyAMA0.
    try:
        _rb = _ensure_rosbridge_running()
        olog.info(
            'ros_preload',
            'startup Direct: rosbridge preload (no bringup)',
            ok=_rb.get('ok'), already_up=_rb.get('already_up'),
            started=_rb.get('started'), detail=str(_rb.get('detail') or '')[:160],
        )
    except Exception as e:
        olog.warn('ros_preload', f'startup Direct rosbridge preload failed: {e}', error=str(e)[:200])


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


def _jpeg_from_client_payload(data):
    """Decode a still the UI already grabbed (data URL or raw base64). None if absent."""
    if not isinstance(data, dict):
        return None
    url = data.get('snapshot_data_url') or data.get('data_url')
    if isinstance(url, str) and url.startswith('data:image') and ',' in url:
        try:
            raw = base64.b64decode(url.split(',', 1)[1])
            if raw:
                return raw
        except Exception:
            pass
    b64 = data.get('image_base64')
    if isinstance(b64, str) and b64.strip():
        try:
            raw = base64.b64decode(b64)
            if raw:
                return raw
        except Exception:
            pass
    return None


_AI_SYSTEM_PROMPT = (
    "You are a vision-capable scout on a Waveshare UGV rover with a forward camera. "
    "When an image is attached, describe what you see clearly and briefly. "
    "Only use tools listed as callable this turn. Do not claim you can drive, pan, "
    "or detect objects unless those tools are listed. "
    "When motion tools are callable: prefer several short timed hops "
    "(duration_ms 800–1600, linear_x ~0.22–0.28) over one long drive, and call stop_motors when done."
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
            {
                'name': 'get_robot_context',
                'label': 'get_robot_context',
                'kind': 'tool',
                'description': 'Robot hardware config: hostname, loadout (base/attachment), camera type/status, lidar, UART, ROS, Hailo, arm status.',
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
                'needs_loadout_attachment': 'ptz',
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
    {
        'name': 'group_roarm',
        'label': 'RoArm-M2',
        'kind': 'group',
        'description': 'USB RoArm control (hangar attachment=roarm2 only).',
        'needs_loadout_attachment': 'roarm2',
        'children': [
            {
                'name': 'send_roarm_command',
                'label': 'send_roarm_command',
                'kind': 'tool',
                'description': 'Control USB RoArm joints or named poses (travel_tuck/scan_ready/home).',
                'needs_loadout_attachment': 'roarm2',
            },
        ],
    },
]

_MOTION_TOOLS = frozenset({'send_motor_command', 'send_gimbal_command', 'stop_motors'})
_ROARM_TOOLS = frozenset({'send_roarm_command'})

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
# RoArm tools stay on (USB, independent of chassis motion infra).
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
        out['get_robot_context'] = bool(data.get('get_robot_context', v))
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
    needs_attachment = meta.get('needs_loadout_attachment')
    
    if not user_on:
        return 'off', 'Toggled off — not offered to the LLM.', user_on
    
    # Check loadout attachment requirement (e.g. ptz, roarm2)
    if needs_attachment:
        try:
            current_attach = _loadout_store.get().get('attachment')
            if current_attach != needs_attachment:
                label = {'ptz': 'PTZ gimbal', 'roarm2': 'RoArm-M2', 'none': 'no attachment'}.get(needs_attachment, needs_attachment)
                return 'unavailable', (
                    f'Requires hangar attachment={needs_attachment} ({label}); '
                    f'current attachment is {current_attach}. Toggle on Loadout tab.'
                ), user_on
        except Exception as e:
            return 'unavailable', f'Loadout check failed: {e}', user_on
    
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
    if not isinstance(msg, dict):
        return ''
    content = msg.get('content')
    if isinstance(content, str):
        text = content.strip()
    elif isinstance(content, list):
        parts = []
        for part in content:
            if isinstance(part, str):
                parts.append(part)
            elif isinstance(part, dict) and part.get('type') == 'text':
                parts.append(str(part.get('text') or ''))
        text = '\n'.join(parts).strip()
    elif isinstance(content, dict):
        text = ''
    else:
        text = ''
    if not text:
        extra = msg.get('reasoning') or msg.get('reasoning_content') or ''
        text = extra.strip() if isinstance(extra, str) else ''
    return text


def _get_robot_context_payload():
    """Robot hardware config and settings for chat AI context."""
    import socket
    hostname = socket.gethostname()
    lo = _loadout_store.get()
    base_id = lo.get('base', 'rover')
    attachment_id = lo.get('attachment', 'ptz')
    use_lidar = bool(lo.get('use_lidar', False))
    camera_prefer = lo.get('camera_prefer', 'auto')
    
    # Camera status
    camera_status = 'unknown'
    camera_type = 'none'
    if cvf.usb_camera_connected:
        camera_type = 'usb'
        camera_status = 'connected'
        if cvf.usb_camera_index is not None:
            camera_status += f' (index {cvf.usb_camera_index})'
    elif cvf.csi_camera_connected:
        camera_type = 'csi'
        camera_status = 'connected'
    elif cvf.oak_camera_connected:
        camera_type = 'oak'
        camera_status = 'connected'
    else:
        camera_type = 'none'
        camera_status = 'disconnected'
    
    # Control mode / UART owner
    control_mode = get_control_mode()
    uart_owner = 'flask' if control_mode == 'direct' else 'ros2'
    
    # ROS / rosbridge
    rosbridge_ok = None
    if control_mode == 'ros2':
        rosbridge_ok = _rosbridge_reachable()
    
    # Hailo detection
    hailo_present = bool(os.path.exists('/dev/hailo0'))
    
    # Arm status
    arm_status = 'no_arm'
    if attachment_id == 'roarm2':
        if roarm_started():
            arm_status = 'roarm_started'
        else:
            arm_status = 'roarm_not_started'
    
    # Battery voltage if available
    voltage = None
    try:
        voltage = round(base.get_base_adc(), 2) if callable(getattr(base, 'get_base_adc', None)) else None
    except Exception:
        pass
    
    return {
        'hostname': hostname,
        'loadout': {
            'base': base_id,
            'attachment': attachment_id,
            'use_lidar': use_lidar,
            'camera_prefer': camera_prefer,
        },
        'camera': {
            'type': camera_type,
            'status': camera_status,
        },
        'control_mode': control_mode,
        'uart_owner': uart_owner,
        'rosbridge_ok': rosbridge_ok,
        'hailo_present': hailo_present,
        'arm_status': arm_status,
        'battery_voltage': voltage,
        'lidar': _lidar_public(),
    }


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
    try:
        data['lidar'] = _lidar_public()
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
    if by_name.get('get_robot_context', {}).get('status') == 'active':
        tools.append({
            'type': 'function',
            'function': {
                'name': 'get_robot_context',
                'description': 'Read robot hardware config and settings (hostname, loadout base/attachment, camera type/status, lidar, UART owner, ROS status, Hailo, arm status, battery voltage). Use this to understand the robot configuration.',
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
                    'Capture a live camera JPEG and attach it for vision analysis on the next turn. '
                    'Use this when the user asks what you see and no still was attached.'
                ),
                'parameters': {'type': 'object', 'properties': {}},
            },
        })
    # RoArm tool (USB, independent of rosbridge; gated on attachment=roarm2)
    if by_name.get('send_roarm_command', {}).get('status') == 'active':
        tools.append({
            'type': 'function',
            'function': {
                'name': 'send_roarm_command',
                'description': (
                    'Control the USB RoArm-M2 by named pose or individual joint angles. '
                    'Named poses: travel_tuck (default compact stance for mobile use), '
                    'scan_ready (slightly open for looking ahead), home (workspace-ready inverted L). '
                    'Joint angles are in radians. Use this to look around, adjust arm position, '
                    'or transition to a safe posture before moving the chassis.'
                ),
                'parameters': {
                    'type': 'object',
                    'properties': {
                        'pose': {
                            'type': 'string',
                            'description': (
                                'Named pose: travel_tuck | scan_ready | home | elbow_in. '
                                'Omit if providing joint angles directly.'
                            ),
                        },
                        'base_rad': {
                            'type': 'number',
                            'description': 'Base yaw angle (radians, ±1.2). Omit to keep current.',
                        },
                        'shoulder_rad': {
                            'type': 'number',
                            'description': 'Shoulder angle (radians, ±0.9). Omit to keep current.',
                        },
                        'elbow_rad': {
                            'type': 'number',
                            'description': 'Elbow angle (radians, 0.85–2.2). Omit to keep current.',
                        },
                        'hand_rad': {
                            'type': 'number',
                            'description': 'Hand/wrist angle (radians, 1.8–3.2). Omit to keep current.',
                        },
                    },
                },
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
    seek_override = _seek_tools_overridden() and name in (
        'send_motor_command', 'stop_motors', 'send_gimbal_command',
        'get_cv_detections', 'get_camera_snapshot',
    )
    if entry and entry.get('status') != 'active' and not seek_override:
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

    if name == 'get_robot_context':
        return {'ok': True, 'context': _get_robot_context_payload()}

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
                'found_goal': bool(
                    filter_label
                    and any(l.lower() == str(filter_label).lower() for l in labels)
                ),
            }
            if warning:
                out['warning'] = warning
            return out
        except Exception as e:
            return {'ok': False, 'error': str(e), 'tool': name}

    if name == 'get_camera_snapshot':
        try:
            jpeg = _grab_jpeg_bytes()
            b64 = base64.b64encode(jpeg).decode('ascii')
            data_url = f'data:image/jpeg;base64,{b64}'
            return {
                'ok': True,
                'mime': 'image/jpeg',
                'bytes': len(jpeg),
                'note': 'Frame captured and attached for vision analysis.',
                '_snapshot_data_url': data_url,  # Internal field for agent loop
            }
        except Exception as e:
            return {'ok': False, 'error': str(e), 'tool': name}

    if name == 'send_roarm_command':
        try:
            if not arm_usb_enabled():
                return {
                    'ok': False,
                    'error': 'USB RoArm not available (hangar attachment is not roarm2)',
                    'tool': name,
                }
            arm = get_roarm()
            if arm is None:
                return {
                    'ok': False,
                    'error': 'USB RoArm driver not started',
                    'tool': name,
                    'detail': 'RoArm USB path may be owned by ROS container or missing',
                }
            import roarm_ctrl
            pose_name = args.get('pose')
            if pose_name:
                pose = roarm_ctrl.POSES.get(str(pose_name).strip().lower())
                if not pose:
                    return {
                        'ok': False,
                        'error': f'unknown pose: {pose_name}',
                        'available_poses': list(roarm_ctrl.POSES.keys()),
                    }
                ok_set, text_set = arm.set_joints(
                    float(pose['base']),
                    float(pose['shoulder']),
                    float(pose['elbow']),
                    float(pose['hand']),
                    spd=0, acc=10,
                )
                return {
                    'ok': bool(ok_set),
                    'text': text_set,
                    'pose': pose_name,
                    'joints': pose,
                    'path': 'roarm_usb',
                }
            base = args.get('base_rad')
            shoulder = args.get('shoulder_rad')
            elbow = args.get('elbow_rad')
            hand = args.get('hand_rad')
            if base is None and shoulder is None and elbow is None and hand is None:
                return {
                    'ok': False,
                    'error': 'must provide pose or at least one joint angle (base_rad/shoulder_rad/elbow_rad/hand_rad)',
                }
            current = arm.status().get('joints') or roarm_ctrl.POSES['travel_tuck']
            target = {
                'base': float(base) if base is not None else float(current.get('base', 0.0)),
                'shoulder': float(shoulder) if shoulder is not None else float(current.get('shoulder', -0.62)),
                'elbow': float(elbow) if elbow is not None else float(current.get('elbow', 0.88)),
                'hand': float(hand) if hand is not None else float(current.get('hand', 3.05)),
            }
            ok_set, text_set = arm.set_joints(
                target['base'], target['shoulder'], target['elbow'], target['hand'],
                spd=0, acc=10,
            )
            return {
                'ok': bool(ok_set),
                'text': text_set,
                'joints': target,
                'path': 'roarm_usb',
            }
        except Exception as e:
            return {'ok': False, 'error': str(e), 'tool': name}

    if name in ('send_motor_command', 'stop_motors', 'send_gimbal_command'):
        try:
            return _execute_motion_via_mode(name, args)
        except Exception as e:
            return {'ok': False, 'error': str(e), 'tool': name}

    return {'ok': False, 'error': f'unmapped tool: {name}'}


def _seek_drive_scope(active=True):
    """Mark this thread as the Seek executor so Chat/UI cannot sneak a hop."""
    return _seek_nav_drive_scope(active)


def _seek_thread_may_drive() -> bool:
    from seek_nav import seek_thread_may_drive as _may
    return _may()


def _autonomy_owns_chassis() -> bool:
    """Seek or Track is running — sticks / Chat must not fight it."""
    try:
        if seek_controller.is_running():
            return True
    except Exception:
        pass
    try:
        if track_controller.is_running():
            return True
    except Exception:
        pass
    return False


def _execute_motion_via_mode(name, args):
    """AI motion tools follow the same control_mode as UI sticks."""
    mode = get_control_mode()
    args = args or {}
    level = 'warn' if name == 'stop_motors' else 'info'
    if name == 'send_motor_command' and _autonomy_owns_chassis() and not _seek_thread_may_drive():
        olog.info(
            'ai_motion',
            'Chassis busy: Seek/Track owns the wheels (Chat hop ignored)',
            tool=name, control_mode=mode,
        )
        return {
            'ok': True,
            'skipped': 'seek_or_track_running',
            'backend': 'none',
            'path': 'blocked',
            'control_mode': mode,
        }
    if name == 'send_motor_command' and not _seek_chassis_allowed():
        olog.info(
            'ai_motion',
            'Seek dry-run: send_motor_command skipped (no chassis)',
            tool=name, dry_run=True, control_mode=mode,
        )
        return {
            'ok': True,
            'dry_run': True,
            'skipped': 'seek_dry_run',
            'backend': 'none',
            'path': 'dry_run',
            'control_mode': mode,
        }

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
        if not _seek_chassis_allowed():
            olog.info(
                'ai_motion',
                'Seek dry-run: send_motor_command skipped (no chassis)',
                tool=name, dry_run=True,
            )
            return {
                'ok': True,
                'dry_run': True,
                'skipped': 'seek_dry_run',
                'backend': 'none',
                'path': 'dry_run',
            }
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
                snapshot_url = result.get('_snapshot_data_url') if isinstance(result, dict) else None
                result_for_model = (
                    {k: v for k, v in result.items() if not str(k).startswith('_')}
                    if isinstance(result, dict) else result
                )
                tool_trace.append({'name': name, 'arguments': args, 'result': result_for_model})
                # Check if tool returned a snapshot for vision injection
                messages.append({
                    'role': 'tool',
                    'tool_call_id': tc.get('id') or name,
                    'content': json.dumps(result_for_model),
                })
                # Inject vision snapshot as a user message after tool result
                if snapshot_url:
                    messages.append({
                        'role': 'user',
                        'content': [
                            {'type': 'text', 'text': 'Here is the camera snapshot you requested. Analyze what you see.'},
                            {'type': 'image_url', 'image_url': {'url': snapshot_url}},
                        ],
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
            jpeg = _jpeg_from_client_payload(data)
            if jpeg is None:
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
    attach = bool(data['attach_snapshot']) if 'attach_snapshot' in data else False

    # Enrich system prompt with control path status and robot hardware/settings context
    mode = get_control_mode()
    backend, bridge = _motion_backend_info()
    system = _AI_SYSTEM_PROMPT
    
    # Add robot hardware/settings context at conversation start
    try:
        robot_ctx = _get_robot_context_payload()
        system += (
            f"\n\nRobot configuration: hostname={robot_ctx['hostname']}, "
            f"base={robot_ctx['loadout']['base']}, "
            f"attachment={robot_ctx['loadout']['attachment']}, "
            f"camera={robot_ctx['camera']['type']} ({robot_ctx['camera']['status']}), "
            f"lidar={'enabled' if robot_ctx['loadout']['use_lidar'] else 'disabled'}, "
            f"arm_status={robot_ctx['arm_status']}"
        )
        if robot_ctx.get('battery_voltage'):
            system += f", battery={robot_ctx['battery_voltage']}V"
        if robot_ctx.get('hailo_present'):
            system += ", Hailo AI accelerator present"
        system += ". "
    except Exception as e:
        print(f"[api_ai_chat] Failed to add robot context: {e}")
    
    active = [t['name'] for t in _ai_tools_catalog() if t.get('status') == 'active']
    inactive = [t['name'] for t in _ai_tools_catalog() if t.get('status') != 'active']
    system += (
        f"Control mode: {mode} ({'ESP32 serial' if mode == 'direct' else 'ROS 2 / rosbridge'}). "
    )
    if active:
        system += f"Callable tools: {', '.join(active)}. Use these tools when appropriate. "
    else:
        system += "No tools currently available. "
    
    # Specific guidance for active capabilities
    has_chassis = any(n in active for n in ('send_motor_command', 'stop_motors'))
    has_gimbal = 'send_gimbal_command' in active
    has_roarm = 'send_roarm_command' in active
    
    if has_chassis:
        system += (
            "Chassis control available: prefer punchy timed moves (duration_ms 800–1600, linear_x 0.22–0.28). "
            "After each drive, re-check with get_cv_detections before moving again. "
            "Call stop_motors if unsure or when done. "
        )
    
    if has_gimbal:
        system += "PTZ gimbal available: use send_gimbal_command to look around. "
    
    if has_roarm:
        system += (
            "USB RoArm available: use send_roarm_command to adjust arm pose (travel_tuck for compact, "
            "scan_ready for looking ahead) or move individual joints. "
        )
    
    # Only mention unavailable tools if motion is blocked but user asked for it
    if not has_chassis and any(n in inactive for n in ('send_motor_command', 'stop_motors')):
        system += (
            "Chassis motion tools are currently disabled. Enable them on /ai before asking to drive. "
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
            jpeg = _jpeg_from_client_payload(data)
            if jpeg is None:
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


# ---------- Chat voice: STT/TTS proxy to external LAN service (env-configured) ----------

def _get_voice_config():
    """Read STT/TTS URLs from .env only. Empty = voice disabled."""
    return {
        'stt_url': (os.environ.get('UGV_STT_URL') or '').strip(),
        'tts_url': (os.environ.get('UGV_TTS_URL') or '').strip(),
    }


@app.route('/api/voice/config', methods=['GET'])
def api_voice_config():
    """Return voice configuration status (URLs present or not, no actual URLs exposed)."""
    cfg = _get_voice_config()
    return jsonify({
        'success': True,
        'stt_enabled': bool(cfg['stt_url']),
        'tts_enabled': bool(cfg['tts_url']),
    })


@app.route('/api/voice/stt', methods=['POST'])
def api_voice_stt():
    """Proxy browser audio → external STT service (UGV_STT_URL from .env)."""
    cfg = _get_voice_config()
    if not cfg['stt_url']:
        return jsonify({'success': False, 'error': 'STT not configured (set UGV_STT_URL in .env)'}), 503
    
    # Forward audio from browser to STT service
    try:
        audio_data = request.data or request.get_data()
        if not audio_data:
            return jsonify({'success': False, 'error': 'No audio data'}), 400
        
        # Forward to STT service
        req = urllib.request.Request(
            cfg['stt_url'],
            data=audio_data,
            headers={'Content-Type': request.content_type or 'audio/webm'},
            method='POST',
        )
        with urllib.request.urlopen(req, timeout=30) as resp:
            result = json.loads(resp.read().decode('utf-8'))
        
        text = result.get('text', '').strip()
        return jsonify({
            'success': True,
            'text': text,
            'language': result.get('language'),
        })
    except urllib.error.HTTPError as e:
        error_body = e.read().decode('utf-8', errors='ignore')[:200]
        return jsonify({'success': False, 'error': f'STT service error: {e.code} {error_body}'}), 502
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)[:200]}), 500


@app.route('/api/voice/tts', methods=['POST'])
def api_voice_tts():
    """Proxy text → external TTS service (UGV_TTS_URL from .env) → return audio."""
    cfg = _get_voice_config()
    if not cfg['tts_url']:
        return jsonify({'success': False, 'error': 'TTS not configured (set UGV_TTS_URL in .env)'}), 503
    
    data = request.get_json(silent=True) or {}
    text = (data.get('text') or '').strip()
    if not text:
        return jsonify({'success': False, 'error': 'No text provided'}), 400
    
    try:
        # Forward to TTS service
        req = urllib.request.Request(
            cfg['tts_url'],
            data=json.dumps({'text': text}).encode('utf-8'),
            headers={'Content-Type': 'application/json'},
            method='POST',
        )
        with urllib.request.urlopen(req, timeout=30) as resp:
            audio_data = resp.read()
            content_type = resp.headers.get('Content-Type', 'audio/wav')
        
        return Response(audio_data, mimetype=content_type)
    except urllib.error.HTTPError as e:
        error_body = e.read().decode('utf-8', errors='ignore')[:200]
        return jsonify({'success': False, 'error': f'TTS service error: {e.code} {error_body}'}), 502
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)[:200]}), 500


@app.route('/api/voice/robot/record', methods=['POST'])
def api_voice_robot_record():
    """Capture audio from robot's mic, send to STT service, return transcript."""
    cfg = _get_voice_config()
    if not cfg['stt_url']:
        return jsonify({'success': False, 'error': 'STT not configured (set UGV_STT_URL in .env)'}), 503
    
    data = request.get_json(silent=True) or {}
    duration_s = float(data.get('duration', 5))
    duration_s = max(1, min(30, duration_s))  # Clamp 1-30 seconds
    
    try:
        # Record audio from robot's mic using arecord (ALSA)
        import tempfile
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as tmp:
            tmp_path = tmp.name
        
        # Record mono 16kHz WAV
        proc = subprocess.run(
            ['arecord', '-D', 'default', '-f', 'cd', '-d', str(int(duration_s)), tmp_path],
            capture_output=True,
            timeout=duration_s + 5,
            check=False,
        )
        
        if proc.returncode != 0 or not os.path.exists(tmp_path):
            try:
                os.unlink(tmp_path)
            except Exception:
                pass
            stderr = (proc.stderr or b'').decode('utf-8', errors='ignore')[:200]
            return jsonify({'success': False, 'error': f'arecord failed: {stderr}'}), 500
        
        # Read recorded audio
        with open(tmp_path, 'rb') as f:
            audio_data = f.read()
        os.unlink(tmp_path)
        
        # Forward to STT service
        req = urllib.request.Request(
            cfg['stt_url'],
            data=audio_data,
            headers={'Content-Type': 'audio/wav'},
            method='POST',
        )
        with urllib.request.urlopen(req, timeout=30) as resp:
            result = json.loads(resp.read().decode('utf-8'))
        
        text = result.get('text', '').strip()
        return jsonify({
            'success': True,
            'text': text,
            'language': result.get('language'),
        })
    except subprocess.TimeoutExpired:
        return jsonify({'success': False, 'error': 'Recording timeout'}), 500
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)[:200]}), 500


@app.route('/api/voice/robot/speak', methods=['POST'])
def api_voice_robot_speak():
    """Get TTS audio from service and play through robot's speakers."""
    cfg = _get_voice_config()
    if not cfg['tts_url']:
        return jsonify({'success': False, 'error': 'TTS not configured (set UGV_TTS_URL in .env)'}), 503
    
    data = request.get_json(silent=True) or {}
    text = (data.get('text') or '').strip()
    if not text:
        return jsonify({'success': False, 'error': 'No text provided'}), 400
    
    try:
        # Get TTS audio from service
        req = urllib.request.Request(
            cfg['tts_url'],
            data=json.dumps({'text': text}).encode('utf-8'),
            headers={'Content-Type': 'application/json'},
            method='POST',
        )
        with urllib.request.urlopen(req, timeout=30) as resp:
            audio_data = resp.read()
        
        # Save to temp file and play with aplay
        import tempfile
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as tmp:
            tmp.write(audio_data)
            tmp_path = tmp.name
        
        # Play audio through robot's speakers (non-blocking)
        subprocess.Popen(
            ['aplay', '-D', 'default', tmp_path],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        
        # Schedule cleanup after a delay (audio should finish playing)
        def cleanup_audio():
            time.sleep(max(2, len(text) // 10))  # Rough estimate: 10 chars per second
            try:
                os.unlink(tmp_path)
            except Exception:
                pass
        
        threading.Thread(target=cleanup_audio, daemon=True).start()
        
        return jsonify({'success': True, 'message': 'Playing audio on robot'})
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)[:200]}), 500


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
    DEFAULT_SEEK_DRY_RUN,
    DEFAULT_SEEK_FOUND_CONF,
    DEFAULT_SEEK_STEP_PAUSE_S,
    parse_forced_yesno,
    normalize_seek_max_steps,
    normalize_seek_timeout_s,
    motion_lock_should_ignore_zero,
    DEFAULT_ON_FOUND,
    DEFAULT_ON_FOUND_TTS,
)
from seek_nav import (  # noqa: E402
    seek_nav_plan as _seek_nav_plan,
    seek_normalize_action as _seek_normalize_action,
    interpret_base_voltage as _interpret_base_voltage,
    seek_battery_block_reason as _seek_battery_block_reason_pure,
    set_seek_dry_run as _set_seek_dry_run,
    seek_dry_run_active as _seek_dry_run_active,
    seek_chassis_allowed as _seek_chassis_allowed,
    seek_drive_log_verb as _seek_drive_log_verb,
    seek_sweep_scorecard as _seek_sweep_scorecard,
    seek_live_start_error as _seek_live_start_error,
    seek_views_are_rear_cruise as _seek_views_are_rear_cruise,
    seek_found_confident as _seek_found_confident,
    begin_seek_dry_run as _begin_seek_dry_run,
    end_seek_dry_run as _end_seek_dry_run,
    seek_drive_scope as _seek_nav_drive_scope,
    register_autonomy_running as _register_autonomy_running,
)

try:
    _register_autonomy_running(seek_controller.is_running)
    _register_autonomy_running(track_controller.is_running)
except Exception:
    pass

_SEEK_JUDGE_SYSTEM = (
    "You are a visual goal referee for a robot camera. "
    "Look only at the provided image. Decide if the described target is clearly visible. "
    "Reply with JSON only — no markdown, no extra keys beyond the schema. "
    "found must be a boolean: true only if the target is clearly present in the image; "
    "false if absent, uncertain, occluded, or too small to be sure. "
    "reason is one short sentence."
)

# Fraction of args_config max_speed*max_rate for seek spins (1.0 = UI Fast)
_SEEK_TURN_SPEED_SCALE = 1.0
# Nav-plan + hop tables live in seek_nav.py (unit-tested). Aliases imported above.


def _seek_side_openness(views):
    """Score cruise LEFT vs RIGHT panels.

    Those panels are camera ±135° (rear-left / rear-right), not port/starboard.
    Use them for reverse-quarter clearance, not as hallway walls.

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
    """Return turn_left/turn_right toward a more open *side* view, or None.

    Cruise LEFT/RIGHT are rear ±135° — not port/starboard. Do not use them
    as a turn hint (that made indoor Seek oscillate L/R).
    """
    if _seek_views_are_rear_cruise(views):
        return None
    info = _seek_side_openness(views)
    if not info.get('scores_ok'):
        return None
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

_SEEK_FRONT_NAV_SCHEMA = {
    'type': 'json_schema',
    'json_schema': {
        'name': 'front_nav',
        'strict': True,
        'schema': {
            'type': 'object',
            'properties': {
                'clear_forward_little': {
                    'type': 'string',
                    'enum': ['yes', 'no'],
                    'description': 'Can drive forward a little (short hop ~0.8s)?',
                },
                'clear_forward_lot': {
                    'type': 'string',
                    'enum': ['yes', 'no'],
                    'description': 'Can drive forward a lot (long hop ~1.6s)?',
                },
                'subject_in_scene': {
                    'type': 'string',
                    'enum': ['yes', 'no'],
                    'description': 'Is the target subject clearly visible in this front view?',
                },
            },
            'required': ['clear_forward_little', 'clear_forward_lot', 'subject_in_scene'],
            'additionalProperties': False,
        },
    },
}

_SEEK_SIDE_NAV_SCHEMA = {
    'type': 'json_schema',
    'json_schema': {
        'name': 'side_nav',
        'strict': True,
        'schema': {
            'type': 'object',
            'properties': {
                'direction_clear': {
                    'type': 'string',
                    'enum': ['yes', 'no'],
                    'description': 'Is this direction clear enough to rotate toward?',
                },
            },
            'required': ['direction_clear'],
            'additionalProperties': False,
        },
    },
}

_SEEK_MULTI_NAV_SCHEMA = {
    'type': 'json_schema',
    'json_schema': {
        'name': 'multi_view_nav',
        'strict': True,
        'schema': {
            'type': 'object',
            'properties': {
                'clear_forward_little': {'type': 'string', 'enum': ['yes', 'no']},
                'clear_forward_lot': {'type': 'string', 'enum': ['yes', 'no']},
                'subject_in_scene': {'type': 'string', 'enum': ['yes', 'no']},
                'left_direction_clear': {'type': 'string', 'enum': ['yes', 'no']},
                'right_direction_clear': {'type': 'string', 'enum': ['yes', 'no']},
            },
            'required': [
                'clear_forward_little', 'clear_forward_lot', 'subject_in_scene',
                'left_direction_clear', 'right_direction_clear',
            ],
            'additionalProperties': False,
        },
    },
}


def opencv_goal_check(goal_label, conf_threshold=DEFAULT_SEEK_CONF, frame=None):
    """On-device detector oracle (MobileNet-SSD). Closed class list only."""
    if frame is None:
        frame = cvf.grab_bgr_frame()
    dets = []
    if frame is not None:
        # Low floor for HUD/logs; found still uses conf_threshold (default 0.85).
        dets = cvf.detect_objects_structured(frame, conf_threshold=min(0.12, float(conf_threshold)))
    else:
        dets = list(getattr(cvf, 'last_detections', []) or [])

    filtered_dets = []
    for d in (dets or []):
        if not isinstance(d, dict) or not d.get('label'):
            continue
        try:
            c = float(d.get('confidence', 0) or 0)
        except (TypeError, ValueError):
            c = 0.0
        if c >= float(conf_threshold):
            filtered_dets.append(d)

    raw_labels = [
        f"{d.get('label')}: {round(float(d.get('confidence', 0))*100)}%"
        for d in filtered_dets if isinstance(d, dict) and d.get('label')
    ]
    olog.info(
        'ai_seek',
        f'CV scan for target "{goal_label}" (conf>={conf_threshold}) — saw {len(filtered_dets)} objects: {", ".join(raw_labels) or "none"}',
        goal=goal_label, conf_threshold=conf_threshold, raw_count=len(filtered_dets), detected_objects=raw_labels,
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


# Seek loop may drive even when Chat motion tools are off. Thread-local so
# we do not persist those caps into .ai_capabilities.json.
_seek_tool_override = threading.local()


def _seek_tools_overridden() -> bool:
    return bool(getattr(_seek_tool_override, 'active', False))


def _seek_force_tools_on():
    """Allow chassis/CV tools for this Seek thread only (does not persist)."""
    _seek_tool_override.active = True


def _seek_clear_tool_override():
    _seek_tool_override.active = False


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
_SEEK_PAN_TOL_DEG = 22.0       # HW often undershoots ±135°; 14° was too tight
_SEEK_PAN_WAIT_MAX_S = 3.5     # Allow time for full 135° slew + damp (was 2.5s, too tight)
_SEEK_PAN_POLL_S = 0.08
_SEEK_PAN_POST_ARRIVE_S = 0.12  # brief damp after arrival before shutter
_SEEK_PAN_FALLBACK_SLEEP_S = 1.5  # if no HW pan feedback at all (135° @ 54°/s = 2.5s + margin)
# Live cam_aim publish throttle (UI polls ~5Hz; wait loop is ~12Hz)
_SEEK_CAM_AIM_MIN_INTERVAL_S = 0.12
_SEEK_CAM_AIM_MIN_LIVE_DELTA_DEG = 3.0
# ESP32 pan feedback is often stuck near 0 while T:133 is mid-swing — estimate
# live angle so the UI needle moves (≈ 135° in ~2.5s on this gimbal = 54°/s).
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
    """Poll HW pan until near target_pan_deg OR estimated slew time elapsed.

    Returns dict: settled, hw_pan, err_deg, waited_s, samples, reason.
    
    Wait strategy (ESP32 often lies with ~0° during ±135° swing):
    1. Calculate estimated slew time: |target - last_cmd| / 54°/s + 0.4s damp
    2. Wait until EITHER:
       - HW feedback is within tolerance of target (and not stuck at ~0), OR
       - Estimated slew time has fully elapsed
    3. Never grab before estimated time if HW is stuck near 0 while |target| > 40
    """
    tol = float(tol_deg if tol_deg is not None else _SEEK_PAN_TOL_DEG)
    max_s = float(max_s if max_s is not None else _SEEK_PAN_WAIT_MAX_S)
    target = float(target_pan_deg)
    
    # Calculate estimated slew time based on commanded travel distance
    last_cmd = float(_cam_aim_pub.get('cmd') or 0.0)
    pan_travel = abs(target - last_cmd)
    estimated_slew = (pan_travel / float(_SEEK_PAN_EST_DPS)) + 0.4  # travel + damp
    min_wait = max(0.6, estimated_slew)  # At least 0.6s for any move
    
    # For large moves (rear angles), ensure minimum settle time
    if abs(target) >= 90:
        min_wait = max(1.0, min_wait)
    
    # Wait at least the estimated time, up to max_s
    required_wait = min(min_wait, max_s)
    
    t0 = time.time()
    samples = 0
    last_pan = None
    saw_feedback = False
    saw_useful_feedback = False
    hw_within_tol = False
    
    while (time.time() - t0) < max_s:
        elapsed = time.time() - t0
        
        if should_stop and should_stop():
            return {
                'settled': False,
                'hw_pan': last_pan,
                'err_deg': None,
                'waited_s': round(elapsed, 3),
                'samples': samples,
                'reason': 'stop_requested',
            }
        
        pan, _tilt, _snap = _seek_read_hw_pan_tilt(request_feedback=True)
        samples += 1
        
        if pan is not None:
            saw_feedback = True
            last_pan = float(pan)
            
            # Detect stuck HW: reports ~0° while commanded far (±40+)
            hw_stuck = (abs(last_pan) < 12.0 and abs(target) > 40.0)
            
            if not hw_stuck:
                saw_useful_feedback = True
                err = abs(last_pan - target)
                if err <= tol:
                    hw_within_tol = True
                try:
                    _seek_publish_cam_aim(
                        target, float(_cam_aim_pub.get('tilt') or 0.0),
                        hw_pan=last_pan, settled=False, live_pan=last_pan,
                    )
                except Exception:
                    pass
            else:
                # HW stuck - publish estimated position instead
                try:
                    _seek_publish_cam_aim(
                        target, float(_cam_aim_pub.get('tilt') or 0.0), settled=False,
                    )
                except Exception:
                    pass
        else:
            # No HW feedback yet
            try:
                _seek_publish_cam_aim(
                    target, float(_cam_aim_pub.get('tilt') or 0.0), settled=False,
                )
            except Exception:
                pass
        
        # Success criteria: HW within tolerance AND minimum time elapsed
        if hw_within_tol and elapsed >= required_wait:
            time.sleep(_SEEK_PAN_POST_ARRIVE_S)
            return {
                'settled': True,
                'hw_pan': last_pan,
                'err_deg': round(abs(last_pan - target), 2) if last_pan else None,
                'waited_s': round(time.time() - t0, 3),
                'samples': samples,
                'reason': 'hw_arrived_and_time_elapsed',
            }
        
        # If HW is useful and within tolerance, but min time not yet elapsed, keep waiting
        if hw_within_tol and elapsed < required_wait:
            time.sleep(_SEEK_PAN_POLL_S)
            continue
        
        # If estimated time has elapsed, we're done (even if HW is lying)
        if elapsed >= required_wait:
            time.sleep(_SEEK_PAN_POST_ARRIVE_S)
            reason = 'estimated_slew_elapsed'
            if saw_useful_feedback and hw_within_tol:
                reason = 'hw_arrived_and_time_elapsed'
            elif saw_feedback and not saw_useful_feedback:
                reason = 'time_elapsed_hw_stuck_at_zero'
            return {
                'settled': True,  # Gimbal had time to move
                'hw_pan': last_pan,
                'err_deg': round(abs(last_pan - target), 2) if last_pan else None,
                'waited_s': round(time.time() - t0, 3),
                'samples': samples,
                'reason': reason,
            }
        
        time.sleep(_SEEK_PAN_POLL_S)
    
    # Max time exhausted before required_wait - shouldn't normally happen
    err = round(abs(last_pan - target), 2) if last_pan else None
    reason = 'timeout_before_min_wait'
    if saw_feedback and not saw_useful_feedback:
        reason = 'timeout_hw_stuck_at_zero'
    elif saw_useful_feedback:
        reason = 'timeout_not_within_tol'
    
    return {
        'settled': False,
        'hw_pan': last_pan,
        'err_deg': err,
        'waited_s': round(time.time() - t0, 3),
        'samples': samples,
        'reason': reason,
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
    """Point camera for Seek look-around.

    PTZ robot: gimbal T:133 (or ROS) with optional HW pan wait.
    Hangar attachment=roarm2: skip PTZ pans; limited RoArm base peek, then settle.
    """
    if tilt_deg is None:
        tilt_deg = float(_SEEK_LOOK_TILT_DEG)
    look = _seek_pan_deg_to_rad(pan_deg, tilt_deg)
    _seek_publish_cam_aim(pan_deg, tilt_deg, settled=False, force=True)

    if arm_usb_enabled():
        res = {'path': 'roarm_look', 'ok': True, 'skipped_ptz': True, 'pan_deg': pan_deg}
        try:
            arm = get_roarm()
            if arm is not None and abs(float(pan_deg)) > 1.0:
                import math as _math
                import roarm_ctrl as _rc
                max_b = 0.35
                base_r = max(-max_b, min(max_b, float(pan_deg) * _math.pi / 180.0 * 0.4))
                tuck = _rc.POSES.get('travel_tuck') or _rc.POSES.get('tuck') or {}
                arm.set_joints(
                    base_r,
                    float(tuck.get('shoulder', -0.62)),
                    float(tuck.get('elbow', 0.88)),
                    float(tuck.get('hand', 3.05)),
                    spd=0,
                    acc=10,
                )
                res['arm_base_rad'] = base_r
                time.sleep(settle_s if settle_s is not None else 0.55)
            else:
                if arm is not None:
                    try:
                        arm.pose('travel_tuck')
                    except Exception:
                        pass
                time.sleep(settle_s if settle_s is not None else 0.25)
            olog.info(
                'seek_look',
                f'Seek look pan={pan_deg}° via RoArm/base-skip (no PTZ)',
                pan_deg=pan_deg, path='roarm_look',
                throttle_s=2.0, throttle_key='seek_roarm_look',
            )
        except Exception as e:
            res = {'path': 'roarm_look', 'ok': False, 'error': str(e), 'skipped_ptz': True}
            olog.warn('seek_look', f'RoArm seek look failed: {e}', error=str(e))
            time.sleep(0.2)
        arrival = {
            'settled': True,
            'reason': 'roarm_no_ptz',
            'hw_pan': None,
        }
        _seek_publish_cam_aim(
            pan_deg, tilt_deg,
            hw_pan=None,
            settled=True,
            force=True,
        )
        return look, res, arrival

    res = _execute_agent_tool('send_gimbal_command', look)
    arrival = None
    if wait_hw:
        arrival = _seek_wait_pan_arrived(
            float(pan_deg),
            should_stop=should_stop,
        )
        # Log arrival for diagnostics
        settled = bool(arrival and arrival.get('settled'))
        reason = arrival.get('reason') if arrival else 'no_arrival'
        hw = arrival.get('hw_pan') if arrival else None
        waited = arrival.get('waited_s') if arrival else 0
        olog.info(
            'seek_pan',
            f'Pan to {pan_deg}° complete: settled={settled} reason={reason} '
            f'hw={hw}° waited={waited:.2f}s',
            pan_deg=pan_deg, settled=settled, reason=reason,
            hw_pan=hw, waited_s=waited,
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


def _seek_grab_jpeg(retries=3):
    """Grab a real camera JPEG. Retry — first frames after a pan are often empty."""
    last_err = None
    for _i in range(max(1, int(retries))):
        try:
            jpeg = _grab_jpeg_bytes(max_width=640, quality=72)
            if jpeg and len(jpeg) >= 800:
                return jpeg
            last_err = 'short_jpeg'
        except Exception as e:
            last_err = e
        time.sleep(0.08)
    if last_err:
        olog.warn(
            'ai_seek',
            f'sweep grab failed after {retries} tries: {last_err}',
            error=str(last_err)[:160],
        )
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
        if not jpeg:
            try:
                time.sleep(0.15)
                _seek_look_deg(pan_deg, wait_hw=True, should_stop=ctrl.should_stop)
            except Exception:
                pass
            jpeg = _seek_grab_jpeg()
        
        # Diagnostic: log pan angles to verify rear captures are not front frames
        hw_pan_diag = arrival.get('hw_pan') if arrival else None
        settle_reason = arrival.get('reason') if arrival else 'no_arrival'
        olog.info(
            'seek_capture',
            f'Captured {name} view: cmd={pan_deg}° hw={hw_pan_diag}° '
            f'settled={settled} reason={settle_reason} bytes={len(jpeg) if jpeg else 0}',
            view=name, commanded_pan=pan_deg, hw_pan=hw_pan_diag,
            settled=settled, settle_reason=settle_reason,
            jpeg_bytes=len(jpeg) if jpeg else 0,
        )
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
    try:
        card = _seek_sweep_scorecard(views)
        ctrl.update(last_sweep=card)
        ctrl.append_log('sweep', card.get('summary') or 'sweep', step=step)
    except Exception:
        pass
    return views, found_check


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
        prefer = _seek_prefer_open_turn(views) or 'turn_left'
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
    if should_stop and should_stop():
        return {
            'ok': False, 'skipped': 'stop_requested',
            'path': 't1_fast',
        }
    if not _seek_chassis_allowed():
        if direction in ('left', 'turn_left'):
            side = 'left'
        else:
            side = 'right'
        dur = max(200, int(duration_ms or 600))
        olog.info(
            'ai_seek',
            f'Seek dry-run: skip Fast tank {side} {dur}ms',
            side=side, duration_ms=dur, dry_run=True,
        )
        return {
            'ok': True,
            'dry_run': True,
            'skipped': 'seek_dry_run',
            'side': side,
            'duration_ms': dur,
            'path': 'dry_run',
        }
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
    Dry-run: still aims the camera, never sends T:1 / T:13.
    """
    plan = _seek_nav_plan(action, drive_distance)
    action = plan['action']
    dist = plan['drive_distance']
    dry = _seek_dry_run_active()
    aim = _seek_aim_for_motion(action, open_side=open_side, should_stop=should_stop)
    if should_stop and should_stop():
        return {
            'ok': False,
            'skipped': 'stop_requested',
            'action': action,
            'drive_distance': dist,
            'summary': plan.get('summary'),
            'dry_run': dry,
            'cam_look': (aim or {}).get('label'),
        }

    prev_scope = _seek_drive_scope(True)
    try:
        return _seek_execute_nav_action_inner(
            plan, action, dist, dry, aim, should_stop=should_stop, open_side=open_side,
        )
    finally:
        _seek_drive_scope(prev_scope)


def _seek_execute_nav_action_inner(
    plan, action, dist, dry, aim, *, should_stop=None, open_side=None,
):
    last_res = None
    last_args = {}
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
        if not dry:
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
        if not dry:
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
        'dry_run': dry,
    }


def _seek_forced_json(messages, schema, timeout, max_tokens=256):
    """json_schema → json_object → plain; return (dict, error_or_none)."""
    last_err = None
    for fmt in (schema, {'type': 'json_object'}, None):
        try:
            msg, _body, _cfg = _openai_chat(
                messages,
                max_tokens=max_tokens,
                temperature=0.0,
                tools=None,
                response_format=fmt,
                timeout=timeout,
            )
            if isinstance(msg, dict) and isinstance(msg.get('content'), dict):
                return msg['content'], None
            parsed = _parse_json_from_text(_message_text_content(msg) or '')
            if isinstance(parsed, dict) and parsed:
                return parsed, None
            last_err = 'unparseable llm json'
        except Exception as e:
            last_err = e
            err_s = str(e).lower()
            if 'timed out' in err_s or 'timeout' in err_s:
                break
            continue
    return {}, (str(last_err)[:200] if last_err else 'empty llm json')


def _seek_view_record(name, pan_deg, jpeg, chk=None):
    data_url = None
    if jpeg:
        data_url = 'data:image/jpeg;base64,' + base64.b64encode(jpeg).decode('ascii')
    return {
        'name': name,
        'pan_deg': pan_deg,
        'jpeg': jpeg,
        'bytes': len(jpeg) if jpeg else 0,
        'data_url': data_url,
        'has_target': bool(chk and chk.get('found')),
        'detected_labels': list((chk or {}).get('labels_found') or []),
        'raw_detections': list((chk or {}).get('raw_detections') or []),
        'check': chk,
    }


def _seek_publish_views(ctrl, views):
    if not ctrl or not views:
        return
    payload = {
        'last_views': [{
            'name': v.get('name'),
            'pan_deg': v.get('pan_deg'),
            'bytes': v.get('bytes') or 0,
            'data_url': v.get('data_url'),
            'has_target': v.get('has_target'),
            'detected_labels': v.get('detected_labels') or [],
        } for v in views if isinstance(v, dict)],
    }
    try:
        pano = _stitch_panorama_views(views)
        if pano:
            payload['panorama_data_url'] = (
                'data:image/jpeg;base64,' + base64.b64encode(pano).decode('ascii')
            )
    except Exception:
        pass
    ctrl.update(**payload)


def _seek_recentre_gimbal(ctrl):
    try:
        _seek_look_deg(0.0, wait_hw=True, should_stop=ctrl.should_stop if ctrl else None)
    except Exception:
        try:
            _seek_look_deg(0.0, wait_hw=False, settle_s=0.3)
        except Exception:
            pass


def _seek_analyze_front_view(jpeg, goal_label, timeout=8):
    """Analyze FRONT view only with forced JSON schema (3 yes/no fields).
    
    Returns dict with clear_forward_little, clear_forward_lot, subject_in_scene.
    """
    if not jpeg:
        return {
            'clear_forward_little': 'no',
            'clear_forward_lot': 'no',
            'subject_in_scene': 'no',
            'error': 'no jpeg',
        }
    
    b64_img = base64.b64encode(jpeg).decode('ascii')
    user_content = [
        {
            'type': 'text',
            'text': (
                f'This is the robot\'s FRONT view (camera 0° straight ahead — drive direction).\n'
                f'Target to seek: "{goal_label}"\n\n'
                f'Answer THREE questions (forced yes/no only):\n'
                f'1. clear_forward_little: Can the robot drive forward a LITTLE (short hop ~0.8s)? '
                f'Say "no" ONLY if there is an immediate VERTICAL obstacle (wall, cabinet face, furniture BODY, baby gate) '
                f'or an object sitting ON the floor (bowl, cables, toys) blocking the path. '
                f'IGNORE: patterned carpet, rugs, floor textures, fabric patterns on the ground. '
                f'These are CLEAR to drive — do not treat floor texture as furniture.\n'
                f'2. clear_forward_lot: Can the robot drive forward A LOT (long hop ~1.6s)? '
                f'Say "yes" only if the path is clearly open with plenty of space ahead. '
                f'IGNORE floor patterns and carpet textures — focus on vertical obstacles and objects ON the floor.\n'
                f'3. subject_in_scene: Is "{goal_label}" clearly visible in THIS front view? '
                f'Say "yes" only if you can see the target object/subject.\n\n'
                f'Reply with ONLY the JSON. No prose.'
            ),
        },
        {'type': 'image_url', 'image_url': {'url': f'data:image/jpeg;base64,{b64_img}'}},
    ]
    
    messages = [
        {
            'role': 'system',
            'content': (
                'You analyze robot camera views for ground-level navigation. Answer exactly 3 yes/no questions. '
                'CRITICAL: Patterned carpets, rugs, and floor textures are CLEAR to drive — these are NOT obstacles. '
                'Block only on vertical surfaces (walls, cabinets), furniture bodies (not floor patterns), '
                'baby gates, and physical objects ON the floor (bowls, cables, toys). '
                'Use the forced JSON schema. No explanations.'
            ),
        },
        {'role': 'user', 'content': user_content},
    ]
    
    result, err = _seek_forced_json(
        messages, _SEEK_FRONT_NAV_SCHEMA, timeout=timeout, max_tokens=256,
    )
    if err:
        olog.warn('ai_seek', f'front view LLM failed: {err}')
        return {
            'clear_forward_little': 'no',
            'clear_forward_lot': 'no',
            'subject_in_scene': 'no',
            'error': err,
        }
    return {
        'clear_forward_little': parse_forced_yesno(result.get('clear_forward_little')),
        'clear_forward_lot': parse_forced_yesno(result.get('clear_forward_lot')),
        'subject_in_scene': parse_forced_yesno(result.get('subject_in_scene')),
    }


def _seek_analyze_side_view(jpeg, pan_deg, timeout=8):
    """Analyze ONE side view with forced JSON schema (direction_clear: yes/no).
    
    Returns dict with direction_clear.
    """
    if not jpeg:
        return {'direction_clear': 'no', 'error': 'no jpeg'}
    
    side_name = 'LEFT' if pan_deg < 0 else 'RIGHT'
    b64_img = base64.b64encode(jpeg).decode('ascii')
    user_content = [
        {
            'type': 'text',
            'text': (
                f'This is the robot\'s {side_name} view (camera {int(pan_deg)}°).\n'
                f'The robot front path is blocked, so it may need to rotate toward this side.\n\n'
                f'ONE question (forced yes/no only):\n'
                f'direction_clear: Is this {side_name} direction clear enough to rotate toward? '
                f'Say "yes" if this direction has free space to turn into. '
                f'Say "no" if this direction is also blocked (wall, furniture, obstacle).\n\n'
                f'Reply with ONLY the JSON. No prose.'
            ),
        },
        {'type': 'image_url', 'image_url': {'url': f'data:image/jpeg;base64,{b64_img}'}},
    ]
    
    messages = [
        {
            'role': 'system',
            'content': (
                'You analyze robot camera views. Answer exactly 1 yes/no question about the image. '
                'Use the forced JSON schema. No explanations.'
            ),
        },
        {'role': 'user', 'content': user_content},
    ]
    
    result, err = _seek_forced_json(
        messages, _SEEK_SIDE_NAV_SCHEMA, timeout=timeout, max_tokens=128,
    )
    if err:
        olog.warn('ai_seek', f'side view LLM failed: {err}')
        return {'direction_clear': 'no', 'error': err}
    return {'direction_clear': parse_forced_yesno(result.get('direction_clear'))}


def _seek_analyze_multi_image(front_jpeg, left_jpeg, right_jpeg, goal_label, timeout=8):
    """Analyze ALL THREE views in one LLM call with forced JSON schema.
    
    Optional mode when seek_multi_image=True. Sends FRONT/REAR-LEFT/REAR-RIGHT labeled.
    Returns dict with clear_forward_little, clear_forward_lot, subject_in_scene,
    left_direction_clear, right_direction_clear.
    """
    if not front_jpeg:
        return {
            'clear_forward_little': 'no',
            'clear_forward_lot': 'no',
            'subject_in_scene': 'no',
            'left_direction_clear': 'no',
            'right_direction_clear': 'no',
            'error': 'no front jpeg',
        }
    
    user_content = [
        {
            'type': 'text',
            'text': (
                f'You are analyzing THREE camera views from a ground robot seeking: "{goal_label}".\n\n'
                f'Image 1 = FRONT view (0° straight ahead, robot drive direction)\n'
                f'Image 2 = REAR-LEFT view (-135°)\n'
                f'Image 3 = REAR-RIGHT view (+135°)\n\n'
                f'Answer FIVE yes/no questions (forced JSON only):\n'
                f'1. clear_forward_little: Can robot drive forward a LITTLE (short hop ~0.8s)? '
                f'Say "no" ONLY if VERTICAL obstacle (wall, cabinet, furniture BODY, baby gate) '
                f'or object ON the floor (bowl, cables) blocking FRONT path. '
                f'IGNORE patterned carpet, rugs, floor textures — these are CLEAR.\n'
                f'2. clear_forward_lot: Can robot drive forward A LOT (long hop ~1.6s)? '
                f'Say "yes" only if FRONT path clearly open. IGNORE floor patterns.\n'
                f'3. subject_in_scene: Is "{goal_label}" clearly visible in the FRONT view? '
                f'Say "yes" only if you see the target in image 1.\n'
                f'4. left_direction_clear: Is the LEFT direction (image 2) clear enough to rotate toward? '
                f'Say "yes" if LEFT has free space, "no" if blocked by walls/furniture.\n'
                f'5. right_direction_clear: Is the RIGHT direction (image 3) clear enough to rotate toward? '
                f'Say "yes" if RIGHT has free space, "no" if blocked by walls/furniture.\n\n'
                f'Reply with ONLY the JSON. No prose.'
            ),
        },
        {'type': 'image_url', 'image_url': {'url': f'data:image/jpeg;base64,{base64.b64encode(front_jpeg).decode("ascii")}'}},
    ]
    
    if left_jpeg:
        user_content.append({'type': 'image_url', 'image_url': {'url': f'data:image/jpeg;base64,{base64.b64encode(left_jpeg).decode("ascii")}'}})
    if right_jpeg:
        user_content.append({'type': 'image_url', 'image_url': {'url': f'data:image/jpeg;base64,{base64.b64encode(right_jpeg).decode("ascii")}'}})
    
    messages = [
        {
            'role': 'system',
            'content': (
                'You analyze robot camera views for ground-level navigation. Answer exactly 5 yes/no questions. '
                'CRITICAL: Patterned carpets, rugs, and floor textures are CLEAR to drive — these are NOT obstacles. '
                'Block only on vertical surfaces (walls, cabinets), furniture bodies (not floor patterns), '
                'baby gates, and physical objects ON the floor (bowls, cables, toys). '
                'Use the forced JSON schema. No explanations.'
            ),
        },
        {'role': 'user', 'content': user_content},
    ]

    result, err = _seek_forced_json(
        messages, _SEEK_MULTI_NAV_SCHEMA, timeout=timeout, max_tokens=150,
    )
    if err:
        olog.warn('ai_seek', f'multi-image LLM failed: {err}')
        return {
            'clear_forward_little': 'no',
            'clear_forward_lot': 'no',
            'subject_in_scene': 'no',
            'left_direction_clear': 'no',
            'right_direction_clear': 'no',
            'error': err,
        }
    return {
        'clear_forward_little': parse_forced_yesno(result.get('clear_forward_little')),
        'clear_forward_lot': parse_forced_yesno(result.get('clear_forward_lot')),
        'subject_in_scene': parse_forced_yesno(result.get('subject_in_scene')),
        'left_direction_clear': parse_forced_yesno(result.get('left_direction_clear')),
        'right_direction_clear': parse_forced_yesno(result.get('right_direction_clear')),
    }


def _seek_loop(ctrl, label, conf, max_steps, timeout_s):
    """Seek loop.

    Mode a (detector, llm_scene_nav=False): L/C/R stills + heuristic nav. No LLM.
    Modes b/c with scene nav: front-first forced-JSON LLM nav; sides only if blocked.
    Modes b/c with scene nav off: scan + referee, no chassis.
    """
    t0 = time.time()
    referee = ctrl.referee()
    max_steps = int(max_steps or 0)
    timeout_s = float(timeout_s or 0)
    unlimited = max_steps <= 0
    steps_label = '∞' if unlimited else str(max_steps)
    
    st_dict = ctrl.status()
    dry = bool((st_dict or {}).get('dry_run', True))
    dry_gen = int((st_dict or {}).get('dry_run_gen') or 0)
    if dry and not dry_gen:
        dry_gen = int(_begin_seek_dry_run() or 0)
    
    multi_image_mode = bool((st_dict or {}).get('seek_multi_image', False))
    use_llm_nav = bool((st_dict or {}).get('llm_scene_nav', True))

    last_drive_action = None
    last_turn_direction = None  # Track last turn to prevent flip-flop
    blocked_cycles = 0  # Track how many times we've been blocked for tapering
    
    def _halt(phase, message, step=0, **kwargs):
        try:
            _execute_agent_tool('stop_motors', {})
        except Exception:
            pass
        try:
            _seek_look_deg(0.0, settle_s=0.2)
        except Exception:
            pass
        try:
            act = 'FOUND' if phase == 'found' else (phase or 'end').upper()[:12]
            _seek_oled_set(
                goal=label, referee=referee, phase=phase, step=step,
                activity=act, detail=(message or '')[:40],
                message=message or '', nav_summary=phase or 'end',
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

    def _halt_if_detector_found(chk, step, view_hits=1):
        if referee != REFEREE_DETECTOR or not chk:
            return False
        ctrl.update(last_detection=chk)
        verdict = _seek_found_confident(
            chk, min_conf=DEFAULT_SEEK_FOUND_CONF, view_hits=view_hits, scan_conf=conf,
        )
        if not verdict.get('ok'):
            return False
        ctrl.append_log(
            'found',
            f'FOUND "{label}" via detector at step {step} · {verdict.get("reason")}',
            step=step,
        )
        _seek_run_on_found(ctrl, label)
        _halt(
            'found',
            message=f'Found {label} via detector at step {step}',
            step=step,
            last_detection=chk,
        )
        return True

    def _halt_if_llm_found(chk, step, source='view'):
        if referee != REFEREE_LLM or not chk or not chk.get('found'):
            return False
        ctrl.update(last_detection=chk)
        ctrl.append_log(
            'found',
            f'FOUND "{label}" via LLM at step {step} ({source})',
            step=step,
        )
        _seek_run_on_found(ctrl, label)
        _halt(
            'found',
            message=f'Found {label} via LLM at step {step}',
            step=step,
            last_detection=chk,
        )
        return True

    def _do_drive(action, dist, step, reason=None):
        nonlocal last_drive_action, last_turn_direction
        plan = _seek_nav_plan(action, dist)
        verb = _seek_drive_log_verb(dry)
        ctrl.append_log(
            'nav',
            f'Step {step}: {verb} {plan["summary"]}'
            + (f' · {reason}' if reason else ''),
            step=step, action=plan['action'], dist=plan['drive_distance'],
        )
        ctrl.update(
            last_nav={
                'action': plan['action'],
                'drive_distance': plan['drive_distance'],
                'summary': plan['summary'],
            },
            message=f'Step {step}/{steps_label}: {verb} {plan["summary"]}',
        )
        _seek_oled_set_nav(
            plan['action'], plan['drive_distance'], plan=plan,
            goal=label, referee=referee, phase='drive', step=step,
            message=f'{plan["action"]} {plan["drive_distance"]}',
        )
        try:
            _seek_execute_nav_action(
                plan['action'], plan['drive_distance'], should_stop=ctrl.should_stop,
            )
            last_drive_action = plan['action']
            if plan['action'] in ('turn_left', 'turn_right'):
                last_turn_direction = plan['action']
            ctrl.append_log(
                'drive',
                f'Step {step}: {verb} {plan["action"]}/{plan["drive_distance"]}',
                step=step,
            )
        except Exception as e:
            ctrl.append_log('warn', f'Step {step}: drive failed: {e}', step=step)
        time.sleep(DEFAULT_SEEK_STEP_PAUSE_S)
    
    try:
        _seek_force_tools_on()
        _seek_disable_steady()
        _seek_oled_set(
            goal=label, referee=referee, phase='running', step=0,
            activity='start', detail=f'ref {referee}',
            message=f'Seeking {label}', nav_summary='start',
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
            if not dry:
                batt_block = _seek_battery_block_reason()
                if batt_block:
                    _halt('failed', batt_block, step=step - 1)
                    return

            # Mode a (and scene-nav off): classic L/C/R. No LLM nav.
            if not use_llm_nav:
                ctrl.update(
                    step=step, seek_phase='triple_scan',
                    message=f'Step {step}/{steps_label}: classic L/C/R scan…',
                )
                ctrl.append_log(
                    'scan',
                    f'Step {step}: classic triple-view '
                    + ('+ heuristic nav' if referee == REFEREE_DETECTOR else '+ LLM found, no drive'),
                    step=step,
                )
                views, found_check = _seek_capture_triple_views(
                    ctrl, step, steps_label, goal_label=label,
                    conf_threshold=conf, referee=REFEREE_DETECTOR,
                )
                hits = sum(1 for v in (views or []) if isinstance(v, dict) and v.get('has_target'))
                if referee == REFEREE_DETECTOR and _halt_if_detector_found(
                    found_check, step, view_hits=max(1, hits),
                ):
                    return
                if referee == REFEREE_LLM:
                    for v in (views or []):
                        jpeg = v.get('jpeg') if isinstance(v, dict) else None
                        if not jpeg:
                            continue
                        chk = llm_goal_check(label, jpeg=jpeg)
                        if _halt_if_llm_found(chk, step, source=v.get('name') or 'view'):
                            return
                    time.sleep(DEFAULT_SEEK_STEP_PAUSE_S)
                    continue
                plan = _seek_nav_decide(views, label)
                _do_drive(
                    plan.get('action') or 'forward',
                    plan.get('drive_distance') or 'short',
                    step,
                    reason=plan.get('reason') or plan.get('source'),
                )
                continue

            # ========== STEP 1: CAPTURE FRONT STILL (0°) ==========
            ctrl.update(
                step=step, seek_phase='front_scan',
                message=f'Step {step}/{steps_label}: capturing front view (0°)...'
            )
            ctrl.append_log('scan', f'Step {step}: pan to front 0° and capture', step=step)
            
            _seek_oled_set(
                goal=label, referee=referee, phase='front_scan', step=step,
                activity='pan 0°', detail='wait settle',
                message=f'step {step} front', nav_summary='front scan',
                obstacle='',
            )
            
            # Wait for real pan settle (commanded angle + time, not lying HW ~0°)
            try:
                _seek_look_deg(0.0, wait_hw=True, should_stop=ctrl.should_stop)
            except Exception as e:
                ctrl.append_log('warn', f'Step {step}: pan wait failed: {e}', step=step)
                time.sleep(0.5)  # Fallback settle time
            
            front_jpeg = _seek_grab_jpeg()
            if not front_jpeg or len(front_jpeg) < 800:
                ctrl.append_log('warn', f'Step {step}: front capture weak/missing, retry', step=step)
                time.sleep(0.2)
                try:
                    _seek_look_deg(0.0, wait_hw=True, should_stop=ctrl.should_stop)
                except Exception:
                    pass
                front_jpeg = _seek_grab_jpeg()
            
            if not front_jpeg:
                ctrl.append_log('error', f'Step {step}: no front image, turn fallback', step=step)
                action = 'turn_left' if (step % 2) == 1 else 'turn_right'
                _do_drive(action, 'short', step, reason='no front image')
                continue
            
            left_jpeg = None
            right_jpeg = None
            front_det = None
            if referee == REFEREE_DETECTOR:
                front_det = seek_goal_check(
                    label, referee=REFEREE_DETECTOR, conf_threshold=conf, jpeg=front_jpeg,
                )
            _seek_publish_views(ctrl, [_seek_view_record('straight', 0.0, front_jpeg, front_det)])

            # ========== STEP 2: LLM ANALYSIS (SINGLE OR MULTI-IMAGE) ==========
            if multi_image_mode:
                # MULTI-IMAGE MODE: Capture all 3 views, one LLM call
                ctrl.update(
                    seek_phase='multi_llm',
                    message=f'Step {step}/{steps_label}: LLM analyzing 3 views...'
                )
                ctrl.append_log('llm', f'Step {step}: LLM multi-image (FRONT/LEFT/RIGHT)', step=step)
                
                _seek_oled_set(
                    goal=label, referee=referee, phase='multi_llm', step=step,
                    activity='LLM 3-view', detail='5 questions',
                    message=f'step {step} LLM', nav_summary='llm',
                    obstacle='',
                )
                
                # Capture left and right views
                left_jpeg = None
                right_jpeg = None
                try:
                    _seek_look_deg(-135.0, wait_hw=True, should_stop=ctrl.should_stop)
                    left_jpeg = _seek_grab_jpeg()
                except Exception as e:
                    ctrl.append_log('warn', f'Step {step}: left capture failed: {e}', step=step)
                
                try:
                    _seek_look_deg(135.0, wait_hw=True, should_stop=ctrl.should_stop)
                    right_jpeg = _seek_grab_jpeg()
                except Exception as e:
                    ctrl.append_log('warn', f'Step {step}: right capture failed: {e}', step=step)
                
                # Return to front
                try:
                    _seek_look_deg(0.0, wait_hw=True, should_stop=ctrl.should_stop)
                except Exception:
                    pass
                
                # Single LLM call with all 3 images
                multi_result = _seek_analyze_multi_image(front_jpeg, left_jpeg, right_jpeg, label, timeout=8)
                llm_error = multi_result.get('error')
                
                clear_little = multi_result.get('clear_forward_little') == 'yes'
                clear_lot = multi_result.get('clear_forward_lot') == 'yes'
                subject_visible = multi_result.get('subject_in_scene') == 'yes'
                left_clear = multi_result.get('left_direction_clear') == 'yes'
                right_clear = multi_result.get('right_direction_clear') == 'yes'
                
                ctrl.append_log(
                    'llm',
                    f'Step {step}: multi LLM → fwd_little={multi_result.get("clear_forward_little")} '
                    f'fwd_lot={multi_result.get("clear_forward_lot")} subject={multi_result.get("subject_in_scene")} '
                    f'left={multi_result.get("left_direction_clear")} right={multi_result.get("right_direction_clear")}',
                    step=step,
                )
                _seek_publish_views(ctrl, [
                    _seek_view_record('left', -135.0, left_jpeg),
                    _seek_view_record('straight', 0.0, front_jpeg, front_det),
                    _seek_view_record('right', 135.0, right_jpeg),
                ])
            else:
                # SINGLE-IMAGE MODE (DEFAULT): Front view only, side scans when blocked
                ctrl.update(
                    seek_phase='front_llm',
                    message=f'Step {step}/{steps_label}: LLM analyzing front view...'
                )
                ctrl.append_log('llm', f'Step {step}: LLM front analysis (3 questions)', step=step)
                
                _seek_oled_set(
                    goal=label, referee=referee, phase='front_llm', step=step,
                    activity='LLM front', detail='3 questions',
                    message=f'step {step} LLM', nav_summary='llm',
                    obstacle='',
                )
                
                front_result = _seek_analyze_front_view(front_jpeg, label, timeout=8)
                llm_error = front_result.get('error')
                
                clear_little = front_result.get('clear_forward_little') == 'yes'
                clear_lot = front_result.get('clear_forward_lot') == 'yes'
                subject_visible = front_result.get('subject_in_scene') == 'yes'
                left_clear = False  # Will scan later if blocked
                right_clear = False
                
                ctrl.append_log(
                    'llm',
                    f'Step {step}: front LLM → little={front_result.get("clear_forward_little")} '
                    f'lot={front_result.get("clear_forward_lot")} '
                    f'subject={front_result.get("subject_in_scene")}',
                    step=step,
                )

            if llm_error:
                ctrl.append_log(
                    'warn',
                    f'Step {step}: LLM nav failed ({llm_error}) → heuristic fallback',
                    step=step,
                )
                plan = _seek_nav_decide(
                    [_seek_view_record('straight', 0.0, front_jpeg, front_det)],
                    label,
                )
                _do_drive(
                    plan.get('action') or 'turn_left',
                    plan.get('drive_distance') or 'short',
                    step,
                    reason=plan.get('reason') or 'heuristic after LLM fail',
                )
                continue

            if subject_visible and _halt_if_llm_found(
                {'found': True, 'parse_ok': True, 'goal_label': label, 'referee': REFEREE_LLM},
                step,
                source='front',
            ):
                return

            if _halt_if_detector_found(front_det, step, view_hits=1):
                return

            # ========== STEP 3: IF CLEAR FORWARD → HOP ==========
            if clear_little or clear_lot:
                blocked_cycles = 0
                dist = 'long' if clear_lot else 'short'
                _do_drive('forward', dist, step, reason='front clear')
                continue
            
            # ========== STEP 4: FRONT BLOCKED → SCAN SIDES (OR USE MULTI-IMAGE RESULTS) ==========
            blocked_cycles += 1
            
            if not multi_image_mode:
                # SINGLE-IMAGE MODE: Need to scan sides individually
                # Taper: first cycle uses ±135, subsequent use narrower
                if blocked_cycles == 1:
                    side_pan_deg = 135
                elif blocked_cycles == 2:
                    side_pan_deg = 90
                else:
                    side_pan_deg = 60  # Corridor glance
                
                ctrl.append_log(
                    'nav',
                    f'Step {step}: front blocked (neither clear) → scan sides ±{side_pan_deg}°',
                    step=step,
                )
                
                # Scan left side
                ctrl.update(
                    seek_phase='side_scan',
                    message=f'Step {step}/{steps_label}: scanning left side (−{side_pan_deg}°)...'
                )
                ctrl.append_log('scan', f'Step {step}: pan left −{side_pan_deg}°', step=step)
                
                _seek_oled_set(
                    goal=label, referee=referee, phase='side_scan', step=step,
                    activity=f'pan −{side_pan_deg}°', detail='wait settle',
                    message=f'step {step} left', nav_summary='side scan',
                    obstacle='',
                )
                
                try:
                    _seek_look_deg(-side_pan_deg, wait_hw=True, should_stop=ctrl.should_stop)
                except Exception as e:
                    ctrl.append_log('warn', f'Step {step}: left pan failed: {e}', step=step)
                    time.sleep(0.5)
                
                left_jpeg = _seek_grab_jpeg()
                if not left_jpeg or len(left_jpeg) < 800:
                    time.sleep(0.2)
                    left_jpeg = _seek_grab_jpeg()
                
                # LLM analyze left
                ctrl.update(message=f'Step {step}/{steps_label}: LLM analyzing left side...')
                ctrl.append_log('llm', f'Step {step}: LLM left side (direction_clear?)', step=step)
                
                left_result = _seek_analyze_side_view(left_jpeg, -side_pan_deg, timeout=8) if left_jpeg else {'direction_clear': 'no'}
                left_clear = left_result.get('direction_clear', 'no') == 'yes'
                
                ctrl.append_log(
                    'llm',
                    f'Step {step}: left LLM → direction_clear={left_result.get("direction_clear")}',
                    step=step,
                )
                
                # Scan right side
                ctrl.update(
                    seek_phase='side_scan',
                    message=f'Step {step}/{steps_label}: scanning right side (+{side_pan_deg}°)...'
                )
                ctrl.append_log('scan', f'Step {step}: pan right +{side_pan_deg}°', step=step)
                
                _seek_oled_set(
                    goal=label, referee=referee, phase='side_scan', step=step,
                    activity=f'pan +{side_pan_deg}°', detail='wait settle',
                    message=f'step {step} right', nav_summary='side scan',
                    obstacle='',
                )
                
                try:
                    _seek_look_deg(+side_pan_deg, wait_hw=True, should_stop=ctrl.should_stop)
                except Exception as e:
                    ctrl.append_log('warn', f'Step {step}: right pan failed: {e}', step=step)
                    time.sleep(0.5)
                
                right_jpeg = _seek_grab_jpeg()
                if not right_jpeg or len(right_jpeg) < 800:
                    time.sleep(0.2)
                    right_jpeg = _seek_grab_jpeg()
                
                # LLM analyze right
                ctrl.update(message=f'Step {step}/{steps_label}: LLM analyzing right side...')
                ctrl.append_log('llm', f'Step {step}: LLM right side (direction_clear?)', step=step)
                
                right_result = _seek_analyze_side_view(right_jpeg, +side_pan_deg, timeout=8) if right_jpeg else {'direction_clear': 'no'}
                right_clear = right_result.get('direction_clear') == 'yes'
                
                ctrl.append_log(
                    'llm',
                    f'Step {step}: right LLM → direction_clear={right_result.get("direction_clear")}',
                    step=step,
                )
                _seek_publish_views(ctrl, [
                    _seek_view_record('left', -side_pan_deg, left_jpeg),
                    _seek_view_record('straight', 0.0, front_jpeg, front_det),
                    _seek_view_record('right', side_pan_deg, right_jpeg),
                ])
                _seek_recentre_gimbal(ctrl)
            else:
                # MULTI-IMAGE MODE: Already have side results from the multi-image LLM call
                ctrl.append_log(
                    'nav',
                    f'Step {step}: front blocked → using multi-image side results '
                    f'(left={left_clear} right={right_clear})',
                    step=step,
                )
                _seek_recentre_gimbal(ctrl)

            for side_name, side_jpeg in (('left', left_jpeg), ('right', right_jpeg)):
                if not side_jpeg:
                    continue
                if referee == REFEREE_DETECTOR:
                    side_chk = seek_goal_check(
                        label, referee=REFEREE_DETECTOR, conf_threshold=conf, jpeg=side_jpeg,
                    )
                    if _halt_if_detector_found(side_chk, step, view_hits=1):
                        return
                elif referee == REFEREE_LLM:
                    side_chk = llm_goal_check(label, jpeg=side_jpeg)
                    if _halt_if_llm_found(side_chk, step, source=side_name):
                        return

            # ========== STEP 5: IF SIDE CLEAR → ROTATE ==========
            if left_clear or right_clear:
                # Hysteresis: prefer continuing in same direction to avoid flip-flop
                # Only reverse if the committed side became blocked
                if left_clear and right_clear:
                    # Both clear: prefer last turn direction (commit), or pick based on step if first time
                    if last_turn_direction in ('turn_left', 'turn_right'):
                        turn_side = last_turn_direction
                    else:
                        turn_side = 'turn_right' if (step % 2) == 0 else 'turn_left'
                elif left_clear and not right_clear:
                    # Only left clear
                    turn_side = 'turn_left'
                elif right_clear and not left_clear:
                    # Only right clear
                    turn_side = 'turn_right'
                else:
                    # Shouldn't reach here, but fallback
                    turn_side = last_turn_direction if last_turn_direction else 'turn_left'
                
                _do_drive(
                    turn_side, 'short', step,
                    reason=f'side clear (last={last_turn_direction or "none"})',
                )
                continue
            
            # ========== STEP 6: ALL BLOCKED → SMALL ROTATE OR STOP ==========
            ctrl.append_log(
                'nav',
                f'Step {step}: all directions blocked (front + left + right) → small rotate',
                step=step,
            )
            
            # Hysteresis: prefer continuing same turn direction instead of flip-flopping
            if last_turn_direction in ('turn_left', 'turn_right'):
                action = last_turn_direction
            else:
                action = 'turn_left' if (step % 2) == 1 else 'turn_right'
            
            _do_drive(action, 'short', step, reason='all blocked, exploratory')
        
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
    finally:
        _seek_clear_tool_override()
        if dry:
            _end_seek_dry_run(dry_gen)
        else:
            _set_seek_dry_run(False)


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


def _parse_bool_flag(raw, default=False):
    if raw is None:
        return bool(default)
    if isinstance(raw, bool):
        return raw
    s = str(raw).strip().lower()
    if s in ('1', 'true', 'yes', 'on'):
        return True
    if s in ('0', 'false', 'no', 'off'):
        return False
    return bool(default)


@app.route('/api/ai/seek/start', methods=['POST'])
def api_ai_seek_start():
    data = request.get_json(silent=True) or {}
    dry_run = _parse_bool_flag(
        data.get('dry_run') if 'dry_run' in data else None,
        default=DEFAULT_SEEK_DRY_RUN,
    )
    confirm_live = _parse_bool_flag(data.get('confirm_live'), default=False)
    live_err = _seek_live_start_error(dry_run=dry_run, confirm_live=confirm_live)
    if live_err:
        return jsonify({
            'success': False,
            'error': live_err,
            'dry_run_required': True,
        }), 400
    batt_block = None if dry_run else _seek_battery_block_reason()
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
    seek_multi_image = bool(data.get('seek_multi_image', False))
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
        seek_multi_image=seek_multi_image,
        dry_run=dry_run,
    )
    code = 200 if result.get('success') else 400
    if result.get('success'):
        olog.info(
            'ai_seek',
            f'Seek started ({referee}) for {result.get("status", {}).get("goal_label")} '
            f'on_found={on_found} dry_run={dry_run}',
            goal=goal, referee=referee, on_found=on_found, dry_run=dry_run,
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
                        if not jpeg:
                            ctrl.append_log(
                                'detect',
                                f'Scan {step} · pan {int(pan)}° tilt {int(tilt)}° · no jpeg — skip',
                            )
                            continue
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
                        verdict = _seek_found_confident(
                            chk,
                            min_conf=DEFAULT_SEEK_FOUND_CONF,
                            view_hits=1,
                            scan_conf=conf,
                        )
                        if chk.get('found') and verdict.get('ok'):
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
            if not jpeg:
                lost += 1
                ctrl.append_log('nav', f'Lock lost ({lost}/4) — no jpeg')
                if lost >= 4:
                    locked = False
                    ctrl.update(locked=False, message=f'Lost {label} — resuming PTZ sweep')
                continue
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
            _track_goto(0.0, 0.0)
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
    # Track is PTZ-only, but line-follow / Chat may still be moving.
    try:
        _emergency_stop_motion(source='track_stop', stop_seek=False)
    except Exception:
        pass
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
    return render_template('index.html', ugv_ai_token=_ai_auth_token())

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
    - T:144 arm UI (E/Z/R) → USB RoArm when hangar attachment=roarm2, else base UART
    - USB-native arm T:100/102/105/114/121/210 → USB when attachment=roarm2
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
    arm_ui_type = f.get('cmd_config', {}).get('cmd_arm_ctrl_ui', 144)
    arm_ui_types = {144, '144', arm_ui_type, str(arm_ui_type) if arm_ui_type is not None else None}
    roarm_raw_types = {100, 102, 105, 114, 121, 210, '100', '102', '105', '114', '121', '210'}

    # Dry-run Seek: refuse non-zero chassis (sticks + leftover heartbeats).
    # Live Seek/Track: same — autonomy owns the wheels. Zeros / STOP still go through.
    if (
        t in chassis_types
        and not force_stop
        and not _chassis_cmd_is_zero(cmd)
        and (
            not _seek_chassis_allowed()
            or (_autonomy_owns_chassis() and not _seek_thread_may_drive())
        )
    ):
        why = 'seek_dry_run' if not _seek_chassis_allowed() else 'autonomy_owns_chassis'
        olog.info('ai_seek', f'Dropped chassis cmd ({why})', T=t, reason=why)
        return {
            'path': 'blocked',
            'ok': True,
            'reason': why,
            'dry_run': not _seek_chassis_allowed(),
        }

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

    # ---- Arm UI stick (T:144 E/Z/R) — USB when hangar attachment=roarm2 ----
    if t in arm_ui_types:
        result = _route_arm_ui_cmd(cmd)
        result['mode'] = mode
        return result

    # ---- USB-native RoArm JSON when hangar wants RoArm ----
    if t in roarm_raw_types and arm_usb_enabled():
        result = _route_roarm_raw(cmd)
        result['mode'] = mode
        return result

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
        use_lidar=bool(getattr(base, 'use_lidar', False)),
        lidar_port=getattr(getattr(base, 'rl', None), 'lidar_port', None),
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
